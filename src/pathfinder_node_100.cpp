#include "pathfinder_node_100.hpp"
#include <sstream>
#include <iomanip>

using namespace std::chrono_literals;

// ==========================================
//              KONSTRUKTOR
// ==========================================
PathfinderNode100::PathfinderNode100() : Node("pathfinder_node_100"), gen_(std::random_device{}())
{
    // Paraméterek: Alapértelmezett méret 100
    this->declare_parameter<bool>("automatic_mode", true);
    this->declare_parameter<int>("map_size", 100); 
    
    map_size_ = this->get_parameter("map_size").as_int();
    automatic_mode_ = this->get_parameter("automatic_mode").as_bool();

    // Labirintus megoldó inicializálása
    solver_ = std::make_unique<MazeSolver>(map_size_);
    // A solver kényszeríthette a páratlan méretet, kérdezzük le
    map_size_ = solver_->getMapSize(); 

    // Publisherek létrehozása (_100 utótaggal)
    grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_grid_100", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_markers_100", 10);

    // Módválasztó (Automata vs Manuális)
    if (automatic_mode_) {
        timer_ = this->create_wall_timer(
            5000ms, std::bind(&PathfinderNode100::timer_callback, this)); 
        RCLCPP_INFO(this->get_logger(), LOG_GREEN "Indítás AUTOMATA módban (100x100). 5 másodpercenként újratervezés." LOG_RESET);
    } else {
        trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
            "trigger_generation_100",
            std::bind(&PathfinderNode100::handle_trigger_service, this, std::placeholders::_1, std::placeholders::_2)
        );
        RCLCPP_INFO(this->get_logger(), LOG_YELLOW "Indítás MANUÁLIS módban (100x100)." LOG_RESET);
        RCLCPP_INFO(this->get_logger(), LOG_YELLOW "Hívd a '/trigger_generation_100' szolgáltatást a generáláshoz." LOG_RESET);
    }
    
    RCLCPP_INFO(this->get_logger(), LOG_GREEN "PathfinderNode100 (Generator + A*) elindult. Térkép méret: %d x %d" LOG_RESET , map_size_, map_size_);
}

// ==========================================
//              CALLBACKEK
// ==========================================

void PathfinderNode100::timer_callback()
{
    std::lock_guard<std::mutex> lock(run_mutex_);
    RCLCPP_INFO(this->get_logger(), "--- [100x100] Új labirintus és útvonal generálása... ---");
    run_generation_cycle();
}

void PathfinderNode100::handle_trigger_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    std::lock_guard<std::mutex> lock(run_mutex_);
    (void)request; // Nem használjuk a bemenetet
    RCLCPP_INFO(this->get_logger(), "--- [100x100] Manuális generálás kérése ---");
    run_generation_cycle();
    response->success = true;
    response->message = "Nagy labirintus és A* útvonal legenerálva.";
}

// ==========================================
//              FŐ LOGIKA
// ==========================================

void PathfinderNode100::run_generation_cycle()
{
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 1. Labirintus generálása
    std::vector<Point> valid_entrances, valid_exits;
    solver_->generateMaze(valid_entrances, valid_exits);

    if (valid_entrances.empty() || valid_exits.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Hiba: Nem sikerült bejáratot generálni.");
        return;
    }

    // 2. Start és Cél pontok véletlenszerű kiválasztása
    std::uniform_int_distribution<> entrance_dist(0, valid_entrances.size() - 1);
    std::uniform_int_distribution<> exit_dist(0, valid_exits.size() - 1);
    
    Point start_point = valid_entrances[entrance_dist(gen_)];
    Point end_point = valid_exits[exit_dist(gen_)];

    // 3. A* Keresés futtatása
    // (Ez iteratív, nem rekurzív, tehát biztonságos nagy pályán is!)
    std::vector<Point> shortest_path = solver_->findShortestPathAStar(start_point, end_point);

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end_time - start_time;

    // Logolás
    if (!shortest_path.empty()) {
        RCLCPP_INFO(this->get_logger(), "Generálás + A* Keresés: " LOG_CYAN "%.2f ms" LOG_RESET, duration.count());
        RCLCPP_INFO(this->get_logger(), "Útvonal hossza: %zu lépés", shortest_path.size());
    } else {
        RCLCPP_WARN(this->get_logger(), "Nem található útvonal!");
    }

    // 4. Vizualizáció (adatok átadása)
    publish_visualizations(shortest_path, start_point, end_point);
}

// ==========================================
//              VIZUALIZÁCIÓ
// ==========================================

void PathfinderNode100::publish_visualizations(const std::vector<Point>& shortest_path, Point start, Point end)
{
    auto now = this->get_clock()->now();
    double map_resolution = 0.1; 
    std::string frame_id = "map";

    // 1. OccupancyGrid (Térkép)
    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.header.stamp = now;
    grid_msg.header.frame_id = frame_id;
    grid_msg.info.resolution = map_resolution;
    grid_msg.info.width = map_size_;
    grid_msg.info.height = map_size_;
    grid_msg.info.origin.position.x = 0.0; 
    grid_msg.info.origin.position.y = 0.0;
    grid_msg.data.resize(map_size_ * map_size_);

    const auto& map_grid = solver_->getMapGrid();
    for (int y = 0; y < map_size_; ++y) {
        for (int x = 0; x < map_size_; ++x) {
            grid_msg.data[y * map_size_ + x] = map_grid[y][x];
        }
    }
    grid_pub_->publish(grid_msg);

    // 2. Markerek (Útvonal + Start/Cél)
    visualization_msgs::msg::MarkerArray marker_array;

    // Előző markerek törlése (hogy ne maradjon ott a régi út)
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = frame_id;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    // A* Útvonal (PIROS VONAL)
    if (!shortest_path.empty()) {
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = frame_id;
        path_marker.header.stamp = now;
        path_marker.ns = "path_100";
        path_marker.id = 1;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.scale.x = map_resolution * 0.4; // Vonal vastagsága
        path_marker.color.r = 1.0; 
        path_marker.color.g = 0.0; 
        path_marker.color.b = 0.0; 
        path_marker.color.a = 1.0; // Piros szín

        for (const auto& p : shortest_path) {
            geometry_msgs::msg::Point gp;
            gp.x = (p.x + 0.5) * map_resolution;
            gp.y = (p.y + 0.5) * map_resolution;
            gp.z = 0.02; // Kicsit a padló felett
            path_marker.points.push_back(gp);
        }
        marker_array.markers.push_back(path_marker);
    }

    // Start (ZÖLD KOCKA) és Cél (KÉK KOCKA)
    auto create_cube = [&](Point p, int id, float r, float g, float b) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = now;
        m.ns = "points_100";
        m.id = id;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = (p.x + 0.5) * map_resolution;
        m.pose.position.y = (p.y + 0.5) * map_resolution;
        m.pose.position.z = 0.02;
        m.scale.x = map_resolution; m.scale.y = map_resolution; m.scale.z = 0.1;
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0;
        return m;
    };

    marker_array.markers.push_back(create_cube(start, 2, 0.0, 1.0, 0.0)); // Zöld
    marker_array.markers.push_back(create_cube(end, 3, 0.0, 0.0, 1.0));   // Kék

    marker_pub_->publish(marker_array);
}

// ==========================================
//              MAIN FÜGGVÉNY
// ==========================================
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathfinderNode100>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}