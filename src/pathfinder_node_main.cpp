#include "var_kqt_feleves/pathfinder_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp> // <-- ÚJ INCLUDE

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // 1. Hozd létre a node-ot
  auto pathfinder_node = std::make_shared<PathfinderNode>();

  // 2. Hozz létre egy Többszálú Végrehajtót (pl. 4 szállal)
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);

  // 3. Add hozzá a node-ot a végrehajtóhoz
  executor.add_node(pathfinder_node);

  // 4. Pörgesd a végrehajtót
  //RCLCPP_INFO(pathfinder_node->get_logger(), "MultiThreadedExecutor indul...");
  executor.spin(); // Ez most már több szálon fogadja a hívásokat

  rclcpp::shutdown();
  return 0;
}