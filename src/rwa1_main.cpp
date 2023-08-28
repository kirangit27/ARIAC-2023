/**
 * @file rwa1_main.cpp
 * @author Kiran S Patil,Aniruddh Balram,Vyshnv Achuthan,Badrinarayanan
 * @brief Main function for rwa1.cpp
 * @version 0.1
 * @date 2023-03-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "../include/ARIAC-2023/rwa1.hpp"
#include "rclcpp/rclcpp.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CompetitionStateSubscriber>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
