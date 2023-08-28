/**
 * @file rwa1.cpp
 * @author Kiran S Patil,Aniruddh Balram,Vyshnv Achuthan,Badrinarayanan 
 * @brief Program that create subscribers and subscribes to orders topic and store the contents of the message in a data structure 
 * @version 0.1
 * @date 2023-03-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "../include/ARIAC-2023/rwa1.hpp"
/**
 * @brief Subscriber function that subscibes to competition state msg and calls the functions to trigger the start/end of the competition accordingly.
 * 
 * @param msg 
 */

void CompetitionStateSubscriber::competitionStateCallback(const ariac_msgs::msg::CompetitionState::SharedPtr msg)
{
    if (msg->competition_state == ariac_msgs::msg::CompetitionState::READY) 
    {
        RCLCPP_INFO(get_logger(), "Competition state is 1, calling service client to Start Competition...");
        callService_start();
    }
    else if(msg->competition_state == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE && submitted == true) 
    {
        RCLCPP_INFO(get_logger(), "Competition state is 3, calling service client to End Competition...");
        callService_end();
    }
}

/**
 * @brief Function that calls the trigger service to start the competition depending on competition state
 * 
 */
void CompetitionStateSubscriber::callService_start()
{
    auto client = create_client<std_srvs::srv::Trigger>("/ariac/start_competition");

    if (!client->wait_for_service(std::chrono::seconds(1))) 
    {
        RCLCPP_ERROR(get_logger(), "Service not available");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);
}

/**
 * @brief Function that calls the trigger service to start the competition depending on competition state
 * 
 */
void CompetitionStateSubscriber::callService_end()
{
    auto client_ = create_client<std_srvs::srv::Trigger>("/ariac/end_competition");

    if (!client_->wait_for_service(std::chrono::seconds(1))) 
    {
        RCLCPP_ERROR(get_logger(), "Service not available");
        return;
    }

    auto request_ = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future_ = client_->async_send_request(request_);
}

/**
 * @brief Callback function that is called whener a new order is received. It stores the orders in the data structure
 * 
 * @param order_msg 
 */
void CompetitionStateSubscriber::orderCallback(const ariac_msgs::msg::Order::SharedPtr order_msg)  
{    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Received order:");
    Orders order;
    order.id = order_msg->id;
    order.type = order_msg->type;
    order.priority = order_msg->priority;

    if(order_msg->type == ariac_msgs::msg::Order::KITTING) {
        const auto& kitting_task = order_msg->kitting_task;
        order.kitting_type.agv_number = static_cast<int>(kitting_task.agv_number);
        order.kitting_type.tray_id = static_cast<int>(kitting_task.tray_id);
        order.kitting_type.destination = kitting_task.destination;
        for (const auto& kitting_part : order_msg->kitting_task.parts) {
            KittingPart ki_part;
            ki_part.quadrant = kitting_part.quadrant;
            ki_part.color = kitting_part.part.color;
            ki_part.type = kitting_part.part.type;
            order.kitting_type.parts.push_back(ki_part);
        }
    }
    else if(order_msg->type == ariac_msgs::msg::Order::ASSEMBLY) {
        const auto& assembly_task = order_msg->assembly_task;
        order.a_c_type.station = static_cast<int>(assembly_task.station);
        
        for (size_t i = 0; i < assembly_task.agv_numbers.size(); ++i) {
            order.a_c_type.agv_numbers.push_back(static_cast<int>(assembly_task.agv_numbers[i]));
        }
        for(const auto& assem_part : assembly_task.parts){
            A_C_Part ac_part;
            ac_part.color = assem_part.part.color;
            ac_part.type = assem_part.part.type;
            ac_part.pose_stamp = assem_part.assembled_pose;
            ac_part.install_direction = assem_part.install_direction;
            order.a_c_type.parts.push_back(ac_part);
        }
    }
    else {
        const auto& combined_task = order_msg->combined_task;
        order.a_c_type.station = static_cast<int>(combined_task.station);
        for(const auto& assem_part : combined_task.parts){
            A_C_Part ac_part;
            ac_part.color = assem_part.part.color;
            ac_part.type = assem_part.part.type;
            ac_part.pose_stamp = assem_part.assembled_pose;
            ac_part.install_direction = assem_part.install_direction;
            order.a_c_type.parts.push_back(ac_part);
        }
    }
    orders_list.push_back(order);

    list_size = orders_list.size();
    
    RCLCPP_INFO_STREAM(rclcpp::get_logger("main"), "Orders with Priority 1 :");
    for(int i = 0; i < int(orders_list.size()); i++) {
        if(orders_list[i].priority == 1) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("main"), orders_list[i].id + "\n");
        }      
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("main"), "Orders with Priority 0 :");
    for(int i = 0; i < int(orders_list.size()); i++) {
        if(orders_list[i].priority == 0) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("main"), orders_list[i].id + "\n");
        }      
    }

    if(list_size == 3)
    {
        for(int i = 0; i < int(orders_list.size()); i++) {
            if(orders_list[i].priority == 1) {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("main"),"Order submitted with Priority 1 :" + orders_list[i].id + "\n");
                callService_submit(orders_list[i].id);
                
            }      
        }
        for(int i = 0; i < int(orders_list.size()); i++) {
            if(orders_list[i].priority == 0) {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("main"), "Order submitted with Priority 0 :" + orders_list[i].id + "\n");
                callService_submit(orders_list[i].id);
            }      
        }

        submitted = true;
    }
}
/**
 * @brief Function that calls the service to submit the order
 * 
 * @param order 
 */

void CompetitionStateSubscriber::callService_submit(std::string order)
{
    auto submit_order_request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
    submit_order_request->order_id = order;

    auto submit_order_future = submit_order_client_->async_send_request(submit_order_request);

    //// code to wait for response
    // if (rclcpp::spin_until_future_complete(get_node_base_interface(), submit_order_future) == rclcpp::FutureReturnCode::SUCCESS) 
    // {
    // auto submit_order_response = submit_order_future.get();
    // if (submit_order_response->success) {
    //     RCLCPP_INFO(get_logger(), "Submit order service call successful: %s", submit_order_response->message.c_str());
    // } else {
    //     RCLCPP_ERROR(get_logger(), "Submit order service call failed: %s", submit_order_response->message.c_str());
    // }
    // } 
    // else {
    // RCLCPP_ERROR(get_logger(), "Submit order service call failed");
    // }
    
}

