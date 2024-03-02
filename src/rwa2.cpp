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
#include "../include/ARIAC-2023/rwa2.hpp"

/**
 * @brief Subscriber function that subscibes to competition state msg and calls the functions to trigger the start/end of the competition accordingly.
 * The function also submits the order in the right order according to the priority and finally the submit order service once all the orders are completed and submitted
 * 
 * @param msg 
 */
void CompetitionARIAC::CompetitionStateCallback(const ariac_msgs::msg::CompetitionState::SharedPtr msg)
{
    if (msg->competition_state == ariac_msgs::msg::CompetitionState::READY)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("CompState Subscriber"), "Competition state is 1, calling service client to Start Competition...");
        callService_start();
    }
    else if (msg->competition_state == ariac_msgs::msg::CompetitionState::STARTED && count_2 == 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("CompState Subscriber"), "Competition state is 2, Competition Started...\n");
        count_2++;
    }
    else if (msg->competition_state == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE && submitted == false)
    {
        RCLCPP_INFO(rclcpp::get_logger("CompState Subscriber"), "Competition state is 3, all orders retrived...\n");

        if(bin_flag && conv_flag && orders_list.size() > 0) 
        {
            OrderAccomplishment(); 
        }
        
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Order Submission    "), "Submitting Orders ...");
        for (int i = 0; i < int(orders_list.size()); i++)
        {
            if (orders_list[i].priority == 1)
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Order Submission    "), "Order submitted with Priority 1 :" + orders_list[i].id);
                callService_submit(orders_list[i].id);
            }
        }
        for (int i = 0; i < int(orders_list.size()); i++)
        {
            if (orders_list[i].priority == 0)
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Order Submission    "), "Order submitted with Priority 0 :" + orders_list[i].id);
                callService_submit(orders_list[i].id);
            }
        }

        submitted = true;
    }
    else if (msg->competition_state == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE && submitted == true)
    {
        RCLCPP_INFO(rclcpp::get_logger("CompState Subscriber"), "Competition state is 3, all orders submitted, calling service client to End Competition...");
        callService_end();
        comp_state_sub.reset();
    }
}


/**
 * @brief Callback function that is called whener a new order is received. It stores the orders in the data structure
 * 
 * @param order_msg 
 */
void CompetitionARIAC::OrderCallback(const ariac_msgs::msg::Order::SharedPtr order_msg)
{
    RCLCPP_INFO(rclcpp::get_logger("Orders    Subscriber"), "Received order !");
    order_::Orders order;
    order.id = order_msg->id;
    order.type = order_msg->type;
    order.priority = order_msg->priority;

    if (order_msg->type == ariac_msgs::msg::Order::KITTING)
    {
        const auto &kitting_task = order_msg->kitting_task;
        order.kitting_type.agv_number = static_cast<int>(kitting_task.agv_number);

        order.kitting_type.tray_id = static_cast<int>(kitting_task.tray_id);
        order.kitting_type.destination = kitting_task.destination;
        for (const auto &kitting_part : order_msg->kitting_task.parts)
        {
            order_::KittingPart ki_part;
            ki_part.quadrant = kitting_part.quadrant;
            ki_part.color = kitting_part.part.color;
            ki_part.type = kitting_part.part.type;
            order.kitting_type.parts.push_back(ki_part);
        }
    }
    else if (order_msg->type == ariac_msgs::msg::Order::ASSEMBLY)
    {
        const auto &assembly_task = order_msg->assembly_task;
        order.AssemComb_type.station = static_cast<int>(assembly_task.station);

        for (size_t i = 0; i < assembly_task.agv_numbers.size(); ++i)
        {
            order.AssemComb_type.agv_numbers.push_back(static_cast<int>(assembly_task.agv_numbers[i]));
        }
        for (const auto &assem_part : assembly_task.parts)
        {
            order_::AssemCombPart ac_part;
            ac_part.color = assem_part.part.color;
            ac_part.type = assem_part.part.type;
            ac_part.pose_stamp = assem_part.assembled_pose;
            ac_part.install_direction = assem_part.install_direction;
            order.AssemComb_type.parts.push_back(ac_part);
        }
    }
    else
    {
        const auto &combined_task = order_msg->combined_task;
        order.AssemComb_type.station = static_cast<int>(combined_task.station);
        for (const auto &assem_part : combined_task.parts)
        {
            order_::AssemCombPart ac_part;
            ac_part.color = assem_part.part.color;
            ac_part.type = assem_part.part.type;
            ac_part.pose_stamp = assem_part.assembled_pose;
            ac_part.install_direction = assem_part.install_direction;
            order.AssemComb_type.parts.push_back(ac_part);
        }
    }

    order_::Orders order_obj;

    RCLCPP_INFO_STREAM(rclcpp::get_logger("Orders    Subscriber"), "  Orders details - ");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Orders    Subscriber"), "\tOrder ID      : " + order.id);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Orders    Subscriber"), "\tOrder type    : " + order_obj.order_type[order.type]);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Orders    Subscriber"), "\tPriority      : " + std::to_string(order.priority)+"\n");

    orders_list.push_back(order);
    list_size = orders_list.size();

}


/**
 * @brief Function to check if the necessary parts are available in the bins and conveyor belt and prompts the user if the order can be completed successfully or not.
 * 
 */
void CompetitionARIAC::OrderAccomplishment()
{
    color_::ColorParts Col_part_obj;
    order_::Orders order = orders_list[order_index];
    std::vector<order_::KittingPart> order_parts = order.kitting_type.parts;
    order_::KittingType kits ;
    pick_part::PartInfo bpart;
    kits.agv_number = order.kitting_type.agv_number;
    kits.destination = order.kitting_type.destination;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Parts availability  "), "Checking for availability of Parts... \n"); 
    if(order.type == ariac_msgs::msg::Order::KITTING) 
    {
        for(int i = 0; i < int(order_parts.size()); i++) {
            if(order_parts[i].part_status == 0) {
                for(int j = 0; j < int(partp.parts.size()); j++) {
                    if(partp.parts[j].quantity != 0) {
                        if(partp.parts[j].color == order_parts[i].color && partp.parts[j].type == order_parts[i].type) {
                            
                            RCLCPP_INFO_STREAM(rclcpp::get_logger("Parts   availability"), "Parts: "); 
                            RCLCPP_INFO_STREAM(rclcpp::get_logger("Parts   availability"), "  -Type  : " + Col_part_obj.PART[order_parts[i].type]);
                            RCLCPP_INFO_STREAM(rclcpp::get_logger("Parts   availability"), "  -Color : " + Col_part_obj.COLOR[order_parts[i].color]);
                            RCLCPP_INFO_STREAM(rclcpp::get_logger("Parts   availability"), "  -Quantity : " + std::to_string(partp.parts[j].quantity));
                            RCLCPP_INFO_STREAM(rclcpp::get_logger("Parts   availability"), "  -Can be found in Bin " + std::to_string(partp.parts[j].bin_number));
                            RCLCPP_INFO_STREAM(rclcpp::get_logger("Ceiling  Robot task "), "   Pick " + Col_part_obj.COLOR[order_parts[i].color]+ " "+Col_part_obj.PART[order_parts[i].type]+" from Bin " + std::to_string(partp.parts[j].bin_number));
                            if(partp.parts[j].quantity == 1)  
                                RCLCPP_INFO_STREAM(rclcpp::get_logger("Parts availability  "), "   " + Col_part_obj.COLOR[order_parts[i].color]+ " "+Col_part_obj.PART[order_parts[i].type] + " exhausted in Bin " + std::to_string(partp.parts[j].bin_number));
                            RCLCPP_INFO_STREAM(rclcpp::get_logger("Ceiling  Robot task "), "   Place " + Col_part_obj.COLOR[order_parts[i].color]+ " "+Col_part_obj.PART[order_parts[i].type]+" on AGV "+std::to_string(kits.agv_number)+" at QUADRANT "+std::to_string(order_parts[i].quadrant)+"\n");
    
                            partp.parts[j].quantity -= 1;
                            order_parts[i].part_status = 1;
                            break;
                        }
                    }
                }
            }
        }
        bool order_status = 1;
        for (int i = 0; i < int(order_parts.size()); i++) {
            if(order_parts[i].part_status == 0) {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Parts availability  "), "Part Missing: [" + Col_part_obj.PART[order_parts[i].type] + "," 
                + Col_part_obj.COLOR[order_parts[i].color] + "]");
                order_status = 0;
            }
        }

        order.order_status = order_status;
        if(order.order_status == 0)
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Parts availability  "), "Order ID : " + order.id + " faces Insufficient Parts Challenge!!! \n");

        RCLCPP_INFO_STREAM(rclcpp::get_logger("AGV Destination task"),"\t Lock AGV " + std::to_string(kits.agv_number));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("AGV Destination task"),"\t Move AGV to " + Col_part_obj.DEST[kits.destination]+"\n");

        if(order_index < int(orders_list.size())) {
            order_index += 1;
        }

    }
}

/**
 * @brief Callback function that gets called when the subscriber subscribes to the topic ariac/bin_parts topic and stores the details
 * about the available parts in a data structure
 * 
 * @param msg 
 */
void CompetitionARIAC::BinPartCallback(const ariac_msgs::msg::BinParts::SharedPtr msg)
{
    color_::ColorParts Col_part_obj;
    RCLCPP_INFO(rclcpp::get_logger("Bin Parts subscriber"), "Parts available in Bins - ");

    //  Loop through all the bin info in the message and store it in the data structure
    for (const auto &bin_info : msg->bins)
    {    
        // Loop through all the part lots in the bin info and store it in the data structure
        for (const auto &part_lot : bin_info.parts)
        {
            
            pick_part::PartInfo info;
            info.quantity = part_lot.quantity;
            info.bin_number = bin_info.bin_number;
            info.pickup_type = "bin";
            // Loop through all the parts in the part lot and store it in the data structure
            std::vector<ariac_msgs::msg::Part> parts_vector;
            // add the Part object to the container
            parts_vector.push_back(part_lot.part);
            // create a new vector from the container
            auto parts_array = std::vector<ariac_msgs::msg::Part>(parts_vector.begin(), parts_vector.end());

            for (auto &parts : parts_array)
            {
                info.color = parts.color;
                info.type = parts.type;
            }
            partp.parts.push_back(info);
        }

    }

    for (long unsigned int i = 0; i < partp.parts.size(); i++)
    {
        if(partp.parts[i].pickup_type == "bin") 
        {    
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Bin Parts subscriber"), "\t- Bin Number : " + std::to_string(partp.parts[i].bin_number));
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Bin Parts subscriber"), "\t  - Parts : ");
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Bin Parts subscriber"), "\t    - Color  : " + Col_part_obj.COLOR[partp.parts[i].color]);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Bin Parts subscriber"), "\t    - Type   : " + Col_part_obj.PART[partp.parts[i].type]);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Bin Parts subscriber"), "\t  - Quantity : " + std::to_string(partp.parts[i].quantity));
        }
    }
    bin_flag = true;
    bin_part_sub.reset();
}


/**
 * @brief Callback function that gets called when the subscriber subscribes to the topic ariac/conveyor_parts topic and stores the details
 * about the available parts in a data structure
 * 
 * @param conv_part_msg 
 */
void CompetitionARIAC::ConvPartCallback(const ariac_msgs::msg::ConveyorParts::SharedPtr conv_part_msg)
{
    if (bin_flag)
    {
        color_::ColorParts Col_part_obj;
        RCLCPP_INFO(rclcpp::get_logger("Conveyor Parts subs "), "Parts available on Conveyor Belt - ");

        for (const auto &part_lot : conv_part_msg->parts)
        {
            pick_part::PartInfo info;
            info.quantity = part_lot.quantity;
            info.pickup_type = "conveyor";
            info.bin_number = 0;
            // Loop through all the parts in the part lot and store it in the data structure
            std::vector<ariac_msgs::msg::Part> parts_vector;
            parts_vector.push_back(part_lot.part);
            auto parts_array = std::vector<ariac_msgs::msg::Part>(parts_vector.begin(), parts_vector.end());

            for (auto &parts : parts_array)
            {
                info.color = parts.color;
                info.type = parts.type;
            }
            partp.parts.push_back(info);
        }

        
        long unsigned int parts_size = partp.parts.size();
        for (long unsigned int j = 0; j < parts_size; j++)
        {
            if(partp.parts[j].pickup_type == "conveyor") 
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Conveyor Parts subs "), "\t- Parts : ");
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Conveyor Parts subs "), "\t    - Color  : " + Col_part_obj.COLOR[partp.parts[j].color]);
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Conveyor Parts subs "), "\t    - Type   : " + Col_part_obj.PART[partp.parts[j].type]);
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Conveyor Parts subs "), "\t  - Quantity : " + std::to_string(partp.parts[j].quantity));
            }
        }
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Conveyor Parts subs "), " ");

        for (long unsigned int j = 0; j < parts_size; j++) 
        {
            order_bins.push_back(partp.parts[j].bin_number);
        }

        for(auto bin: bin_space){
            if(std::find(order_bins.begin(), order_bins.end(), bin) == order_bins.end()){
                empty_bins.push_back(bin);
            }
        } 

        for (long unsigned int j = 0; j < parts_size; j++)
        {
            if(partp.parts[j].pickup_type == "conveyor") 
            {
                partp.parts[j].bin_number = empty_bins[0];
                for(int i=0 ; i<partp.parts[j].quantity ; i++)
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor    Robot task "), "\t Pick " + Col_part_obj.COLOR[partp.parts[j].color] + " " + Col_part_obj.PART[partp.parts[j].type] + " and place in Bin " + std::to_string(partp.parts[j].bin_number) +" - slot " + std::to_string(slot));
                    slot++;
                }               
            }
        }

        FloorRobot();

        conv_flag = true;
        conv_part_sub.reset();
    }
}


/**
 * @brief Function that makes the floor robot pick tray and put it on a AGV
 * 
 */

void CompetitionARIAC::FloorRobot()
{
    color_::ColorParts Col_part_obj;
    order_::Orders order = orders_list[order_index];
    order_::KittingType kits ;

    kits.agv_number = order.kitting_type.agv_number;
    kits.tray_id = order.kitting_type.tray_id;

    if(order.type == ariac_msgs::msg::Order::KITTING) 
    {    
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor    Robot task "),"");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor    Robot task "),"\t Change to tray gripper ");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor    Robot task "),"\t Pick Tray number " + std::to_string(kits.tray_id));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor    Robot task "),"\t Place tray on AGV " + std::to_string(kits.agv_number));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor    Robot task "),"\t Change to part gripper\n");
    }
}



/**
 * @brief Function that calls the trigger service to start the competition depending on competition state
 * 
 */
void CompetitionARIAC::callService_start()
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
void CompetitionARIAC::callService_end()
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
 * @brief Function that calls the service to submit the order
 * 
 * @param order 
 */
void CompetitionARIAC::callService_submit(std::string order)
{
    auto submit_order_request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
    submit_order_request->order_id = order;

    auto submit_order_future = submit_order_client_->async_send_request(submit_order_request);

}


