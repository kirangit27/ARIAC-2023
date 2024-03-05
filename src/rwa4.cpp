/**
 * @file rwa4.cpp
 * @author Kiran S Patil,Aniruddh Balram,Vyshnv Achuthan,Badrinarayanan
 * @brief Program that create subscribers and subscribes to orders topic and store the contents of the message in a data structure.
 * The program also checks the parts available in bins and conveyor belt and storees them in a data structure. Finally, the program
 * prompts if the order can be succefully completed or not, depending on the available parts.
 * @version 0.1
 * @date 2023-04-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "../include/ARIAC-2023/rwa4.hpp"
#include "../include/ARIAC-2023/data_maps.hpp"


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
    else if ((msg->competition_state == ariac_msgs::msg::CompetitionState::STARTED || msg->competition_state == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE) && orders_list.size() > 0 )
    {
        while(beam_flag  && orders_list.size() > 0 && order_counter!=0) 
        {
            RCLCPP_INFO(rclcpp::get_logger("CompState Subscriber"), "Competition state is 2, Competition Started...\n");
            OrderAccomplishment(); 
        }  
    }
    else if (msg->competition_state == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE && submitted == false && orders_list.size() == 0)
    {
          
        RCLCPP_INFO(rclcpp::get_logger("CompState Subscriber"), "Competition state is 3, all orders retrived...\n");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Order Submission    "), "Submitting Orders ...");
        for (int i = 0; i < int(orders_list_submit.size()); i++)
        {
            if (orders_list_submit[i].priority == 1)
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Order Submission    "), "Order submitted with Priority 1 :" + orders_list[i].id);
                callService_submit(orders_list_submit[i].id);
            }
        }
        for (int i = 0; i < int(orders_list_submit.size()); i++)
        {
            if (orders_list_submit[i].priority == 0)
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Order Submission    "), "Order submitted with Priority 0 :" + orders_list[i].id);
                callService_submit(orders_list_submit[i].id);
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
        else if (msg->competition_state == ariac_msgs::msg::CompetitionState::ENDED)
    {
        RCLCPP_INFO(rclcpp::get_logger("CompState Subscriber"), "Competition state is 4, Competition Ended...");
        // comp_end = true;
    }
}


/**
 * @brief Callback function that is called whener a new order is received. It stores the orders in the data structure
 * 
 * @param order_msg 
 */
void CompetitionARIAC::OrderCallback(const ariac_msgs::msg::Order::SharedPtr order_msg)
{
    RCLCPP_INFO(rclcpp::get_logger("Orders    Subscriber"), " ");
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
    order_counter++;
    
    if(order.priority)
        priority_index = order_counter - 1;
    priority_order = order.priority;
    order_::Orders order_obj;

    RCLCPP_INFO_STREAM(rclcpp::get_logger("Orders    Subscriber"), "  Orders details - ");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Orders    Subscriber"), "\tOrder ID      : " + order.id);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Orders    Subscriber"), "\tOrder type    : " + order_obj.order_type[order.type]);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Orders    Subscriber"), "\tPriority      : " + std::to_string(order.priority)+"\n");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Orders    Subscriber"), "\torder_counter : " + std::to_string(order_counter)+"\n");

    orders_list.push_back(order);
    orders_list_submit.push_back(order);
    
    list_size = orders_list.size();

}


/**
 * @brief Function to check if the necessary parts are available in the bins and conveyor belt and prompts the user if the order can be completed successfully or not.
 * 
 */
void CompetitionARIAC::OrderAccomplishment()
{
    color_::ColorParts Col_part_obj;
    order_::Orders order = orders_list[0];

    if(priority_order && priority_index!=-1) {
        order = orders_list[priority_index];        
    }
    if(priority_order == true) {
        priority_order = false;
    }

    constants::Constants const_obj;
    std::vector<order_::KittingPart> order_parts = order.kitting_type.parts;
    order_::KittingType kits ;
    order_::AssemCombPart assem;
    int asmbly=order.AssemComb_type.station;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Station"),static_cast<int>(asmbly)); 
    std::vector<order_::AssemCombPart> order_p= order.AssemComb_type.parts;
    ariac_msgs::msg::PartPose part_to_pick;
    geometry_msgs::msg::PoseStamped pose; 
    // pick_part::PartInfo bpart;
    
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Parts availability  "), "Checking for availability of Parts... \n"); 
    if(order.type == ariac_msgs::msg::Order::KITTING) 
    {
        kits.agv_number = order.kitting_type.agv_number;
        kits.destination = order.kitting_type.destination;
        kits.tray_id = order.kitting_type.tray_id;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor    Robot task "),"");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor    Robot task "),"\t Change to tray gripper ");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor    Robot task "),"\t Pick Tray number " + std::to_string(kits.tray_id));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor    Robot task "),"\t Place tray on AGV " + std::to_string(kits.agv_number));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor    Robot task "),"\t Change to part gripper\n");

        if(priority_order) { OrderAccomplishment(); }
        
        FloorRobotPickandPlaceTray(kits.tray_id, kits.agv_number);
        // tray_flag = true;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor    Robot task "),"\t tray pick and place done\n");
        if(priority_order) { OrderAccomplishment(); }
        for(int i = 0; i < int(order_parts.size()); i++) {
            if(priority_order) { OrderAccomplishment(); }
            if(order_parts[i].part_status == 0) {
                if(priority_order) { OrderAccomplishment(); }
                for(int j = 0; j < int(partp.parts.size()); j++) {
                    if(priority_order) { OrderAccomplishment(); }
                    if(partp.parts[j].quantity != 0) {
                        if(priority_order) { OrderAccomplishment(); }
                        if(partp.parts[j].color == order_parts[i].color && partp.parts[j].type == order_parts[i].type) {
                            if(priority_order) { OrderAccomplishment(); }                                              
                            RCLCPP_INFO_STREAM(rclcpp::get_logger("Parts   availability"), "Parts: "); 
                            RCLCPP_INFO_STREAM(rclcpp::get_logger("Parts   availability"), "  -Type  : " + Col_part_obj.PART[order_parts[i].type]);
                            RCLCPP_INFO_STREAM(rclcpp::get_logger("Parts   availability"), "  -Color : " + Col_part_obj.COLOR[order_parts[i].color]);
                            RCLCPP_INFO_STREAM(rclcpp::get_logger("Parts   availability"), "  -Quantity : " + std::to_string(partp.parts[j].quantity));
                            RCLCPP_INFO_STREAM(rclcpp::get_logger("Parts   availability"), "  -Can be found in Bin " + std::to_string(partp.parts[j].bin_number));
                            RCLCPP_INFO_STREAM(rclcpp::get_logger("Ceiling  Robot task "), "   Pick " + Col_part_obj.COLOR[order_parts[i].color]+ " "+Col_part_obj.PART[order_parts[i].type]+" from Bin " + std::to_string(partp.parts[j].bin_number));
                            const_obj.quad_parts_map[int(order_parts[i].quadrant)].first = order_parts[i].color; 
                            const_obj.quad_parts_map[int(order_parts[i].quadrant)].second = order_parts[i].type;              
                            if(priority_order) { OrderAccomplishment(); }
                            FloorRobotPickBinPart(order_parts[i]);
                            FloorRobotPlacePartOnKitTray(kits.agv_number, order_parts[i].quadrant);  
                            if(priority_order) { OrderAccomplishment(); }                      
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
        if(priority_order) { OrderAccomplishment(); }
        bool order_status = 1;
        for (int i = 0; i < int(order_parts.size()); i++) {
            if(priority_order) { OrderAccomplishment(); }
            if(order_parts[i].part_status == 0) {
                if(priority_order) { OrderAccomplishment(); }
                const_obj.quad_parts_map[int(order_parts[i].quadrant)].first = order_parts[i].color; 
                const_obj.quad_parts_map[int(order_parts[i].quadrant)].second = order_parts[i].type;
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Parts availability  "), "Part Missing: [" + Col_part_obj.PART[order_parts[i].type] + "," 
                + Col_part_obj.COLOR[order_parts[i].color] + "]");
                order_status = 0;
            }
        }
        if(priority_order) { OrderAccomplishment(); }
        order.order_status = order_status;
        if(order.order_status == 0)
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Parts availability  "), "Order ID : " + order.id + " faces Insufficient Parts Challenge!!! \n");

        // if(order_index < int(orders_list.size())) {
        //     order_index += 1;
        // }
        if(priority_order) { OrderAccomplishment(); }
        for (const auto& pair : const_obj.quad_parts_map) 
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Quadrant Maps "), "quadrant - "+std::to_string(pair.first)+", type -  "+ Col_part_obj.COLOR[pair.second.first]+ " "+ Col_part_obj.PART[pair.second.second]);
        }
        if(priority_order) { OrderAccomplishment(); }
            // Check quality
            auto request1 = std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
            request1->order_id = order.id;
            if(priority_order) { OrderAccomplishment(); }
            auto result1 = quality_checker_->async_send_request(request1);
            result1.wait();
            RCLCPP_INFO_STREAM(get_logger(), "Result at 0 sec: " + std::to_string(result1.get()->all_passed));
            // Check quality 2
            rclcpp::sleep_for(std::chrono::seconds(3));
            auto request = std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
            request->order_id = order.id;
            if(priority_order) { OrderAccomplishment(); }
            auto result = quality_checker_->async_send_request(request);
            result.wait();
            
            RCLCPP_INFO_STREAM(get_logger(), "Result after 3 sec: " + std::to_string(result.get()->all_passed));
            if(priority_order) { OrderAccomplishment(); }
            if (!result.get()->all_passed) 
            {
                if(priority_order) { OrderAccomplishment(); }
                RCLCPP_ERROR(get_logger(), "Issue with shipment");
                RCLCPP_INFO_STREAM(get_logger(), "Faulty part ? : " + std::to_string(result.get()->quadrant1.faulty_part));
                
                if (result.get()->incorrect_tray) 
                {
                    RCLCPP_ERROR(get_logger(), "Incorrect Tray");
                    if(priority_order) { OrderAccomplishment(); }
                }

                // Quadarant 1 faulty parts
                if (result.get()->quadrant1.faulty_part) 
                {
                    if(priority_order) { OrderAccomplishment(); }
                    RCLCPP_ERROR(get_logger(), "Faulty part in Quadrant 1");
                    FloorRobotDisposeParts(kits.agv_number,1);
                    FloorRobotPickBinPartReplaceMissing(const_obj.quad_parts_map[1].first,const_obj.quad_parts_map[1].second);
                    FloorRobotPlacePartOnKitTrayMissing(kits.agv_number, 1,const_obj.quad_parts_map[1].second);
                    if(priority_order) { OrderAccomplishment(); }
                }
                if (result.get()->quadrant1.flipped_part) 
                {
                    if(priority_order) { OrderAccomplishment(); }
                    RCLCPP_ERROR(get_logger(), "Flipped part in Quadrant 1");
                    FloorRobotPickFlippedPart(kits.agv_number,1);
                    CeilingRobotPlaceFlippedPartOnKitTray(kits.agv_number,1);
                    if(priority_order) { OrderAccomplishment(); }
                }
                if (result.get()->quadrant1.incorrect_part_color) 
                {
                    if(priority_order) { OrderAccomplishment(); }
                    RCLCPP_ERROR(get_logger(), "Incorrect part color in Quadrant 1");
                    if(priority_order) { OrderAccomplishment(); }
                }
                if (result.get()->quadrant1.incorrect_part_type) 
                {
                    if(priority_order) { OrderAccomplishment(); }
                    RCLCPP_ERROR(get_logger(), "Incorrect part type in Quadrant 1");
                    FloorRobotDisposeParts(kits.agv_number,1);
                    if(priority_order) { OrderAccomplishment(); }
                }
                if (result.get()->quadrant1.missing_part) 
                {
                    if(priority_order) { OrderAccomplishment(); }
                    RCLCPP_ERROR(get_logger(), "Missing part in Quadrant 1");
                    FloorRobotPickBinPartReplaceMissing(const_obj.quad_parts_map[1].first,const_obj.quad_parts_map[1].second);
                    FloorRobotPlacePartOnKitTrayMissing(kits.agv_number, 1,const_obj.quad_parts_map[1].second);
                    if(priority_order) { OrderAccomplishment(); }
                }


                // Quadarant 2 faulty parts
                if (result.get()->quadrant2.faulty_part) 
                {
                    if(priority_order) { OrderAccomplishment(); }
                    RCLCPP_ERROR(get_logger(), "Faulty part in Quadrant 2");
                    FloorRobotDisposeParts(kits.agv_number,2);
                    FloorRobotPickBinPartReplaceMissing(const_obj.quad_parts_map[2].first,const_obj.quad_parts_map[2].second);
                    FloorRobotPlacePartOnKitTrayMissing(kits.agv_number, 2,const_obj.quad_parts_map[2].second);
                    if(priority_order) { OrderAccomplishment(); }
                }
                if (result.get()->quadrant2.flipped_part) 
                {
                    if(priority_order) { OrderAccomplishment(); }
                    RCLCPP_ERROR(get_logger(), "Flipped part in Quadrant 2");
                    FloorRobotPickFlippedPart(kits.agv_number,1);
                    CeilingRobotPlaceFlippedPartOnKitTray(kits.agv_number,1);
                    if(priority_order) { OrderAccomplishment(); }
                }
                if (result.get()->quadrant2.incorrect_part_color) 
                {
                    if(priority_order) { OrderAccomplishment(); }
                    RCLCPP_ERROR(get_logger(), "Incorrect part color in Quadrant 2");
                    if(priority_order) { OrderAccomplishment(); }
                }
                if (result.get()->quadrant2.incorrect_part_type) 
                {
                    if(priority_order) { OrderAccomplishment(); }
                    RCLCPP_ERROR(get_logger(), "Incorrect part type in Quadrant 2");
                    FloorRobotDisposeParts(kits.agv_number,2);
                    if(priority_order) { OrderAccomplishment(); }
                }
                if (result.get()->quadrant2.missing_part) 
                {
                    if(priority_order) { OrderAccomplishment(); }
                    RCLCPP_ERROR(get_logger(), "Missing part in Quadrant 2");
                    FloorRobotPickBinPartReplaceMissing(const_obj.quad_parts_map[2].first,const_obj.quad_parts_map[2].second);
                    FloorRobotPlacePartOnKitTrayMissing(kits.agv_number, 2,const_obj.quad_parts_map[2].second);
                    if(priority_order) { OrderAccomplishment(); }
                }


                // Quadarant 3 faulty parts
                if (result.get()->quadrant3.faulty_part) 
                {
                    if(priority_order) { OrderAccomplishment(); }
                    RCLCPP_ERROR(get_logger(), "Faulty part in Quadrant 3");
                    FloorRobotDisposeParts(kits.agv_number,3);
                    FloorRobotPickBinPartReplaceMissing(const_obj.quad_parts_map[3].first,const_obj.quad_parts_map[3].second);
                    FloorRobotPlacePartOnKitTrayMissing(kits.agv_number, 3,const_obj.quad_parts_map[3].second);
                    if(priority_order) { OrderAccomplishment(); }
                }
                if (result.get()->quadrant3.flipped_part) 
                {
                    if(priority_order) { OrderAccomplishment(); }
                    RCLCPP_ERROR(get_logger(), "Flipped part in Quadrant 3");
                    FloorRobotPickFlippedPart(kits.agv_number,1);
                    CeilingRobotPlaceFlippedPartOnKitTray(kits.agv_number,1);
                    if(priority_order) { OrderAccomplishment(); }
                }
                if (result.get()->quadrant3.incorrect_part_color) 
                {
                    RCLCPP_ERROR(get_logger(), "Incorrect part color in Quadrant 3");
                    if(priority_order) { OrderAccomplishment(); }
                }
                if (result.get()->quadrant3.incorrect_part_type) 
                {
                    if(priority_order) { OrderAccomplishment(); }
                    RCLCPP_ERROR(get_logger(), "Incorrect part type in Quadrant 3");
                    FloorRobotDisposeParts(kits.agv_number,3);
                    if(priority_order) { OrderAccomplishment(); }
                }
                if (result.get()->quadrant3.missing_part) 
                {
                    if(priority_order) { OrderAccomplishment(); }
                    RCLCPP_ERROR(get_logger(), "Missing part in Quadrant 3");
                    FloorRobotPickBinPartReplaceMissing(const_obj.quad_parts_map[3].first,const_obj.quad_parts_map[3].second);
                    FloorRobotPlacePartOnKitTrayMissing(kits.agv_number, 3,const_obj.quad_parts_map[3].second);
                    if(priority_order) { OrderAccomplishment(); }
                }


                // Quadarant 4 faulty parts
                if (result.get()->quadrant4.faulty_part) 
                {
                    if(priority_order) { OrderAccomplishment(); }
                    RCLCPP_ERROR(get_logger(), "Faulty part in Quadrant 4");
                    FloorRobotDisposeParts(kits.agv_number,4);
                    FloorRobotPickBinPartReplaceMissing(const_obj.quad_parts_map[4].first,const_obj.quad_parts_map[4].second);
                    FloorRobotPlacePartOnKitTrayMissing(kits.agv_number, 4,const_obj.quad_parts_map[4].second);
                    if(priority_order) { OrderAccomplishment(); }
                }
                if (result.get()->quadrant4.flipped_part) 
                {
                    if(priority_order) { OrderAccomplishment(); }
                    RCLCPP_ERROR(get_logger(), "Flipped part in Quadrant 4");
                    FloorRobotPickFlippedPart(kits.agv_number,1);
                    CeilingRobotPlaceFlippedPartOnKitTray(kits.agv_number,1);
                    if(priority_order) { OrderAccomplishment(); }
                }
                if (result.get()->quadrant4.incorrect_part_color) 
                {
                    RCLCPP_ERROR(get_logger(), "Incorrect part color in Quadrant 4");
                    if(priority_order) { OrderAccomplishment(); }
                }
                if (result.get()->quadrant4.incorrect_part_type) 
                {
                    if(priority_order) { OrderAccomplishment(); }
                    RCLCPP_ERROR(get_logger(), "Incorrect part type in Quadrant 4");
                    FloorRobotDisposeParts(kits.agv_number,4);
                    if(priority_order) { OrderAccomplishment(); }
                }
                if (result.get()->quadrant4.missing_part) 
                {
                    if(priority_order) { OrderAccomplishment(); }
                    RCLCPP_ERROR(get_logger(), "Missing part in Quadrant 4");
                    FloorRobotPickBinPartReplaceMissing(const_obj.quad_parts_map[4].first,const_obj.quad_parts_map[4].second);
                    FloorRobotPlacePartOnKitTrayMissing(kits.agv_number, 4,const_obj.quad_parts_map[4].second); 
                    if(priority_order) { OrderAccomplishment(); }
                }

                quality_check_flag = true;

            }
            if (result.get()->all_passed || quality_check_flag) 
            {
                if(priority_order) { OrderAccomplishment(); }
                RCLCPP_INFO_STREAM(rclcpp::get_logger("AGV Destination task"),"\t Lock AGV " + std::to_string(kits.agv_number));
                RCLCPP_INFO_STREAM(rclcpp::get_logger("AGV Destination task"),"\t Move AGV to " + Col_part_obj.DEST[kits.destination]+"\n");
                MoveAGV(kits.agv_number,kits.destination);
                if(priority_order) { OrderAccomplishment(); }
            }

            order_counter--;
            if(priority_index!=-1) {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Erasing priority order id:"),order.id);
                orders_list.erase(orders_list.begin() + priority_index);
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Order List Size :"), std::to_string(orders_list.size()));
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Order List Submit Size :"), std::to_string(orders_list_submit.size()));

                priority_index =  -1;
            }
            else {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Erasing order id:"),order.id);
                orders_list.erase(orders_list.begin());
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Order List Size :"), std::to_string(orders_list.size()));
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Order List Submit Size :"), std::to_string(orders_list_submit.size()));
            }

    }
    else if(order.type == ariac_msgs::msg::Order::ASSEMBLY) 
    {
        if(priority_order) { OrderAccomplishment(); }
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Assembly task"),"\t Assembly " );

        ariac_msgs::msg::AssemblyTask task;

        for(int i=0;i<order.AssemComb_type.agv_numbers.size();i++)
        {
            if(priority_order) { OrderAccomplishment(); }
            int destination;
            if (asmbly == 1 || asmbly == 3) {
            destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_FRONT;
            } else if (asmbly == 2 || asmbly == 4) {
            destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_BACK;
            }
            if(priority_order) { OrderAccomplishment(); }
            MoveAGV(order.AssemComb_type.agv_numbers[i],destination);
            if(priority_order) { OrderAccomplishment(); }
        }

        RCLCPP_INFO_STREAM(rclcpp::get_logger("Parts   availability"), "ASSEMBLY TASK: ");
        // CeilingRobotMoveToAssemblyStation(asmbly.station);
        if(priority_order) { OrderAccomplishment(); }
        CeilingRobotMoveToAssemblyStation(order.AssemComb_type.station);
        ariac_msgs::msg::Part Bart;
        std::vector<order_::AssemCombPart> assem_parts = order.AssemComb_type.parts;
        std::vector<ariac_msgs::msg::Part> agv_parts;
        if(priority_order) { OrderAccomplishment(); }
        for (const auto& assem_part : assem_parts) {
        ariac_msgs::msg::Part agv_part;
        agv_part.type = assem_part.type;
        agv_part.color = assem_part.color;
        agv_parts.push_back(agv_part);
        }

        
        if(priority_order) { OrderAccomplishment(); }
        auto client = create_client<ariac_msgs::srv::GetPreAssemblyPoses>("/ariac/get_pre_assembly_poses");
        auto request = std::make_shared<ariac_msgs::srv::GetPreAssemblyPoses::Request>();
        request->order_id = order.id;
        auto result = client->async_send_request(request);
        std::vector<ariac_msgs::msg::PartPose> agv_part_poses; 
        if (result.get()->valid_id) {
            agv_part_poses = result.get()->parts;
            if(priority_order) { OrderAccomplishment(); }
            if (agv_part_poses.size() == 0) {
            RCLCPP_WARN(get_logger(), "No part poses recieved");
            }
        } else {
            RCLCPP_WARN(get_logger(), "Not a valid order ID");
        }
        for (int i = 0; i < int(order_p.size()); i++) {  
            if(priority_order) { OrderAccomplishment(); }  
            ariac_msgs::msg::PartPose part_to_pick;
            part_to_pick.part.type = order.AssemComb_type.parts[i].type;
            part_to_pick.part.color = order.AssemComb_type.parts[i].color;

            for (auto const &agv_part: agv_part_poses) {
                if(priority_order) { OrderAccomplishment(); }
                if (agv_part.part.type == part_to_pick.part.type && agv_part.part.color == part_to_pick.part.color) 
                    part_to_pick.pose = agv_part.pose;
                    
                }
                if(priority_order) { OrderAccomplishment(); }
                CeilingRobotPickAGVPart(part_to_pick);
                if(priority_order) { OrderAccomplishment(); }
                ariac_msgs::msg::AssemblyPart part_to_assemble;
                part_to_assemble.part.type = order.AssemComb_type.parts[i].type;
                part_to_assemble.part.color = order.AssemComb_type.parts[i].color;
                part_to_assemble.assembled_pose=order.AssemComb_type.parts[i].pose_stamp;
                part_to_assemble.install_direction=order.AssemComb_type.parts[i].install_direction;
                // part_to_assemble.pose = geometry_msgs::msg::Pose();
                if(priority_order) { OrderAccomplishment(); }
                CeilingRobotMoveToAssemblyStation(order.AssemComb_type.station);
                if(priority_order) { OrderAccomplishment(); }
                CeilingRobotAssemblePart(order.AssemComb_type.station, part_to_assemble);
                if(priority_order) { OrderAccomplishment(); }
                CeilingRobotMoveToAssemblyStation(order.AssemComb_type.station);
            }
        order_counter--;
        if(priority_index!=-1) {
            orders_list.erase(orders_list.begin() + priority_index);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Order List Size :"), std::to_string(orders_list.size()));
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Order List Submit Size :"), std::to_string(orders_list_submit.size()));
            priority_index =  -1;
        }
        else {
            orders_list.erase(orders_list.begin());
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Order List Size :"), std::to_string(orders_list.size()));
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Order List Submit Size :"), std::to_string(orders_list_submit.size()));
        }
    }
    else if(order.type == ariac_msgs::msg::Order::COMBINED) 
    {
        if(priority_order) { OrderAccomplishment(); }
        // FloorRobotSendHome();
        ariac_msgs::msg::CombinedTask task;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Assembly Order Station :"), std::to_string(order.AssemComb_type.station));

        uint8_t agv_number;
        if (order.AssemComb_type.station == 1)
            agv_number = 1;
        else if(order.AssemComb_type.station == 2)
            agv_number = 2;
        else if(order.AssemComb_type.station == 3)
            agv_number = 3;
        else if(order.AssemComb_type.station == 4)
            agv_number = 4;
        if(priority_order) { OrderAccomplishment(); }
        int id;
        if (kts2_trays_.size() != 0) {
            id = kts2_trays_[0].id;
        }  
        else if (kts1_trays_.size() != 0) {
            id = kts1_trays_[0].id;
        }else {
            RCLCPP_ERROR(get_logger(), "No trays available.");
        }
        if(priority_order) { OrderAccomplishment(); }
        FloorRobotPickandPlaceTray(id, agv_number);
        if(priority_order) { OrderAccomplishment(); }
        // tray_flag=true;
        // FloorRobot();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Parts   availability"), std::to_string(task.station));

        int quad_count = 1;
        if(priority_order) { OrderAccomplishment(); }   
        for (int i=0; i<order.AssemComb_type.parts.size();i++) {
            if(priority_order) { OrderAccomplishment(); }
            ariac_msgs::msg::Part agv_part;
            agv_part.type = order.AssemComb_type.parts[i].type;
            agv_part.color = order.AssemComb_type.parts[i].color;
            if(priority_order) { OrderAccomplishment(); }
            const_obj.quad_parts_map[quad_count].first =  agv_part.color ; 
            const_obj.quad_parts_map[quad_count].second = agv_part.type; 
            FloorRobotPickBinPartCombined(agv_part);
            FloorRobotPlacePartOnKitTray(agv_number, quad_count);
            if(priority_order) { OrderAccomplishment(); }
            quad_count++;
            
        } 
        ///////////////END of KITTING ///////////////////////////////////////
        
        if(priority_order) { OrderAccomplishment(); }
        for (const auto& pair : const_obj.quad_parts_map) 
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Quadrant Maps "), "quadrant - "+std::to_string(pair.first)+", type -  "+ Col_part_obj.COLOR[pair.second.first]+ " "+ Col_part_obj.PART[pair.second.second]);
        }
        if(priority_order) { OrderAccomplishment(); }
        // Check quality
        auto request1 = std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
        request1->order_id = order.id;
        if(priority_order) { OrderAccomplishment(); }
        auto result1 = quality_checker_->async_send_request(request1);
        result1.wait();
        RCLCPP_INFO_STREAM(get_logger(), "Result at 0 sec: " + std::to_string(result1.get()->all_passed));
        // Check quality 2
        rclcpp::sleep_for(std::chrono::seconds(3));
        auto request = std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
        request->order_id = order.id;
        if(priority_order) { OrderAccomplishment(); }
        auto result = quality_checker_->async_send_request(request);
        result.wait();
        RCLCPP_INFO_STREAM(get_logger(), "Result at 3 sec : " + std::to_string(result.get()->all_passed));
        if(priority_order) { OrderAccomplishment(); }
        if (!result.get()->all_passed) 
        {
            if(priority_order) { OrderAccomplishment(); }
            RCLCPP_ERROR(get_logger(), "Issue with shipment");
            RCLCPP_INFO_STREAM(get_logger(), "Faulty part ? : " + std::to_string(result.get()->quadrant2.faulty_part));
            
            if (result.get()->incorrect_tray) 
            {
                RCLCPP_ERROR(get_logger(), "Incorrect Tray");
                if(priority_order) { OrderAccomplishment(); }
            }

            // Quadarant 1 faulty parts
            if (result.get()->quadrant1.faulty_part) 
            {
                if(priority_order) { OrderAccomplishment(); }
                RCLCPP_ERROR(get_logger(), "Faulty part in Quadrant 1");
                FloorRobotDisposeParts(agv_number,1);
                FloorRobotPickBinPartReplaceMissing(const_obj.quad_parts_map[1].first,const_obj.quad_parts_map[1].second);
                FloorRobotPlacePartOnKitTrayMissing(agv_number, 1,const_obj.quad_parts_map[1].second);
                if(priority_order) { OrderAccomplishment(); }
            }
            if (result.get()->quadrant1.flipped_part) 
            {
                if(priority_order) { OrderAccomplishment(); }
                RCLCPP_ERROR(get_logger(), "Flipped part in Quadrant 1");
                FloorRobotPickFlippedPart(agv_number,1);
                CeilingRobotPlaceFlippedPartOnKitTray(agv_number,1);
                if(priority_order) { OrderAccomplishment(); }
            }
            if (result.get()->quadrant1.incorrect_part_color) 
            {
                if(priority_order) { OrderAccomplishment(); }
                RCLCPP_ERROR(get_logger(), "Incorrect part color in Quadrant 1");
                if(priority_order) { OrderAccomplishment(); }
            }
            if (result.get()->quadrant1.incorrect_part_type) 
            {
                if(priority_order) { OrderAccomplishment(); }
                RCLCPP_ERROR(get_logger(), "Incorrect part type in Quadrant 1");
                FloorRobotDisposeParts(agv_number,1);
                if(priority_order) { OrderAccomplishment(); }
            }
            if (result.get()->quadrant1.missing_part) 
            {
                if(priority_order) { OrderAccomplishment(); }
                RCLCPP_ERROR(get_logger(), "Missing part in Quadrant 1");
                FloorRobotPickBinPartReplaceMissing(const_obj.quad_parts_map[1].first,const_obj.quad_parts_map[1].second);
                FloorRobotPlacePartOnKitTrayMissing(agv_number, 1,const_obj.quad_parts_map[1].second);
                if(priority_order) { OrderAccomplishment(); }
            }


            // Quadarant 2 faulty parts
            if (result.get()->quadrant2.faulty_part) 
            {
                if(priority_order) { OrderAccomplishment(); }
                RCLCPP_ERROR(get_logger(), "Faulty part in Quadrant 2");
                FloorRobotDisposeParts(agv_number,2);
                FloorRobotPickBinPartReplaceMissing(const_obj.quad_parts_map[2].first,const_obj.quad_parts_map[2].second);
                FloorRobotPlacePartOnKitTrayMissing(agv_number, 2,const_obj.quad_parts_map[2].second);
                if(priority_order) { OrderAccomplishment(); }
            }
            if (result.get()->quadrant2.flipped_part) 
            {
                if(priority_order) { OrderAccomplishment(); }
                RCLCPP_ERROR(get_logger(), "Flipped part in Quadrant 2");
                FloorRobotPickFlippedPart(agv_number,1);
                CeilingRobotPlaceFlippedPartOnKitTray(agv_number,1);
                if(priority_order) { OrderAccomplishment(); }
            }
            if (result.get()->quadrant2.incorrect_part_color) 
            {
                if(priority_order) { OrderAccomplishment(); }
                RCLCPP_ERROR(get_logger(), "Incorrect part color in Quadrant 2");
                if(priority_order) { OrderAccomplishment(); }
            }
            if (result.get()->quadrant2.incorrect_part_type) 
            {
                if(priority_order) { OrderAccomplishment(); }
                RCLCPP_ERROR(get_logger(), "Incorrect part type in Quadrant 2");
                FloorRobotDisposeParts(agv_number,2);
                if(priority_order) { OrderAccomplishment(); }
            }
            if (result.get()->quadrant2.missing_part) 
            {
                if(priority_order) { OrderAccomplishment(); }
                RCLCPP_ERROR(get_logger(), "Missing part in Quadrant 2");
                FloorRobotPickBinPartReplaceMissing(const_obj.quad_parts_map[2].first,const_obj.quad_parts_map[2].second);
                FloorRobotPlacePartOnKitTrayMissing(agv_number, 2,const_obj.quad_parts_map[2].second);
                if(priority_order) { OrderAccomplishment(); }
            }


            // Quadarant 3 faulty parts
            if (result.get()->quadrant3.faulty_part) 
            {
                if(priority_order) { OrderAccomplishment(); }
                RCLCPP_ERROR(get_logger(), "Faulty part in Quadrant 3");
                FloorRobotDisposeParts(agv_number,3);
                FloorRobotPickBinPartReplaceMissing(const_obj.quad_parts_map[3].first,const_obj.quad_parts_map[3].second);
                FloorRobotPlacePartOnKitTrayMissing(agv_number, 3,const_obj.quad_parts_map[3].second);
                if(priority_order) { OrderAccomplishment(); }
            }
            if (result.get()->quadrant3.flipped_part) 
            {
                if(priority_order) { OrderAccomplishment(); }
                RCLCPP_ERROR(get_logger(), "Flipped part in Quadrant 3");
                FloorRobotPickFlippedPart(agv_number,1);
                CeilingRobotPlaceFlippedPartOnKitTray(agv_number,1);
                if(priority_order) { OrderAccomplishment(); }
            }
            if (result.get()->quadrant3.incorrect_part_color) 
            {
                RCLCPP_ERROR(get_logger(), "Incorrect part color in Quadrant 3");
                if(priority_order) { OrderAccomplishment(); }
            }
            if (result.get()->quadrant3.incorrect_part_type) 
            {
                if(priority_order) { OrderAccomplishment(); }
                RCLCPP_ERROR(get_logger(), "Incorrect part type in Quadrant 3");
                FloorRobotDisposeParts(agv_number,3);
                if(priority_order) { OrderAccomplishment(); }
            }
            if (result.get()->quadrant3.missing_part) 
            {
                if(priority_order) { OrderAccomplishment(); }
                RCLCPP_ERROR(get_logger(), "Missing part in Quadrant 3");
                FloorRobotPickBinPartReplaceMissing(const_obj.quad_parts_map[3].first,const_obj.quad_parts_map[3].second);
                FloorRobotPlacePartOnKitTrayMissing(agv_number, 3,const_obj.quad_parts_map[3].second);
                if(priority_order) { OrderAccomplishment(); }
            }


            // Quadarant 4 faulty parts
            if (result.get()->quadrant4.faulty_part) 
            {
                if(priority_order) { OrderAccomplishment(); }
                RCLCPP_ERROR(get_logger(), "Faulty part in Quadrant 4");
                FloorRobotDisposeParts(agv_number,4);
                FloorRobotPickBinPartReplaceMissing(const_obj.quad_parts_map[4].first,const_obj.quad_parts_map[4].second);
                FloorRobotPlacePartOnKitTrayMissing(agv_number, 4,const_obj.quad_parts_map[4].second);
                if(priority_order) { OrderAccomplishment(); }
            }
            if (result.get()->quadrant4.flipped_part) 
            {
                if(priority_order) { OrderAccomplishment(); }
                RCLCPP_ERROR(get_logger(), "Flipped part in Quadrant 4");
                FloorRobotPickFlippedPart(agv_number,1);
                CeilingRobotPlaceFlippedPartOnKitTray(agv_number,1);
                if(priority_order) { OrderAccomplishment(); }
            }
            if (result.get()->quadrant4.incorrect_part_color) 
            {
                RCLCPP_ERROR(get_logger(), "Incorrect part color in Quadrant 4");
                if(priority_order) { OrderAccomplishment(); }
            }
            if (result.get()->quadrant4.incorrect_part_type) 
            {
                if(priority_order) { OrderAccomplishment(); }
                RCLCPP_ERROR(get_logger(), "Incorrect part type in Quadrant 4");
                FloorRobotDisposeParts(agv_number,4);
                if(priority_order) { OrderAccomplishment(); }
            }
            if (result.get()->quadrant4.missing_part) 
            {
                if(priority_order) { OrderAccomplishment(); }
                RCLCPP_ERROR(get_logger(), "Missing part in Quadrant 4");
                FloorRobotPickBinPartReplaceMissing(const_obj.quad_parts_map[4].first,const_obj.quad_parts_map[4].second);
                FloorRobotPlacePartOnKitTrayMissing(agv_number, 4,const_obj.quad_parts_map[4].second); 
                if(priority_order) { OrderAccomplishment(); }
            }

            quality_check_flag = true;

        }
        if (result.get()->all_passed || quality_check_flag) 
        {
            int destination;
            if (order.AssemComb_type.station == 1 or order.AssemComb_type.station == 3) {
                destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_FRONT;
            } else {
                destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_BACK;
            }
            if(priority_order) { OrderAccomplishment(); }
            RCLCPP_INFO_STREAM(rclcpp::get_logger("AGV Destination task"),"\t Lock AGV " + std::to_string(agv_number));
            RCLCPP_INFO_STREAM(rclcpp::get_logger("AGV Destination task"),"\t Move AGV to " + Col_part_obj.DEST[destination]+"\n");
            if(priority_order) { OrderAccomplishment(); }
            MoveAGV(agv_number, destination);
        }

        //////////////////////////////////////////////////END of Quality Check//////////////////////////////////////


        
        if(priority_order) { OrderAccomplishment(); }
        CeilingRobotMoveToAssemblyStation(order.AssemComb_type.station);
        if(priority_order) { OrderAccomplishment(); }
        auto client = create_client<ariac_msgs::srv::GetPreAssemblyPoses>("/ariac/get_pre_assembly_poses");
        if(priority_order) { OrderAccomplishment(); }
        auto request_ = std::make_shared<ariac_msgs::srv::GetPreAssemblyPoses::Request>();
        request_->order_id = order.id;
        if(priority_order) { OrderAccomplishment(); }
        auto result_ = client->async_send_request(request_);
        std::vector<ariac_msgs::msg::PartPose> agv_part_poses; 
        if (result_.get()->valid_id) {
            agv_part_poses = result_.get()->parts;
            if(priority_order) { OrderAccomplishment(); }
            if (agv_part_poses.size() == 0) {
            RCLCPP_WARN(get_logger(), "No part poses recieved");
            }
        } else {
            RCLCPP_WARN(get_logger(), "Not a valid order ID");
        }
        for (int i = 0; i < int(order_p.size()); i++) 
        {    
            if(priority_order) { OrderAccomplishment(); }
            ariac_msgs::msg::PartPose part_to_pick;
            part_to_pick.part.type = order.AssemComb_type.parts[i].type;
            part_to_pick.part.color = order.AssemComb_type.parts[i].color;

            for (auto const &agv_part: agv_part_poses) {
                if(priority_order) { OrderAccomplishment(); }
                if (agv_part.part.type == part_to_pick.part.type && agv_part.part.color == part_to_pick.part.color) 
                    part_to_pick.pose = agv_part.pose;    
                }
                if(priority_order) { OrderAccomplishment(); }
                CeilingRobotPickAGVPart(part_to_pick);
                if(priority_order) { OrderAccomplishment(); }
                ariac_msgs::msg::AssemblyPart part_to_assemble;
                part_to_assemble.part.type = order.AssemComb_type.parts[i].type;
                part_to_assemble.part.color = order.AssemComb_type.parts[i].color;
                part_to_assemble.assembled_pose=order.AssemComb_type.parts[i].pose_stamp;
                part_to_assemble.install_direction=order.AssemComb_type.parts[i].install_direction;
                // part_to_assemble.pose = geometry_msgs::msg::Pose();
                if(priority_order) { OrderAccomplishment(); }
                CeilingRobotMoveToAssemblyStation(order.AssemComb_type.station);
                if(priority_order) { OrderAccomplishment(); }
                CeilingRobotAssemblePart(order.AssemComb_type.station, part_to_assemble);
                if(priority_order) { OrderAccomplishment(); }
                CeilingRobotMoveToAssemblyStation(order.AssemComb_type.station);
                if(priority_order) { OrderAccomplishment(); }
        }
        order_counter--;
        if(priority_index!=-1) {
            orders_list.erase(orders_list.begin() + priority_index);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Order List Size :"), std::to_string(orders_list.size()));
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Order List Submit Size :"), std::to_string(orders_list_submit.size()));
            priority_index =  -1;
        }
        else {
            orders_list.erase(orders_list.begin());
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Order List Size :"), std::to_string(orders_list.size()));
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Order List Submit Size :"), std::to_string(orders_list_submit.size()));
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
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Bin Parts subscriber"), " ");
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
                conv_part_quantity+=partp.parts[j].quantity;
            }
        }
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Conveyor Parts subs "), " ");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Conveyor Parts subs "), "Total number of parts on conveyor : " + std::to_string(conv_part_quantity));
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

        for(int i:p_bins)
        {
            for(int j:empty_bins)
            {
                if(i==j)
                {
                    empty_bin = i;
                    break;
                }         
            }
            if(empty_bin) break;               
        }

        RCLCPP_INFO_STREAM(rclcpp::get_logger("Conveyor Parts subs "), " ");
        conv_flag = true;
        conv_part_sub.reset();      
    }
}


void CompetitionARIAC::BreakBeamCallback(const ariac_msgs::msg::BreakBeamStatus::SharedPtr break_beam_msg)
{
    if(conv_flag)
    {
        if(break_beam_msg->object_detected)
        {                                                                
            conv_part_detected++;
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Break Beam subscriber"), "Part Detected, count = " + std::to_string(conv_part_detected));

            if(slot<=9)
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor Robot conv task"), "\t Pick "+conv_part_idetified_color +" "+conv_part_idetified_type+" from conveyor lane "+std::to_string(conv_lane_offset)+" and place in Bin " + std::to_string(empty_bin) +" - slot " + std::to_string(slot));
            else
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor Robot conv task"), "empty slots exhausted");

            FloorRobotPickConveyorPart(conv_part_idetified_color, conv_part_idetified_type,conv_lane_offset);   
            FloorRobotPlacePartEmptyBin(conv_part_idetified_type, empty_bin, slot);
            slot+=2;

        }
        if(conv_part_detected >= conv_part_quantity)
        {
            slot = 2;
            beam_flag = true;
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Break Beam Callback "), "flags set true");
            break_beam_sub.reset();
            break_beam_sub2.reset();
        }
    }
          
}

void CompetitionARIAC::BreakBeamCallback2(const ariac_msgs::msg::BreakBeamStatus::SharedPtr break_beam_msg)
{
    
    if(break_beam_msg->object_detected)
    {                                                                
        conv_part_detected++; 
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Break Beam Subscriber2"), "Part Detected, count= "+std::to_string(conv_part_detected));
        rclcpp::sleep_for(std::chrono::seconds(1));                    
    }
    
}


void CompetitionARIAC::KitTrayTable1Callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    
    kts1_trays_ =  msg->tray_poses;

    for(auto trays:kts1_trays_)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub kts1 "), "on kts1 tray id :" + std::to_string(trays.id));
        kit1_pose = trays.pose;
        
        RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub kts1 "), "pose :" );
        RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub kts1 "), "    position x :" + std::to_string(kit1_pose.position.x));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub kts1 "), "    position y :" + std::to_string(kit1_pose.position.y));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub kts1 "), "    position z :" + std::to_string(kit1_pose.position.z));

    }
    
    kts1_camera_pose_ = msg->sensor_pose;
    kit_tray_table1_camera_sub_.reset();

}

void CompetitionARIAC::KitTrayTable2Callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    
    kts2_trays_ =  msg->tray_poses;

    for(auto trays:kts2_trays_)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub kts2 "), "on kts2 tray id :" + std::to_string(trays.id));

        kit2_pose = trays.pose;
        
        RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub kts2 "), "pose :" );
        RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub kts2 "), "    position x :" + std::to_string(kit2_pose.position.x));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub kts2 "), "    position y :" + std::to_string(kit2_pose.position.y));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub kts2 "), "    position z :" + std::to_string(kit2_pose.position.z));

    }
    
    kts2_camera_pose_ = msg->sensor_pose;
    kit_tray_table2_camera_sub_.reset();

}

void CompetitionARIAC::LeftBinsCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    
    if((beam_flag || conv_part_quantity == 0) && conv_flag )
    {
        color_::ColorParts Col_part_obj;
        left_bins_parts_ =  msg->part_poses;
        left_bins_camera_pose_ = msg->sensor_pose;
        geometry_msgs::msg::Pose part_pose;
        for(auto bin_part:left_bins_parts_)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub LBin "), "Parts on left bins :" + Col_part_obj.COLOR[bin_part.part.color]+" "+Col_part_obj.PART[bin_part.part.type]);

            lbin_pose = bin_part.pose;
            
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub LBin "), "pose :" );
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub LBin "), "    position    x :" + std::to_string(lbin_pose.position.x));
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub LBin "), "    position    y :" + std::to_string(lbin_pose.position.y));
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub LBin "), "    position    z :" + std::to_string(lbin_pose.position.z));
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub LBin "), "    orientation w :" + std::to_string(lbin_pose.orientation.w));
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub LBin "), "    orientation x :" + std::to_string(lbin_pose.orientation.x));
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub LBin "), "    orientation y :" + std::to_string(lbin_pose.orientation.y));
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub LBin "), "    orientation z :" + std::to_string(lbin_pose.orientation.z));
            part_pose = MultiplyPose(left_bins_camera_pose_,lbin_pose);

            tf2::Quaternion q(
            part_pose.orientation.x,
            part_pose.orientation.y,
            part_pose.orientation.z,
            part_pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub LBin "), "    position    r :" + std::to_string(roll));
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub LBin "), "    position    p :" + std::to_string(pitch));
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub LBin "), "    position    y :" + std::to_string(yaw));

        }
        
       
        left_bins_camera_sub_.reset();
    }

}

void CompetitionARIAC::RightBinsCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    
    if((beam_flag || conv_part_quantity == 0) && conv_flag )
    {
        color_::ColorParts Col_part_obj;
        right_bins_parts_ =  msg->part_poses;
        right_bins_camera_pose_ = msg->sensor_pose;
        geometry_msgs::msg::Pose part_pose;
        for(auto bin_part:right_bins_parts_)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub rBin "), "Parts on right bins :" + Col_part_obj.COLOR[bin_part.part.color]+" "+Col_part_obj.PART[bin_part.part.type]);

            rbin_pose = bin_part.pose;
            
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub RBin "), "pose :" );
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub RBin "), "    position    x :" + std::to_string(rbin_pose.position.x));
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub RBin "), "    position    y :" + std::to_string(rbin_pose.position.y));
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub RBin "), "    position    z :" + std::to_string(rbin_pose.position.z));
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub RBin "), "    orientation w :" + std::to_string(rbin_pose.orientation.w));
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub RBin "), "    orientation x :" + std::to_string(rbin_pose.orientation.x));
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub RBin "), "    orientation y :" + std::to_string(rbin_pose.orientation.y));
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub RBin "), "    orientation z :" + std::to_string(rbin_pose.orientation.z));
            part_pose = MultiplyPose(right_bins_camera_pose_,rbin_pose);

            tf2::Quaternion q(
            part_pose.orientation.x,
            part_pose.orientation.y,
            part_pose.orientation.z,
            part_pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub RBin "), "    position    r :" + std::to_string(roll));
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub RBin "), "    position    p :" + std::to_string(pitch));
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub RBin "), "    position    y :" + std::to_string(yaw));

        }
                                               
        right_bins_camera_sub_.reset();
    }

}

void CompetitionARIAC::ConveyorCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    color_::ColorParts Col_part_obj;
    
    conveyor_parts_ =  msg->part_poses;
    conveyor_camera_pose_ = msg->sensor_pose;

    for(auto conv_part:conveyor_parts_)
    {
        
        if(Col_part_obj.PART[conv_part.part.type].length()!=0)
        {
            conv_part_idetified_type = Col_part_obj.PART[conv_part.part.type];
            conv_part_idetified_color = Col_part_obj.COLOR[conv_part.part.color];

            conv_part_pose = conv_part.pose;
            
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub conveyor "), "pose : " + conv_part_idetified_color+" "+conv_part_idetified_type);
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub conveyor "), "    position z :" + std::to_string(conv_part_pose.position.z));

            conv_lane_offset = conv_part_pose.position.z;
        }         
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



geometry_msgs::msg::Pose CompetitionARIAC::MultiplyPose(
    geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2)
{
    KDL::Frame f1;
    KDL::Frame f2;

    tf2::fromMsg(p1, f1);
    tf2::fromMsg(p2, f2);

    KDL::Frame f3 = f1 * f2;

    return tf2::toMsg(f3);
}

void CompetitionARIAC::LogPose(geometry_msgs::msg::Pose p)
{
    tf2::Quaternion q(
        p.orientation.x,
        p.orientation.y,
        p.orientation.z,
        p.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    roll *= 180 / M_PI;
    pitch *= 180 / M_PI;
    yaw *= 180 / M_PI;

    RCLCPP_INFO(get_logger(), "(X: %.2f, Y: %.2f, Z: %.2f, R: %.2f, P: %.2f, Y: %.2f)",
                p.position.x, p.position.y, p.position.z,
                roll, pitch, yaw);
}

geometry_msgs::msg::Pose CompetitionARIAC::BuildPose(
    double x, double y, double z, geometry_msgs::msg::Quaternion orientation)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = orientation;

    return pose;
}

geometry_msgs::msg::Pose CompetitionARIAC::FrameWorldPose(std::string frame_id)
{
    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::Pose pose;

    try
    {
        t = tf_buffer->lookupTransform("world", frame_id, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(get_logger(), "Could not get transform");
    }

    pose.position.x = t.transform.translation.x;
    pose.position.y = t.transform.translation.y;
    pose.position.z = t.transform.translation.z;
    pose.orientation = t.transform.rotation;

    return pose;
}

double CompetitionARIAC::GetYaw(geometry_msgs::msg::Pose pose)
{
    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

double CompetitionARIAC::GetPitch(geometry_msgs::msg::Pose pose)
{
    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return pitch;
}

geometry_msgs::msg::Quaternion CompetitionARIAC::QuaternionFromRPY(double r, double p, double y)
{
    tf2::Quaternion q;
    geometry_msgs::msg::Quaternion q_msg;

    q.setRPY(r, p, y);

    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();

    return q_msg;
}

void CompetitionARIAC::AddModelToPlanningScene(
    std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose)
{
    moveit_msgs::msg::CollisionObject collision;

    collision.id = name;
    collision.header.frame_id = "world";

    shape_msgs::msg::Mesh mesh;
    shapes::ShapeMsg mesh_msg;

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("test_competitor");
    std::stringstream path;
    path << "file://" << package_share_directory << "/meshes/" << mesh_file;
    std::string model_path = path.str();

    shapes::Mesh *m = shapes::createMeshFromResource(model_path);
    shapes::constructMsgFromShape(m, mesh_msg);

    mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    collision.meshes.push_back(mesh);
    collision.mesh_poses.push_back(model_pose);

    collision.operation = collision.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision);

    planning_scene_.addCollisionObjects(collision_objects);
}

void CompetitionARIAC::AddModelsToPlanningScene()
{
    // Add bins
    std::map<std::string, std::pair<double, double>> bin_positions = {
        {"bin1", std::pair<double, double>(-1.9, 3.375)},
        {"bin2", std::pair<double, double>(-1.9, 2.625)},
        {"bin3", std::pair<double, double>(-2.65, 2.625)},
        {"bin4", std::pair<double, double>(-2.65, 3.375)},
        {"bin5", std::pair<double, double>(-1.9, -3.375)},
        {"bin6", std::pair<double, double>(-1.9, -2.625)},
        {"bin7", std::pair<double, double>(-2.65, -2.625)},
        {"bin8", std::pair<double, double>(-2.65, -3.375)}};

    geometry_msgs::msg::Pose bin_pose;
    for (auto const &bin : bin_positions)
    {
        bin_pose.position.x = bin.second.first;
        bin_pose.position.y = bin.second.second;
        bin_pose.position.z = 0;
        bin_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

        AddModelToPlanningScene(bin.first, "bin.stl", bin_pose);
    }

    // Add assembly stations
    std::map<std::string, std::pair<double, double>> assembly_station_positions = {
        {"as1", std::pair<double, double>(-7.3, 3)},
        {"as2", std::pair<double, double>(-12.3, 3)},
        {"as3", std::pair<double, double>(-7.3, -3)},
        {"as4", std::pair<double, double>(-12.3, -3)},
    };

    geometry_msgs::msg::Pose assembly_station_pose;
    for (auto const &station : assembly_station_positions)
    {
        assembly_station_pose.position.x = station.second.first;
        assembly_station_pose.position.y = station.second.second;
        assembly_station_pose.position.z = 0;
        assembly_station_pose.orientation = QuaternionFromRPY(0, 0, 0);

        AddModelToPlanningScene(station.first, "assembly_station.stl", assembly_station_pose);
    }

    // Add assembly briefcases
    std::map<std::string, std::pair<double, double>> assembly_insert_positions = {
        {"as1_insert", std::pair<double, double>(-7.7, 3)},
        {"as2_insert", std::pair<double, double>(-12.7, 3)},
        {"as3_insert", std::pair<double, double>(-7.7, -3)},
        {"as4_insert", std::pair<double, double>(-12.7, -3)},
    };

    geometry_msgs::msg::Pose assembly_insert_pose;
    for (auto const &insert : assembly_insert_positions)
    {
        assembly_insert_pose.position.x = insert.second.first;
        assembly_insert_pose.position.y = insert.second.second;
        assembly_insert_pose.position.z = 1.011;
        assembly_insert_pose.orientation = QuaternionFromRPY(0, 0, 0);

        AddModelToPlanningScene(insert.first, "assembly_insert.stl", assembly_insert_pose);
    }

    geometry_msgs::msg::Pose conveyor_pose;
    conveyor_pose.position.x = -0.6;
    conveyor_pose.position.y = 0;
    conveyor_pose.position.z = 0;
    conveyor_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene("conveyor", "conveyor.stl", conveyor_pose);

    geometry_msgs::msg::Pose kts1_table_pose;
    kts1_table_pose.position.x = -1.3;
    kts1_table_pose.position.y = -5.84;
    kts1_table_pose.position.z = 0;
    kts1_table_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

    AddModelToPlanningScene("kts1_table", "kit_tray_table.stl", kts1_table_pose);

    geometry_msgs::msg::Pose kts2_table_pose;
    kts2_table_pose.position.x = -1.3;
    kts2_table_pose.position.y = 5.84;
    kts2_table_pose.position.z = 0;
    kts2_table_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene("kts2_table", "kit_tray_table.stl", kts2_table_pose);
}

geometry_msgs::msg::Quaternion CompetitionARIAC::SetRobotOrientation(double rotation)
{
    tf2::Quaternion tf_q;
    tf_q.setRPY(0, 3.14159, rotation);

    geometry_msgs::msg::Quaternion q;

    q.x = tf_q.x();
    q.y = tf_q.y();
    q.z = tf_q.z();
    q.w = tf_q.w();

    return q;
}

bool CompetitionARIAC::FloorRobotMovetoTarget()
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(floor_robot_.plan(plan));

    if (success)
    {
        return static_cast<bool>(floor_robot_.execute(plan));
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate plan");
        return false;
    }
}

bool CompetitionARIAC::FloorRobotMoveCartesian(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf)
{
    moveit_msgs::msg::RobotTrajectory trajectory;

    double path_fraction = floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (path_fraction < 0.9)
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
        return false;
    }

    // Retime trajectory
    robot_trajectory::RobotTrajectory rt(floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
    rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
    totg_.computeTimeStamps(rt, vsf, asf);
    rt.getRobotTrajectoryMsg(trajectory);

    return static_cast<bool>(floor_robot_.execute(trajectory));
}

void CompetitionARIAC::FloorRobotMoveUp()
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;
    waypoints.clear();
    starting_pose.position.z += 0.003;
    waypoints.push_back(starting_pose);
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
}

void CompetitionARIAC::FloorRobotWaitForAttach(double timeout)
{
    // Wait for part to be attached
    rclcpp::Time start = now();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;

    while (!floor_gripper_state_.attached)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
        
        waypoints.clear();
        starting_pose.position.z -= 0.001;
        waypoints.push_back(starting_pose);

        FloorRobotMoveCartesian(waypoints, 0.1, 0.1);

        usleep(200);

        if (now() - start > rclcpp::Duration::from_seconds(timeout))
        {
            RCLCPP_ERROR(get_logger(), "Unable to pick up object");
            return;
        }
    }
}

void CompetitionARIAC::FloorRobotWaitForAttachPump(double timeout)
{
    // Wait for part to be attached
    rclcpp::Time start = now();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;

    // works for slot 1,2,3,4
    while (!floor_gripper_state_.attached)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach to PUMP");
        
        waypoints.clear();
        starting_pose.position.z -= 0.0005;
        waypoints.push_back(starting_pose);

        FloorRobotMoveCartesian(waypoints, 0.1, 0.1);

        usleep(600);

        if (now() - start > rclcpp::Duration::from_seconds(timeout))
        {
            RCLCPP_ERROR(get_logger(), "Unable to pick up object");
            return;
        }
    }

}

void CompetitionARIAC::FloorRobotWaitForAttachKitTrayPart(double timeout)
{
    // Wait for part to be attached
    rclcpp::Time start = now();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Wait timer Subscriber"), "gripper state :"+std::to_string(floor_gripper_state_.attached));
    while (!floor_gripper_state_.attached)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
        waypoints.clear();
        starting_pose.position.z -= 0.001;
        waypoints.push_back(starting_pose);

        FloorRobotMoveCartesian(waypoints, 0.1, 0.1);

        usleep(200);

        if (now() - start > rclcpp::Duration::from_seconds(timeout))
        {
            RCLCPP_ERROR(get_logger(), "Unable to pick up object");
            return;
        }
    }
}


void CompetitionARIAC::FloorRobotWaitForDrop(double timeout)
{
    // Wait for part to be attached
    rclcpp::Time start = now();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;

    while (floor_gripper_state_.attached)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for drop");

        waypoints.clear();
        starting_pose.position.z -= 0.01;
        waypoints.push_back(starting_pose);

        FloorRobotMoveCartesian(waypoints, 0.1, 0.1);

        // usleep(200);

        if (now() - start > rclcpp::Duration::from_seconds(timeout))
        {
            FloorRobotSetGripperState(false);
            RCLCPP_ERROR(get_logger(), "Unable to drop");
            return;
        }
    }
}



void CompetitionARIAC::floor_gripper_state_cb(
    const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg)
{
    floor_gripper_state_ = *msg;
}



bool CompetitionARIAC::FloorRobotSetGripperState(bool enable)
{
    
    
    floor_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/floor_robot_enable_gripper");

    RCLCPP_INFO_STREAM(rclcpp::get_logger("SET GRIPPER FUNCTION"), " ");
    if (floor_gripper_state_.enabled == enable)
    {
        if (floor_gripper_state_.enabled)
            RCLCPP_INFO(get_logger(), "Already enabled");
        else
            RCLCPP_INFO(get_logger(), "Already disabled");

        return false;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("SET GRIPPER FUNCTION"), " ");
    // Call enable service
    auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
    request->enable = enable;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("SET GRIPPER disable"), " ");
    auto result = floor_robot_gripper_enable_->async_send_request(request);
    result.wait();

    if (!result.get()->success)
    {
        RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
        return false;
    }

    return true;
}

void CompetitionARIAC::FloorRobotSendHome()
{
    // Move floor robot to home joint state
    floor_robot_.setNamedTarget("home");
    FloorRobotMovetoTarget();

}


bool CompetitionARIAC::FloorRobotPickConveyorPart(std::string part_color, std::string part_type,double lane)
{
    j_s_maps::FloorRobotJS js_obj;

    RCLCPP_INFO_STREAM(rclcpp::get_logger("CONVEYOR PART PICK"), " ");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("CONVEYOR PART PICK"), "Pick " + part_color + " " + part_type + " from lane - " + std::to_string(lane));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("CONVEYOR PART PICK"), " ");

    rclcpp::Time start = now();
    double timeout{10.0};

    if(lane >= 0.0725)
    {
        if(part_type=="pump")
        {   
            
            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_neg1_high_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);
            
            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }
            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }

            //////////////////////////////////////vertical lift/////////////////////////////////////////
            // floor_robot_.setJointValueTarget(js_obj.conv_pick_neg1_high_flat_pick);
            // FloorRobotMovetoTarget();
            // FloorRobotSetGripperState(true);
            
            // while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            // {
            //     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            // }
            // if (floor_gripper_state_.attached)
            // {
            //     floor_robot_.setJointValueTarget(js_obj.conv_pick_neg1_high_flat_pick_up);
            //     FloorRobotMovetoTarget();
            //     floor_robot_.setJointValueTarget(js_obj.conv_pick_neg1_high_flat_pick_up2);
            //     FloorRobotMovetoTarget();
            // }


            
        }
        else if (part_type=="regulator" || part_type=="sensor")
        {
            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_neg1_mid_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);

            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }

            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
            
        }
        else if (part_type=="battery")
        {

            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_neg1_low_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);

            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }

            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
        }
    }
    else if(lane<0.0725 && lane>= 0.0375)
    {
        if(part_type=="pump")
        {   
            
            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_neg75_high_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);
            
            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }
            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
            
        }
        else if (part_type=="regulator" || part_type=="sensor")
        {
            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_neg75_mid_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);

            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }

            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
            
        }
        else if (part_type=="battery")
        {

            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_neg75_low_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);

            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }

            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
        }
    }
    else if(lane<0.0375 && lane>= 0.0025)
    {
        if(part_type=="pump")
        {   
            
            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
 
            floor_robot_.setJointValueTarget(js_obj.conv_pick_neg50_high_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);
            
            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }
            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
            
        }
        else if (part_type=="regulator" || part_type=="sensor")
        {
            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_neg50_mid_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);

            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }

            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
            
        }
        else if (part_type=="battery")
        {

            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_neg50_low_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);

            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }

            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
        }
    }
    else if(lane<0.0025 && lane>= -0.0325)
    {
        if(part_type=="pump")
        {   
            
            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);

            floor_robot_.setJointValueTarget(js_obj.conv_pick_neg25_high_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);
            
            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }
            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
            
        }
        else if (part_type=="regulator" || part_type=="sensor")
        {
            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_neg25_mid_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);

            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }

            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
            
        }
        else if (part_type=="battery")
        {

            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_neg25_low_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);

            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }

            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
        }
    }
    else if(lane<-0.0325 && lane>= -0.0675)
    {
        if(part_type=="pump")
        {   
            
            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_0_high_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);
            
            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }
            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
            
        }
        else if (part_type=="regulator" || part_type=="sensor")
        {
            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_0_mid_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);

            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }

            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
            
        }
        else if (part_type=="battery")
        {

            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_0_low_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);

            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }

            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
        }
    }
    else if(lane<-0.0675 && lane>= -0.1025)
    {
        if(part_type=="pump")
        {   
            
            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_pos25_high_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);
            
            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }
            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
            
        }
        else if (part_type=="regulator" || part_type=="sensor")
        {
            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_pos25_mid_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);

            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }

            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
            
        }
        else if (part_type=="battery")
        {

            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_pos25_low_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);

            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }

            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
        }
    }
    else if(lane<-0.1025 && lane>= -0.1375)
    {
        if(part_type=="pump")
        {   
            
            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_pos50_high_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);
            
            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }
            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
            
        }
        else if (part_type=="regulator" || part_type=="sensor")
        {
            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_pos50_mid_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);

            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }

            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
            
        }
        else if (part_type=="battery")
        {

            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_pos50_low_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);

            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }

            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
        }
    }
    else if(lane<-0.1375 && lane>= -0.1725)
    {
        if(part_type=="pump")
        {   
            
            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_pos75_high_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);
            
            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }
            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
            
        }
        else if (part_type=="regulator" || part_type=="sensor")
        {
            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_pos75_mid_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);

            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }

            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
            
        }
        else if (part_type=="battery")
        {

            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_pos75_low_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);

            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }

            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
        }
    }
    else if(lane < -0.1725)
    {
        if(part_type=="pump")
        {   
            
            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_pos1_high_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);
            
            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }
            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
            
        }
        else if (part_type=="regulator" || part_type=="sensor")
        {
            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_pos1_mid_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);

            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }

            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
            
        }
        else if (part_type=="battery")
        {

            RCLCPP_INFO(get_logger(), "Detected pump part with z  value: %lf", lane);
            floor_robot_.setJointValueTarget(js_obj.conv_pick_pos1_low_js);
            FloorRobotMovetoTarget();
            FloorRobotSetGripperState(true);

            while (!floor_gripper_state_.attached && !(now() - start > rclcpp::Duration::from_seconds(timeout)))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
            }

            if (floor_gripper_state_.attached)
            {
                floor_robot_.setJointValueTarget(js_obj.conv_pick_right_up_js);
                FloorRobotMovetoTarget();
            }
        }
    }
    else
    {
        RCLCPP_INFO(get_logger(), "No parts detected");
    }
    // conv_pick_flag = true;
    return true;
}

bool CompetitionARIAC::FloorRobotPlacePartEmptyBin(std::string part_type, int bin_num, int slot)
{
    j_s_maps::FloorRobotJS js_obj;
    
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("PLACE PART EMPTY BIN"), " ");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("PLACE PART EMPTY BIN"), "Bin number : "+std::to_string(bin_num));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("PLACE PART EMPTY BIN"), "    slot : "+std::to_string(slot));
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("PLACE PART EMPTY BIN"), " ");

    switch(bin_num)
    {
        case 1: floor_robot_.setJointValueTarget(js_obj.bin_1_up);
                FloorRobotMovetoTarget();
                // rclcpp::sleep_for(std::chrono::milliseconds(200));
                switch(slot)
                {
                    case 1: floor_robot_.setJointValueTarget(js_obj.bin_1_slot_1);
                        break;
                    case 2: floor_robot_.setJointValueTarget(js_obj.bin_1_slot_2);
                        break;
                    case 3: floor_robot_.setJointValueTarget(js_obj.bin_1_slot_3);
                        break;
                    case 4: floor_robot_.setJointValueTarget(js_obj.bin_1_slot_4);
                        break;
                    case 5: floor_robot_.setJointValueTarget(js_obj.bin_1_slot_5);
                        break;
                    case 6: floor_robot_.setJointValueTarget(js_obj.bin_1_slot_6);
                        break;
                    case 7: floor_robot_.setJointValueTarget(js_obj.bin_1_slot_7);
                        break;
                    case 8: floor_robot_.setJointValueTarget(js_obj.bin_1_slot_8);
                        break;
                    case 9: floor_robot_.setJointValueTarget(js_obj.bin_1_slot_9);
                        break;
                }
                FloorRobotMovetoTarget();
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                FloorRobotSetGripperState(false);
                floor_robot_.setJointValueTarget(js_obj.bin_1_up);
                FloorRobotMovetoTarget();
                break;
        case 2: floor_robot_.setJointValueTarget(js_obj.bin_2_up);
                FloorRobotMovetoTarget();
                // rclcpp::sleep_for(std::chrono::milliseconds(200));
                switch(slot)
                {
                    case 1: floor_robot_.setJointValueTarget(js_obj.bin_2_slot_1);
                        break;
                    case 2: floor_robot_.setJointValueTarget(js_obj.bin_2_slot_2);
                        break;
                    case 3: floor_robot_.setJointValueTarget(js_obj.bin_2_slot_3);
                        break;
                    case 4: floor_robot_.setJointValueTarget(js_obj.bin_2_slot_4);
                        break;
                    case 5: floor_robot_.setJointValueTarget(js_obj.bin_2_slot_5);
                        break;
                    case 6: floor_robot_.setJointValueTarget(js_obj.bin_2_slot_6);
                        break;
                    case 7: floor_robot_.setJointValueTarget(js_obj.bin_2_slot_7);
                        break;
                    case 8: floor_robot_.setJointValueTarget(js_obj.bin_2_slot_8);
                        break;
                    case 9: floor_robot_.setJointValueTarget(js_obj.bin_2_slot_9);
                        break;
                }
                FloorRobotMovetoTarget();
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                FloorRobotSetGripperState(false);
                floor_robot_.setJointValueTarget(js_obj.bin_2_up);
                FloorRobotMovetoTarget();
                break;
        case 3: 
                switch(slot)
                {
                    case 1: floor_robot_.setJointValueTarget(js_obj.bin_3_slot_7);
                        break;
                    case 2: floor_robot_.setJointValueTarget(js_obj.bin_3_slot_8);
                        break;
                    case 3: floor_robot_.setJointValueTarget(js_obj.bin_3_slot_9);
                        break;
                }
                FloorRobotMovetoTarget();
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                break;
        case 4: 
                switch(slot)
                {
                    case 1: floor_robot_.setJointValueTarget(js_obj.bin_4_slot_7);
                        break;
                    case 2: floor_robot_.setJointValueTarget(js_obj.bin_4_slot_8);
                        break;
                    case 3: floor_robot_.setJointValueTarget(js_obj.bin_4_slot_9);
                        break;
                }
                FloorRobotMovetoTarget();
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                break;
        case 5: floor_robot_.setJointValueTarget(js_obj.bin_5_up);
                FloorRobotMovetoTarget();
                // rclcpp::sleep_for(std::chrono::milliseconds(200));
                switch(slot)
                {
                    case 1: floor_robot_.setJointValueTarget(js_obj.bin_5_slot_1);
                        break;
                    case 2: floor_robot_.setJointValueTarget(js_obj.bin_5_slot_2);
                        break;
                    case 3: floor_robot_.setJointValueTarget(js_obj.bin_5_slot_3);
                        break;
                    case 4: floor_robot_.setJointValueTarget(js_obj.bin_5_slot_4);
                        break;
                    case 5: floor_robot_.setJointValueTarget(js_obj.bin_5_slot_5);
                        break;
                    case 6: floor_robot_.setJointValueTarget(js_obj.bin_5_slot_6);
                        break;
                    case 7: floor_robot_.setJointValueTarget(js_obj.bin_5_slot_7);
                        break;
                    case 8: floor_robot_.setJointValueTarget(js_obj.bin_5_slot_8);
                        break;
                    case 9: floor_robot_.setJointValueTarget(js_obj.bin_5_slot_9);
                        break;
                }
                FloorRobotMovetoTarget();
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                FloorRobotSetGripperState(false);
                floor_robot_.setJointValueTarget(js_obj.bin_5_up);
                FloorRobotMovetoTarget();
                break;
        case 6: floor_robot_.setJointValueTarget(js_obj.bin_6_up);
                FloorRobotMovetoTarget();
                // rclcpp::sleep_for(std::chrono::milliseconds(200));
                switch(slot)
                {
                    case 1: floor_robot_.setJointValueTarget(js_obj.bin_6_slot_1);
                        break;
                    case 2: floor_robot_.setJointValueTarget(js_obj.bin_6_slot_2);
                        break;
                    case 3: floor_robot_.setJointValueTarget(js_obj.bin_6_slot_3);
                        break;
                    case 4: floor_robot_.setJointValueTarget(js_obj.bin_6_slot_4);
                        break;
                    case 5: floor_robot_.setJointValueTarget(js_obj.bin_6_slot_5);
                        break;
                    case 6: floor_robot_.setJointValueTarget(js_obj.bin_6_slot_6);
                        break;
                    case 7: floor_robot_.setJointValueTarget(js_obj.bin_6_slot_7);
                        break;
                    case 8: floor_robot_.setJointValueTarget(js_obj.bin_6_slot_8);
                        break;
                    case 9: floor_robot_.setJointValueTarget(js_obj.bin_6_slot_9);
                        break;
                }
                FloorRobotMovetoTarget();
                rclcpp::sleep_for(std::chrono::milliseconds(200));

                FloorRobotSetGripperState(false);
                floor_robot_.setJointValueTarget(js_obj.bin_6_up);
                FloorRobotMovetoTarget();
                break;
        case 7: 
                switch(slot)
                {
                    case 1: floor_robot_.setJointValueTarget(js_obj.bin_7_slot_7);
                        break;
                    case 2: floor_robot_.setJointValueTarget(js_obj.bin_7_slot_8);
                        break;
                    case 3: floor_robot_.setJointValueTarget(js_obj.bin_7_slot_9);
                        break;
                }
                FloorRobotMovetoTarget();
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                break;
        case 8: 
                switch(slot)
                {
                    case 1: floor_robot_.setJointValueTarget(js_obj.bin_8_slot_7);
                        break;
                    case 2: floor_robot_.setJointValueTarget(js_obj.bin_8_slot_8);
                        break;
                    case 3: floor_robot_.setJointValueTarget(js_obj.bin_8_slot_9);
                        break;
                }
                FloorRobotMovetoTarget();
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                break;            
    }

    FloorRobotSetGripperState(false);
    
    return true;
}



bool CompetitionARIAC::FloorRobotPickandPlaceTray(int tray_id, int agv_num)
{
    // Check if kit tray is on one of the two tables

    geometry_msgs::msg::Pose tray_pose;
    std::string station;
    bool found_tray = false;

    constants::Constants const_obj;
    j_s_maps::FloorRobotJS js_obj;

    // Check table 1
    // for (auto tray : kts1_trays_)
    // {
    //     if (tray.id == tray_id)
    //     {
    //         station = "kts1";
    //         tray_pose = MultiplyPose(kts1_camera_pose_, tray.pose);
    //         found_tray = true;
    //         break;
    //     }
    // }
    for (auto it = kts1_trays_.begin(); it != kts1_trays_.end(); ++it)
    {
        if (it->id == tray_id)
        {
            station = "kts1";
            tray_pose = MultiplyPose(kts1_camera_pose_, it->pose);
            found_tray = true;
            kts1_trays_.erase(it);
            break;
        }
    }
    // Check table 2
    // if (!found_tray)
    // {   
    //     for (auto tray : kts2_trays_)
    //     {   
    //         if (tray.id == tray_id)
    //         {
    //             station = "kts2";
    //             tray_pose = MultiplyPose(kts2_camera_pose_, tray.pose);
    //             found_tray = true;
    //             break;
    //         }
    //     }
    // }
    if (!found_tray)
    {
        for (auto it = kts2_trays_.begin(); it != kts2_trays_.end(); ++it)
        {
            if (it->id == tray_id)
            {
                station = "kts2";
                tray_pose = MultiplyPose(kts2_camera_pose_, it->pose);
                found_tray = true;
                kts2_trays_.erase(it);
                break;
            }
        }

    }
    if (!found_tray)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("TRAY TAKEN"), " THE TASK TRAY WAS TAKEN, ATTEMPTING TO TAKE ANOTHER ");
        if(kts1_trays_.size() !=0)
        {
            for (auto it = kts1_trays_.begin(); it != kts1_trays_.end(); ++it)
            {
                station = "kts1";
                tray_pose = MultiplyPose(kts1_camera_pose_, it->pose);
                found_tray = true;
                kts1_trays_.erase(it);
                break;
            }

        }
        if(kts2_trays_.size() !=0){
            for (auto it = kts2_trays_.begin(); it != kts2_trays_.end(); ++it)
            {
                station = "kts2";
                tray_pose = MultiplyPose(kts2_camera_pose_, it->pose);
                found_tray = true;
                kts2_trays_.erase(it);
                break;
            }
        }
    }
    if (!found_tray)
        return false;

    double tray_rotation = GetYaw(tray_pose);

    // Move floor robot to the corresponding kit tray table
    if (station == "kts1")
    {
        floor_robot_.setJointValueTarget(js_obj.floor_kts1_js_);
    }
    else
    {
        floor_robot_.setJointValueTarget(js_obj.floor_kts2_js_);
    }
    FloorRobotMovetoTarget();

    // Change gripper to tray gripper
    if (floor_gripper_state_.type != "tray_gripper")
    {
        FloorRobotChangeGripper(station, "trays");
    }

    // Move to tray
    std::vector<geometry_msgs::msg::Pose> waypoints;

    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                  tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));
    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                  tray_pose.position.z + const_obj.pick_offset_, SetRobotOrientation(tray_rotation))); 

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
    FloorRobotSetGripperState(true);
    FloorRobotWaitForAttach(5.0);
   
    // Add kit tray to planning scene
    std::string tray_name = "kit_tray_" + std::to_string(tray_id);
    AddModelToPlanningScene(tray_name, "kit_tray.stl", tray_pose);
    floor_robot_.attachObject(tray_name);

    // Move up slightly
    waypoints.clear();
    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                  tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    floor_robot_.setJointValueTarget("linear_actuator_joint", const_obj.rail_positions_["agv" + std::to_string(agv_num)]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");
    auto agv_rotation = GetYaw(agv_tray_pose);

    waypoints.clear();
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + 0.3, SetRobotOrientation(agv_rotation)));

    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + const_obj.kit_tray_thickness_ + const_obj.drop_height_, SetRobotOrientation(agv_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

    FloorRobotSetGripperState(false);
    floor_robot_.detachObject(tray_name);

    // publish to robot state
    waypoints.clear();
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + 0.3, SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

    return true;
}

bool CompetitionARIAC::FloorRobotChangeGripper(std::string station, std::string gripper_type)
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("CHANGE GRIPPER FUNCTION"), " ");
    floor_robot_tool_changer_ = this->create_client<ariac_msgs::srv::ChangeGripper>("/ariac/floor_robot_change_gripper");

    // Move gripper into tool changer
    auto tc_pose = FrameWorldPose(station + "_tool_changer_" + gripper_type + "_frame");

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                  tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

    waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                  tc_pose.position.z, SetRobotOrientation(0.0)));

    if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1))
        return false;

    // Call service to change gripper
    auto request = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();

    if (gripper_type == "trays")
    {
        request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;
    }
    else if (gripper_type == "parts")
    {
        request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
    }

    auto result = floor_robot_tool_changer_->async_send_request(request);
    result.wait();
    if (!result.get()->success)
    {
        RCLCPP_ERROR(get_logger(), "Error calling gripper change service");
        return false;
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("CHANGE GRIPPER FUNCTION"), " ");
    waypoints.clear();
    waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                  tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

    if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1))
        return false;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("CHANGE GRIPPER FUNCTION"), " ");
    return true;
}


// bool CompetitionARIAC::FloorRobotPickBinPart(order_::KittingPart part_to_pick)
// {
//     color_::ColorParts Col_part_obj;

//     RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick a " + Col_part_obj.COLOR[part_to_pick.color] + " " + Col_part_obj.PART[part_to_pick.type]);

//     // Check if part is in one of the bins
//     geometry_msgs::msg::Pose part_pose;
//     geometry_msgs::msg::Pose check_flip_part_pose;
//     bool flipped{false};
//     bool vert_bin{false};
//     bool found_part = false;
//     std::string bin_side;

//     constants::Constants const_obj;
//     j_s_maps::FloorRobotJS js_obj;


//     // Check left bins
//     for (unsigned int i=0 ; i< left_bins_parts_.size();i++)
//     {   auto part = left_bins_parts_[i];

//         if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
//         {
//             part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
//             // check_flip_part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
//             tf2::Quaternion q(
//             part_pose.orientation.x,
//             part_pose.orientation.y,
//             part_pose.orientation.z,
//             part_pose.orientation.w);
//             tf2::Matrix3x3 m(q);
//             double roll, pitch, yaw;
//             m.getRPY(roll, pitch, yaw);

//             if(int(abs(roll))==3)
//                 flipped = true;

//             if(int(abs(pitch))==1)
//                 vert_bin = true;

//             found_part = true;
//             bin_side = "left_bins";
//             left_bins_parts_.erase(left_bins_parts_.begin() + i);
//             break;
//         }
//     }
//     // Check right bins
//     if (!found_part)
//     {
//         for (unsigned int i=0 ; i< right_bins_parts_.size();i++)
//         {
//             auto part = right_bins_parts_[i];

//             if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
//             {
//                 part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
//                 // check_flip_part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
//                 tf2::Quaternion q(
//                 part_pose.orientation.x,
//                 part_pose.orientation.y,
//                 part_pose.orientation.z,
//                 part_pose.orientation.w);
//                 tf2::Matrix3x3 m(q);
//                 double roll, pitch, yaw;
//                 m.getRPY(roll, pitch, yaw);

//                 if(int(abs(roll))==3)
//                     flipped = true;
                
//                 if(int(abs(pitch))==1)
//                     vert_bin = true;

//                 found_part = true;
//                 bin_side = "right_bins";
//                 right_bins_parts_.erase(right_bins_parts_.begin() + i);
//                 break;
//             }
//         }
//     }
//     if (!found_part)
//     {
//         RCLCPP_ERROR(get_logger(), "Unable to locate the required part (any color) !!!");
//         FloorRobotPickBinPartReplaceMissing(part_to_pick.color,part_to_pick.type);
//         return false;
//     }

//     double part_rotation = GetYaw(part_pose);
//     double part_rotation2 = GetPitch(part_pose);

//     // Change gripper at location closest to part
//     if (floor_gripper_state_.type != "part_gripper")
//     {
//         std::string station;
//         if (part_pose.position.y < 0)
//         {
//             station = "kts1";
//         }
//         else
//         {
//             station = "kts2";
//         }

//         // Move floor robot to the corresponding kit tray table
//         if (station == "kts1")
//         {
//             floor_robot_.setJointValueTarget(js_obj.floor_kts1_js_);
//         }
//         else
//         {
//             floor_robot_.setJointValueTarget(js_obj.floor_kts2_js_);
//         }
//         FloorRobotMovetoTarget();

//         FloorRobotChangeGripper(station, "parts");
//     }

//     floor_robot_.setJointValueTarget("linear_actuator_joint", js_obj.rail_positions_[bin_side]);
//     floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
//     FloorRobotMovetoTarget();  

//     std::vector<geometry_msgs::msg::Pose> waypoints;


//     if(flipped)
//     {
//         RCLCPP_INFO_STREAM(rclcpp::get_logger("Flipped Part Challenge"), "Part is flipped "); 
//         waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
//                                   part_pose.position.z + 0.005 , SetRobotOrientation(part_rotation)));
                                
//     }
//     else if(vert_bin)
//     {
//         RCLCPP_INFO_STREAM(rclcpp::get_logger("Vertical Pump"), "Vertical Pump "); 
//         RCLCPP_INFO_STREAM(rclcpp::get_logger("Vertical Pump"), "Vertical Pump x coord " + std::to_string(part_pose.position.x)); 
//         RCLCPP_INFO_STREAM(rclcpp::get_logger("Vertical Pump"), "Vertical Pump y coord " + std::to_string(part_pose.position.y)); 
//         RCLCPP_INFO_STREAM(rclcpp::get_logger("Vertical Pump"), "Vertical Pump z coord " + std::to_string(part_pose.position.z)); 
//         RCLCPP_INFO_STREAM(rclcpp::get_logger("Vertical Pump"), "Vertical Pump "); 
//         waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y+0.05,
//                                   part_pose.position.z + 0.067 , SetRobotOrientation(part_rotation2)));
//     }
//     else
//     {
//         RCLCPP_INFO_STREAM(rclcpp::get_logger("Flipped Part Challenge"), "Part is not flipped ");
//         waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
//                                   part_pose.position.z + 0.5, SetRobotOrientation(part_rotation)));
        
//         waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
//                                   part_pose.position.z + const_obj.part_heights_[part_to_pick.type] + const_obj.pick_offset_, SetRobotOrientation(part_rotation)));
//     }
    

//     FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

//     FloorRobotSetGripperState(true);
//     if(part_to_pick.type == ariac_msgs::msg::Part::PUMP)
//         FloorRobotWaitForAttachPump(10.0);
//     else
//         FloorRobotWaitForAttach(5.0);

//     // Add part to planning scene
//     std::string part_name = Col_part_obj.COLOR[part_to_pick.color] + "_" + Col_part_obj.PART[part_to_pick.type];
//     // AddModelToPlanningScene(part_name, Col_part_obj.PART[part_to_pick.type] + ".stl", part_pose);
//     floor_robot_.attachObject(part_name);
//     floor_robot_attached_part_ = part_to_pick;

//     // Move up slightly
//     waypoints.clear();
//     waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
//                                   part_pose.position.z + 0.3, SetRobotOrientation(0)));
     
//     FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

//     return true;
// }

bool CompetitionARIAC::FloorRobotPickBinPart(order_::KittingPart part_to_pick)
{
    color_::ColorParts Col_part_obj;

    RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick a " + Col_part_obj.COLOR[part_to_pick.color] + " " + Col_part_obj.PART[part_to_pick.type]);

    // Check if part is in one of the bins
    geometry_msgs::msg::Pose part_pose;
    geometry_msgs::msg::Pose check_flip_part_pose;
    bool flipped{false};
    bool found_part = false;
    std::string bin_side;

    constants::Constants const_obj;
    j_s_maps::FloorRobotJS js_obj;


    // Check left bins
    for (unsigned int i=0 ; i< left_bins_parts_.size();i++)
    {   auto part = left_bins_parts_[i];

        if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
        {
            part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
            // check_flip_part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
            tf2::Quaternion q(
            part_pose.orientation.x,
            part_pose.orientation.y,
            part_pose.orientation.z,
            part_pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            if(int(abs(roll))==3)
                flipped = true;

            found_part = true;
            bin_side = "left_bins";
            left_bins_parts_.erase(left_bins_parts_.begin() + i);
            break;
        }
    }
    // Check right bins
    if (!found_part)
    {
        for (unsigned int i=0 ; i< right_bins_parts_.size();i++)
        {
            auto part = right_bins_parts_[i];

            if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
            {
                part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
                // check_flip_part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
                tf2::Quaternion q(
                part_pose.orientation.x,
                part_pose.orientation.y,
                part_pose.orientation.z,
                part_pose.orientation.w);
                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                if(int(abs(roll))==3)
                    flipped = true;
                found_part = true;
                bin_side = "right_bins";
                right_bins_parts_.erase(right_bins_parts_.begin() + i);
                break;
            }
        }
    }
    if (!found_part)
    {
        RCLCPP_ERROR(get_logger(), "Unable to locate the required part (any color) !!!");
        FloorRobotPickBinPartReplaceMissing(part_to_pick.color,part_to_pick.type);
        return false;
    }

    double part_rotation = GetYaw(part_pose);

    // Change gripper at location closest to part
    if (floor_gripper_state_.type != "part_gripper")
    {
        std::string station;
        if (part_pose.position.y < 0)
        {
            station = "kts1";
        }
        else
        {
            station = "kts2";
        }

        // Move floor robot to the corresponding kit tray table
        if (station == "kts1")
        {
            floor_robot_.setJointValueTarget(js_obj.floor_kts1_js_);
        }
        else
        {
            floor_robot_.setJointValueTarget(js_obj.floor_kts2_js_);
        }
        FloorRobotMovetoTarget();

        FloorRobotChangeGripper(station, "parts");
    }

    floor_robot_.setJointValueTarget("linear_actuator_joint", js_obj.rail_positions_[bin_side]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();  

    std::vector<geometry_msgs::msg::Pose> waypoints;




    if(flipped)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Parts   availability"), "Part is flipped "); 
        waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + 0.005 , SetRobotOrientation(part_rotation)));
                                
    }
    else
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Parts   availability"), "Part is not flipped ");
        waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + 0.5, SetRobotOrientation(part_rotation)));
        
        waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + const_obj.part_heights_[part_to_pick.type] + const_obj.pick_offset_, SetRobotOrientation(part_rotation)));
    

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    FloorRobotSetGripperState(true);
    }
    
    if(part_to_pick.type == ariac_msgs::msg::Part::PUMP)
    {

        for (auto part : right_bins_parts_)
        {
            if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
            {
                
                RCLCPP_INFO_STREAM(rclcpp::get_logger("OI KUNT"), part.pose.position.y); 
                if ((part.pose.position.y >= 0.335 && part.pose.position.y <= 0.4) || (part.pose.position.y <= -0.410 && part.pose.position.y >= -0.45 ))
                {
                    
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("OI KUNT"), part.pose.position.y); 
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("OI KUNT"), "ATTEMPTING PICK"); 
                    FloorRobotWaitForAttachPump(7.4);
                    // right_bins_parts_.erase(it);
                }
                
                //SLOT 4
                else if ((part.pose.position.y >= 0.515 && part.pose.position.y <= 0.545 ) || (part.pose.position.y <= -0.234 && part.pose.position.y >= -0.240 )) 
                {
                    if(part.pose.position.x <= 1.076851964642432)
                    {
                       FloorRobotSetGripperState(true);
                        RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 1"), part.pose.position.x); 
                        RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 1"), "ATTEMPTING PICK"); 
                        
                        FloorRobotSetGripperState(true);
                        FloorRobotWaitForAttachPump(7.7);
                    }
                    else{

                    FloorRobotSetGripperState(true);
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 4"), part.pose.position.x); 
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 4"), "ATTEMPTING PICK"); 
                    
                    FloorRobotSetGripperState(true);
                    FloorRobotWaitForAttachPump(7.55);
                    }

                }
                //SLOT 6
                else if ((part.pose.position.y >= 0.155 && part.pose.position.y <= 0.25) || (part.pose.position.y <= -0.594 && part.pose.position.y >= -0.630 ))
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 6"), part.pose.position.y); 
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 6"), "ATTEMPTING PICK"); 
                    
                    FloorRobotSetGripperState(true);
                    FloorRobotWaitForAttachPump(7.85);
                    // AddModelToPlanningScene(part_name,"pump.stl", part.pose);
                }
                // }
                else{
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("OI KUNT"), "ATTEMPTING PICK ANOTHER");
                    FloorRobotWaitForAttachPump(8.0);
            
                }
            }
        }

        for (auto part : left_bins_parts_)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("LEFT CAMERA"), part.pose.position.y); 
            if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("LEFT CAMERA"), part.pose.position.y); 
                if ((part.pose.position.y >= 0.415 && part.pose.position.y <= 0.450) || (part.pose.position.y <= -0.334 && part.pose.position.y >= -0.36 ))
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("LEFT CAMERA"), part.pose.position.y); 
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("LEFT CAMERA"), "ATTEMPTING PICK"); 
                    FloorRobotWaitForAttachPump(7.4);
                }
                else if ((part.pose.position.y >= 0.595 && part.pose.position.y <= 0.625 ) || (part.pose.position.y <= -0.15 && part.pose.position.y >= -0.225)) 
                {
                    if(part.pose.position.x >= 1.077186840622127)
                    {
                       FloorRobotSetGripperState(true);
                        RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 1"), part.pose.position.y); 
                        RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 1"), "ATTEMPTING PICK"); 
                        
                        FloorRobotSetGripperState(true);
                        FloorRobotWaitForAttachPump(7.7);
                   }
                    else{

                    FloorRobotSetGripperState(true);
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 4"), part.pose.position.y); 
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 4"), "ATTEMPTING PICK"); 
                    
                    FloorRobotSetGripperState(true);
                    FloorRobotWaitForAttachPump(7.55);
                    }
                }
                else{
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("LEFT CAMERA"), "ATTEMPTING PICK ANOTHER");
                    FloorRobotWaitForAttachPump(8.2);
            
                }
            }
        }


    }
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("PUMP PICK FROM"), std::to_string(slot)); 
    //     FloorRobotWaitForAttachPump(9.4);
    //     if(slot==6 or slot==4)
    //         FloorRobotWaitForAttachPump(5.0);

    else
        FloorRobotWaitForAttach(5.0);

    // Add part to planning scene
    std::string part_name = Col_part_obj.COLOR[part_to_pick.color] + "_" + Col_part_obj.PART[part_to_pick.type];
    // AddModelToPlanningScene(part_name, Col_part_obj.PART[part_to_pick.type] + ".stl", part_pose);
    floor_robot_.attachObject(part_name);
    floor_robot_attached_part_ = part_to_pick;

    // Move up slightly
    waypoints.clear();
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + 0.3, SetRobotOrientation(0)));
     
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
    // rclcpp::sleep_for(std::chrono::seconds(30));
    return true;
}

// bool CompetitionARIAC::FloorRobotPickBinPartCombined(ariac_msgs::msg::Part part_to_pick)
// {
//     color_::ColorParts Col_part_obj;

//     RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick a " << Col_part_obj.COLOR[part_to_pick.color] << " " << Col_part_obj.PART[part_to_pick.type]);

//     // Check if part is in one of the bins
//     geometry_msgs::msg::Pose part_pose;
//     bool found_part = false;
//     std::string bin_side;

//     constants::Constants const_obj;
//     j_s_maps::FloorRobotJS js_obj;

//     // Check left bins
//     for (auto part : left_bins_parts_)
//     {
//         if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
//         {
//             part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
//             found_part = true;
//             bin_side = "left_bins";
//             break;
//         }
//     }
//     // Check right bins
//     if (!found_part)
//     {
//         for (auto part : right_bins_parts_)
//         {
//             if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
//             {
//                 part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
//                 found_part = true;
//                 bin_side = "right_bins";
//                 break;
//             }
//         }
//     }
//     if (!found_part)
//     {
//         // Check left bins for same type
//         for (auto part : left_bins_parts_)
//         {
//             if (part.part.type == part_to_pick.type)
//             {
//                 part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
//                 found_part = true;
//                 bin_side = "left_bins";
//                 break;
//             }
//         }
//     }
//     // Check right bins for same type
//     if (!found_part)
//     {
//         for (auto part : right_bins_parts_)
//         {
//             if (part.part.type == part_to_pick.type)
//             {
//                 part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
//                 found_part = true;
//                 bin_side = "right_bins";
//                 break;
//             }
//         }
//     }

//     if (!found_part)
//     {
//         RCLCPP_ERROR(get_logger(), "Unable to locate part");
//         return false;
//     }

//     double part_rotation = GetYaw(part_pose);

//     // Change gripper at location closest to part
//     if (floor_gripper_state_.type != "part_gripper")
//     {
//         std::string station;
//         if (part_pose.position.y < 0)
//         {
//             station = "kts1";
//         }
//         else
//         {
//             station = "kts2";
//         }

//         // Move floor robot to the corresponding kit tray table
//         if (station == "kts1")
//         {
//             floor_robot_.setJointValueTarget(js_obj.floor_kts1_js_);
//         }
//         else
//         {
//             floor_robot_.setJointValueTarget(js_obj.floor_kts2_js_);
//         }
//         FloorRobotMovetoTarget();

//         FloorRobotChangeGripper(station, "parts");
//     }

//     floor_robot_.setJointValueTarget("linear_actuator_joint", js_obj.rail_positions_[bin_side]);
//     floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
//     FloorRobotMovetoTarget();
    

//     std::vector<geometry_msgs::msg::Pose> waypoints;
    
    
//     if(part_to_pick.type == 11)
//     {
//         waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
//                                   part_pose.position.z + 0.6, SetRobotOrientation(part_rotation)));

//         waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
//                                   part_pose.position.z + const_obj.part_heights_[part_to_pick.type] + const_obj.pick_offset_, SetRobotOrientation(part_rotation)));

//     }
    
//     else{
//     waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
//                                   part_pose.position.z + 0.5, SetRobotOrientation(part_rotation)));

//     waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
//                                   part_pose.position.z + const_obj.part_heights_[part_to_pick.type] + const_obj.pick_offset_, SetRobotOrientation(part_rotation)));
//     }

//     FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

//     FloorRobotSetGripperState(true);
//     FloorRobotWaitForAttach(4.0);

//     // Add part to planning scene
//     std::string part_name = Col_part_obj.COLOR[part_to_pick.color] + "_" + Col_part_obj.PART[part_to_pick.type];
//     // AddModelToPlanningScene(part_name, Col_part_obj.PART[part_to_pick.type] + ".stl", part_pose);
//     floor_robot_.attachObject(part_name);
//     floor_robot_attached_part_comb = part_to_pick;

//     // Move up slightly
//     waypoints.clear();
//     waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
//                                   part_pose.position.z + 0.3, SetRobotOrientation(0)));
    
    
//     FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
//     // rclcpp::sleep_for(std::chrono::seconds(30));

//     return true;
// }

bool CompetitionARIAC::FloorRobotPickBinPartCombined(ariac_msgs::msg::Part part_to_pick)
{
    color_::ColorParts Col_part_obj;
    j_s_maps::FloorRobotJS js_obj;

    RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick a " << Col_part_obj.COLOR[part_to_pick.color] << " " << Col_part_obj.PART[part_to_pick.type]);

    // Check if part is in one of the bins
    geometry_msgs::msg::Pose part_pose;
    bool found_part = false;
    std::string bin_side;

    constants::Constants const_obj;

   // Check left bins
    for (auto part : left_bins_parts_)
    {
        if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
        {
            part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
            found_part = true;
            bin_side = "left_bins";
            break;
        }
    }
    // Check right bins
    if (!found_part)
    {
        for (auto part : right_bins_parts_)
        {
            if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
            {
                part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
                found_part = true;
                bin_side = "right_bins";
                break;
            }
        }
    }
    if (!found_part)
    {
        // Check left bins for same type
        for (auto part : left_bins_parts_)
        {
            if (part.part.type == part_to_pick.type)
            {
                part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
                found_part = true;
                bin_side = "left_bins";
                break;
            }
        }
    }
    // Check right bins for same type
    if (!found_part)
    {
        for (auto part : right_bins_parts_)
        {
            if (part.part.type == part_to_pick.type)
            {
                part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
                found_part = true;
                bin_side = "right_bins";
                break;
            }
        }
    }

    if (!found_part)
    {
        RCLCPP_ERROR(get_logger(), "Unable to locate part");
        return false;
    }


    double part_rotation = GetYaw(part_pose);

    // Change gripper at location closest to part
    if (floor_gripper_state_.type != "part_gripper")
    {
        std::string station;
        if (part_pose.position.y < 0)
        {
            station = "kts1";
        }
        else
        {
            station = "kts2";
        }

        // Move floor robot to the corresponding kit tray table
        if (station == "kts1")
        {
            floor_robot_.setJointValueTarget(js_obj.floor_kts1_js_);
        }
        else
        {
            floor_robot_.setJointValueTarget(js_obj.floor_kts2_js_);
        }
        FloorRobotMovetoTarget();

        FloorRobotChangeGripper(station, "parts");
    }

    floor_robot_.setJointValueTarget("linear_actuator_joint", js_obj.rail_positions_[bin_side]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();
    

    std::vector<geometry_msgs::msg::Pose> waypoints;
    
    if(part_to_pick.type == 11)
    {
        waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + 0.7, SetRobotOrientation(part_rotation)));

        waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + const_obj.part_heights_[part_to_pick.type] + const_obj.pick_offset_, SetRobotOrientation(part_rotation)));
          
        FloorRobotMoveCartesian(waypoints, 0.0, 0.1);

        FloorRobotSetGripperState(true);
        // FloorRobotWaitForAttachPump(9.9);

                    // std::string part_name = Col_part_obj.COLOR[part_to_pick.color] + "_" + Col_part_obj.PART[part_to_pick.type];
                    // AddModelToPlanningScene(part_name,"pump.stl", part.pose);


        for (auto part : right_bins_parts_)
        {
            if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
            {
                
                RCLCPP_INFO_STREAM(rclcpp::get_logger("OI KUNT"), part.pose.position.y); 
                if ((part.pose.position.y >= 0.335 && part.pose.position.y <= 0.4) || (part.pose.position.y <= -0.410 && part.pose.position.y >= -0.45 ))
                {
                    
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("OI KUNT"), part.pose.position.y); 
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("OI KUNT"), "ATTEMPTING PICK"); 
                    FloorRobotWaitForAttachPump(7.4);
                    // right_bins_parts_.erase(it);
                }
                
                //SLOT 4
                else if ((part.pose.position.y >= 0.515 && part.pose.position.y <= 0.545 ) || (part.pose.position.y <= -0.234 && part.pose.position.y >= -0.240 )) 
                {
                    // waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                    //               part_pose.position.z + 0.8, SetRobotOrientation(part_rotation)));

                    // waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                    //                         part_pose.position.z + const_obj.part_heights_[part_to_pick.type] + const_obj.pick_offset_, SetRobotOrientation(part_rotation)));
                    
                    // FloorRobotMoveCartesian(waypoints, 0.0, 0.1);
                    if(part.pose.position.x <= 1.076851964642432)
                    {
                       FloorRobotSetGripperState(true);
                        RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 1"), part.pose.position.y); 
                        RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 1"), "ATTEMPTING PICK"); 
                        
                        FloorRobotSetGripperState(true);
                        FloorRobotWaitForAttachPump(7.7);
                    }
                    else{

                    FloorRobotSetGripperState(true);
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 4"), part.pose.position.y); 
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 4"), "ATTEMPTING PICK"); 
                    
                    FloorRobotSetGripperState(true);
                    FloorRobotWaitForAttachPump(7.55);
                    }

                }
                // else if()
                // {
                //     // waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                //     //               part_pose.position.z + 0.8, SetRobotOrientation(part_rotation)));

                //     // waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                //     //                         part_pose.position.z + const_obj.part_heights_[part_to_pick.type] + const_obj.pick_offset_, SetRobotOrientation(part_rotation)));
                    
                //     // FloorRobotMoveCartesian(waypoints, 0.0, 0.1);
                //     RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 4 BIN 1"), part.pose.position.y); 
                //     RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 4 BIN 1"), "ATTEMPTING PICK"); 
                //     FloorRobotSetGripperState(true);
                //     FloorRobotWaitForAttachPump(7.55);
                // }
                //SLOT 6
                else if ((part.pose.position.y >= 0.155 && part.pose.position.y <= 0.25) || (part.pose.position.y <= -0.594 && part.pose.position.y >= -0.630 ))
                {
                    // waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                    //               part_pose.position.z + 0.8, SetRobotOrientation(part_rotation)));

                    // waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                    //                         part_pose.position.z + const_obj.part_heights_[part_to_pick.type] + const_obj.pick_offset_, SetRobotOrientation(part_rotation)));
                    
                    // FloorRobotMoveCartesian(waypoints, 0.0, 0.1);
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 6"), part.pose.position.y); 
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 6"), "ATTEMPTING PICK"); 
                    
                    FloorRobotSetGripperState(true);
                    FloorRobotWaitForAttachPump(7.85);
                    // AddModelToPlanningScene(part_name,"pump.stl", part.pose);
                }
                // else if()
                // {
                //     // waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                //     //               part_pose.position.z + 0.6, SetRobotOrientation(part_rotation)));

                //     // waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                //     //                         part_pose.position.z + const_obj.part_heights_[part_to_pick.type] + const_obj.pick_offset_, SetRobotOrientation(part_rotation)));
                    
                //     // FloorRobotMoveCartesian(waypoints, 0.0, 0.1);
                //     RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 6 BIN 1"), part.pose.position.y); 
                //     RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 6 BIN 1"), "ATTEMPTING PICK"); 
                    
                //     FloorRobotSetGripperState(true);
                //     // floor_robot_.setJointValueTarget(js_obj.bin_1_slot_6_pump);
                //     FloorRobotWaitForAttachPump(7.85);
                //     // AddModelToPlanningScene(part_name,"pump.stl", part.pose);
                // }
                else{
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("OI KUNT"), "ATTEMPTING PICK ANOTHER");
                    FloorRobotWaitForAttachPump(8.0);
            
                }
            }
        }
        for (auto part : left_bins_parts_)
        {
            if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("LEFT CAMERA"), part.pose.position.y); 
                if ((part.pose.position.y >= 0.415 && part.pose.position.y <= 0.450) || (part.pose.position.y <= -0.334 && part.pose.position.y >= -0.36 ))
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("LEFT CAMERA"), part.pose.position.y); 
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("LEFT CAMERA"), "ATTEMPTING PICK"); 
                    FloorRobotWaitForAttachPump(7.4);
                }
                else if ((part.pose.position.y >= 0.595 && part.pose.position.y <= 0.625 ) || (part.pose.position.y <= -0.15 && part.pose.position.y >= -0.225)) 
                {
                    // waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                    //               part_pose.position.z + 0.8, SetRobotOrientation(part_rotation)));

                    // waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                    //                         part_pose.position.z + const_obj.part_heights_[part_to_pick.type] + const_obj.pick_offset_, SetRobotOrientation(part_rotation)));
                    
                    // FloorRobotMoveCartesian(waypoints, 0.0, 0.1);
                    if(part.pose.position.x >= 1.077186840622127)
                    {
                       FloorRobotSetGripperState(true);
                        RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 1"), part.pose.position.y); 
                        RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 1"), "ATTEMPTING PICK"); 
                        
                        FloorRobotSetGripperState(true);
                        FloorRobotWaitForAttachPump(7.7);
                     
                    }
                    else{

                    FloorRobotSetGripperState(true);
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 4"), part.pose.position.y); 
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("SLOT 4"), "ATTEMPTING PICK"); 
                    
                    FloorRobotSetGripperState(true);
                    FloorRobotWaitForAttachPump(7.55);
                    }
                }
                else{
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("LEFT CAMERA"), "ATTEMPTING PICK ANOTHER");
                    FloorRobotWaitForAttachPump(8.2);
            
                }
            }
        }

    }
    
    else{
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + 0.5, SetRobotOrientation(part_rotation)));

    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + const_obj.part_heights_[part_to_pick.type] + const_obj.pick_offset_, SetRobotOrientation(part_rotation)));
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    FloorRobotSetGripperState(true);
    FloorRobotWaitForAttach(4.0);
    }
    // FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    // FloorRobotSetGripperState(true);
    // FloorRobotWaitForAttach(4.0);

    // Add part to planning scene
    std::string part_name = Col_part_obj.COLOR[part_to_pick.color] + "_" + Col_part_obj.PART[part_to_pick.type];
    // AddModelToPlanningScene(part_name, Col_part_obj.PART[part_to_pick.type] + ".stl", part_pose);
    floor_robot_.attachObject(part_name);
    floor_robot_attached_part_comb = part_to_pick;

    // Move up slightly
    waypoints.clear();
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + 0.3, SetRobotOrientation(0)));
    
    
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
    // rclcpp::sleep_for(std::chrono::seconds(30));

    return true;
}

bool CompetitionARIAC::FloorRobotPickBinPartReplaceMissing(int part_color, int part_type)
{
    color_::ColorParts Col_part_obj;

    RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick a " +Col_part_obj.COLOR[part_color] + " "+  Col_part_obj.PART[part_type]);

    // Check if part is in one of the bins
    geometry_msgs::msg::Pose part_pose;
    bool found_part = false;
    std::string bin_side;

    constants::Constants const_obj;
    j_s_maps::FloorRobotJS js_obj;

    // Check left bins
    for (unsigned int i=0 ; i< left_bins_parts_.size();i++)
    {   auto part = left_bins_parts_[i];

        if (part.part.type == part_type && part.part.color == part_color)
        {
            part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
            found_part = true;
            bin_side = "left_bins";
            left_bins_parts_.erase(left_bins_parts_.begin() + i);
            break;
        }
    }
    // Check right bins
    if (!found_part)
    {
        for (unsigned int i=0 ; i< right_bins_parts_.size();i++)
        {
            auto part = right_bins_parts_[i];
            if (part.part.type == part_type && part.part.color == part_color)
            {
                part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
                found_part = true;
                bin_side = "right_bins";
                right_bins_parts_.erase(right_bins_parts_.begin() + i);
                break;
            }
        }
    }
    if (!found_part)
    {
        RCLCPP_ERROR(get_logger(), "Unable to locate the required part (specific color) !!! ");

        // Check left bins
        for (unsigned int i=0 ; i< left_bins_parts_.size();i++)
        {   auto part = left_bins_parts_[i];

            if (part.part.type == part_type )
            {
                part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
                found_part = true;
                bin_side = "left_bins";
                left_bins_parts_.erase(left_bins_parts_.begin() + i);
                break;
            }
        }
        // Check right bins
        if (!found_part)
        {
            for (unsigned int i=0 ; i< right_bins_parts_.size();i++)
            {
                auto part = right_bins_parts_[i];
                if (part.part.type == part_type)
                {
                    part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
                    found_part = true;
                    bin_side = "right_bins";
                    right_bins_parts_.erase(right_bins_parts_.begin() + i);
                    break;
                }
            }
        }
    }

    double part_rotation = GetYaw(part_pose);

    floor_robot_.setJointValueTarget("linear_actuator_joint", js_obj.rail_positions_[bin_side]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + 0.5, SetRobotOrientation(part_rotation)));

    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + const_obj.part_heights_[part_type] + const_obj.pick_offset_, SetRobotOrientation(part_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
    FloorRobotSetGripperState(true);
    FloorRobotWaitForAttach(5.0);

    // Add part to planning scene
    std::string part_name = "red_" + Col_part_obj.PART[part_type];
    AddModelToPlanningScene(part_name, Col_part_obj.PART[part_type] + ".stl", part_pose);
    floor_robot_.attachObject(part_name);

    // Move up slightly
    waypoints.clear();
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + 0.3, SetRobotOrientation(0)));
   
    
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
    

    return true;
}

bool CompetitionARIAC::FloorRobotPlacePartOnKitTray(int agv_num, int quadrant)
{
    color_::ColorParts Col_part_obj;
    RCLCPP_ERROR(get_logger(), "attached" + static_cast<int>(quadrant));


    if (!floor_gripper_state_.attached)
    {
        RCLCPP_ERROR(get_logger(), "No part attached");
        return false;
    }

    constants::Constants const_obj;

    // Move to agv
    floor_robot_.setJointValueTarget("linear_actuator_joint", const_obj.rail_positions_["agv" + std::to_string(agv_num)]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    // Determine target pose for part based on agv_tray pose
    auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");

    auto part_drop_offset = BuildPose(const_obj.quad_offsets_[quadrant].first, const_obj.quad_offsets_[quadrant].second, 0.0,
                                      geometry_msgs::msg::Quaternion());

    auto part_drop_pose = MultiplyPose(agv_tray_pose, part_drop_offset);

    std::vector<geometry_msgs::msg::Pose> waypoints;

    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + 0.3, SetRobotOrientation(0)));

    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + const_obj.part_heights_[floor_robot_attached_part_.type] + const_obj.drop_height_+0.15,
                                  SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    // Drop part in quadrant
    FloorRobotSetGripperState(false);

    std::string part_name = Col_part_obj.COLOR[floor_robot_attached_part_.color] +
                            "_" + Col_part_obj.PART[floor_robot_attached_part_.type];
    floor_robot_.detachObject(part_name);

    waypoints.clear();
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + 0.3,
                                  SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

    return true;
}

bool CompetitionARIAC::FloorRobotDisposeParts(int agv_num, int quadrant)
{
    color_::ColorParts Col_part_obj;

    j_s_maps::FloorRobotJS js_obj;
    constants::Constants const_obj;

    // Move to agv
    floor_robot_.setJointValueTarget("linear_actuator_joint", const_obj.rail_positions_["agv" + std::to_string(agv_num)]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    // Determine target pose for part based on agv_tray pose
    auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");

    auto part_drop_offset = BuildPose(const_obj.quad_offsets_[quadrant].first, const_obj.quad_offsets_[quadrant].second, 0.0,
                                      geometry_msgs::msg::Quaternion());

    auto part_drop_pose = MultiplyPose(agv_tray_pose, part_drop_offset);

    std::vector<geometry_msgs::msg::Pose> waypoints;

    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + const_obj.part_heights_kit_pick[floor_robot_attached_part_.type]+0.065, SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    FloorRobotSetGripperState(true);
    FloorRobotWaitForAttachKitTrayPart(10.0);

    // Move up slightly
    waypoints.clear();
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + 0.3, SetRobotOrientation(0)));
   
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    waypoints.clear();
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + 0.3,
                                  SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

    floor_robot_.setJointValueTarget(js_obj.drop_faulty_part);
    FloorRobotMovetoTarget();
    // Drop part in bin
    FloorRobotSetGripperState(false);

    return true;
}

bool CompetitionARIAC::FloorRobotPickFlippedPart(int agv_num, int quadrant)
{
    color_::ColorParts Col_part_obj;
    j_s_maps::FloorRobotJS js_obj;
    constants::Constants const_obj;

    // Move to agv
    floor_robot_.setJointValueTarget("linear_actuator_joint", const_obj.rail_positions_["agv" + std::to_string(agv_num)]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    // Determine target pose for part based on agv_tray pose
    auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");

    auto part_drop_offset = BuildPose(const_obj.quad_offsets_[quadrant].first, const_obj.quad_offsets_[quadrant].second, 0.0,
                                      geometry_msgs::msg::Quaternion());

    auto part_drop_pose = MultiplyPose(agv_tray_pose, part_drop_offset);

    std::vector<geometry_msgs::msg::Pose> waypoints;

    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + const_obj.part_heights_kit_pick[floor_robot_attached_part_.type]+0.065, SetRobotOrientation(0)));


    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    FloorRobotSetGripperState(true);
    FloorRobotWaitForAttachKitTrayPart(5.0);


    // Move up slightly
    waypoints.clear();
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + 0.3, SetRobotOrientation(0)));
   
    
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    waypoints.clear();
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + 0.3,  SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

    // floor_robot_.setJointValueTarget(js_obj.floor_flipped_js_);
    // FloorRobotMovetoTarget();
    

    // ceiling_robot_.setJointValueTarget(js_obj.ceiling_flipped_js_up);
    // CeilingRobotMovetoTarget();
    // ceiling_robot_.setJointValueTarget(js_obj.ceiling_flipped_js_);
    // CeilingRobotMovetoTarget();
    // CeilingRobotSetGripperState(true);
    // CeilingRobotWaitForAttachFlipped(3); 
    // // Drop part in bin
    // FloorRobotSetGripperState(false);
    // ceiling_robot_.setJointValueTarget(js_obj.ceiling_flipped_js_up);
    // CeilingRobotMovetoTarget();

    floor_robot_.setJointValueTarget(js_obj.floor_flipped_js_up);
    FloorRobotMovetoTarget();
    ceiling_robot_.setJointValueTarget(js_obj.ceiling_flipped_js_down);
    CeilingRobotMovetoTarget();
    CeilingRobotSetGripperState(true);
    rclcpp::sleep_for(std::chrono::seconds(1)); 
    FloorRobotSetGripperState(false);

    return true;
}

bool CompetitionARIAC::CeilingRobotPlaceFlippedPartOnKitTray(int agv_num, int quadrant)
{
    color_::ColorParts Col_part_obj;
    j_s_maps::FloorRobotJS js_obj;
    RCLCPP_ERROR(get_logger(), "attached" + static_cast<int>(quadrant));

    constants::Constants const_obj;

    floor_robot_.setJointValueTarget(js_obj.floor_straight_js_);
    FloorRobotMovetoTarget();

    ceiling_robot_.setJointValueTarget(js_obj.ceiling_flipped_js_down1);
    CeilingRobotMovetoTarget();
    // ceiling_robot_.setJointValueTarget(js_obj.ceiling_flipped_js_down2);
    // CeilingRobotMovetoTarget();
    

    switch(agv_num)
    {
        case 1: ceiling_robot_.setJointValueTarget(js_obj.ceiling_agv1_js_drop);
                break;
        case 2: ceiling_robot_.setJointValueTarget(js_obj.ceiling_agv2_js_up);
                CeilingRobotMovetoTarget();
                ceiling_robot_.setJointValueTarget(js_obj.ceiling_agv2_js_drop);
                break;
        case 3: ceiling_robot_.setJointValueTarget(js_obj.ceiling_agv3_js_drop);
                break;
        case 4: ceiling_robot_.setJointValueTarget(js_obj.ceiling_agv4_js_drop);
                break;
    }
    // moveit_msgs::msg::JointConstraint wrist_2_constraint;
    // wrist_2_constraint.joint_name = "ceiling_wrist_3_joint";
    // wrist_2_constraint.position = 0.0;
    // wrist_2_constraint.tolerance_above = M_PI_2;
    // wrist_2_constraint.tolerance_below = M_PI_2;
    // wrist_2_constraint.weight = 1.0;
    // moveit_msgs::msg::Constraints constraints;
    // constraints.joint_constraints.push_back(wrist_2_constraint);
    // ceiling_robot_.setPathConstraints(constraints);

    CeilingRobotMovetoTarget();
    CeilingRobotSetGripperState(false);
    CeilingRobotSendHome();
    return true;
}

bool CompetitionARIAC::FloorRobotPlacePartOnKitTrayMissing(int agv_num, int quadrant,int type)
{
    color_::ColorParts Col_part_obj;
    RCLCPP_ERROR(get_logger(), "attached" + static_cast<int>(quadrant));

    constants::Constants const_obj;

    // Move to agv
    floor_robot_.setJointValueTarget("linear_actuator_joint", const_obj.rail_positions_["agv" + std::to_string(agv_num)]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    // Determine target pose for part based on agv_tray pose
    auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");

    auto part_drop_offset = BuildPose(const_obj.quad_offsets_[quadrant].first, const_obj.quad_offsets_[quadrant].second, 0.0,
                                      geometry_msgs::msg::Quaternion());

    auto part_drop_pose = MultiplyPose(agv_tray_pose, part_drop_offset);

    std::vector<geometry_msgs::msg::Pose> waypoints;

    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + 0.3, SetRobotOrientation(0)));

    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + const_obj.part_heights_[type] + const_obj.drop_height_,
                                  SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    // Drop part in quadrant
    FloorRobotSetGripperState(false);

    std::string part_name = "red_" + Col_part_obj.PART[type];
    floor_robot_.detachObject(part_name);

    waypoints.clear();
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + 0.3,
                                  SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

    return true;
}


bool CompetitionARIAC::LockAGV(int agv_num)
{
    auto lock_agv = create_client<std_srvs::srv::Trigger>("/ariac/agv"+std::to_string(agv_num)+"_lock_tray");
    
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = lock_agv->async_send_request(request);
    result.wait();
    if (!result.get()->success)
    {
        RCLCPP_ERROR(get_logger(), "Error locking the AGV ");
        return false;
    }
    return true;
}

bool CompetitionARIAC::MoveAGV(int agv_num , int destination)
{
    LockAGV(agv_num);
    auto move_agv=create_client<ariac_msgs::srv::MoveAGV>("/ariac/move_agv" + std::to_string(agv_num));
    auto request = std::make_shared<ariac_msgs::srv::MoveAGV::Request>();
    request->location=destination;
    auto result = move_agv->async_send_request(request);
    result.wait();
    if (!result.get()->success)
    {
        RCLCPP_ERROR(get_logger(), "Error moving the AGV ");
        return false;
    }
    return true;

}

/*===================================
Assembly And Cieling Bot
=====================================*/

void CompetitionARIAC::ceiling_gripper_state_cb(
  const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg) 
{
  ceiling_gripper_state_ = *msg;
}
void CompetitionARIAC::as1_state_cb(
  const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS1, *msg);
}

void CompetitionARIAC::as2_state_cb(
  const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS2, *msg);
}

void CompetitionARIAC::as3_state_cb(
  const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS3, *msg);
}
void CompetitionARIAC::as4_state_cb(
  const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS4, *msg);
}

void CompetitionARIAC::CeilingRobotSendHome()
{
  // Move ceiling robot to home joint state
  ceiling_robot_.setNamedTarget("home");
  CeilingRobotMovetoTarget();
}

bool CompetitionARIAC::CeilingRobotSetGripperState(bool enable)
{
    ceiling_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/ceiling_robot_enable_gripper");


  if (ceiling_gripper_state_.enabled == enable) {
    if (ceiling_gripper_state_.enabled)
      RCLCPP_INFO(get_logger(), "Already enabled");
    else 
      RCLCPP_INFO(get_logger(), "Already disabled");
    
    return false;
  }

  // Call enable service
  auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
  request->enable = enable;

  auto result = ceiling_robot_gripper_enable_->async_send_request(request);
  result.wait();

  if (!result.get()->success) {
    RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
    return false;
  }

  return true;
}
void CompetitionARIAC::CeilingRobotWaitForAttach(double timeout)
{
 // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = ceiling_robot_.getCurrentPose().pose;

  while (!ceiling_gripper_state_.attached) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

    waypoints.clear();
    starting_pose.position.z -= 0.001;
    waypoints.push_back(starting_pose);

    CeilingRobotMoveCartesian(waypoints, 0.01, 0.01, false);

    usleep(200);

    if (now() - start > rclcpp::Duration::from_seconds(timeout)){
      RCLCPP_ERROR(get_logger(), "Unable to pick up object");
      return;
    }
  } 
}
void CompetitionARIAC::CeilingRobotWaitForAttachFlipped(double timeout)
{
 // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = ceiling_robot_.getCurrentPose().pose;

  while (!ceiling_gripper_state_.attached) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Waiting for gripper attach");

    waypoints.clear();
    starting_pose.position.z -= 0.001;
    waypoints.push_back(starting_pose);

    CeilingRobotMoveCartesian(waypoints, 0.005, 0.005, true);

    usleep(200);

    if (now() - start > rclcpp::Duration::from_seconds(timeout)){
      RCLCPP_ERROR(get_logger(), "Unable to pick up object");
      return;
    }
  } 
}
bool CompetitionARIAC::CeilingRobotMovetoTarget()
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(ceiling_robot_.plan(plan));

  if (success) {
    return static_cast<bool>(ceiling_robot_.execute(plan));
  } else {
    RCLCPP_ERROR(get_logger(), "Unable to generate plan");
    return false;
  }
}

bool CompetitionARIAC::CeilingRobotMoveCartesian(
  std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions)
{
  moveit_msgs::msg::RobotTrajectory trajectory;

  double path_fraction = ceiling_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, avoid_collisions);

  if (path_fraction < 0.9) {
    RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
    return false;
  }
    
  // Retime trajectory 
  robot_trajectory::RobotTrajectory rt(ceiling_robot_.getCurrentState()->getRobotModel(), "ceiling_robot");
  rt.setRobotTrajectoryMsg(*ceiling_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, vsf, asf);
  rt.getRobotTrajectoryMsg(trajectory);

  return static_cast<bool>(ceiling_robot_.execute(trajectory));
}
bool CompetitionARIAC::CeilingRobotPickAGVPart(ariac_msgs::msg::PartPose part)
{
  constants::Constants const_obj;
  color_::ColorParts color;
  double part_rotation = GetYaw(part.pose);

  std::vector<geometry_msgs::msg::Pose> waypoints;

  double dx = 0;
  double dy = 0;

  if (part.part.type == ariac_msgs::msg::Part::BATTERY) {
    dx = const_obj.battery_grip_offset_*cos(part_rotation);
    dy = const_obj.battery_grip_offset_*sin(part_rotation);
  }

  waypoints.push_back(BuildPose(part.pose.position.x + dx, part.pose.position.y + dy, 
    part.pose.position.z + 0.4, SetRobotOrientation(part_rotation)));
  
  waypoints.push_back(BuildPose(part.pose.position.x + dx, part.pose.position.y + dy, 
    part.pose.position.z + const_obj.part_heights_[part.part.type] + const_obj.pick_offset_, SetRobotOrientation(part_rotation)));


  CeilingRobotMoveCartesian(waypoints, 0.7, 0.7, true);

  CeilingRobotSetGripperState(true);

  CeilingRobotWaitForAttach(5.0);

  // Add part to planning scene
  std::string part_name = color.COLOR[part.part.color] + "_" + color.PART[part.part.type];
  RCLCPP_INFO_STREAM(rclcpp::get_logger("CeilingRobotPickAGVPart"), "Picking up " << part_name);

  AddModelToPlanningScene(part_name, color.PART[part.part.type] + ".stl", part.pose);
  ceiling_robot_.attachObject(part_name);
  ceiling_robot_attached_part_ = part.part;

  // Move up slightly
  auto current_pose = ceiling_robot_.getCurrentPose().pose;
  current_pose.position.z += 0.2;
  
  waypoints.clear();
  waypoints.push_back(current_pose);

  CeilingRobotMoveCartesian(waypoints, 0.7, 0.7, true);

  return true;

}
bool CompetitionARIAC::CeilingRobotMoveToAssemblyStation(int station)
{
  j_s_maps::FloorRobotJS js_obj;
  switch (station) {
    case 1:
      ceiling_robot_.setJointValueTarget(js_obj.ceiling_as1_js_);
      break;
    case 2:
      ceiling_robot_.setJointValueTarget(js_obj.ceiling_as2_js_);
      break;
    case 3:
      ceiling_robot_.setJointValueTarget(js_obj.ceiling_as3_js_);
      break;
    case 4:
      ceiling_robot_.setJointValueTarget(js_obj.ceiling_as4_js_);
      break;
    default:
      RCLCPP_WARN(get_logger(), "Not a valid assembly station");
      return false;
  }

  return CeilingRobotMovetoTarget();
}

bool CompetitionARIAC::CeilingRobotAssemblePart(int station, ariac_msgs::msg::AssemblyPart part)
{
    constants::Constants const_obj;
    color_::ColorParts color;

    RCLCPP_INFO_STREAM(rclcpp::get_logger("CeilingRobotPickAGVPart"), "Picking up " << std::to_string(part.part.type));

  // Check that part is attached and matches part to assemble
  if (!ceiling_gripper_state_.attached) {
    RCLCPP_WARN(get_logger(), "No part attached");
    return false;
  }
      
  if (part.part != ceiling_robot_attached_part_){
    RCLCPP_WARN(get_logger(), "Incorrect part attached for this assembly");
    return false;
  }
  
  // Calculate assembled pose in world frame
  std::string insert_frame_name;
  switch (station) {
    case 1:
      insert_frame_name = "as1_insert_frame";
      break;
    case 2:
      insert_frame_name = "as2_insert_frame";
      break;
    case 3:
      insert_frame_name = "as3_insert_frame";
      break;
    case 4:
      insert_frame_name = "as4_insert_frame";
      break;
    default:
      RCLCPP_WARN(get_logger(), "Not a valid assembly station");
      return false;
  }

  // Calculate robot positions at assembly and approach
  KDL::Vector install(part.install_direction.x, part.install_direction.y, part.install_direction.z);

  KDL::Frame insert;
  tf2::fromMsg(FrameWorldPose(insert_frame_name), insert);

  KDL::Frame part_assemble;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("CeilingRobotPickAGVPart"), "Picking up " << std::to_string(part.assembled_pose.pose.position.x));

  tf2::fromMsg(part.assembled_pose.pose, part_assemble);

  KDL::Frame part_to_gripper;

  // Build approach waypoints
  std::vector<geometry_msgs::msg::Pose> waypoints;

  if (part.part.type == ariac_msgs::msg::Part::BATTERY) {
    tf2::fromMsg(BuildPose(const_obj.battery_grip_offset_, 0, const_obj.part_heights_[part.part.type], QuaternionFromRPY(M_PI, 0, M_PI)), part_to_gripper);

    KDL::Vector up(0, 0, 0.1);
    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(up) * KDL::Frame(install * -0.06) * part_assemble * part_to_gripper));
    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.06) * part_assemble * part_to_gripper));    

  } else if (part.part.type == ariac_msgs::msg::Part::PUMP) {
    tf2::fromMsg(BuildPose(0, 0, const_obj.part_heights_[part.part.type], QuaternionFromRPY(M_PI, 0, M_PI)), part_to_gripper);

    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.09) * part_assemble * part_to_gripper));
  }
  else{
    tf2::fromMsg(BuildPose(0, 0, const_obj.part_heights_[part.part.type], QuaternionFromRPY(M_PI, 0, M_PI)), part_to_gripper);

    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.1) * part_assemble * part_to_gripper));
  }
  
  // Move to approach position
  CeilingRobotMoveCartesian(waypoints, 0.3, 0.3, true);

  // Move to just before assembly position
  waypoints.clear();
  waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.003) * part_assemble * part_to_gripper));
  CeilingRobotMoveCartesian(waypoints, 0.1, 0.1, true);

  CeilingRobotWaitForAssemble(station, part);

  CeilingRobotSetGripperState(false);

  std::string part_name = color.COLOR[ceiling_robot_attached_part_.color] + 
    "_" + color.PART[ceiling_robot_attached_part_.type];
  ceiling_robot_.detachObject(part_name);

  // Move away slightly
  auto current_pose = ceiling_robot_.getCurrentPose().pose;

  if (part.part.type == ariac_msgs::msg::Part::REGULATOR) {
    current_pose.position.x -= 0.05;
  }
  else {
    current_pose.position.z += 0.1;
  }
  
  waypoints.clear();
  waypoints.push_back(current_pose);

  CeilingRobotMoveCartesian(waypoints, 0.3, 0.3, true);
  
  return true;

}
bool CompetitionARIAC::CeilingRobotWaitForAssemble(int station, ariac_msgs::msg::AssemblyPart part)
{
  // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = ceiling_robot_.getCurrentPose().pose;

  bool assembled = false;
  while (!assembled) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for part to be assembled");

    // Check if part is assembled
    switch (part.part.type) {
      case ariac_msgs::msg::Part::BATTERY:
        assembled = assembly_station_states_[station].battery_attached;
        break;
      case ariac_msgs::msg::Part::PUMP:
        assembled = assembly_station_states_[station].pump_attached;
        break;
      case ariac_msgs::msg::Part::SENSOR:
        assembled = assembly_station_states_[station].sensor_attached;
        break;
      case ariac_msgs::msg::Part::REGULATOR:
        assembled = assembly_station_states_[station].regulator_attached;
        break;
      default:
        RCLCPP_WARN(get_logger(), "Not a valid part type");
        return false;
    }

    double step = 0.0005;

    waypoints.clear();
    starting_pose.position.x += step * part.install_direction.x;
    starting_pose.position.y += step * part.install_direction.y;
    starting_pose.position.z += step * part.install_direction.z;
    waypoints.push_back(starting_pose);

    CeilingRobotMoveCartesian(waypoints, 0.01, 0.01, false);

    usleep(500);

    if (now() - start > rclcpp::Duration::from_seconds(5)){
      RCLCPP_ERROR(get_logger(), "Unable to assemble object");
      ceiling_robot_.stop();
      return false;
    }
  }

  RCLCPP_INFO(get_logger(), "Part is assembled");
  
  return true;
}

