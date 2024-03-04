/**
 * @file rwa3.cpp
 * @author Kiran S Patil,Aniruddh Balram,Vyshnv Achuthan,Badrinarayanan
 * @brief Program that create subscribers and subscribes to orders topic and store the contents of the message in a data structure.
 * The program also checks the parts available in bins and conveyor belt and storees them in a data structure. Finally, the program
 * prompts if the order can be succefully completed or not, depending on the available parts.
 * @version 0.1
 * @date 2023-03-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "../include/ARIAC-2023/rwa3.hpp"
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
    else if (msg->competition_state == ariac_msgs::msg::CompetitionState::STARTED && count_2 == 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("CompState Subscriber"), "Competition state is 2, Competition Started...\n");
        count_2++;
    }
    else if (msg->competition_state == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE && submitted == false && bin_flag && conv_flag && tray_flag)
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
                            FloorRobotPickBinPart(order_parts[i]);
                            FloorRobotPlacePartOnKitTray(kits.agv_number, order_parts[i].quadrant); 
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


        // RCLCPP_INFO_STREAM(rclcpp::get_logger("AGV Destination task"),"\t Lock AGV " + std::to_string(kits.agv_number));
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("AGV Destination task"),"\t Move AGV to " + Col_part_obj.DEST[kits.destination]+"\n");
        // MoveAGV(kits.agv_number,kits.destination);
        if(order_index < int(orders_list.size())) {
            order_index += 1;
        }

        RCLCPP_INFO_STREAM(rclcpp::get_logger("AGV Destination task"),"\t Lock AGV " + std::to_string(kits.agv_number));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("AGV Destination task"),"\t Move AGV to " + Col_part_obj.DEST[kits.destination]+"\n");
        MoveAGV(kits.agv_number,kits.destination);

    }
    else if(order.type == ariac_msgs::msg::Order::ASSEMBLY) 
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Assembly task"),"\t Assembly " );
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
    // if(tray_flag)
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
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Break Beam subscriber"), "part detected , count = " + std::to_string(conv_part_detected));
            // std::this_thread::sleep_for(std::chrono::milliseconds(500));
            if(slot<=9)
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor Robot conv task"), "\t Pick "+conv_part_idetified_color +" "+conv_part_idetified_type+" from conveyor lane "+std::to_string(conv_lane_offset)+" and place in Bin " + std::to_string(empty_bin) +" - slot " + std::to_string(slot));
            else
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor Robot conv task"), "empty slots exhausted");

            // FloorRobotSendHome();
            FloorRobotPickConveyorPart(conv_part_idetified_color, conv_part_idetified_type,conv_lane_offset);
            
            FloorRobotPlacePartEmptyBin(conv_part_idetified_type, empty_bin, slot);

            slot+=2;
            // rclcpp::sleep_for(std::chrono::seconds(2)); 
            
        }
        if(conv_part_detected == conv_part_quantity)
        {
            slot = 2;
            beam_flag = true;
            FloorRobot();
            tray_flag = true;
            break_beam_sub.reset();
        }
    }
          
}

void CompetitionARIAC::BreakBeamCallback2(const ariac_msgs::msg::BreakBeamStatus::SharedPtr break_beam_msg)
{
    if(conv_flag)
    {
        if(break_beam_msg->object_detected)
        {                                                                
            conv_part_detected++;            
        }
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
    
    if(beam_flag)
    {
        left_bins_parts_ =  msg->part_poses;

        for(auto bin_part:left_bins_parts_)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub LBin "), "Parts on left bins :" + std::to_string(bin_part.part.color)+" "+std::to_string(bin_part.part.type));

            lbin_pose = bin_part.pose;
            
            RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub LBin "), "pose :" );
            RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub LBin "), "    position x :" + std::to_string(lbin_pose.position.x));
            RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub LBin "), "    position y :" + std::to_string(lbin_pose.position.y));
            RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub LBin "), "    position z :" + std::to_string(lbin_pose.position.z));

        }
        
        left_bins_camera_pose_ = msg->sensor_pose;
        left_bins_camera_sub_.reset();
    }

}

void CompetitionARIAC::RightBinsCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    
    if(beam_flag)
    {
        right_bins_parts_ =  msg->part_poses;

        for(auto bin_part:right_bins_parts_)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub rBin "), "Parts on right bins :" + std::to_string(bin_part.part.color)+" "+std::to_string(bin_part.part.type));

            rbin_pose = bin_part.pose;
            
            RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub RBin "), "pose :" );
            RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub RBin "), "    position x :" + std::to_string(rbin_pose.position.x));
            RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub RBin "), "    position y :" + std::to_string(rbin_pose.position.y));
            RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub RBin "), "    position z :" + std::to_string(rbin_pose.position.z));

        }
        
        right_bins_camera_pose_ = msg->sensor_pose;
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
            
            RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub conveyor "), "pose : " + conv_part_idetified_color+" "+conv_part_idetified_type);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("adv camera sub conveyor "), "    position z :" + std::to_string(conv_part_pose.position.z));

            conv_lane_offset = conv_part_pose.position.z;
        }         
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
    
    FloorRobotPickandPlaceTray(kits.tray_id, kits.agv_number);
    tray_flag = true;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Floor    Robot task "),"\t tray pick and place done\n");

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
    // FloorRobotMovetoTarget();
    // rclcpp::sleep_for(std::chrono::milliseconds(200));

    

    // if (part_type=="regulator" || part_type=="sensor")
    // {
    //     FloorRobotWaitForDrop(1);
    //     FloorRobotSetGripperState(false);
    // }
    // else if (part_type=="battery")
    // {
    //     FloorRobotWaitForDrop(2);
    //     FloorRobotSetGripperState(false);

    // }
    // else if(part_type=="pump" && (bin_num == 5 || bin_num == 1))
    // {
    //     FloorRobotWaitForDrop(1);
    //     FloorRobotSetGripperState(false);
    //     // FloorRobotSetGripperState(false);
    // }
    // rclcpp::sleep_for(std::chrono::milliseconds(200));
    FloorRobotSetGripperState(false);
    // FloorRobotMoveUp();
    // FloorRobotSendHome();
    
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
    for (auto tray : kts1_trays_)
    {
        if (tray.id == tray_id)
        {
            station = "kts1";
            tray_pose = MultiplyPose(kts1_camera_pose_, tray.pose);
            found_tray = true;
            break;
        }
    }
    // Check table 2
    if (!found_tray)
    {   
        for (auto tray : kts2_trays_)
        {   
            if (tray.id == tray_id)
            {
                station = "kts2";
                tray_pose = MultiplyPose(kts2_camera_pose_, tray.pose);
                found_tray = true;
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


bool CompetitionARIAC::FloorRobotPickBinPart(order_::KittingPart part_to_pick)
{
    color_::ColorParts Col_part_obj;

    RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick a " << Col_part_obj.COLOR[part_to_pick.color] << " " << Col_part_obj.PART[part_to_pick.type]);

    // Check if part is in one of the bins
    geometry_msgs::msg::Pose part_pose;
    bool found_part = false;
    std::string bin_side;

    constants::Constants const_obj;
    j_s_maps::FloorRobotJS js_obj;

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
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + 0.5, SetRobotOrientation(part_rotation)));

    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + const_obj.part_heights_[part_to_pick.type] + const_obj.pick_offset_, SetRobotOrientation(part_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    FloorRobotSetGripperState(true);
    FloorRobotWaitForAttach(5.0);

    // Add part to planning scene
    std::string part_name = Col_part_obj.COLOR[part_to_pick.color] + "_" + Col_part_obj.PART[part_to_pick.type];
    AddModelToPlanningScene(part_name, Col_part_obj.PART[part_to_pick.type] + ".stl", part_pose);
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
                                  part_drop_pose.position.z + const_obj.part_heights_[floor_robot_attached_part_.type] + const_obj.drop_height_,
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

