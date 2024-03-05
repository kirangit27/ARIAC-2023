/**
 * @file rwa4.hpp
 * @author Kiran S Patil, Aniruddh Balram, Vyshnv Achuthan, Badrinarayanan
 * @brief Header file that defines data structures to store the contents of the orders received, and also to store details about the parts availble in bins and conveyor belts to perform each task
 * @version 0.1
 * @date 2023-04-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
//Added necessary header files
#include <chrono>
#include <memory>
#include <thread>
#include <map>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ariac_msgs/msg/competition_state.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <string>

#include "ariac_msgs/msg/order.hpp"
#include "ariac_msgs/msg/assembly_task.hpp"
#include "ariac_msgs/msg/kitting_task.hpp"
#include "ariac_msgs/msg/kitting_part.hpp"
#include "ariac_msgs/msg/bin_parts.hpp"
#include "ariac_msgs/msg/conveyor_parts.hpp"
#include "ariac_msgs/msg/bin_info.hpp"
#include "ariac_msgs/msg/part_lot.hpp"
#include "ariac_msgs/msg/part.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "ariac_msgs/msg/break_beam_status.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.h>

#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <ariac_msgs/msg/kit_tray_pose.hpp>
#include <ariac_msgs/msg/vacuum_gripper_state.hpp>
#include <ariac_msgs/msg/part_pose.hpp>
#include <ariac_msgs/msg/assembly_state.hpp>
#include <ariac_msgs/msg/agv_status.hpp>

#include "ariac_msgs/srv/submit_order.hpp"
#include <ariac_msgs/srv/change_gripper.hpp>
#include <ariac_msgs/srv/vacuum_gripper_control.hpp>
#include <ariac_msgs/srv/get_pre_assembly_poses.hpp>
#include <ariac_msgs/srv/perform_quality_check.hpp>
#include <ariac_msgs/srv/move_agv.hpp>

#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <unistd.h>
#include <cmath>
#include <ament_index_cpp/get_package_share_directory.hpp>


/**
 * @brief A namespace defined to identify the data structure used to store order details
 * 
 */
namespace order_
{   
    /**
     * @brief Structure that stores the Quadrant, color ,type of a Kitting part. Part status stores a bool value according to availability of the specific part
     * 
     */
    struct KittingPart{
    uint8_t quadrant;
    uint8_t color;
    uint8_t type;
    bool part_status{0};
    };

    /**
     * @brief Structure that stores information about the Kitting Task itself. It stores the AGV number, tray_id and destination and different kitting parts.
     * 
     */
    struct KittingType{
        uint8_t quadrant;
        uint8_t agv_number;
        int tray_id;
        uint8_t destination;
        std::vector<KittingPart> parts;
    };

    /**
     * @brief Structure that stores the color,type of parts, the pose and direction of the parts uused for both Assembly and Combined task
     * 
     */
    struct AssemCombPart{
        uint8_t color;
        uint8_t type;
        geometry_msgs::msg::Pose pose;
        geometry_msgs::msg::PoseStamped pose_stamp;
        geometry_msgs::msg::Vector3 install_direction;
    };

    /**
     * @brief Structure that stores information about Assembly and the Combined task. It stores the station number, the different AGV numbers and parts necessary to complete the task
     * 
     */
    struct AssemCombType{
        uint8_t station;
        std::vector<uint8_t> agv_numbers;
        std::vector<AssemCombPart> parts;
    };

    /**
     * @brief Class that defines members that define the order itself. Id stores the order id, type stores the type of task, priority is a boolean
     * variable that stores either 0 or 1 depending on the order priority. Additionally, it consists of a map, which maps the integer received to the type of task
     * 
     */
    class Orders{
    public:
        std::string id;
        uint8_t type;
        bool priority;
        KittingType kitting_type;
        bool order_status{0};
        AssemCombType AssemComb_type;

        std::map<int,std::string> order_type={
            {0,"KITTING"},
            {1,"ASSEMBLY"},
            {2,"COMBINED"}};
    };

    
}

/**
 * @brief Namespace defined to identify specific data structures that define part colors and types
 * 
 */
namespace color_
{   
    /**
     * @brief Class defined to store information about part colors and types. It uses a map to store that information
     * 
     */
    class ColorParts{
        
        public:

        std::map<int, std::string> COLOR = {
            {ariac_msgs::msg::Part::RED, "red"},
            {ariac_msgs::msg::Part::GREEN, "green"},
            {ariac_msgs::msg::Part::BLUE, "blue"},
            {ariac_msgs::msg::Part::ORANGE, "orange"},
            {ariac_msgs::msg::Part::PURPLE, "purple"}};

        std::map<int,std::string> PART={
            {ariac_msgs::msg::Part::BATTERY,"battery"},
            {ariac_msgs::msg::Part::PUMP,"pump"},
            {ariac_msgs::msg::Part::SENSOR,"sensor"},
            {ariac_msgs::msg::Part::REGULATOR,"regulator"}};

        std::map<int,std::string> DEST={
            {ariac_msgs::msg::KittingTask::KITTING, "KITTING"},
            {ariac_msgs::msg::KittingTask::ASSEMBLY_FRONT, "ASSEMBLY_FRONT"},
            {ariac_msgs::msg::KittingTask::ASSEMBLY_BACK, "ASSEMBLY_BACK"},
            {ariac_msgs::msg::KittingTask::WAREHOUSE, "WAREHOUSE"}};
            
    };
    
}

/**
 * @brief Namespace defined to identify data structures that define the parts itself
 * 
 */
namespace pick_part 
{   
    /**
     * @brief A sttructure that stores the part color and type
     * 
     */
    struct Part{
        uint8_t color;
        uint8_t type;
        geometry_msgs::msg::Pose pose;
    };

    /**
     * @brief Structure to store information about the parts. It stores data like quantity,color,type and bin number in which part needs to go into.
     * Additionally, it also stores whether the part has to be picked up by ceiling robot or the floor robot in a string.
     * 
     */
    struct PartInfo{
        uint8_t quantity;
        std::string pickup_type;
        uint8_t color;
        uint8_t type;
        uint8_t bin_number;
        geometry_msgs::msg::Pose pose;
    };

    /**
     * @brief Class that uses the structures that have been defined above to store all data about parts.
     * 
     */
    class Parts {
        public:
        std::vector<PartInfo> parts;
    };
}


/**
 * @brief Namespace to store constants such as dimensions and poses of objects in the environment
 * 
 */
namespace constants
{   /**
    * @brief Class that stores constants
    * 
    */
    class Constants{

        public:
            /**
             * @brief Kit tray thickness
             * 
             */
            double kit_tray_thickness_ = 0.01;
            /**
             * @brief Height from which part will be dropped at minimum
             * 
             */            
            double drop_height_ = 0.002;
            /**
             * @brief Final offset from the part to pick it up
             * 
             */
            double pick_offset_ = 0.007;
            double pick_offset_2 = 0.008;
            /**
             * @brief Battery grip offset
             * 
             */
            double battery_grip_offset_ = -0.05;

            // Part heights
            std::map<int, double> part_heights_ = {
                {ariac_msgs::msg::Part::BATTERY, 0.04},
                {ariac_msgs::msg::Part::PUMP, 0.12},
                {ariac_msgs::msg::Part::REGULATOR, 0.07},
                {ariac_msgs::msg::Part::SENSOR, 0.07}};

            // Part heights
            std::map<int, double> part_heights_kit_pick = {
                {ariac_msgs::msg::Part::BATTERY, 0.0},
                {ariac_msgs::msg::Part::PUMP, 0.08},
                {ariac_msgs::msg::Part::REGULATOR, 0.02},
                {ariac_msgs::msg::Part::SENSOR, 0.02}};

            // Quadrant Offsets
            std::map<int, std::pair<double, double>> quad_offsets_ = {
                {ariac_msgs::msg::KittingPart::QUADRANT1, std::pair<double, double>(-0.08, 0.12)},
                {ariac_msgs::msg::KittingPart::QUADRANT2, std::pair<double, double>(0.08, 0.12)},
                {ariac_msgs::msg::KittingPart::QUADRANT3, std::pair<double, double>(-0.08, -0.12)},
                {ariac_msgs::msg::KittingPart::QUADRANT4, std::pair<double, double>(0.08, -0.12)}};

            std::map<std::string, double> rail_positions_ = {
                {"agv1", -4.5},
                {"agv2", -1.2},
                {"agv3", 1.2},
                {"agv4", 4.5},
                {"left_bins", 3},
                {"right_bins", -3}};

            std::map<int, std::pair<int,int>> quad_parts_map = {
                {1, std::pair<int,int>(ariac_msgs::msg::Part::RED,ariac_msgs::msg::Part::BATTERY)},
                {2, std::pair<int,int>(ariac_msgs::msg::Part::RED,ariac_msgs::msg::Part::PUMP)},
                {3, std::pair<int,int>(ariac_msgs::msg::Part::RED,ariac_msgs::msg::Part::REGULATOR)},
                {4, std::pair<int,int>(ariac_msgs::msg::Part::RED,ariac_msgs::msg::Part::SENSOR)}};
    
    };
}

/**
 * @brief Construct a new Competition State Subscriber object. Create subscriber to subscribe to competition state and orders topic.
 * Subscribers to get the parts available in bins and conveyor belt have also been created. Also a client is created to send message when an order is submitted.
 * 
 */
class CompetitionARIAC : public rclcpp::Node
{
    public:
        /**
         * @brief Construct a new Competition A R I A C object
         * 
         */
        CompetitionARIAC() : Node("competition_subscriber"),floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "floor_robot"),
        ceiling_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "ceiling_robot"),planning_scene_()
        {
            
            floor_robot_.setMaxAccelerationScalingFactor(1.0);
            floor_robot_.setMaxVelocityScalingFactor(1.0);
            ceiling_robot_.setMaxAccelerationScalingFactor(1.0);
            ceiling_robot_.setMaxVelocityScalingFactor(1.0);
           
            m_callback_group_1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            m_callback_group_2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            m_callback_group_3 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            m_callback_group_4 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            m_callback_group_5 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            // sensor callback
            cb_group_bin_cameras_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            cb_group_kit_tray_cameras_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            cb_group_assembly_state_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            

            //subscription callback groups
            auto subscription_option1 = rclcpp::SubscriptionOptions();
            subscription_option1.callback_group = m_callback_group_1;
            auto subscription_option2 = rclcpp::SubscriptionOptions();
            subscription_option2.callback_group = m_callback_group_2;
            auto subscription_option3 = rclcpp::SubscriptionOptions();
            subscription_option3.callback_group = m_callback_group_3;
            auto subscription_option4 = rclcpp::SubscriptionOptions();
            subscription_option4.callback_group = m_callback_group_4;
            auto subscription_option5 = rclcpp::SubscriptionOptions();
            subscription_option5.callback_group = m_callback_group_5;
            auto subscription_option6 = rclcpp::SubscriptionOptions();
            subscription_option6.callback_group = m_callback_group_6;
            auto subscription_option7 = rclcpp::SubscriptionOptions();
            subscription_option7.callback_group = m_callback_group_7;
            // sensor callback
            auto bin_cameras_option = rclcpp::SubscriptionOptions();
            bin_cameras_option.callback_group = cb_group_bin_cameras_;
            auto kit_tray_cameras_option = rclcpp::SubscriptionOptions();
            kit_tray_cameras_option.callback_group = cb_group_kit_tray_cameras_;
            auto assembly_state_option = rclcpp::SubscriptionOptions();
            assembly_state_option.callback_group = cb_group_assembly_state_;
            // Subscriber objects            
            comp_state_sub = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 10, 
                                                                        std::bind(&CompetitionARIAC::CompetitionStateCallback, this, std::placeholders::_1),subscription_option1);

            order_sub = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders", 10, 
                                                                        std::bind(&CompetitionARIAC::OrderCallback, this, std::placeholders::_1),subscription_option2);

            bin_part_sub = this->create_subscription<ariac_msgs::msg::BinParts>("/ariac/bin_parts", 10, 
                                                                        std::bind(&CompetitionARIAC::BinPartCallback, this, std::placeholders::_1),subscription_option3); 

            conv_part_sub = this->create_subscription<ariac_msgs::msg::ConveyorParts>("/ariac/conveyor_parts", 10, 
                                                                        std::bind(&CompetitionARIAC::ConvPartCallback, this, std::placeholders::_1),subscription_option4); 

            break_beam_sub = this->create_subscription<ariac_msgs::msg::BreakBeamStatus>("/ariac/sensors/breakbeam_0/status",  rclcpp::SensorDataQoS(), 
                                                                        std::bind(&CompetitionARIAC::BreakBeamCallback, this, std::placeholders::_1),subscription_option5); 

            break_beam_sub2 = this->create_subscription<ariac_msgs::msg::BreakBeamStatus>("/ariac/sensors/breakbeam_1/status",  rclcpp::SensorDataQoS(), 
                                                                        std::bind(&CompetitionARIAC::BreakBeamCallback2, this, std::placeholders::_1),subscription_option5);                                                           

            kit_tray_table1_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>("/ariac/sensors/kts1_camera/image", rclcpp::SensorDataQoS(),
                                                                    std::bind(&CompetitionARIAC::KitTrayTable1Callback, this, std::placeholders::_1), kit_tray_cameras_option);

            kit_tray_table2_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>("/ariac/sensors/kts2_camera/image", rclcpp::SensorDataQoS(),
                                                                    std::bind(&CompetitionARIAC::KitTrayTable2Callback, this, std::placeholders::_1), kit_tray_cameras_option);

            left_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>("/ariac/sensors/left_bins_camera/image", rclcpp::SensorDataQoS(),
                                                                    std::bind(&CompetitionARIAC::LeftBinsCameraCallback, this, std::placeholders::_1), bin_cameras_option);

            right_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>("/ariac/sensors/right_bins_camera/image", rclcpp::SensorDataQoS(),
                                                                    std::bind(&CompetitionARIAC::RightBinsCameraCallback, this, std::placeholders::_1), bin_cameras_option);                                                       

            conv_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>("/ariac/sensors/conveyor_camera/image", rclcpp::SensorDataQoS(),
                                                                    std::bind(&CompetitionARIAC::ConveyorCameraCallback, this, std::placeholders::_1), bin_cameras_option);                                                                                                      

            floor_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>("/ariac/floor_robot_gripper_state", rclcpp::SensorDataQoS(),
                                                                    std::bind(&CompetitionARIAC::floor_gripper_state_cb, this, std::placeholders::_1), subscription_option6);

            ceiling_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>("/ariac/ceiling_robot_gripper_state", rclcpp::SensorDataQoS(),
                                                                    std::bind(&CompetitionARIAC::ceiling_gripper_state_cb, this, std::placeholders::_1), subscription_option7); 

            as1_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>("/ariac/assembly_insert_1_assembly_state", rclcpp::SensorDataQoS(), 
                                                                    std::bind(&CompetitionARIAC::as1_state_cb, this, std::placeholders::_1), assembly_state_option);

            as2_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>("/ariac/assembly_insert_2_assembly_state", rclcpp::SensorDataQoS(), 
                                                                    std::bind(&CompetitionARIAC::as2_state_cb, this, std::placeholders::_1), assembly_state_option);

            as3_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>("/ariac/assembly_insert_3_assembly_state", rclcpp::SensorDataQoS(), 
                                                                    std::bind(&CompetitionARIAC::as3_state_cb, this, std::placeholders::_1), assembly_state_option);

            as4_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>("/ariac/assembly_insert_4_assembly_state", rclcpp::SensorDataQoS(), 
                                                                    std::bind(&CompetitionARIAC::as4_state_cb, this, std::placeholders::_1), assembly_state_option);                                                       
            
            submit_order_client_ = create_client<ariac_msgs::srv::SubmitOrder>("/ariac/submit_order");

            quality_checker_ = this->create_client<ariac_msgs::srv::PerformQualityCheck>("/ariac/perform_quality_check");

            AddModelsToPlanningScene();

            RCLCPP_INFO(this->get_logger(), "Initialization successful.");
        }

        /**
         * @brief Destroy the Competition A R I A C object
         * 
         */
        ~CompetitionARIAC()
        {
            floor_robot_.~MoveGroupInterface();
            ceiling_robot_.~MoveGroupInterface();
        }
        
        // variable to check if order is retrieved
        bool order_retrived{false};
        // variable to check if bin callback is completed        
        bool bin_flag{false};
        // variable to check if conveyor callback is completed
        bool conv_flag{false};
        // variable to check if all the parts in conveyor have passed the sensor
        bool beam_flag{false};
        // variable is set true after part passes beam and gets picked
        bool pick_beam_flag{false};
        // variable to check if tray filled
        bool tray_flag{false};
        // variable is set true after al necessary parts picked from conveyor
        bool conv_pick_flag{false};
        // variable that is set true when quality check is completed
        bool quality_check_flag{false};
        // bin slots
        std::vector<int> bin_space{1,2,3,4,5,6,7,8};
        // priority bins
        std::vector<int> p_bins{6,5,2,1,7,8,3,4};
        // empty bins
        std::vector<int> empty_bins;
        // flag empty bins
        int empty_bin{0};
        int order_counter{0};
        // order bin vector
        std::vector<int> order_bins;
        // boolean vector to check quality
        std::vector<bool> quality_check_vec{};
        int slot{2};
        int conv_part_quantity{0};
        int conv_part_detected{0};
        std::string conv_part_idetified_type{""};
        std::string conv_part_idetified_color{""};
        double conv_lane_offset{0};


        // Floor Robot Public Functions
        void FloorRobotSendHome();
        bool FloorRobotSetGripperState(bool enable);
        bool FloorRobotChangeGripper(std::string station, std::string gripper_type);
        bool FloorRobotPickConveyorPart(std::string part_color, std::string part_type, double conv_lane);
        bool FloorRobotPlacePartEmptyBin(std::string part_type, int bin_num, int slot);
        bool FloorRobotPickandPlaceTray(int tray_id, int agv_num);
        bool FloorRobotPickBinPart(order_::KittingPart part_to_pick);
        bool FloorRobotPickBinPartCombined(ariac_msgs::msg::Part part_to_pick);
        bool FloorRobotPickBinPartReplaceMissing(int part_color, int part_type);
        bool FloorRobotPlacePartOnKitTray(int agv_num, int quadrant);
        bool FloorRobotDisposeParts(int agv_num, int quadrant);
        bool FloorRobotPickFlippedPart(int agv_num, int quadrant);
        bool FloorRobotPlacePartOnKitTrayMissing(int agv_num, int quadrant, int type);

        // Ceiling Robot Public Functions
        void CeilingRobotSendHome();
        bool CeilingRobotSetGripperState(bool enable);
        bool CeilingRobotPickAGVPart(ariac_msgs::msg::PartPose part);

        bool MoveAGV(int agv_num , int destination);
        bool LockAGV(int agv_num);


    private:
        // order submitted
        bool submitted{false};
        // list size
        int list_size{0};
        int count_2{0};
        int order_index{0};
        bool priority_order = false;
        int priority_index = -1;

        /**
         * @brief Vector to store list of orders
         * 
         */
        std::vector<order_::Orders> orders_list;
        std::vector<order_::Orders> orders_list_submit;
        pick_part::Parts partp;

        /**
         * @brief Function that checks the avaiilable parts in bins and conveyor and prompts the user if the task can be completed succesfully or not.
         * 
         */
        void OrderAccomplishment();


        // Callback Groups
        rclcpp::CallbackGroup::SharedPtr m_callback_group_1;
        rclcpp::CallbackGroup::SharedPtr m_callback_group_2;
        rclcpp::CallbackGroup::SharedPtr m_callback_group_3;
        rclcpp::CallbackGroup::SharedPtr m_callback_group_4;
        rclcpp::CallbackGroup::SharedPtr m_callback_group_5;
        rclcpp::CallbackGroup::SharedPtr m_callback_group_6;
        rclcpp::CallbackGroup::SharedPtr m_callback_group_7;
        rclcpp::CallbackGroup::SharedPtr cb_group_bin_cameras_;
        rclcpp::CallbackGroup::SharedPtr cb_group_kit_tray_cameras_;
        rclcpp::CallbackGroup::SharedPtr cb_group_assembly_state_;


        /*==============
        Subscribers
        ==============*/
        rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr comp_state_sub;
        rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_sub;
        rclcpp::Subscription<ariac_msgs::msg::BinParts>::SharedPtr bin_part_sub;
        rclcpp::Subscription<ariac_msgs::msg::ConveyorParts>::SharedPtr conv_part_sub;

        /*!< Subscriber to camera image over kit tray table1. */
        rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kit_tray_table1_camera_sub_;
        /*!< Subscriber to camera image over kit tray table2. */
        rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kit_tray_table2_camera_sub_;
        /*!< Subscriber to camera image over left bins. */
        rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr left_bins_camera_sub_;
        /*!< Subscriber to camera image over right bins. */
        rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr right_bins_camera_sub_;
        /*!< Subscriber to camera image over conveyor */
        rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr conv_camera_sub_;
        /*!< Subscriber to break beam sensors */
        rclcpp::Subscription<ariac_msgs::msg::BreakBeamStatus>::SharedPtr break_beam_sub;
        /*!< Subscriber to break beam sensors */
        rclcpp::Subscription<ariac_msgs::msg::BreakBeamStatus>::SharedPtr break_beam_sub2;
        /*!< Subscriber to floor gripper state */
        rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr floor_gripper_state_sub_;
        /*!< Subscriber to ceiling gripper state */
        rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr ceiling_gripper_state_sub_;
        /*!< Subscriber to Aseembly state */
        rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as1_state_sub_;
        rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as2_state_sub_;
        rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as3_state_sub_;
        rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as4_state_sub_;



        // Floor Robot Move Functions
        bool FloorRobotMovetoTarget();
        bool FloorRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf);
        void FloorRobotWaitForAttach(double timeout);
        void FloorRobotWaitForAttachPump(double timeout);
        void FloorRobotWaitForAttachKitTrayPart(double timeout);
        void FloorRobotWaitForDrop(double timeout);
        void FloorRobotMoveUp();

        //Ceiling Robot Move Functions
        bool CeilingRobotMovetoTarget();
        bool CeilingRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions);
        void CeilingRobotWaitForAttach(double timeout);
        void CeilingRobotWaitForAttachFlipped(double timeout);
        bool CeilingRobotPlaceFlippedPartOnKitTray(int agv_num, int quadrant);
        bool CeilingRobotWaitForAssemble(int station, ariac_msgs::msg::AssemblyPart part);
        bool CeilingRobotMoveToAssemblyStation(int station);
        bool CeilingRobotAssemblePart(int station, ariac_msgs::msg::AssemblyPart part);

        // Set robot orientation
        geometry_msgs::msg::Quaternion SetRobotOrientation(double rotation);
        // Pose robot
        void LogPose(geometry_msgs::msg::Pose p);
        geometry_msgs::msg::Pose MultiplyPose(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2);
        geometry_msgs::msg::Pose BuildPose(double x, double y, double z, geometry_msgs::msg::Quaternion orientation);
        geometry_msgs::msg::Pose FrameWorldPose(std::string frame_id);
        double GetYaw(geometry_msgs::msg::Pose pose);
        double GetPitch(geometry_msgs::msg::Pose pose);
        geometry_msgs::msg::Quaternion QuaternionFromRPY(double r, double p, double y);
        /**
         * @brief Moveit function to add the planning to the scene
         * 
         * @param name 
         * @param mesh_file 
         * @param model_pose 
         */
        void AddModelToPlanningScene(std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose);
        /**
         * @brief Moveit function to add model to the scene
         * 
         */
        void AddModelsToPlanningScene();

        // MoveIt Interfaces
        moveit::planning_interface::MoveGroupInterface floor_robot_;
        moveit::planning_interface::MoveGroupInterface ceiling_robot_;

        moveit::planning_interface::PlanningSceneInterface planning_scene_;

        trajectory_processing::TimeOptimalTrajectoryGeneration totg_;

        // TF
        std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        /**
         * @brief Function that calls the service to start the competition
         * 
         * @param msg 
         */
        void CompetitionStateCallback(const ariac_msgs::msg::CompetitionState::SharedPtr msg);

        /**
         * @brief Callback function that gets called whenever an order is received
         * 
         * @param order_msg 
         */
        void OrderCallback(const ariac_msgs::msg::Order::SharedPtr order_msg); 

        /**
         * @brief Callback function that gets called when the information about parts in bins are received
         * 
         * @param bin_part_msg 
         */
        void BinPartCallback(const ariac_msgs::msg::BinParts::SharedPtr bin_part_msg); 

        /**
         * @brief Callback function that gets called when the information about parts in the conveyor belt are received
         * 
         * @param conv_part_msg 
         */
        void ConvPartCallback(const ariac_msgs::msg::ConveyorParts::SharedPtr conv_part_msg); 

        /*===================================
            Sensor Subscriber Callbacks
        =====================================*/
        void KitTrayTable1Callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
        void KitTrayTable2Callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
        void LeftBinsCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
        void RightBinsCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
        void ConveyorCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
        void BreakBeamCallback(const ariac_msgs::msg::BreakBeamStatus::SharedPtr break_beam_msg); 
        void BreakBeamCallback2(const ariac_msgs::msg::BreakBeamStatus::SharedPtr break_beam_msg); 

        // Gripper State Callback
        void floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);
        void ceiling_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);

        //Assembly State Callbacks
        void as1_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
        void as2_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
        void as3_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
        void as4_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
        // Assembly States
        std::map<int, ariac_msgs::msg::AssemblyState> assembly_station_states_;


        /**
         * @brief Function that calls the service to start the competition
         * 
         */
        void callService_start();

        /**
         * @brief Function that calls the service to end the competition
         * 
         */
        void callService_end();
        
        /**
         * @brief Function that calls service to submit the order.
         * 
         * @param order 
         */
        void callService_submit(std::string order);


        // Gripper State
        ariac_msgs::msg::VacuumGripperState floor_gripper_state_;
        ariac_msgs::msg::Part floor_robot_attached_part_comb;
        order_::KittingPart floor_robot_attached_part_;
        ariac_msgs::msg::VacuumGripperState ceiling_gripper_state_;
        ariac_msgs::msg::Part ceiling_robot_attached_part_;

        // Sensor poses
        geometry_msgs::msg::Pose kts1_camera_pose_;
        geometry_msgs::msg::Pose kts2_camera_pose_;
        geometry_msgs::msg::Pose left_bins_camera_pose_;
        geometry_msgs::msg::Pose right_bins_camera_pose_;
        geometry_msgs::msg::Pose conveyor_camera_pose_;
        geometry_msgs::msg::Pose conveyor_camera_pick_pose_;

        // Trays
        std::vector<ariac_msgs::msg::KitTrayPose> kts1_trays_;
        std::vector<ariac_msgs::msg::KitTrayPose> kts2_trays_;
        geometry_msgs::msg::Pose kit1_pose;
        geometry_msgs::msg::Pose kit2_pose;

        // Bins
        std::vector<ariac_msgs::msg::PartPose> left_bins_parts_;
        std::vector<ariac_msgs::msg::PartPose> right_bins_parts_;
        geometry_msgs::msg::Pose lbin_pose;
        geometry_msgs::msg::Pose rbin_pose;
        std::vector<ariac_msgs::msg::PartPose> conveyor_parts_;
        std::vector<ariac_msgs::msg::PartPose> conveyor_parts_pick;
        geometry_msgs::msg::Pose conv_part_pose;
        geometry_msgs::msg::Pose conv_part_pick_pose;
   

        // Sensor Callbacks
        // bool kts1_camera_received_data = false;
        // bool kts2_camera_received_data = false;
        // bool left_bins_camera_received_data = false;
        // bool right_bins_camera_received_data = false;
        // bool floor_robot_task_received_data_ = false;
        

        // ARIAC Services
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_comp_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr end_comp_client_;
        rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr submit_order_client_;
        rclcpp::Client<ariac_msgs::srv::PerformQualityCheck>::SharedPtr quality_checker_;
        rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr floor_robot_tool_changer_;
        rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr floor_robot_gripper_enable_;
        rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr ceiling_robot_gripper_enable_;

};
