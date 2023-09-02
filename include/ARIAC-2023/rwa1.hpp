
/**
 * @file rwa1.hpp
 * @author Kiran S Patil,Aniruddh Balram,Vyshnv Achuthan,Badrinarayanan 
 * @brief Header file that defines data structures to store the contents of the orders received
 * @version 0.1
 * @date 2023-03-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "ariac_msgs/msg/competition_state.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <string>

#include "ariac_msgs/msg/order.hpp"
#include "ariac_msgs/msg/assembly_task.hpp"
#include "ariac_msgs/msg/kitting_task.hpp"
#include "ariac_msgs/msg/kitting_part.hpp"
#include "ariac_msgs/msg/bin_parts.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "ariac_msgs/srv/submit_order.hpp"



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


// /**
//  * @brief Structure that stores the Quadrant, color and type of a Kitting part
//  * 
//  */
// struct KittingPart {
//     uint8_t quadrant;
//     uint8_t color;
//     uint8_t type;
// };

// /**
//  * @brief Structure that stores information about the Kitting Task itself. It stores the AGV number, tray_id and destination and different kitting parts.
//  * 
//  */
// struct KittingType {
//     uint8_t quadrant;
//     uint8_t agv_number;
//     int tray_id;
//     uint8_t destination;
//     std::vector<KittingPart> parts;
// };

// /**
//  * @brief Structure that stores the color,type of parts, the pose and direction of the parts uused for both Assembly and Combined task
//  * 
//  */
// struct A_C_Part {
//     uint8_t color;
//     uint8_t type;
//     geometry_msgs::msg::PoseStamped pose_stamp;
//     geometry_msgs::msg::Vector3 install_direction;
// };

// /**
//  * @brief Structure that stores information about Assembly and the Combined task. It stores the station number, the different AGV numbers and parts necessary to complete the task
//  * 
//  */
// struct A_C_Type {
//     uint8_t station;
//     std::vector<uint8_t> agv_numbers;
//     std::vector<A_C_Part> parts;
// };

// /**
//  * @brief Class that defines members that define the order itself. Id stores the order id, type stores the type of task, priority is a boolean
//  * variable that stores either 0 or 1 depending on the order priority.
//  * 
//  */
// class Orders{
//   public:
//     std::string id;
//     uint8_t type;
//     bool priority;
//     KittingType kitting_type;
//     A_C_Type a_c_type;
// };



/**
 * @brief Class that contains subscriber and callback methods whenever an order or the competitition state is received. 
 * 
 */
class CompetitionARIAC : public rclcpp::Node
{
    public:
    /**
     * @brief Construct a new Competition State Subscriber object. Create subscriber to subscribe to competition state and orders topic.
     * Also a client is created to send message when an order is submitted
     * 
     */
    CompetitionARIAC() : Node("competition_subscriber")
    {
        m_callback_group_1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        m_callback_group_2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        //subscription callback groups
        auto subscription_option1 = rclcpp::SubscriptionOptions();
        subscription_option1.callback_group = m_callback_group_1;
        auto subscription_option2 = rclcpp::SubscriptionOptions();
        subscription_option2.callback_group = m_callback_group_2;

        // Subscriber objects            
        comp_state_sub = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 10, 
                                                                    std::bind(&CompetitionARIAC::CompetitionStateCallback, this, std::placeholders::_1),subscription_option1);

        order_sub = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders", 10, 
                                                                    std::bind(&CompetitionARIAC::OrderCallback, this, std::placeholders::_1),subscription_option2);

        submit_order_client_ = create_client<ariac_msgs::srv::SubmitOrder>("/ariac/submit_order");


        // comp_state_sub = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 10, 
        //                                                             std::bind(&CompetitionStateSubscriber::competitionStateCallback, this, std::placeholders::_1));

        // order_sub = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders", 10, 
        //                                                             std::bind(&CompetitionStateSubscriber::orderCallback, this, std::placeholders::_1));                                               

        // submit_order_client_ = create_client<ariac_msgs::srv::SubmitOrder>("/ariac/submit_order");
    }

    private:
        
        // flag to check if order is submitted
        bool submitted{false};

        bool priority_order = false;
        int priority_index = -1;
        int order_counter{0};
        int list_size{0};

        /**
         * @brief Vector to store list of orders
         * 
         */
        std::vector<order_::Orders> orders_list;
        std::vector<order_::Orders> orders_list_submit;

        // Callback Groups
        rclcpp::CallbackGroup::SharedPtr m_callback_group_1;
        rclcpp::CallbackGroup::SharedPtr m_callback_group_2;

        /*==============
        Subscribers
        ==============*/
        rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr comp_state_sub;
        rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_sub;


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


        // ARIAC Services
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_comp_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr end_comp_client_;
        rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr submit_order_client_;


        // /**
        //  * @brief Callback function definition that is called when competition state is received
        //  * 
        //  * @param msg 
        //  */
        // void competitionStateCallback(const ariac_msgs::msg::CompetitionState::SharedPtr msg);
        //  /**
        //  * @brief Function that calls the service to start the competition
        //  * 
        //  */  
        // void callService_start();
        // /**
        //  * @brief Function that calls the service to end the competition
        //  * 
        //  */        
        // void callService_end();


        // rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr comp_state_sub;

        // bool submitted{false};
        // /**
        //  * @brief Vector to store list of orders
        //  * 
        //  */
        // std::vector<Orders> orders_list;
        // /**
        //  * @brief Subscriber that subscribes to ariac/orders msg.
        //  * 
        //  */
        // rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_sub;

        // int list_size{0};
        // /**
        //  * @brief Callback function that gets called whenever an order is received
        //  * 
        //  * @param order_msg 
        //  */

        // void orderCallback(const ariac_msgs::msg::Order::SharedPtr order_msg); 
        // /**
        //  * @brief Function that calls service to submit the order.
        //  * 
        //  * @param order 
        //  */
        // void callService_submit(std::string order);


        // rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr submit_order_client_;
};

