
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
 * @brief Structure that stores the Quadrant, color and type of a Kitting part
 * 
 */
struct KittingPart {
    uint8_t quadrant;
    uint8_t color;
    uint8_t type;
};

/**
 * @brief Structure that stores information about the Kitting Task itself. It stores the AGV number, tray_id and destination and different kitting parts.
 * 
 */
struct KittingType {
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
struct A_C_Part {
    uint8_t color;
    uint8_t type;
    geometry_msgs::msg::PoseStamped pose_stamp;
    geometry_msgs::msg::Vector3 install_direction;
};

/**
 * @brief Structure that stores information about Assembly and the Combined task. It stores the station number, the different AGV numbers and parts necessary to complete the task
 * 
 */
struct A_C_Type {
    uint8_t station;
    std::vector<uint8_t> agv_numbers;
    std::vector<A_C_Part> parts;
};

/**
 * @brief Class that defines members that define the order itself. Id stores the order id, type stores the type of task, priority is a boolean
 * variable that stores either 0 or 1 depending on the order priority.
 * 
 */
class Orders{
  public:
    std::string id;
    uint8_t type;
    bool priority;
    KittingType kitting_type;
    A_C_Type a_c_type;
};

/**
 * @brief Class that contains subscriber and callback methods whenever an order or the competitition state is received. 
 * 
 */
class CompetitionStateSubscriber : public rclcpp::Node
{
    public:
    /**
     * @brief Construct a new Competition State Subscriber object. Create subscriber to subscribe to competition state and orders topic.
     * Also a client is created to send message when an order is submitted
     * 
     */
    CompetitionStateSubscriber() : Node("competition_state_subscriber")
    {
        comp_state_sub = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 10, 
                                                                    std::bind(&CompetitionStateSubscriber::competitionStateCallback, this, std::placeholders::_1));

        order_sub = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders", 10, 
                                                                    std::bind(&CompetitionStateSubscriber::orderCallback, this, std::placeholders::_1));                                               

        submit_order_client_ = create_client<ariac_msgs::srv::SubmitOrder>("/ariac/submit_order");
    }

    private:
        /**
         * @brief Callback function definition that is called when competition state is received
         * 
         * @param msg 
         */
        void competitionStateCallback(const ariac_msgs::msg::CompetitionState::SharedPtr msg);
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


        rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr comp_state_sub;

        bool submitted{false};
        /**
         * @brief Vector to store list of orders
         * 
         */
        std::vector<Orders> orders_list;
        /**
         * @brief Subscriber that subscribes to ariac/orders msg.
         * 
         */
        rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_sub;

        int list_size{0};
        /**
         * @brief Callback function that gets called whenever an order is received
         * 
         * @param order_msg 
         */

        void orderCallback(const ariac_msgs::msg::Order::SharedPtr order_msg); 
        /**
         * @brief Function that calls service to submit the order.
         * 
         * @param order 
         */
        void callService_submit(std::string order);


        rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr submit_order_client_;
};

