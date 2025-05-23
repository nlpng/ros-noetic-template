/**
 * @file template_node.h
 * @brief Header file for the ROS Template Node
 * 
 * This header defines a reusable ROS node template that demonstrates
 * basic subscriber and publisher functionality with proper timer-based
 * periodic tasks following ROS best practices.
 */

#ifndef ROS_TEMPLATE_NODE_TEMPLATE_NODE_H
#define ROS_TEMPLATE_NODE_TEMPLATE_NODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <memory>

namespace ros_template_node {

/**
 * @class TemplateNode
 * @brief A reusable ROS node template for basic pub/sub functionality
 * 
 * This class provides a template for creating ROS nodes that need to:
 * - Subscribe to various topic types (String, LaserScan, etc.)
 * - Publish data periodically using timers
 * - Handle ROS shutdown gracefully
 * - Follow C++17 and ROS best practices
 */
class TemplateNode {
public:
    /**
     * @brief Constructor - initializes the node with default parameters
     * @param nh ROS NodeHandle for communication
     */
    explicit TemplateNode(ros::NodeHandle& nh);
    
    /**
     * @brief Destructor - ensures clean shutdown
     */
    ~TemplateNode();
    
    /**
     * @brief Initialize the node - sets up publishers, subscribers, and timers
     * @return true if initialization successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Main execution loop - keeps the node running
     */
    void run();

private:
    // ROS NodeHandle
    ros::NodeHandle& nh_;
    
    // Publishers
    ros::Publisher string_publisher_;
    ros::Publisher cmd_vel_publisher_;
    
    // Subscribers
    ros::Subscriber string_subscriber_;
    ros::Subscriber laser_subscriber_;
    
    // Timers for periodic tasks
    ros::Timer publish_timer_;
    ros::Timer status_timer_;
    
    // Node parameters
    double publish_rate_;
    double status_rate_;
    std::string input_topic_;
    std::string output_topic_;
    std::string cmd_vel_topic_;
    std::string laser_topic_;
    
    // Internal state
    bool is_initialized_;
    std::string last_received_message_;
    int message_count_;
    
    /**
     * @brief Load parameters from the parameter server
     * @return true if all required parameters loaded successfully
     */
    bool loadParameters();
    
    /**
     * @brief Callback for string messages
     * @param msg Received string message
     */
    void stringCallback(const std_msgs::String::ConstPtr& msg);
    
    /**
     * @brief Callback for laser scan messages
     * @param msg Received laser scan message
     */
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    
    /**
     * @brief Timer callback for publishing periodic messages
     * @param event Timer event information
     */
    void publishTimerCallback(const ros::TimerEvent& event);
    
    /**
     * @brief Timer callback for status updates
     * @param event Timer event information
     */
    void statusTimerCallback(const ros::TimerEvent& event);
    
    /**
     * @brief Publish a string message
     * @param message The message content to publish
     */
    void publishStringMessage(const std::string& message);
    
    /**
     * @brief Publish a velocity command (example of geometry_msgs usage)
     */
    void publishVelocityCommand();
};

} // namespace ros_template_node

#endif // ROS_TEMPLATE_NODE_TEMPLATE_NODE_H