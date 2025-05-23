/**
 * @file template_node.cpp
 * @brief Implementation of the ROS Template Node
 * 
 * This file implements a reusable ROS node template that demonstrates
 * proper subscriber/publisher patterns, timer usage, and parameter handling
 * following ROS and C++17 best practices.
 */

#include "ros_template_node/template_node.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>

namespace ros_template_node {

TemplateNode::TemplateNode(ros::NodeHandle& nh) 
    : nh_(nh)
    , is_initialized_(false)
    , message_count_(0)
    , publish_rate_(1.0)
    , status_rate_(0.2)
    , input_topic_("/template_input")
    , output_topic_("/template_output")
    , cmd_vel_topic_("/cmd_vel")
    , laser_topic_("/scan")
{
    ROS_INFO("TemplateNode constructor called");
}

TemplateNode::~TemplateNode() {
    ROS_INFO("TemplateNode destructor called");
}

bool TemplateNode::loadParameters() {
    // Load parameters with default values
    nh_.param("publish_rate", publish_rate_, 1.0);
    nh_.param("status_rate", status_rate_, 0.2);
    nh_.param("input_topic", input_topic_, std::string("/template_input"));
    nh_.param("output_topic", output_topic_, std::string("/template_output"));
    nh_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
    nh_.param("laser_topic", laser_topic_, std::string("/scan"));
    
    // Validate parameters
    if (publish_rate_ <= 0.0 || status_rate_ <= 0.0) {
        ROS_ERROR("Invalid rate parameters: publish_rate=%.2f, status_rate=%.2f", 
                  publish_rate_, status_rate_);
        return false;
    }
    
    ROS_INFO("Parameters loaded successfully:");
    ROS_INFO("  publish_rate: %.2f Hz", publish_rate_);
    ROS_INFO("  status_rate: %.2f Hz", status_rate_);
    ROS_INFO("  input_topic: %s", input_topic_.c_str());
    ROS_INFO("  output_topic: %s", output_topic_.c_str());
    ROS_INFO("  cmd_vel_topic: %s", cmd_vel_topic_.c_str());
    ROS_INFO("  laser_topic: %s", laser_topic_.c_str());
    
    return true;
}

bool TemplateNode::initialize() {
    ROS_INFO("Initializing TemplateNode...");
    
    // Load parameters first
    if (!loadParameters()) {
        ROS_ERROR("Failed to load parameters");
        return false;
    }
    
    // Initialize publishers with appropriate queue sizes
    string_publisher_ = nh_.advertise<std_msgs::String>(output_topic_, 10);
    cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    
    // Initialize subscribers with appropriate queue sizes
    string_subscriber_ = nh_.subscribe(input_topic_, 10, 
                                     &TemplateNode::stringCallback, this);
    laser_subscriber_ = nh_.subscribe(laser_topic_, 1, 
                                    &TemplateNode::laserCallback, this);
    
    // Initialize timers - using ros::Timer for periodic tasks (ROS best practice)
    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), 
                                   &TemplateNode::publishTimerCallback, this);
    status_timer_ = nh_.createTimer(ros::Duration(1.0 / status_rate_), 
                                  &TemplateNode::statusTimerCallback, this);
    
    is_initialized_ = true;
    ROS_INFO("TemplateNode initialized successfully");
    ROS_INFO("Publishing to: %s", output_topic_.c_str());
    ROS_INFO("Subscribing to: %s, %s", input_topic_.c_str(), laser_topic_.c_str());
    
    return true;
}

void TemplateNode::run() {
    if (!is_initialized_) {
        ROS_ERROR("Node not initialized! Call initialize() first.");
        return;
    }
    
    ROS_INFO("TemplateNode running...");
    
    // ROS spin - handles callbacks and maintains node operation
    ros::spin();
    
    ROS_INFO("TemplateNode shutting down");
}

void TemplateNode::stringCallback(const std_msgs::String::ConstPtr& msg) {
    last_received_message_ = msg->data;
    ROS_DEBUG("Received string message: %s", msg->data.c_str());
    
    // Echo the received message with modification
    std::ostringstream response;
    response << "Echo: " << msg->data << " (processed by template_node)";
    publishStringMessage(response.str());
}

void TemplateNode::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ROS_DEBUG("Received laser scan with %zu ranges", msg->ranges.size());
    
    // Example processing: find minimum range
    if (!msg->ranges.empty()) {
        auto min_range = *std::min_element(msg->ranges.begin(), msg->ranges.end());
        
        // React to obstacles (example behavior)
        if (min_range < 0.5 && min_range > msg->range_min) {
            ROS_WARN("Obstacle detected at %.2f meters!", min_range);
            // Could trigger emergency stop or avoidance behavior
        }
    }
}

void TemplateNode::publishTimerCallback(const ros::TimerEvent& event) {
    // Periodic publishing example
    std::ostringstream message;
    message << "Template node heartbeat #" << ++message_count_ 
            << " at " << ros::Time::now();
    
    publishStringMessage(message.str());
    
    // Also publish a simple velocity command as example
    publishVelocityCommand();
}

void TemplateNode::statusTimerCallback(const ros::TimerEvent& event) {
    // Periodic status updates
    ROS_INFO("Node status: Running, processed %d messages, last: '%s'", 
             message_count_, last_received_message_.c_str());
}

void TemplateNode::publishStringMessage(const std::string& message) {
    if (!is_initialized_) {
        ROS_WARN("Attempting to publish before initialization");
        return;
    }
    
    std_msgs::String msg;
    msg.data = message;
    string_publisher_.publish(msg);
    
    ROS_DEBUG("Published: %s", message.c_str());
}

void TemplateNode::publishVelocityCommand() {
    if (!is_initialized_) {
        return;
    }
    
    // Example: publish a simple forward motion command
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.1;  // 0.1 m/s forward
    cmd_vel.angular.z = 0.0; // No rotation
    
    cmd_vel_publisher_.publish(cmd_vel);
}

} // namespace ros_template_node

/**
 * @brief Main function - entry point for the ROS node
 */
int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "template_node");
    
    // Create node handle
    ros::NodeHandle nh("~"); // Private node handle for parameters
    
    try {
        // Create and initialize the template node
        ros_template_node::TemplateNode node(nh);
        
        if (!node.initialize()) {
            ROS_ERROR("Failed to initialize TemplateNode");
            return 1;
        }
        
        // Run the node
        node.run();
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in main: %s", e.what());
        return 1;
    }
    
    ROS_INFO("Template node terminated");
    return 0;
}