/**********************************************************************************
Project: Lab Powder Manipulation 
Name: record_tf_path.cpp
Author: Liang Yan
Purpose: as the user toggles joint state publisher and moves the manipulator
         in RViz, this node tracks its end effector pose and records it in
         trajectory.yaml. It also plots the end effector position in real time
         in RViz 
Description: this node owns a wall timer that fires every 0.05s. every time it 
             fires, it calls timerCallback, whose tf_listener fills the most recent
             transform from fixed_frame to target_frame in the tf_buffer_. The end
             effector position and orientation at each time sample are then extracted
             and converted into a PoseStamped. This is then first written into a yaml
             file, and then published into the /end_effector_path topic, which can be 
             visualized in RViz by addition of this topic.
**********************************************************************************/

#include <fstream>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <string>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"

using namespace std;


// defining class TFPathRecorder, public inheritance from the rclcpp::Node class
class TfPathRecorder : public rclcpp::Node
{
public:
    //constructor
    //calls the parent constructor, and name ROS2 node tf_path_recorder

    /*
    Initializer List:

    Node("tf_path_recorder")
        create the ROS2 node base object

    tf_buffer_(this->get_clock())
        create the TF buffer and give it the node's clock

    tf_listener_(tf_buffer_)
        create the TF listener and connect it to this buffer
        the listener will fill this buffer with new transforms
    */

    TfPathRecorder()
    : Node("tf_path_recorder"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        //that logger carries the node name, [tf_path_recorder]
        RCLCPP_INFO(this->get_logger(), "tf_path_recorder started");

        /*
        this is to avoid these as magic numbers
        the same can also be achieved by using regular strings and doubles
        but using parameters allows for direct editing in terminal
        */
        this->fixed_frame_ = this->declare_parameter<string> ("fixed_frame","base_link");
        this->target_frame_ = this->declare_parameter<string> ("target_frame","tip_link");
        this->sampling_period_ = this->declare_parameter<double> ("sampling_period",0.05);

        // RCLCPP_INFO(this->get_logger(), "fixed_frame: %s", this->fixed_frame_.c_str());
        // RCLCPP_INFO(this->get_logger(), "target_frame: %s", this->target_frame_.c_str());
        // RCLCPP_INFO(this->get_logger(), "sampling_period: %.3f", this->sampling_period_);

        //setting the header frame of the path_i.e. the path is expressed in the fixed frame
        this->path_.header.frame_id = fixed_frame_;

        //create a ROS2 publisher. The message type is nav_msgs::msg::Path. The topic name is /end_effector_path
        //queue size is 10
        this->path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/end_effector_path",10);

        yaml_file_.open(yaml_file_path_);

        if (!yaml_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file: %s", yaml_file_path_.c_str());
        } 
        else 
        {
            yaml_file_ << "trajectory:\n";
            RCLCPP_INFO(this->get_logger(), "Recording trajectory to %s", yaml_file_path_.c_str());
        }

        /*
        wall timer fires at set period
        std::chrono::duration<double>(0.5) sets the period
        std::bind(&TFPathRecorder::timerCallback,this) means that everytime
        the timer fires, timerCallback is called
        */
        this->timer_ = this->create_wall_timer(
            std::chrono::duration<double>(this->sampling_period_),
            //TfPathRecorder: the class name, this specifies that timerCallback 
            //belongs to the class TfPathRecorder

            //this: run timerCallback on this specific object every time the timer fires
            std::bind(&TfPathRecorder::timerCallback,this)
        );

    }

private:

    //this node owns a timer object
    //keep this object alive as long as the node object exists
    //This variable is defined in the constuctor
    rclcpp::TimerBase::SharedPtr timer_;

    string fixed_frame_;
    string target_frame_;
    double sampling_period_;

    //declaring the tf_buffer_ and tf_listener_ variables. 
    //the tf2_ros::TransformListener tf_listener_ internally fills the buffer from
    //tf2_ros::TransformListener tf_listener_
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    nav_msgs::msg::Path path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

    std::ofstream yaml_file_;
    std::string yaml_file_path_ = "trajectory.yaml";


    void writePoseToYaml(const geometry_msgs::msg::PoseStamped& pose_msg)
    {
        if (!this->yaml_file_.is_open()) {
            RCLCPP_WARN(this->get_logger(), "YAML file is not open. Skipping write.");
            return;
        }

        this->yaml_file_ << "  - header:\n";
        this->yaml_file_ << "      stamp:\n";
        this->yaml_file_ << "        sec: " << pose_msg.header.stamp.sec << "\n";
        this->yaml_file_ << "        nanosec: " << pose_msg.header.stamp.nanosec << "\n";
        this->yaml_file_ << "      frame_id: " << pose_msg.header.frame_id << "\n";

        this->yaml_file_ << "    pose:\n";
        this->yaml_file_ << "      position:\n";
        this->yaml_file_ << "        x: " << pose_msg.pose.position.x << "\n";
        this->yaml_file_ << "        y: " << pose_msg.pose.position.y << "\n";
        this->yaml_file_ << "        z: " << pose_msg.pose.position.z << "\n";

        this->yaml_file_ << "      orientation:\n";
        this->yaml_file_ << "        x: " << pose_msg.pose.orientation.x << "\n";
        this->yaml_file_ << "        y: " << pose_msg.pose.orientation.y << "\n";
        this->yaml_file_ << "        z: " << pose_msg.pose.orientation.z << "\n";
        this->yaml_file_ << "        w: " << pose_msg.pose.orientation.w << "\n";

        this->yaml_file_.flush();
    }


    //function called every time the wall timer fires
    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(), "timer is running");

        geometry_msgs::msg::TransformStamped current_transform;
        
        try {
          //look up the current transform from fixed frame to target frame
          current_transform = this->tf_buffer_.lookupTransform(fixed_frame_ ,target_frame_,tf2::TimePointZero);
        } 
        catch (const tf2::TransformException & ex) {
          RCLCPP_WARN( this->get_logger(), "Could not transform %s to %s: %s",
          this->fixed_frame_.c_str(), this->target_frame_.c_str(), ex.what());

          return;
        }

        geometry_msgs::msg::PoseStamped current_pose_stamped;

        current_pose_stamped.pose.position.x = current_transform.transform.translation.x;
        current_pose_stamped.pose.position.y = current_transform.transform.translation.y;
        current_pose_stamped.pose.position.z  = current_transform.transform.translation.z;

        current_pose_stamped.pose.orientation.x = current_transform.transform.rotation.x;
        current_pose_stamped.pose.orientation.y = current_transform.transform.rotation.y;
        current_pose_stamped.pose.orientation.z = current_transform.transform.rotation.z;
        current_pose_stamped.pose.orientation.w = current_transform.transform.rotation.w;

        current_pose_stamped.header.stamp = this->now();
        current_pose_stamped.header.frame_id = fixed_frame_;

        //append this current pose stamped to path_
        this->path_.poses.push_back(current_pose_stamped);

        //append the most recent pose_stamped to yaml file
        writePoseToYaml(current_pose_stamped);

        //publish path to /end_effector_path topic for visualization
        this->path_publisher_->publish(this->path_);
        RCLCPP_INFO(this->get_logger(), "Publishing path with %zu poses.", this->path_.poses.size());
    }
};

int main(int argc, char** argv)
{
    //initializes ROS2
    //lets the program use ROS2 communication, node names, parameters, logging, etc.
    rclcpp::init(argc, argv);

    /*
    std::shared_ptr<TfPathRecorder> node =
    std::make_shared<TfPathRecorder>();

    rclcpp::spin(node) expects a shared ptr because many internal things depend on this node running till
    ctrl+C is pressed. The shared ptr guarantees that as long as something stills owns the node, the node
    object will not be destroyed
    */
    //this line also runs the constructor
    auto node = std::make_shared<TfPathRecorder>();
 
    //checks timer, subscriptions, service callbacks, action callbacks, etc
    //if timer fires, call timerCallback()
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
