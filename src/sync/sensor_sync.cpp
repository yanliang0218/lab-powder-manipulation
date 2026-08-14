/**********************************************************************************
Project: Lab Powder Manipulation 
Name: sensor_sync
Author: Liang Yan
Purpose: 
Description: 
**********************************************************************************/


#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>


#include "arm_v2_description/msg/april_tag_detection.hpp"
#include "arm_v2_description/msg/april_tag_detection_array.hpp"
#include "arm_v2_description/msg/synced_msg.hpp"

using namespace std;


class SensorSync: public rclcpp::Node
{
public:
    //constructor
    SensorSync()
    : Node("sensor_sync")
      
    {
        //that logger carries the node name, [tf_path_recorder]
        RCLCPP_INFO(this->get_logger(), "sensor_sync started");

        //all poses are defined in base_link frame
        this->frame_ = this->declare_parameter<string> ("frame","base_link");

        /**********************************************************************************************
        Subscribers: for all incoming messages from FK, webcam, and endoscope modules
        all bound to their respective callback functions, which runs every time a new message is received
        *************************************************************************************************/
        this->fk_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/FK_pose",1,std::bind(&SensorSync::fkPoseCallback,this,std::placeholders::_1)
        );

        this->joint_angles_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_angles",1,std::bind(&SensorSync::jointAnglesCallback,this,std::placeholders::_1)
        );

        this->tag_detections_sub_ = this->create_subscription<arm_v2_description::msg::AprilTagDetectionArray>(
            "/webcam/tag_pose",1,std::bind(&SensorSync::tagPoseCallback,this,std::placeholders::_1)
        );
    
        this->image_raw_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/webcam/image_raw/compressed",1,std::bind(&SensorSync::imageRawCallback,this,std::placeholders::_1)
        );

        /******************************************
        Publishers, for the synchronized message
        ******************************************/
        this->synced_msg_pub_ = this->create_publisher<arm_v2_description::msg::SyncedMsg>("/synced_msg",1);

    }

private:
    string frame_;

    /****************************************************************************
    Subscribers: for all incoming messages from FK, webcam, and endoscope modules
    *****************************************************************************/
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr fk_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_angles_sub_;
    rclcpp::Subscription<arm_v2_description::msg::AprilTagDetectionArray>::SharedPtr tag_detections_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_raw_sub_;

    /******************************************
    Publishers, for the synchronized message
    ******************************************/
    rclcpp::Publisher<arm_v2_description::msg::SyncedMsg>::SharedPtr synced_msg_pub_;


    /***********************************************************
    private variable to store the latest message from each topic
    ***********************************************************/
    geometry_msgs::msg::PoseStamped latest_fk_pose_;
    bool fk_received_ = false;

    sensor_msgs::msg::JointState latest_joint_angles_;
    bool joint_angles_received_ = false;

    arm_v2_description::msg::AprilTagDetectionArray latest_tag_detections_;
    bool tag_received_ = false;

    sensor_msgs::msg::CompressedImage latest_image_raw_;
  
    /***********************************************************
    callback functions for each subscriber, to store the latest message
    ***********************************************************/
    void fkPoseCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> msg)
    {
        this->latest_fk_pose_ = *msg;
        this->fk_received_ = true;
    }

    void jointAnglesCallback(const std::shared_ptr<const sensor_msgs::msg::JointState> msg)
    {
        this->latest_joint_angles_ = *msg;
        this->joint_angles_received_ = true;
    }

    void tagPoseCallback(const std::shared_ptr<const arm_v2_description::msg::AprilTagDetectionArray> msg)
    {
        this->latest_tag_detections_ = *msg;
        this->tag_received_ = true;
    }


    //since the image raw topic (and the tag detection topic) at the lowest frequency,
    //we will use it to trigger synchronization of all messages
    void imageRawCallback(const std::shared_ptr<const sensor_msgs::msg::CompressedImage> msg)
    {
        this->latest_image_raw_ = *msg;

        arm_v2_description::msg::SyncedMsg latest_synced_msg;

        //if (this->fk_received_ && this->joint_angles_received_ && this->tag_received_ )
        if (this->fk_received_  && this->tag_received_ )
        {
            latest_synced_msg.synced_header = this->latest_image_raw_.header;
            latest_synced_msg.frame_id= this->frame_;
            latest_synced_msg.fk_pose = this->latest_fk_pose_;
            latest_synced_msg.joint_angles = this->latest_joint_angles_;
            latest_synced_msg.tag_detections = this->latest_tag_detections_;
            latest_synced_msg.image_raw = this->latest_image_raw_;

            this->synced_msg_pub_->publish(latest_synced_msg);
        }
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
    auto node = std::make_shared<SensorSync>();
 
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

