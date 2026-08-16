/**********************************************************************************
Project: Lab Powder Manipulation
Name: sensor_sync.cpp
Author: Liang Yan
Contributor: Lang Yun

Purpose: Package camera images with the latest robot FK pose for offline
         hand-eye calibration.
Description:
    AprilTag detection is intentionally not performed online.  usb_cam keeps
    publishing images independently, and this node publishes one SyncedMsg for
    every new compressed image after an FK pose has been received.
**********************************************************************************/

#include <cstdint>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "arm_v2_description/msg/synced_msg.hpp"

class SensorSync : public rclcpp::Node
{
public:
    SensorSync()
    : Node("sensor_sync")
    {
        frame_ = declare_parameter<std::string>("frame", "base_link");
             
        const auto fk_topic =
            declare_parameter<std::string>("fk_topic", "/FK_pose");
             
        const auto joint_angles_topic = declare_parameter<std::string>(
            "joint_angles_topic", "/joint_angles");
             
        const auto image_topic = declare_parameter<std::string>(
            "image_topic", "/webcam/image_raw/compressed");
             
        const auto synced_topic = declare_parameter<std::string>(
            "synced_topic", "/synced_msg");

        // FK is low-bandwidth state data, so the normal reliable QoS is used.
        fk_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            fk_topic,
            rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&SensorSync::fkPoseCallback, this, std::placeholders::_1)
        );

        joint_angles_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            joint_angles_topic,
            rclcpp::SensorDataQoS().keep_last(1),
            std::bind(
                &SensorSync::jointAnglesCallback,
                this,
                std::placeholders::_1)
        );

        // Match usb_cam/image_transport's sensor-data QoS.  This callback is
        // triggered by the arrival of a camera frame; there is no timer and no
        // cached-frame re-publication.
        image_raw_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            image_topic,
            rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&SensorSync::imageRawCallback, this, std::placeholders::_1)
        );

        // Best-effort prevents a slow recorder from feeding back into the
        // 150 Hz camera path.  Missing an occasional frame is harmless for
        // hand-eye calibration because only a few dozen stable poses are used.
        synced_msg_pub_ = create_publisher<arm_v2_description::msg::SyncedMsg>(
            synced_topic,
            rclcpp::SensorDataQoS().keep_last(1)
        );

        RCLCPP_INFO(
            get_logger(),
            "sensor_sync started: image='%s', FK='%s', output='%s'; "
            "AprilTag detection is offline",
            image_topic.c_str(),
            fk_topic.c_str(),
            synced_topic.c_str());
    }

private:
    std::string frame_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr fk_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_angles_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_raw_sub_;

    rclcpp::Publisher<arm_v2_description::msg::SyncedMsg>::SharedPtr synced_msg_pub_;

    geometry_msgs::msg::PoseStamped latest_fk_pose_;
    bool fk_received_ = false;

    sensor_msgs::msg::JointState latest_joint_angles_;
    bool joint_angles_received_ = false;

    bool image_stamp_received_ = false;
    std::int32_t last_image_stamp_sec_ = 0;
    std::uint32_t last_image_stamp_nanosec_ = 0;
    std::uint64_t published_count_ = 0;
    std::uint64_t duplicate_image_count_ = 0;

    void fkPoseCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> msg)
    {
        latest_fk_pose_ = *msg;
        fk_received_ = true;
    }

    void jointAnglesCallback(const std::shared_ptr<const sensor_msgs::msg::JointState> msg)
    {
        latest_joint_angles_ = *msg;
        joint_angles_received_ = true;
    }

    void imageRawCallback(const std::shared_ptr<const sensor_msgs::msg::CompressedImage> msg)
    {
        const auto & stamp = msg->header.stamp;
        // A valid camera frame must only produce one packaged message.  
        // This check also protects an offline bag from an upstream duplicate.
        if (
            image_stamp_received_ &&
            stamp.sec == last_image_stamp_sec_ &&
            stamp.nanosec == last_image_stamp_nanosec_)
        {
            ++duplicate_image_count_;
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Dropped duplicated camera timestamp; duplicates=%llu",
                static_cast<unsigned long long>(duplicate_image_count_));
                
            return;
        }

        image_stamp_received_ = true;
        last_image_stamp_sec_ = stamp.sec;
        last_image_stamp_nanosec_ = stamp.nanosec;

        if (!fk_received_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Waiting for the first FK pose; camera frame not packaged");
            return;
        }

        arm_v2_description::msg::SyncedMsg latest_synced_msg;
        latest_synced_msg.synced_header = msg->header;
        latest_synced_msg.frame_id = frame_;
        latest_synced_msg.fk_pose = latest_fk_pose_;
             
        if (joint_angles_received_)
        {
            latest_synced_msg.joint_angles = latest_joint_angles_;
        }

        latest_synced_msg.image_raw = *msg;
        synced_msg_pub_->publish(latest_synced_msg);
        ++published_count_;

        // if (published_count_ % 500 == 0)
        // {
        //     RCLCPP_INFO(
        //         get_logger(),
        //         "Packaged %llu unique image/FK groups",
        //         static_cast<unsigned long long>(published_count_));
        // }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorSync>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
