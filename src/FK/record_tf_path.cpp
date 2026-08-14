/**********************************************************************************
Project: Lab Powder Manipulation
File: record_tf_path.cpp
Author: Liang Yan
Contributor: Lang Yun

Purpose:
    Compute one base_link -> tip_link forward-kinematics pose for every received
    /joint_states message and publish exactly one /FK_pose message.

Data flow:
    /joint_states[k]
        -> jointStateCallback()
        -> KDL forward kinematics
        -> /FK_pose[k]

Important properties:
    1. There is no wall timer or TF lookup.
    2. /FK_pose copies the exact timestamp from the triggering /joint_states, and 
    3. end_effector pose is computed via KDL tree and chain, which is constructed from the same URDF used by robot_state_publisher
    4. robot_state_publisher remains independent and can continue serving RViz.
**********************************************************************************/

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

class JointStateFkPublisher : public rclcpp::Node
{
public:
    JointStateFkPublisher()
    : Node("joint_state_fk_publisher")
    {
        //these declare_parameters calls declare each parameter name and type
        //and provide a default value if the parameter is not set externally
        const std::string robot_description =
            declare_parameter<std::string>("robot_description", "");

        base_frame_ =
            declare_parameter<std::string>("base_frame", "base_link");
        
        tip_frame_ =
            declare_parameter<std::string>("tip_frame", "tip_link");
        
        joint_states_topic_ =
            declare_parameter<std::string>("joint_states_topic", "/joint_states");
        
        fk_pose_topic_ =
            declare_parameter<std::string>("fk_pose_topic", "/FK_pose");

        //after node initialization, check if the robot_description parameter is empty
        if (robot_description.empty()) {
            throw std::runtime_error(
                "robot_description is empty; pass the same URDF used by "
                "robot_state_publisher");
        }

        KDL::Tree tree;

        // Construct a KDL tree from the URDF string. This is a one-time operation
        // a KDL tree can have branches
        if (!kdl_parser::treeFromString(robot_description, tree)) {
            throw std::runtime_error(
                "Failed to construct a KDL tree from robot_description");
        }

        // Construct a KDL chain from the base frame to the tip frame. This is also a one-time operation
        // a KDL chain is a single branch of the KDL tree from base_frame_ to tip_frame_
        if (!tree.getChain(base_frame_, tip_frame_, chain_))
        {
            throw std::runtime_error(
                "Failed to construct the KDL chain from '" + base_frame_ +
                "' to '" + tip_frame_ + "'");
        }

        // Check if the KDL chain has any movable joints. If not, throw an error
        if (chain_.getNrOfJoints() == 0U)
        {
            throw std::runtime_error(
                "The selected KDL chain contains no movable joints");
        }

        // Store the moving joint names in exactly the order expected by KDL.
        chain_joint_names_.reserve(chain_.getNrOfJoints());

        // Iterate through each segment of the KDL chain to extract the joint names
        for (unsigned int segment_index = 0;
             segment_index < chain_.getNrOfSegments();
             ++segment_index)
        {
            //First, get the joint associated with the current segment of the KDL chain
            const KDL::Joint & joint =
                chain_.getSegment(segment_index).getJoint();

            //Only consider joints that are not of type KDL::Joint::None (i.e., movable joints)
            if (joint.getType() != KDL::Joint::None)
            {
                chain_joint_names_.push_back(joint.getName());
            }
        }

        //chain joint names size should match the number of joints in the KDL chain. 
        //If not, throw an error
        if (chain_joint_names_.size() != chain_.getNrOfJoints()) {
            throw std::runtime_error(
                "Internal KDL joint-order construction failed");
        }

        // Create a KDL forward kinematics solver for the constructed KDL chain. 
        // This solver will be used to compute the end-effector pose from joint angles.
        fk_solver_ =
            std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

        // Reliable QoS and a moderate queue provide headroom for the current
        // approximately 545 Hz stream without allowing unbounded growth.
        // reduced to 20 to avoid excessive queuing to ensure real-time publication of end-effector path
        auto qos = rclcpp::QoS(rclcpp::KeepLast(20));
        qos.reliable();

        // Create a publisher for the computed FK pose, which will publish PoseStamped messages to the specified topic
        fk_pose_pub_ =
            create_publisher<geometry_msgs::msg::PoseStamped>(
                fk_pose_topic_, qos);
        
        // Create a subscription to the /joint_states topic, which will trigger the jointStateCallback function whenever a new JointState message is received
        joint_state_sub_ =
            create_subscription<sensor_msgs::msg::JointState>(
                joint_states_topic_,
                qos,
                std::bind(&JointStateFkPublisher::jointStateCallback,this,std::placeholders::_1)
            );

        RCLCPP_INFO(
            get_logger(),
            "Event-driven FK ready: %s -> %s, input=%s, output=%s, "
            "%u moving joints",
            base_frame_.c_str(),
            tip_frame_.c_str(),
            joint_states_topic_.c_str(),
            fk_pose_topic_.c_str(),
            chain_.getNrOfJoints());
    }

private:
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::vector<std::string> chain_joint_names_;

    std::string base_frame_;
    std::string tip_frame_;
    std::string joint_states_topic_;
    std::string fk_pose_topic_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        fk_pose_pub_;

    std::uint64_t received_count_ = 0;
    std::uint64_t published_count_ = 0;
    std::uint64_t rejected_count_ = 0;

    //boolean function to convert a JointState message into a KDL::JntArray, returning true if successful
    //and false if the message is invalid 

    bool makeJointArray(
        const sensor_msgs::msg::JointState & message,
        KDL::JntArray & joint_array)
    {
        if (message.name.size() != message.position.size())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Rejected /joint_states: name size (%zu) differs from "
                "position size (%zu)",
                message.name.size(),
                message.position.size());
            return false;
        }

        //build a map of joint names to positions for fast lookup
        //
        std::unordered_map<std::string, double> map_joint_names_to_angles;

        //reserve space to avoid repeated reallocation and copying
        map_joint_names_to_angles.reserve(message.name.size());

        for (std::size_t index = 0; index < message.name.size(); ++index) {

        //iterating through every joint name in the JointState message, check if the position is finite, 
        //and if so, store it in the positions_by_name map

            if (!std::isfinite(message.position[index])) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Rejected /joint_states: joint '%s' has a non-finite position",
                    message.name[index].c_str());
                return false;
            }
            
            //if not infinite, store the joint name and position in the map
            //populate an unordered map with joint names as keys and their corresponding positions as values
            map_joint_names_to_angles[message.name[index]] = message.position[index];
        }

        //iterate through the KDL chain joint names 
        //and fill the joint_array with the corresponding positions from the map
        for (std::size_t kdl_index = 0;
             kdl_index < chain_joint_names_.size();
             ++kdl_index)
        { 
            //for each KDL joint name, find its position in the map_joint_names_to_angles map
            //the search uses chain_joint_names_[kdl_index] as the search key, and
            //if the key is found, the iterator points to the specific key-value pair in the map, 
            //allowing access to the corresponding joint position
            //if the key does not exist, the iterator will point to map_joint_names_to_angles.end(), indicating that the joint name is missing from the message
            const auto found = map_joint_names_to_angles.find(chain_joint_names_[kdl_index]);
            
            //if the joint name is not found in the map, log a warning and return false
            if (found == map_joint_names_to_angles.end()) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Rejected /joint_states: required KDL joint '%s' is missing",
                    chain_joint_names_[kdl_index].c_str());
                return false;
            }
            
            //if the joint name is found, assign its angular position (second in the key-value pair) to the joint_array at the corresponding index
            //at this point, joint_array is filled with the joint positions in the exact order expected by KDL for forward kinematics computation
            joint_array(kdl_index) = found->second;
        }
        
        //if all required joints are found and assigned, return true
        return true;
    }
 
    //callback function for the /joint_states subscription
    void jointStateCallback(
        const sensor_msgs::msg::JointState::SharedPtr message)
    {
        //increment the received count for every valid /joint_states message received
        ++received_count_;
        
        KDL::JntArray joint_array(chain_.getNrOfJoints());

        //convert the JointState message into a KDL::JntArray, returning false if the message is invalid
        //if the conversion fails, increment the rejected count and return early
        if (!makeJointArray(*message, joint_array)) {
            ++rejected_count_;
            return;
        }

        //compute the forward kinematics using the KDL solver, and publish the resulting pose to /FK_pose
        KDL::Frame tip_pose;
        const int status = fk_solver_->JntToCart(joint_array, tip_pose);
        if (status < 0) {
            ++rejected_count_;
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "KDL forward kinematics failed with status %d",
                status);
            return;
        }

        geometry_msgs::msg::PoseStamped pose_message;

        // This is the key one-to-one synchronization rule: the FK result keeps
        // the exact timestamp of the JointState that triggered this callback.
        pose_message.header.stamp = message->header.stamp;
        pose_message.header.frame_id = base_frame_;

        pose_message.pose.position.x = tip_pose.p.x();
        pose_message.pose.position.y = tip_pose.p.y();
        pose_message.pose.position.z = tip_pose.p.z();

        double qx = 0.0;
        double qy = 0.0;
        double qz = 0.0;
        double qw = 1.0;
        tip_pose.M.GetQuaternion(qx, qy, qz, qw);

        pose_message.pose.orientation.x = qx;
        pose_message.pose.orientation.y = qy;
        pose_message.pose.orientation.z = qz;
        pose_message.pose.orientation.w = qw;

        fk_pose_pub_->publish(pose_message);
        ++published_count_;

        if (published_count_ == 1U) {
            RCLCPP_INFO(
                get_logger(),
                "Published first event-driven FK pose with the source "
                "/joint_states timestamp");
        }

        if (received_count_ % 10000U == 0U) {
            RCLCPP_INFO(
                get_logger(),
                "FK counters: received=%llu, published=%llu, rejected=%llu",
                static_cast<unsigned long long>(received_count_),
                static_cast<unsigned long long>(published_count_),
                static_cast<unsigned long long>(rejected_count_));
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateFkPublisher>());
    rclcpp::shutdown();
    return 0;
}

