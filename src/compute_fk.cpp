/**********************************************************************************
Project: Lab Powder Manipulation 
Name: compute_fk.cpp
Author: Liang Yan
Purpose: computes the forward kinematics of the data acquisition manipulator
Description: takes in current joint angles, converts these into a KDL:JntArray. This array is 
             then converted into a KDL chain, which is then used by ChainFkSolverPos_recursive fk_solver
             to compute the end effector pose.
**********************************************************************************/



#include <iostream>
#include <map>
#include <string>
#include <stdexcept>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

constexpr double PI = 3.14159265358979323846;

//function, maps joint variables into KDL joint array order
KDL::JntArray makeJointArray(
    const KDL::Chain& chain,//chain is here
    double q_base,
    double q_low_A,
    double q_high_A,
    double q_wrist_1,
    double q_wrist_2,
    double q_wrist_3)
{
    //using a map
    std::map<std::string, double> joint_value;

    //independent joints
    joint_value["base_joint"] = q_base;
    joint_value["low_joint_A"] = q_low_A;
    joint_value["high_joint_A"] = q_high_A;
    joint_value["wrist_1_joint"] = q_wrist_1;
    joint_value["wrist_2_joint"] = q_wrist_2;
    joint_value["wrist_3_joint"] = q_wrist_3;

    // Mimic/dependent joints according to your current URDF
    joint_value["low_joint_C"] = -q_low_A;
    joint_value["high_joint_C"] = q_high_A;
    
    // chain.getNrOfJoints() returns number of moving joints in the selected KDL chain
    // creates array that the KDL FK needs
    KDL::JntArray q(chain.getNrOfJoints());

    unsigned int q_index = 0;
    
    //iterating through every segment in the chain
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        
        // gets the joint associated with the current segment 
        const KDL::Joint& joint = chain.getSegment(i).getJoint();

        // KDL::Joint: None means "this segment has no moving joint"
        if (joint.getType() == KDL::Joint::None) {
            //skip non-moving joint
            continue;
        }
        
        // gets the joint name
        std::string name = joint.getName();
        
        // defensive programming: verify the current joint name was actually assigned a value
        // if not, throw a runtime_error
        if (joint_value.find(name) == joint_value.end()) {
            throw std::runtime_error("No value provided for joint: " + name);
        }

        //put this value into the joint array
        q(q_index) = joint_value[name];

        std::cout << "q[" << q_index << "] = "
                  << name << " = "
                  << q(q_index) << std::endl;

        q_index++;
    }

    return q;
}

int main()
{
    std::string urdf_path =
        "/home/stephen/ros2_ws/src/arm_v2_description/urdf/Arm_V2_edited_2.urdf";

    std::string base_link = "base_link";
    std::string tip_link = "tip_link";

    KDL::Tree tree;

    if (!kdl_parser::treeFromFile(urdf_path, tree)) {
        std::cerr << "Failed to parse URDF into KDL tree." << std::endl;
        return 1;
    }

    KDL::Chain chain;
     
    //loads the chain in this if 
    if (!tree.getChain(base_link, tip_link, chain)) {
        std::cerr << "Failed to get chain from "
                  << base_link << " to " << tip_link << std::endl;
        return 1;
    }

    std::cout << "KDL chain loaded." << std::endl;
    std::cout << "Number of Segments: " << chain.getNrOfSegments() << std::endl;
    std::cout << "Number of Moving joints: " << chain.getNrOfJoints() << std::endl;
    std::cout << std::endl;

    // Example joint inputs in radians
    // double q_base = PI / 2.000;
    double q_base = 0.0;
    double q_low_A = 0.0;
    double q_high_A = 0.0;
    double q_wrist_1 = 0.0;
    double q_wrist_2 = 0.0;
    double q_wrist_3 = 0.0;

    KDL::JntArray q = makeJointArray( //calls function defined earlier!!!
        chain,
        q_base,
        q_low_A,
        q_high_A,
        q_wrist_1,
        q_wrist_2,
        q_wrist_3
    );

     
    //joint axes are considered here!
    KDL::ChainFkSolverPos_recursive fk_solver(chain);

    KDL::Frame end_pose;

    //end pose is store in the JntToCart(q, end_pose)
    int status = fk_solver.JntToCart(q, end_pose);


    //!!!!where is the end effector point!!!

//     KDL::Frame wrist3_to_tool(
//         KDL::Rotation::Identity(),
//         KDL::Vector(0.037, -0.0325, 0.030)
//     );
//     make a new fixed joint on that new plane and edit this transform!

//    end_pose = end_pose * wrist3_to_tool;

    if (status < 0) {
        std::cerr << "FK solver failed." << std::endl;
        return 1;
    }

    std::cout << std::endl;
    std::cout << "FK result: " << base_link
              << " -> " << tip_link << std::endl;

    std::cout << "Position [m]:" << std::endl;
    std::cout << "  x = " << end_pose.p.x() << std::endl;
    std::cout << "  y = " << end_pose.p.y() << std::endl;
    std::cout << "  z = " << end_pose.p.z() << std::endl;

    double roll, pitch, yaw;
    end_pose.M.GetRPY(roll, pitch, yaw);

    std::cout << "Orientation [rad]:" << std::endl;
    std::cout << "  roll  = " << roll << std::endl;
    std::cout << "  pitch = " << pitch << std::endl;
    std::cout << "  yaw   = " << yaw << std::endl;

    return 0;
}
