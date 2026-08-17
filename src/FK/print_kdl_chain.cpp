/**********************************************************************************
Project: Lab Powder Manipulation 
Name: print_kdl_chain.cpp
Author: Liang Yan
Description: 
    This program reads a URDF file and extracts the KDL chain between the specified base link and tip link.
    It prints out the number of segments, number of moving joints, and details about each segment and joint in the chain.
**********************************************************************************/


#include <iostream>
#include <string>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl_parser/kdl_parser.hpp>

int main(int argc, char** argv)
{
    if (argc != 4) {
        std::cerr << "Usage: print_kdl_chain <urdf_path> <base_link> <tip_link>" << std::endl;
        std::cerr << "Example:" << std::endl;
        std::cerr << "  print_kdl_chain "
                  << "/home/stephen/ros2_ws/src/arm_v2_description/urdf/Arm_V2_edited_2.urdf "
                  << "base_link tip_link" << std::endl;
        return 1;
    }

    std::string urdf_path = argv[1];
    std::string base_link = argv[2];
    std::string tip_link = argv[3];

    KDL::Tree tree;

    if (!kdl_parser::treeFromFile(urdf_path, tree)) {
        std::cerr << "Failed to parse URDF into KDL tree: " << urdf_path << std::endl;
        return 1;
    }

    KDL::Chain chain;

    if (!tree.getChain(base_link, tip_link, chain)) {
        std::cerr << "Failed to extract KDL chain from "
                  << base_link << " to " << tip_link << std::endl;
        return 1;
    }

    std::cout << "Parsed URDF to KDL tree." << std::endl;
    std::cout << "Kinematic Chain from " << base_link
              << " to " << tip_link << ":" << std::endl;
    std::cout << "Number of segments: " << chain.getNrOfSegments() << std::endl;
    std::cout << "Number of moving joints: " << chain.getNrOfJoints() << std::endl;
    std::cout << std::endl;

    std::cout << "Chain Structure: ";
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        std::cout << chain.getSegment(i).getName();
        if (i + 1 < chain.getNrOfSegments()) {
            std::cout << " -> ";
        }
    }
    std::cout << std::endl << std::endl;

    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        const KDL::Segment& segment = chain.getSegment(i);
        const KDL::Joint& joint = segment.getJoint();

        std::cout << "Segment " << i << ": " << segment.getName() << std::endl;
        std::cout << "  Joint Name: " << joint.getName() << std::endl;
        std::cout << "  Joint Type: " << joint.getTypeName() << std::endl;

        if (joint.getType() != KDL::Joint::None) {
            KDL::Vector axis = joint.JointAxis();
            std::cout << "  Axis: ["
                      << axis.x() << ", "
                      << axis.y() << ", "
                      << axis.z() << "]" << std::endl;
        }

        std::cout << std::endl;
    }

    return 0;
}
