
/**********************************************************************************
Project: Lab Powder Manipulation 
Name: print_kdl_tree.cpp
Author: Liang Yan
Purpose: prints the structure of a KDL tree
Description: takes in a URDF file, parses it into a KDL tree, and prints the tree structure,
             including segments, joints, and joint axes.
**********************************************************************************/

#include <iostream>
#include <string>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl_parser/kdl_parser.hpp>

void printSegment(
    const KDL::SegmentMap::const_iterator& segment_it,
    int depth)
{
    const KDL::Segment& segment = segment_it->second.segment;
    const KDL::Joint& joint = segment.getJoint();

    std::string indent(depth * 4, ' ');

    std::cout << indent << "Segment: " << segment.getName() << std::endl;
    std::cout << indent << "  Joint: " << joint.getName()
              << " (" << joint.getTypeName() << ")" << std::endl;

    if (joint.getType() != KDL::Joint::None) {
        KDL::Vector axis = joint.JointAxis();
        std::cout << indent << "  Axis: ["
                  << axis.x() << ", "
                  << axis.y() << ", "
                  << axis.z() << "]" << std::endl;
    }

    for (const auto& child_it : segment_it->second.children) {
        printSegment(child_it, depth + 1);
    }
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "Usage: print_kdl_tree <path_to_urdf>" << std::endl;
        return 1;
    }

    std::string urdf_path = argv[1];

    KDL::Tree tree;

    if (!kdl_parser::treeFromFile(urdf_path, tree)) {
        std::cerr << "Failed to parse URDF into KDL tree: "
                  << urdf_path << std::endl;
        return 1;
    }

    std::cout << "========== KDL Tree Structure ==========" << std::endl;
    std::cout << "Root: " << tree.getRootSegment()->first << std::endl;
    std::cout << "Number of segments: " << tree.getNrOfSegments() << std::endl;
    std::cout << "Number of joints: " << tree.getNrOfJoints() << std::endl;
    std::cout << std::endl;

    printSegment(tree.getRootSegment(), 0);

    std::cout << "========================================" << std::endl;

    return 0;
}
