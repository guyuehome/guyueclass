#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>
#include <iostream>
#include <kdl/frames.hpp>

using namespace KDL;
using namespace std;

int main(int argc,char** argv){
    Tree my_tree;
    kdl_parser::treeFromFile("/home/alex/ur5_ws/kdl/src/ur5.urdf", my_tree);
    Chain chain;
    bool exit_value = my_tree.getChain("base_link", "wrist_3_link", chain);
    unsigned int ns = chain.getNrOfSegments();
    for(int i = 0; i < ns; i++) {
      cout << "Type: ";
      cout << chain.getSegment(i).getJoint().getTypeName();
      cout << endl;
      cout << "Origin: ";
      cout << chain.getSegment(i).getJoint().JointOrigin();
      cout << endl;
      cout << "Axis: ";
      cout << chain.getSegment(i).getJoint().JointAxis();
      cout << endl;
      cout << "Frame: ";
      cout << chain.getSegment(i).getFrameToTip();
      cout << endl;
      cout << "Mass: ";
      cout << chain.getSegment(i).getInertia().getMass();
      cout << endl;
      cout << "COG: ";
      for(int j = 0; j < 3; j++) {
        cout << chain.getSegment(i).getInertia().getCOG().data[j] << " ";        
      }
      cout << endl;
      cout << "Inertia: ";
      for(int j = 0; j < 9; j++) {
	cout << chain.getSegment(i).getInertia().getRotationalInertia().data[j] << " ";
      }
      cout << endl;
    } 
    return 0;
}
