#include "ARTracker.h"
#include <ros/ros.h>

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "ar_tracker");
  ros::NodeHandle n;

  ROS_INFO("Starting ar_tracker..");
  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.

  // Create a new ARTracker object.
  ARTracker *arstereo = new ARTracker(n);


  // Tell ROS how fast to run this node.
  ros::Rate r(40);

  // Main loop.
  while (n.ok())
  {
    arstereo->mainLoop();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}; // end main()
