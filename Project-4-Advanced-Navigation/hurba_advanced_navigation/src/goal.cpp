#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "tf/tf.h"
#include <random>


// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the nav_goals node
  ros::init(argc, argv, "nav_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(0.1))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define the bounds of the map
  float map_min_x = -5.0;
  float map_max_x = 5.0;
  float map_min_y = -5.0;
  float map_max_y = 5.0;

  // Define the random number generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> x_distr(map_min_x, map_max_x);
  std::uniform_real_distribution<float> y_distr(map_min_y, map_max_y);

  while(true){
    // Generate random x and y coordinates
    float x = x_distr(gen);
    float y = y_distr(gen);

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal to x: %.2f, y: %.2f", x, y);
    ac.sendGoal(goal);

    // Wait for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Goal reached!");
    else
      ROS_INFO("Failed to reach goal for some reason");

    // Wait for some time before sending the next goal
    ros::Duration(1.0).sleep();
  }

  return 0;
} 
