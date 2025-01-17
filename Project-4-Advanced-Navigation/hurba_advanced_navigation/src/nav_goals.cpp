#include "ros/ros.h" //
#include "actionlib/client/simple_action_client.h" //
#include "actionlib/client/terminal_state.h" //
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include <cstdlib>
#include <chrono>
#include <thread>


// #include "geometry_msgs/PoseStamped.h"

// #include "test_kobuki_action/Kobuki1Action.h" //


#include "thread"
using std::thread;



void stop() {
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    char input;
    ac.waitForResult();
    bool finished_before_timeout = ac.waitForResult(ros::Duration(0.1));
    std::cout << "Stop : Press 'S' : ";
    std::cin >> input;
    if (input == 'S'){
        ac.cancelGoal();
    }
}

void goal() {
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    move_base_msgs::MoveBaseGoal goal;
    
    // actionlib::SimpleActionClient<geometry_msgs::PoseStamped> ac("/move_base_simple/goal",100);
    // geometry_msgs::PoseStamped msg;
 
    char input;
    double x, y;

while(true){

    std::cout << "Current goal: A\nPress 'A' to stay at A 눌러보세요 여러분 A, B, C, D : ";
    std::cin >> input;

    if (input == 'A') {
        x = 1.7;
        y = 0.2;
    }
    else if (input == 'B'){
        x = 1.7;
        y = 4.8;
    }
    else if (input == 'C'){
        x = -1.3;
        y = 3.7;
    }
    else if (input == 'D'){
        x = -1.3;
        y = 0.1;
    }
    else if (input == 'S'){
      std::this_thread::sleep_for(std::chrono::seconds(10));
    }

    
    // Set new goal as A 
    goal.target_pose.header.seq = 0;
    goal.target_pose.header.stamp.sec = 0;
    goal.target_pose.header.stamp.nsec = 0;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.025547;
    goal.target_pose.pose.orientation.w = 0.98381429;       
        
    ac.sendGoal(goal);

    std::cout << "Stop : Press 'S' : ";
    std::cin >> input;
    if (input == 'S'){
       ac.cancelGoal();
    }

    // Wait for the robot to reach the goal or for the goal to be cancelled
    ac.waitForResult();
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if(finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
        ROS_INFO("Succeeded!");
    }
    else {
        ROS_WARN("Faile!");
    }
}
}
int main (int argc, char **argv)
{
    ros::init(argc, argv, "A_B_action_client");
    
    thread t1(goal);
    //thread t2(stop);

    t1.join();
    // t2.join();

}
