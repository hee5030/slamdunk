#include <ros/ros.h>
   31 
   32 #include <actionlib/client/simple_action_client.h>
   33 #include <move_base_msgs/MoveBaseAction.h>
   34 #include <nav_msgs/Path.h>
   35 
   36 #include <neonavigation_common/compatibility.h>
   37 
   38 class PatrolActionNode
   39 {
   40 protected:
   41   using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
   42 
   43   ros::NodeHandle nh_;
   44   ros::NodeHandle pnh_;
   45 
   46   ros::Subscriber sub_path_;
   47   std::shared_ptr<MoveBaseClient> act_cli_;
   48 
   49   nav_msgs::Path path_;
   50   size_t pos_;
   51 
   52   void cbPath(const nav_msgs::Path::ConstPtr& msg)
   53   {
   54     path_ = *msg;
   55     pos_ = 0;
   56   }
   57 
   58 public:
   59   PatrolActionNode()
   60     : nh_()
   61     , pnh_("~")
   62   {
   63     neonavigation_common::compat::checkCompatMode();
   64     sub_path_ = neonavigation_common::compat::subscribe(
   65         nh_, "patrol_nodes",
   66         pnh_, "path", 1, &PatrolActionNode::cbPath, this);
   67     act_cli_.reset(new MoveBaseClient("move_base", false));
   68 
   69     pos_ = 0;
   70   }
   71   bool sendNextGoal()
   72   {
   73     move_base_msgs::MoveBaseGoal goal;
   74 
   75     if (path_.poses.size() <= pos_)
   76     {
   77       ROS_WARN("Patrol finished. Waiting next path.");
   78       path_.poses.clear();
   79 
   80       return false;
   81     }
   82 
   83     goal.target_pose.header = path_.poses[pos_].header;
   84     goal.target_pose.header.stamp = ros::Time::now();
   85     goal.target_pose.pose = path_.poses[pos_].pose;
   86 
   87     act_cli_->sendGoal(goal);
   88     pos_++;
   89 
   90     return true;
   91   }
   92   void spin()
   93   {
   94     ros::Rate rate(1.0);
   95 
   96     while (ros::ok())
   97     {
   98       ros::spinOnce();
   99       rate.sleep();
  100 
  101       if (path_.poses.size() == 0)
  102       {
  103         continue;
  104       }
  105 
  106       if (pos_ == 0)
  107       {
  108         sendNextGoal();
  109         continue;
  110       }
  111 
  112       actionlib::SimpleClientGoalState state = act_cli_->getState();
  113       if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  114       {
  115         ROS_INFO("Action has been finished.");
  116         sendNextGoal();
  117       }
  118       else if (state == actionlib::SimpleClientGoalState::ABORTED)
  119       {
  120         ROS_ERROR("Action has been aborted. Skipping.");
  121         sendNextGoal();
  122       }
  123       else if (state == actionlib::SimpleClientGoalState::LOST)
  124       {
  125         ROS_WARN_ONCE("Action server is not ready.");
  126       }
  127     }
  128   }
  129 };
  130 
  131 int main(int argc, char** argv)
  132 {
  133   ros::init(argc, argv, "patrol");
  134 
  135   PatrolActionNode pa;
  136   pa.spin();
  137 
  138   return 0;
  139 }