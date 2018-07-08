#include "ros/ros.h"
#include "simple_navigation_goals/GoalAndRespond.h"
#include <move_base_msgs/MoveBaseAction.h>  
#include <actionlib/client/simple_action_client.h>  
#include <geometry_msgs/Pose.h>
#include <vector>
#include <unistd.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;  

bool move(simple_navigation_goals::GoalAndRespond::Request  &req,
         simple_navigation_goals::GoalAndRespond::Response &res)
{
    MoveBaseClient ac("move_base", true);  

    while(!ac.waitForServer(ros::Duration(5.0))){  
    ROS_INFO("Waiting for the move_base action server to come up");  
    }  

    move_base_msgs::MoveBaseGoal goal;  
    goal.target_pose.header.frame_id = "map";  
    goal.target_pose.header.stamp = ros::Time::now();  
    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.position.z = 0;
    //resp.plan.poses.resize(global_plan.size());
    //for(unsigned int i = 0; i < global_plan.size(); ++i){
      //resp.plan.poses[i] = global_plan[i];
    //}

    int success = 0;
    unsigned int num = req.goals.poses.size();
    std::vector<geometry_msgs::Pose> goals_nav;
    goals_nav.resize(num);
    for(unsigned int i = 0; i < num; ++i){
        goals_nav.push_back(req.goals.poses[i]);
        //给move_base赋值
        goal.target_pose.pose.position.x = req.goals.poses[i].position.x;
        goal.target_pose.pose.position.y = req.goals.poses[i].position.y;
        goal.target_pose.pose.orientation.z = req.goals.poses[i].orientation.z;
        goal.target_pose.pose.orientation.w = req.goals.poses[i].orientation.w;
        ROS_INFO("goal.position:%f,%f,%f",goal.target_pose.pose.position.x,goal.target_pose.pose.position.y,goal.target_pose.pose.position.z);
        ROS_INFO("goal.orientation:%f,%f,%f,%f",goal.target_pose.pose.orientation.x,goal.target_pose.pose.orientation.y,goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w);        
        ROS_INFO("Sending goal");  
        ac.sendGoal(goal);  
        ac.waitForResult();  

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)  
        {
            ROS_INFO("The %dth point is reached!", i+1); 
            ++success; 
            //sleep(1);
        }
        else
        {
            ROS_INFO("The %dth point failed!", i+1);  
        }
    }

    if(success == num)  
    {
        ROS_INFO("Hooray, service successful!");  
        res.success = true;
        ROS_INFO("sending back response");
        return true;
    }
    else
    {
        res.success = false;
        ROS_INFO("%d points success, %d points failed!", success, num - success);  
        return true;
    }

    return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "send_goal_receive_respond_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("abc", move);
  ROS_INFO("Ready to receive goal.");
  ros::spin();

  return 0;
}
