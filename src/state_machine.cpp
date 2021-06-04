#include "ros/ros.h"
#include "rt2_ass2/Command.h"
#include "rt2_ass2/Position.h"
#include "rt2_ass2/RandomPosition.h"
#include "rt2_ass2/go_to_pointAction.h"
#include <actionlib/client/simple_action_client.h>
bool send_goal = false;
bool stop_goal=false;
bool start=false;
float lin_vel=0;
float ang_vel=0;
bool user_interface(rt2_ass2::Command::Request &req, rt2_ass2::Command::Response &res){
    if (req.command == "start"){
      start = true;
      send_goal=true;
      stop_goal=false;
      lin_vel=req.lin_vel;
      ang_vel=req.ang_vel;
    }
    if (req.command == "stop"){
      start=false;
      send_goal=false;
      stop_goal=true;
    }
    return true;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_ass2::RandomPosition>("/position_server");
   //ros::ServiceClient client_p = n.serviceClient<rt2_ass1_ros1::Position>("/go_to_point");
   actionlib::SimpleActionClient<rt2_ass2::go_to_pointAction> ac("/go_to_point_action", true);

   while(!ac.waitForServer(ros::Duration(1.0))){
     ROS_INFO("Waiting for the go_to_point action server to come up");
   }

   rt2_ass2::go_to_pointGoal goal;

   rt2_ass2::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   while(ros::ok()){
    
    ros::spinOnce();
    if (start && send_goal){
      send_goal=false;
   		client_rp.call(rp);
      goal.target_position.x= rp.response.x;
      goal.target_position.y = rp.response.y;
      goal.theta = rp.response.theta;
      goal.lin_vel=lin_vel;
      goal.ang_vel=ang_vel;
      std::cout << "\nGoing to the position: x= " << goal.target_position.x << " y= " <<goal.target_position.y << " theta = " <<goal.theta << std::endl;
      ac.sendGoal(goal);
    }
    if(start && !send_goal){
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, target reached!");
        send_goal=true;
      }
    }
    if(!start && stop_goal){

      stop_goal=false;
      ac.cancelGoal();

    }
    if(!start && !stop_goal){

      //nulla da fare

    }
   }
   return 0;
}
