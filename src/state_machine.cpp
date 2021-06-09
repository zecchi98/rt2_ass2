/**
 * \file state_machine.cpp
 * \brief This file will let the user interface comunicate with the go_to_point_action
 * \author Federico Zecchi
 * \version 1
 * \date 18/06/21
 * \param [in] world_width Define the width of the discretized world
 *
 * \details
 *
 * Services:<BR>
 *  °/user_interface
 *
 * Clients:<BR>
 *  °/position_server
 *
 * Action Client:<BR>
 *  °/go_to_point_action
 * Description :
 *
 * This node will be the bridge between the user interface, and the go_to_point_action.
 * It will receive commands from the user interface.
 * It can elaborate new random target from the position_server.
 * It can start or stop the go_to_point_action
 *
 */


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

/**The user_interface function
 *
 * This function is the callback function for the server
 * of /user_interface service
 *@param req  the request received from the client of the user_interface.py.
 *@param res  the response has not been used
 *@retval A boolean value
 */
bool user_interface(rt2_ass2::Command::Request &req, rt2_ass2::Command::Response &res){
/* if the request.command="start" than start bool is set to true, send_goal is set to true, stop_goal is set to false.
   Also lin_vel and ang_vel are updated with the requested ones. I have added this two values to the "standard srv file" in order to add the possibility of requiring specific velocities
*/
  if (req.command == "start"){
      start = true;
      send_goal=true;
      stop_goal=false;
      lin_vel=req.lin_vel;
      ang_vel=req.ang_vel;
    }
/* if the request.command="stop" than start bool is set to false, send_goal is set to false, stop_goal is set to true.

*/
  if (req.command == "stop"){
      start=false;
      send_goal=false;
      stop_goal=true;
    }
    return true;
}

/**The main funtion
 *
 * This function initializes everithing that is needed
 * waits for commands and then performes the various requests
 * to services and action
 */
int main(int argc, char **argv)
{
  /* Initialising the state_machine node*/
   ros::init(argc, argv, "state_machine");
   /* setting-up the node handler n*/
   ros::NodeHandle n;
   /* initialising the /user_interface service */
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   /* initialising the client for retreving the random position by means of the /position_server service */
   ros::ServiceClient client_rp = n.serviceClient<rt2_ass2::RandomPosition>("/position_server");
   /* creating the action client */
   /* true causes the client to spin its own thread */
   actionlib::SimpleActionClient<rt2_ass2::go_to_pointAction> ac("/go_to_point_action", true);

   /*Wait until the go_to_point_action server its not ready*/
   while(!ac.waitForServer(ros::Duration(1.0))){
     ROS_INFO("Waiting for the go_to_point action server to come up");
   }

   /* initialising a custom  message of typer GoarReaching goal */
   rt2_ass2::go_to_pointGoal goal;
   /* initialising a custom message of type RandomPosition */
   rt2_ass2::RandomPosition rp;
   /* filling the custom message request fields */
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   while(ros::ok()){
    
    ros::spinOnce();
    /* if the robot is in start mode and a goal need to be sent*/
    if (start && send_goal){
      //send_goal set to false in order to send only one goal
      send_goal=false;
      //ask to the position_server for new random values
   		client_rp.call(rp);
      //update the new goal with the new values just received
      goal.target_position.x= rp.response.x;
      goal.target_position.y = rp.response.y;
      goal.theta = rp.response.theta;
      goal.lin_vel=lin_vel;
      goal.ang_vel=ang_vel;
      std::cout << "\nGoing to the position: x= " << goal.target_position.x << " y= " <<goal.target_position.y << " theta = " <<goal.theta << std::endl;
      //ask to the go_to_point_action for a new goal
      ac.sendGoal(goal);
    }
    //if the robot is in start mode, but does not need to send any goal
    if(start && !send_goal){
      //if the action has completed the goal
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, target reached!");
        //send_goal set to true in order to prepare a new goal for the action
        send_goal=true;
      }
    }
    //if stop_goal is true than the robot need to be stopped
    if(!start && stop_goal){

      //it is set to false in order to not repeat this action
      stop_goal=false;
      //the goal is deleted
      ac.cancelGoal();

    }
   }
   return 0;
}
