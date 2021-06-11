#!/usr/bin/env python


"""
.. module:: go_to_point_action
   :platform: Unix
   :synopsis: Python module for piloting the robot to the target

.. moduleauthor:: Carmine Recchiuto <carmine.recchiuto@dibris.unige.it>

ROS node for driving a robot to a specific point

Subscribes to:
    /odom topic where the simulator publishes the robot position

Publishes to:
    /cmd_vel the desired robot position

Service :
    /go_to_point to start the robot motion.
"""

## @package rt2_ass2
# \file go_to_point_action.py
# \brief This file will directly operate on the robot based on the goal received
# \author Federico Zecchi
# \date 18/06/21
# \details
#
# Action Server:<BR>
#  °/go_to_point_action
#
#
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from rt2_ass2.srv import Position
import math
import actionlib
import actionlib.msg
import rt2_ass2.msg

# robot state variables
position_ = Point()
yaw_ = 0
position_ = 0
state_ = 0
pub_ = None


# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6



# action_server
act_s = None




## The clbk_odom function.
#
#  This Callback is for the subscriber to topic odom
#  it keeps track of the robot pose.
#
# @arg msg the message carrying the information
def clbk_odom(msg):
    """
    This Callback is for the subscriber to topic odom. It keeps track of the robot pose.

    Args:
        msg(Odometry): the message received
    Returns: none
    """
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

## The change_state function.
#
#  It controls the state of the robot, allowing to translate
#  or to rotate when needed.

# @param state indicates the new state to chancge to
# @var state_ the current state of the robot
def change_state(state):
    """
    It controls the state of the robot, allowing to translate or to rotate when needed.

    Args:
        state: state of the robot

    Returns: none
    """
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


## The normalize_angle function.
#
#  It performs normalazation of an angle .
def normalize_angle(angle):
    """
    Function for normalizing the angle between -pi and pi.

    Args:
        angle(Float): the input angle
    Returns:angle(Float):
        the normalized angle.
    """
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

## The fix_yah function.
#
#  It rotates the robot to fix it's yah in the desired direction
#  by publishing a twist message on /cmd_vel
#  @param des_pos from this desired position computes the desired yah
def fix_yaw(des_pos):
    """
    Function for fixing the angle of the robot.
    It rotates the robot to fix it's yah in the desired direction by publishing a twist message on /cmd_vel

    Args:
        des_pos(Point): the desiderd pose
    Returns: none
    """
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        change_state(1)




## The go_straight_ahead function.
#
#  It allows the robot to go straight or to change it's state if needed
def go_straight_ahead(des_pos):
    """
    Function for going straight ahead to the target.

    Args:
        des_pos(Point): the desiderd pose
    Returns: none
    """
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = kp_d
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub_.publish(twist_msg)
    else: # state change conditions
        change_state(2)

## The fix_final_yah function.
#
#  It allows the robot to fix it's yah and reach the final state(done)
#  if the conditions are met
def fix_final_yaw(des_yaw):
    """
    Function for fixing the final angle of the robot, to the final position

    Args:
        des_yaw(float): the desiderd yaw
    Returns: none
    """
    err_yaw = normalize_angle(des_yaw - yaw_)
    #rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        change_state(3)

## The done function.
#
#  It stops the robot meaning that the pose has been reached
def done():
    """
    Calling this function, the robot will stop

    Args: none
    Returns: none
    """
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)


## The planning function.
#
#  It is the action callback. The script will remain inside this function, due to the while loop.
#  Here it will wait for new goal, ask the robot to reach them and delete goal if required. The state will also be updated
# @param goal It is the goal required in the action server
def planning(goal):
    """
    It is the action callback. The script will remain inside this function, due to the while loop.
    Here it will wait for new goal, ask the robot to reach them and delete goal if required. The state will also be updated

    Args:
        goal(rt2_ass2.msg.go_to_pointActionGoal): goal of the robot
    Returns: none
    """
    global state_, desired_position_,act_s,kp_a,kp_d

#   desider position and orientation acquired
    desired_position_ = Point()
    desired_position_ = goal.target_position
    des_yaw = goal.theta
    kp_d=goal.lin_vel
    kp_a=-goal.ang_vel
    state_ = 0
    rate = rospy.Rate(20)
    success = True

    feedback = rt2_ass2.msg.go_to_pointFeedback()
    result = rt2_ass2.msg.go_to_pointResult()

    while not rospy.is_shutdown():
        #during each state, the feedback is updated

        if act_s.is_preempt_requested():
            #if the goal is deleted then the system stop the robot through the function done()
            rospy.loginfo('Goal was preempted')
            act_s.set_preempted()
            done()

            success = False
            break

        # the yaw will be fixed
        elif state_ == 0:
            feedback.stat = "Fixing the yaw"
            feedback.actual_position = position_
            feedback.actual_theta=yaw_
            act_s.publish_feedback(feedback)
            fix_yaw(desired_position_)
        # The angle will be aligned
        elif state_ == 1:
            feedback.stat = "Angle aligned"
            feedback.actual_position = position_
            feedback.actual_theta=yaw_
            act_s.publish_feedback(feedback)
            go_straight_ahead(desired_position_)
        # The position is reached, so the angle will be fixed
        elif state_ == 2:
            feedback.stat = "position reached,fixing angle!"
            feedback.actual_position = position_
            feedback.actual_theta=yaw_
            act_s.publish_feedback(feedback)
            fix_final_yaw(des_yaw)
        #The target has been reached
        elif state_ == 3:
            feedback.stat = "Target reached!"
            feedback.actual_position = position_
            feedback.actual_theta=yaw_
            act_s.publish_feedback(feedback)
            done()
            break
        else:
            rospy.logerr('Unknown state!')

        rate.sleep()
    if success:
        rospy.loginfo('Goal: Succeeded!')
        act_s.set_succeeded(result)



## Documentation for the main function.
#
#  More details.
#
# @param None
def main():
    global pub_,active_, act_s
    rospy.init_node('go_to_point_action')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    act_s = actionlib.SimpleActionServer(
        '/go_to_point_action', rt2_ass2.msg.go_to_pointAction, planning, auto_start=False)
    act_s.start()



    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
