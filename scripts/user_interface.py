import rospy
import time
from rt2_ass2.srv import Command
def main():
    robot_moving=False;
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(10)
    rate = rospy.Rate(20)
    x=0
    while not rospy.is_shutdown() and x!=-1:
        x = int(input("\nFrom now on, you can press '0','1','-1' whenever you want.\nPress 0 to stop the robot\nPress 1 to start the robot\nPress -1 to close the user interface\n\n\n"))
        if (x == 1):
            ui_client("start")
            robot_moving=True
        elif (x==0 and robot_moving):
            ui_client("stop")
            robot_moving=False
            
if __name__ == '__main__':
    main()
