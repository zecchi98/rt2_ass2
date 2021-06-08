import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import matplotlib.pyplot as plt
import jupyros as jr
import rospy
from nav_msgs.msg import Odometry
from matplotlib import animation, rc
from geometry_msgs.msg import Twist
import ipywidgets as widgets

rospy.init_node('robot_control_node')

plt.style.use('fivethirtyeight')

x_vals = []
y_vals = []
x=0
y1=0
y2=0
index = count()


def odom_callback(msg):
    global x,y1,y2
    y1=msg.pose.pose.position.y
    y2=y1
    x=msg.pose.pose.position.x
    print(x)
    print(y1)


def animate(i):
    global x,y1,y2
    plt.cla()

    plt.plot(x, y1, label='Channel 1')
    
    plt.plot(x, y2, label='Channel 2')
    plt.legend(loc='upper left')
    plt.tight_layout()


ani = FuncAnimation(plt.gcf(), animate, interval=1000)


rospy.Subscriber('/odom', Odometry, odom_callback)


plt.tight_layout()
plt.show()
