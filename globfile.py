import cv2
import numpy as np
import cv2.aruco as aruco
from threading import Thread, Lock
from datetime import datetime
from pymunk import Vec2d
import time
import math

# Table constants
table_width = 1600.0
table_height = 700.0
table_center_x = table_width/2
table_center_y = table_height/2
camera_height = 1055.0
goal_size = 300.0

#Puck constants
puck_radius = 69.2/2
workAreaX = 700.0
workAreaY = table_height

# Puck max position
puck_x_minPos = puck_radius
puck_x_maxPos = workAreaX - puck_radius
puck_y_minPos = puck_radius
puck_y_maxPos = workAreaY - puck_radius


# Pusher
pusher_height = 95.0
pusher_radius = 95/2

# Pusher max values
pusher_y_minPos = 65.0
pusher_y_maxPos = table_height - 65.0
pusher_x_minPos = 70.0
pusher_x_maxPos = 620.0
