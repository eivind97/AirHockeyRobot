import time

from comProtocol import Commands
import threading

import serial
from globfile import *
import puck
from videoStream import VideoStream

class Bot():
    def __init__(self, camera, serialCom):
        self.camera = camera
        self.started = False
        self.startedLock = Lock()
        self.serialCom = serialCom
        self.last_yPos = 0
        self.last_puckEndPos = [0, 0]
        self.last_botPos = Vec2d.zero()
        self.calc_TimeDif = True
        self.time_difference = 0
        self.targetPos = [0, 0]
        self.lastVel = Vec2d.zero()
        self.puckPos = [0, 0]
        self.lastTime = 0



    def startAlgorithm(self):
        if not self.started: # if started is false
            self.startedLock.acquire() # lock thread
            self.started = True
            self.startedLock.release() # release thread
            self.thread = Thread(target=self.botThread, daemon=True) # target botThread and set it as a background thread
            self.thread.start() #start thread


    def stopAlgorithm(self):
        self.startedLock.acquire()
        self.started = False
        self.startedLock.release()


    def botThread(self):
        self.startedLock.acquire()
        started = self.started
        self.startedLock.release()
        i = 1

        puck_pos_vel_points = []
        time.sleep(3)

        while len(puck_pos_vel_points) < 1: # while table length is less then 1
            frame2, current_time, puck_pos = self.camera.get_puck_coordinates() # get frame, time and puck position from method in VideoStream.py
            if puck_pos != -1: # if puck has a found position
                puck_pos_vel_points.append([puck_pos, current_time])
                last_puck_pos = puck_pos
        puck_dir = Vec2d.zero()

        while started:
            _, current_time, puck_pos = self.camera.get_puck_coordinates()
            if puck_pos != -1:
                self.puckPos = puck_pos
                if current_time != puck_pos_vel_points[-1][1]: # if time is not equal the last time value in table

                    # add puck position and time to table
                    puck_pos_vel_points.append([puck_pos, current_time])

                    if len(puck_pos_vel_points) == 5: # if table consist of 5 item
                        i += 1
                        puck_vel = puck.velocity(puck_pos_vel_points) # find puck velocity
                        points, times, last_velocity = puck.path_points2(puck_vel, puck_pos) # get puck positions, time and last_velocity
                        # points is a 2d list [x, y] and empty when puck_vel > 0

                        botSpeed = 3000 # speed of robot (change as necessary)
                        botPos = self.serialCom.readData(Commands.GET_BOTPOS) # get the robot position from camera capture

                        collision_y = self.defence(points, last_velocity)  # point along y-axis robot should move to intercept puck
                        collision_pos = self.attack(points, botPos, times, last_velocity)

                        if collision_pos != -1 and collision_y != -1:
                            self.targetPos = collision_pos
                            if puck_pos[0] < botPos[0]:
                                self.serialCom.writeData(Commands.MOVE_TO, 120, table_center_y, 2000)
                                time.sleep(0.005)
                                if 100 < collision_y < 600 and puck_pos[0] >= 120:
                                    self.serialCom.writeData(Commands.MOVE_TO, 120, collision_y, 2000)
                                    time.sleep(0.01)
                            elif 100 < collision_y < 600 and puck_pos[0] < 800:
                                self.serialCom.writeData(Commands.MOVE_TO, 120, collision_y, 2000)
                                time.sleep(0.01)
                                if 100 < collision_pos[0] < 600 and 100 < collision_pos[1] < 600:
                                    _, current_time, puck_pos = self.camera.get_puck_coordinates()
                                    if puck_pos[0] < collision_pos[0] + (puck_vel[0]*0.1):
                                        self.serialCom.writeData(Commands.MOVE_TO, collision_pos[0], collision_pos[1], botSpeed)
                                        time.sleep(0.01)

                        puck_pos_vel_points.pop(0) # remove item at index 0
                        i = 0
                        time.sleep(0.001)

            self.startedLock.acquire() # lock thread
            started = self.started
            self.startedLock.release() # release thread

       # Algorithm for defending the goal
    def defence(self, puck_points, last_velocity):
        puck_endPoint_y = puck_points[-1][1] # y-position of puck when it reach the left wall

        if last_velocity[0] < 0: # if the puck velocity in x-direction is less than -100mm/s
            # the if statement under seams to make problems as the robot might go to center and then 50mm difference make noe meaning
            if abs(self.last_yPos - puck_endPoint_y) > 50:# if difference between current and last y-pos is more than 50 difference
                self.last_yPos = puck_endPoint_y # copy puck_endPoint_y so it can be used for next calculation
                return puck_endPoint_y

            else:
                return -1

        else:
            return -1

    # Algorithm for attack mode
    def attack(self, puck_positions, bot_pos, times, puck_vel):
        velocity = puck_vel
        if velocity[0] < 0:
            if len(puck_positions) > 1:
                puck_lastPos = Vec2d(puck_positions[-1][0], puck_positions[-1][1])
                puck_secondLastPos = Vec2d(puck_positions[-2][0], puck_positions[-2][0])
            else:
                puck_lastPos = Vec2d(puck_positions[-1][0], puck_positions[-1][1])
                puck_secondLastPos = self.puckPos

            collision_pos = (puck_lastPos + puck_secondLastPos) / 2
            if len(times) > 1:
                puck_time = times[-1] - ((times[-1] - times[-2]) / 2)
            else:
                puck_time = times[-1] / 2

            botPos = bot_pos
            bot_time = (collision_pos - botPos).length / 3000
            bot_time = bot_time * 1.05 # robot does not start at full speed
            t_diff = puck_time - bot_time # time difference to reach same position

            return collision_pos
        else:
            return -1

    def targetPosition(self):
        tPos = self.puckPos
        return tPos