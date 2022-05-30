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
        j = 0
        k = 0
        ii = 0

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
            self.puckPos = puck_pos
            if puck_pos != -1:
                if current_time != puck_pos_vel_points[-1][1]: # if time is not equal the last time value in table

                    # add puck position and time to table
                    puck_pos_vel_points.append([puck_pos, current_time])
                    j += 1
                    k += 1

                    if len(puck_pos_vel_points) == 5: # if table consist of 5 item
                        i += 1
                        puck_vel = puck.velocity(puck_pos_vel_points) # find puck velocity
                        points, times, last_velocity = puck.path_points2(puck_vel, puck_pos) # get puck positions, time and last_velocity
                        # points is a 2d list [x, y] and empty when puck_vel > 0
                        # puck_points2 gives the assumed path of the puck, time it takes to reach each position and velocity when reaching left most position

                        botSpeed = 3000 # speed of robot (change as necessary)
                        botPos = self.serialCom.readData(Commands.GET_BOTPOS)  # get the robot position from motor angles

                        collision_y = self.defence(points, last_velocity)  # point along y-axis robot should move to intercept puck
                        collision_pos, slope, col_start, col_stop = self.attack(points, botPos, times, last_velocity) # collision position, puck slope, collision start pos, collision end pos

                        if type(collision_pos) == Vec2d:
                            self.targetPos = collision_pos

                        if puck_pos[0] < botPos[0]:
                            self.serialCom.writeData(Commands.MOVE_TO, 100, table_center_y, 3000)  # send command move_to, x-pos=100, y-pos=center, speed
                            self.last_botPos = Vec2d(100, table_center_y)
                        elif collision_pos != -1: # if a collision position is calculated
                            if 140 < collision_pos[0] < 600 and 120 < collision_pos[1] < 580 and last_velocity[0] < 0 \
                                    and col_start[0] > 80 and 100 < col_start[1] < 600:
                                self.serialCom.writeData(Commands.MOVE_TO, col_start[0], col_start[1], 2500)
                                self.last_botPos = Vec2d(col_start[0], col_start[1])
                                if puck_pos[0] < (collision_pos[0] + 250) and col_stop[0] > col_start[0] \
                                        and 100 <= col_stop[0] <= 600 and col_stop[1] <= 600:
                                    self.serialCom.writeData(Commands.MOVE_TO, col_stop[0], col_stop[1], 2500)
                                    self.last_botPos = Vec2d(col_stop[0], col_stop[1])
                                    time.sleep(0.05)
                                else:
                                    self.serialCom.writeData(Commands.MOVE_TO, 100, table_center_y, 3000) # send command move_to, x-pos=100, y-pos=center, speed
                                    self.last_botPos = Vec2d(100, table_center_y)
                                    time.sleep(0.05)
                            else:
                                self.serialCom.writeData(Commands.MOVE_TO, 100, table_center_y, 3000) # send command move_to, x-pos=100, y-pos=center, speed
                                self.last_botPos = Vec2d(100, table_center_y)
                                time.sleep(0.05)

                        #elif self.time_difference <= 2.0:
                         #   self.serialCom.writeData(Commands.MOVE_TO, pusher_bottomPos + 40, round(collision_y, 2), botSpeed)
                          #  time.sleep(time_dif)
                           # if last_velocity[1] < 0:
                            #    self.serialCom.writeData(Commands.MOVE_TO, collision_pos[0] + 20, collision_pos[1] + 20, botSpeed)
                           # elif last_velocity[1] > 0:
                            #    self.serialCom.writeData(Commands.MOVE_TO, collision_pos[0] + 20, collision_pos[1] - 20, botSpeed)
                            #else:
                             #   self.serialCom.writeData(Commands.MOVE_TO, collision_pos[0] + 20, collision_pos[1], botSpeed)

                       # elif -5 <= last_velocity[0] <= 5 and collision_pos[0] > 100:
                           # self.serialCom.writeData(Commands.MOVE_TO, collision_pos[0]-10, table_center_y, 2000)
                           # time.sleep(0.01)


                        puck_pos_vel_points.pop(0) # remove item at index 0
                        i = 0
                        time.sleep(0.001)

            self.startedLock.acquire() # lock thread
            started = self.started
            self.startedLock.release() # release thread

       # Algorithm for defending the goal
    def defence(self, puck_points, last_velocity):
        puck_endPoint_y = puck_points[-1][1] # y-position of puck when it reach the left wall

        if last_velocity[0] < -100: # if the puck velocity in x-direction is less than -100mm/s
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
            if len(puck_positions) >= 2:
                puck_lastPos = Vec2d(puck_positions[-1][0], puck_positions[-1][1])
                puck_secondLastPos = Vec2d(puck_positions[-2][0], puck_positions[-2][0])
            else:
                puck_lastPos = Vec2d(puck_positions[-1][0], puck_positions[-1][1])
                _, current_time, puck_pos = self.camera.get_puck_coordinates()
                if type(puck_pos) == Vec2d:
                    puck_secondLastPos = puck_pos
                else:
                    puck_secondLastPos = Vec2d.zero()

            collision_pos = (puck_lastPos + puck_secondLastPos) / 2
            collision_start_pos = (collision_pos + puck_lastPos) / 2
            collision_stop_pos = (collision_pos + puck_secondLastPos) / 2
            slope = (puck_lastPos[1] - puck_secondLastPos[1]) / (puck_lastPos[0] - puck_secondLastPos[0])
            if len(times) >= 2:
                puck_time = times[-1] - ((times[-1] - times[-2]) / 2)
            else:
                puck_time = times[-1] / 2

            botPos = bot_pos
            bot_time = (collision_pos - botPos).length / 3000
            bot_time = bot_time * 1.05 # robot does not start at full speed
            t_diff = puck_time - bot_time # time difference to reach same position

            return collision_pos, slope, collision_start_pos, collision_stop_pos
        else:
            return -1


    # Algoritm for making the robot attack and defende at the same time
    def attackDefense(self, bot_pos, puck_positions, botpos):
        pass


    def targetPosition(self):
        tPos = self.puckPos
        return tPos