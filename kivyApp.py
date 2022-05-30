import time
from os import truncate
from pickle import TRUE, FALSE

import numpy as np

from comProtocol import SerialCom, Commands
from videoStream import VideoStream
from globfile import *
from bot import Bot
import threading
from kivy.app import App
import cv2
from kivy.core import window
import kivy
from kivy.uix.widget import Widget
from kivy.properties import ObjectProperty
from kivy.uix.label import Label
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.lang import Builder
from kivy.uix.boxlayout import BoxLayout
from kivy.core.window import Window
from kivy.graphics.texture import Texture
from kivy.clock import Clock
from kivy.uix.image import Image
from kivy.uix.popup import Popup
from kivy.uix.progressbar import ProgressBar
from kivy.properties import NumericProperty
from kivy.config import Config

Config.set('graphics', 'width', '800')
Config.set('graphics', 'height', '480')
Config.write()

# Serial object to communicate
try:
    serialCom = SerialCom('/dev/ttyACM0', 115200) # port connection between raspberry pi and arduino
    serialCom.connect() # try to connect to the port
except:
    print("Could not open port")


camera = VideoStream()
camera.start()
robot = Bot(camera, serialCom) # robot connection

done_zeroing = False
ROI_found = False


class InitializeWindow(Screen): # first window to appear when program start
    font_size = 25
    numOfSearches = 0
    bot_covering = True # Assume bot is covering aruco markers
    start_zeroing = False
    limit_switches_pressed = [0, 0, 0, 0]
    color_red = 1,0,0
    color_green = 0,1,0
    color_white = 1,1,1

    # Find ROI
    def on_enter(self, *args):
        self.arucoClock()


    def arucoClock(self):
        self.clock_interval = Clock.schedule_interval(self.findAruco, 0.3) # run findAruco every 0.3 seconds


    def findAruco(self, *args):
        not_done = True
        self.numOfSearches += 1 # add 1 to number of searches
        self.ids.progressbar.value += 0.015 # increase progressbar by 1.5% for each run

        aruco_corner, roi_found = camera.corner_detection()  # gives feeback on found aruco markers and roi detection

        if roi_found: # if all corners where found
            global ROI_found # make ROI_found global
            ROI_found = True
            not_done = False
            self.ids.roi.text = "Found" # create text for box
            self.ids.roi.color = self.color_green
            self.ids.roi.text = ""
            self.numOfSearches = 0
            serialCom.writeData(Commands.MOVE, 0, 25, 100) # move downward until all aruco markers are found
            self.ids.progressbar.value = 0.6 # go to 60% finished
            time.sleep(0.2)
            self.zeroClock() # run zeroClock method

        elif self.numOfSearches%10 == 0 and self.bot_covering: # if the reminder after numOfSearches is divided by 10 is zero and botcovering is true
            bot_pos = camera.get_robot_coordinates_full_fov() # get robot center position from method

            if bot_pos != -1: # if a robot position is found
                if bot_pos[0] < 80: # if robot x-position is less than 80
                    x_dist = (80-bot_pos[0]) * 2.5
                    serialCom.writeData(Commands.MOVE, x_dist, 0, 100) # move along x-direction for each run
                self.bot_covering = False # robot is not covered

            else:
                limit_switches = serialCom.readData(Commands.GET_LIMIT_SWITCHES) # connect to all limit switches
                if limit_switches[2] == 0: #If limit switch 2 is not pressed
                    serialCom.writeData(Commands.MOVE_OUT_OF_LIMITSWITCHES) # move away from switch
                    self.bot_covering = False
                    self.limit_switches_pressed[2] = 1 # if limit switch is pressed

                else:
                    serialCom.writeData(Commands.MOVE, 50, 0, 100) # move robot forward
                    time.sleep(0.2)

        else:
            self.ids.roi_id.text = "" # empty text box indicating aruco marker ids
            i = 0
            for corner in aruco_corner: # for each aruco corner marker
                if corner == True: # if aruco corner marker is found
                    self.ids.roi_id.text += str(i) + " " # add id to text
                i += 1

        if self.numOfSearches > 30: # if aruco markers is searched for more then 30 times
            not_done = False
            self.parent.current = "Main" # go to main window
        return not_done

    # Zeroing
    def zeroClock(self):
        self.clock_interval = Clock.schedule_interval(self.zeroing, 0.5) # run zeroing every 0.5sec

    def zeroing(self, *args):
        bot_pos = camera.get_robot_coordinates_roi() # get robot position
        self.numOfSearches += 1 # add 1 to numOfSearhces
        self.ids.progressbar.value += 0.02 # increase progressbar
        not_done = True # roi not found

        if self.start_zeroing == True or self.numOfSearches == 20: # if robot has started zeroing or number of searches reaches 20
            self.ids.progressbar.value = 1.0 # Fill progressbar (finished status)
            serialCom.writeData(Commands.ZERO) # go to zero position
            while serialCom.getLastReceivedMessage() != "Done: zero": # While not receiveing done zeroing message
                time.sleep(0.1)
            serialCom.writeData(Commands.CENTER, 600) # go to center position
            time.sleep(1) # wait for 1 second
            global done_zeroing # create global variable
            done_zeroing = True
            not_done = False # done
            self.parent.current = "Main" # go to main

        if bot_pos != -1: # if robot postion is found
            x_dist = 0.0
            y_dist = 0.0
            if bot_pos[0] > 75: # if robot is more than 75mm from left wall
                x_dist = 75-bot_pos[0] # distance to min position
            if bot_pos[1] > 75: # if robot is more than 75mm from top wall
                y_dist = 75-bot_pos[1] # distance from min top position

            if x_dist < 0 or y_dist < 0:
                serialCom.writeData(Commands.MOVE, x_dist, y_dist, 200) # move robot x_dist along x and y_dist along y
                waitingTime = math.sqrt(x_dist**2 + y_dist**2)/200 + 0.1 # time to finish
                time.sleep(waitingTime) # wait
            self.ids.robot.text = "Found" # change robot textbox to Found
            self.ids.robot.color = self.color_green # change text color to green

            self.start_zeroing = True # make it possible to start zeroing

        elif self.numOfSearches%4 == 0: # if numOfSearches divided by 4 has zero remainder
            limit_switches = serialCom.readData(Commands.GET_LIMIT_SWITCHES) # get all limit switches
            move_out_of_limit_switches = False # it is not necessary for robot to move away from limit switches

            if limit_switches[0] == 0: # if not pressed
                print("OK")
                self.limit_switches_pressed[0] = 1
                move_out_of_limit_switches = True

            if limit_switches[3] == 0: # if not pressed
                self.limit_switches_pressed[3] = 1
                move_out_of_limit_switches = True

            if move_out_of_limit_switches: # if limit switch is pressed
                serialCom.writeData(Commands.MOVE_OUT_OF_LIMITSWITCHES) # move away from limit switches
                time.sleep(0.2)

            if self.limit_switches_pressed[3] == 1: # if pressed
                x_dist = 5
            else:
                x_dist = -5

            if self.limit_switches_pressed[0] == 1: # if pressed
                y_dist = 5
            else:
                y_dist = -5

            if not move_out_of_limit_switches: # if false
                serialCom.writeData(Commands.MOVE, x_dist, y_dist, 50)
            time.sleep(0.1)

        if self.start_zeroing == True or self.numOfSearches == 19:
            self.ids.zero.text = "Starting"
            self.ids.zero.color = self.color_green
            if self.ids.robot.text == "Waiting":
                self.ids.robot.text = "Not found"
                self.ids.robot.color = self.color_red
            self.ids.progressbar.value = 0.85 # increase progressbar to 85%

        return not_done


class MainWindow(Screen):
    font_size_buttons = 20

    def quitApp(self):
        App.get_running_app().stop() # stop program
        Window.close() # exit HMI


class PlayGameWindow(Screen):
    font_size_score = 175
    font_size = 20
    font_size_buttons = 25

    def on_leave(self, *args):
        self.stopGame()


    def score(self, increment, robot=False):
        score = 0
        if robot: # if true
            score = int(self.ids.score_robot.text) # read robot score text as integer
        else:
            score = int(self.ids.score_player.text) # read player score text as integer

        if increment == 1:
            score += 1
        else:
            score -= 1

        if score <= 0:
            score = 0
        elif score >= 12:
            score = 12

        if robot:
            self.ids.score_robot.text = "{}".format(score) # write robot score to textbox
        else:
            self.ids.score_player.text = "{}".format(score) # write player score to textbox

    def restartGame(self):
        self.ids.score_player.text = "0" # sett player score to zero
        self.ids.score_robot.text = "0" # sett robot score to zero


    def startGame(self):
        serialCom.writeData(Commands.FAN, 1) # Start fans
        serialCom.writeData(Commands.MOVE_TO, 100, table_center_y, 1000) # move to center position
        robot.startAlgorithm() # start robot program
        self.ids.start_game.disabled = False # disable start game button

    def stopGame(self):
        serialCom.writeData(Commands.FAN, 0) # turn off fans
        serialCom.writeData(Commands.MOVE_TO, 200, table_center_y, 500) # move to center
        time.sleep(2) # wait for 2 seconds such that robot can move to center
        robot.stopAlgorithm() # stop robot program
        self.ids.start_game.disabled = True # enable button
        self.ids.stop_game.disabled = False # disable button


    def solenoid_btn(self, push):
        serialCom.writeData(Commands.SOLENOID, push)


class LiveCalculationWindow(Screen):
        pass


class ColorDetectionWindow(Screen):
    lock = threading.Lock()
    font_size = 20
    font_size_buttons = 25
    color_blue = 0,0,1
    color_yellow = 1,1,0
    color_green = 0,1,0

    hue_min = 50
    hue_max = 170
    sat_min = 150
    sat_max = 255
    val_min = 0
    val_max = 255

    def on_enter(self, *args):
        self.getColors()
        self.startClock()


    def on_leave(self, *args):
        self.stopClock()


    def startClock(self):
        self.clock_interval = Clock.schedule_interval(self.update, 0.1) # run update every 0.1 sec


    def stopClock(self, *args):
        self.clock_interval.cancel()


    def update(self, *args):
        frame = camera.frame # get frame from camera
        imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # turn image into HSV color format
        lower = np.array([self.hue_min, self.sat_min, self.val_min]) # lower HSV limit
        upper = np.array([self.hue_max, self.sat_max, self.val_max]) # upper HSV limit
        frame = cv2.inRange(imgHSV, lower, upper) # find objects in picture

        texture = Texture.create(size=(frame.shape[1], frame.shape[0]), colorfmt='luminance') # create luminance texture with size shape
        # if possible use colorfmt='rgba', but use 'bgr' in buffer
        texture.blit_buffer(frame.tobytes(order=None), colorfmt='luminance', bufferfmt='ubyte')
        texture.flip_vertical()
        self.ids.image.texture = texture # show live image of texture


    def getColors(self):
        try:
            f = open('test_color.txt', 'r') # import file with color values
            data = []
            read = f.readline() # read line from file
            doneReading = False

            while(not doneReading): # while false
                data.append(read) # add line to end of table
                read = f.readline()

                if "END" in read: # if the letter commbination END is found in read
                    data.append(read) # add read to end of table
                    f.close() # stop receiving text
                    doneReading = True
                    break # break from while loop
            self.lock.acquire() # lock thread
            for color in data: # for each color information in data
                if 'LAST' in color: # if LAST is read from color
                    self.last_col = color[color.find("LAST: ") + len("LAST: "): color.find(";")] # last color text
                    self.last_col_val = self.last_col.split(" ") # split text into words
                    self.hue_min = int(self.last_col_val[0]) # get hue_min from text
                    self.hue_max = int(self.last_col_val[1])
                    self.sat_min = int(self.last_col_val[2])
                    self.sat_max = int(self.last_col_val[3])
                    self.val_min = int(self.last_col_val[4])
                    self.val_max = int(self.last_col_val[5])

                    # Insert info into textboxes and sliders
                    self.ids.hue_min_label.text = "Hue min: " + str(self.hue_min)
                    self.ids.hue_min_slider.value = self.hue_min
                    self.ids.hue_max_label.text = "Hue max: " + len(self.hue_max)
                    self.ids.hue_max_slider.value = self.hue_max

                    self.ids.sat_min_label.text = "Sat min: " + str(self.sat_min)
                    self.ids.sat_min_slider.value = self.sat_min
                    self.ids.sat_max_label.text = "Sat max: " + str(self.sat_max)
                    self.ids.sat_max_slider.value = self.sat_max

                    self.ids.val_min_label.text = "Val min: " + str(self.val_min)
                    self.ids.val_min_slider.value = self.val_min
                    self.ids.val_max_label.text = "Val max: " + str(self.val_max)
                    self.ids.val_max_slider.value = self.val_min

                if 'BLUE' in color:
                    self.blue_col = color[color.find("BLUE: ") + len("BLUE: "): color.find(";")]
                    blue_col_val = self.blue_col.split(" ")
                    self.hue_min_blue = int(blue_col_val[0])
                    self.hue_max_blue = int(blue_col_val[1])
                    self.sat_min_blue = int(blue_col_val[2])
                    self.sat_max_blue = int(blue_col_val[3])
                    self.val_min_blue = int(blue_col_val[4])
                    self.val_max_blue = int(blue_col_val[5])

                if 'YELLOW' in color:
                   self.yellow_col = color[color.find("YELLOW: ") + len("YELLOW: "): color.find(";")]
                   yellow_col_val = self.yellow_col.split(" ")
                   self.hue_min_yellow = int(yellow_col_val[0])
                   self.hue_max_yellow = int(yellow_col_val[1])
                   self.sat_min_yellow = int(yellow_col_val[2])
                   self.sat_max_yellow = int(yellow_col_val[3])
                   self.val_min_yellow = int(yellow_col_val[4])
                   self.val_max_yellow = int(yellow_col_val[5])

                if 'GREEN' in color:
                    self.green_col = color[color.find("GREEN: ") + len("GREEN: "): color.find(";")]
                    green_col_val = self.green_col.split(" ")
                    self.hue_min_green = int(green_col_val[0])
                    self.hue_max_green = int(green_col_val[1])
                    self.sat_min_green = int(green_col_val[2])
                    self.sat_max_green = int(green_col_val[3])
                    self.val_min_green = int(green_col_val[4])
                    self.val_max_green = int(green_col_val[5])

            self.lock.release() # release thread
        except:
            print("Failed to get colors from file")


    def updateColors(self):
        try:
            self.lock.acquire()
            self.last_col = "{0} {1} {2} {3} {4} {5}".format(self.hue_min, self.hue_max, self.sat_min, self.sat_max, self.val_min, self.val_max)
            data = [
                "COLOR: h_min h_max s_min s_max v_min v_max;",
                " ",
                "LAST: " + self.last_col + ";",
                "BLUE: " + self.blue_col + ";",
                "YELLOW: " + self.yellow_col + ";",
                "GREEN: " + self.green_col + ";",
                "END"]

            f = open('test_color.text', 'w') # edit color text file

            for l in range(len(data)):
                f.write(data[l] + "\n")
            f.close()
            self.lock.release()
        except:
            print("Failed to write colors from file")

   # Unessecary many methods for color detection, try to make into 1 method
    def changeHSVcolor(self, color): # update color based on changed slider values
        if color == "LAST":
            self.hue_min = int(self.last_col_val[0])
            self.hue_max = int(self.last_col_val[1])
            self.sat_min = int(self.last_col_val[2])
            self.sat_max = int(self.last_col_val[3])
            self.val_min = int(self.last_col_val[4])
            self.val_max = int(self.last_col_val[5])

        elif color == "BLUE":
            self.hue_min = self.hue_min_blue
            self.hue_max = self.hue_max_blue
            self.sat_min = self.sat_min_blue
            self.sat_max = self.sat_max_blue
            self.val_min = self.sat_min_blue
            self.val_max = self.val_max_blue

        elif color == "YELLOW":
            self.hue_min = self.hue_min_yellow
            self.hue_max = self.hue_max_yellow
            self.sat_min = self.sat_min_yellow
            self.sat_max = self.sat_max_yellow
            self.val_min = self.val_min_yellow
            self.val_max = self.val_max_yellow

        elif color == "GREEN":
            self.hue_min = self.hue_min_green
            self.hue_max = self.hue_max_green
            self.sat_min = self.sat_min_green
            self.sat_max = self.sat_max_green
            self.val_min = self.val_min_green
            self.val_max = self.val_max_green

        else:
            self.hue_min = self.ids.hue_min_slider.value
            self.hue_max = self.ids.hue_max_slider.value
            self.sat_min = self.ids.sat_min_slider.value
            self.sat_max = self.ids.sat_max_slider.value
            self.val_min = self.ids.val_min_slider.value
            self.val_max = self.ids.val_max_slider.value

        self.ids.hue_min_label.text = "Hue min: " + str(self.hue_min)
        self.ids.hue_min_slider.value = self.hue_min
        self.ids.hue_max_label.text = "Hue max: " + len(self.hue_max)
        self.ids.hue_max_slider.value = self.hue_max

        self.ids.sat_min_label.text = "Sat min: " + str(self.sat_min)
        self.ids.sat_min_slider.value = self.sat_min
        self.ids.sat_max_label.text = "Sat max: " + str(self.sat_max)
        self.ids.sat_max_slider.value = self.sat_max

        self.ids.val_min_label.text = "Val min: " + str(self.val_min)
        self.ids.val_min_slider.value = self.val_min
        self.ids.val_max_label.text = "Val max: " + str(self.val_max)
        self.ids.val_max_slider.value = self.val_min


    # Not in use, consider to remove
    def slide_hue_min(self, *args):
        self.hue_min = int(args[1])
        self.ids.hue_min_label.text = "Hue min: " + str(self.hue_min)


    def slide_hue_max(self, *args):
        self.hue_max = int(args[1])
        self.ids.hue_max_label.text = "Hue max: " + str(self.hue_max)


    def slide_sat_min(self, *args):
        self.sat_min = int(args[1])
        self.ids.sat_min_label.text = "Sat min: " + str(self.sat_min)


    def slide_sat_max(self, *args):
        self.sat_max = int(args[1])
        self.ids.sat_max_label.text = "Sat max: " + str(self.sat_max)


    def slide_val_min(self, *args):
        self.val_min = int(args[1])
        self.ids.val_min_label.text = "Val min: " + str(self.val_min)


    def slide_val_max(self, *args):
        self.val_max = int(args[1])
        self.ids.val_max_label.text = "Val max: " + str(self.val_max)


    def help_btn(self):
        popUpHelp() # start method


class CalibrationWindow(Screen):
    font_size_pos = 55
    font_size_buttons = 25
    axis = "x"
    color_red = 1,0,0
    color_green = 0,1,0
    color_black = 0,0,0
    color_white = 1,1,1
    doneHoming = False

    def on_enter(self, *args):
        self.startClock() # start method


    def on_leave(self, *args):
        self.stopClock()


    def startClock(self):
        self.clock_interval = Clock.schedule_interval(self.update, 0.5)


    def stopClock(self, *args):
        self.clock_interval.cancel()


    def update(self, *args):
        global done_zeroing
        global ROI_found

        if ROI_found == False: # if not found

            aruco_corner, ROI_found = camera.corner_detection() # get aruco markers and roi found or not

            if ROI_found == False: # if still not found
                i = 0
                self.ids.roi_id.text = ""
                self.ids.roi.text = "Not found"
                self.ids.roi.color = self.color_red
                for corner in aruco_corner:
                    if corner == True:
                        self.ids.roi_id.text += str(i) + " " # add whitch markers is found
                    i += 1

        if ROI_found == True: # if found
            self.ids.roi.text = "Found"
            self.ids.roi.color = self.color_green
            self.ids.roi_id.text = ""

        if ROI_found == True and done_zeroing == True:
            self.ids.moveTo_btn.disabled = False # enable button

        if ROI_found == True:
            aruco_robot = camera.robot_detection() # robot status

            if aruco_robot == True: # if robot found
                self.ids.robot_aruco.text = "Found"
                self.ids.robot_aruco.color = self.color_green
            else:
                self.ids.robot_aruco.text = "Not Found"
                self.ids.robot_aruco.color = self.color_red

        if serialCom.getLastReceivedMessage() == "Done: zero":
            self.ids.moveTo_btn.disabled = False # enable button
            self.doneHoming = True
            done_zeroing = True

        if done_zeroing:
            bot_pos = serialCom.readData(Commands.GET_BOTPOS) # get robot position
            self.ids.pos_x.text = str(int(bot_pos[0]) + 1)
            self.ids.pos_y.text = str(int(bot_pos[1]) + 1)


    def numPadPress(self, number):
        x = int(self.ids.text_x_pos.text) # robot x pos
        y = int(self.ids.text_y_pos.text) # robot y pos

        if x == 0 and self.axis == "x":
            if number == -1: # if not an accepted number is inserted
                self.ids.text_x_pos.text = "0" # set x pos textbox to 0
            else:
                self.ids.text_x_pos.text = str(number) # set x pos textbox to input

        elif y == 0 and self.axis == "y":
            if number == -1:
                self.ids.text_y_pos.text = "0"
            else:
                self.ids.text_y_pos.text = str(number)

        else:
            if self.axis == "x": # change x pos
                if number >= 0:
                    self.ids.text_x_pos.text += str(number) # add number to textbox
                else:
                    self.ids.text_x_pos.text = "0"

                x = int(self.ids.text_x_pos.text)

                if len(self.ids.text_x_pos.text) > 5: # if length of x is more than 5
                    self.ids.text_x_pos.text = str(number) # x pos textbox is equal to the input number

                if x < 70 or x > 620: # if x is outside robot playing area
                    self.ids.text_x_pos.foreground_color = self.color_red
                else:
                    self.ids.text_x_pos.foreground_color = self.color_green

            else: # change y pos
                if number >= 0:
                    self.ids.text_y_pos.text += str(number)
                else:
                    self.ids.text_y_pos.text = "0"

                y = int(self.ids.text_y_pos.text)

                if len(self.ids.text_y_pos.text) > 5:
                    self.ids.text_y_pos.text = str(number)

                if y < 65 or y > 635:
                    self.ids.text_y_pos.foreground_color = self.color_red
                else:
                    self.ids.text_y_pos.foreground_color = self.color_green

            if x < 70 or x > 620 or y < 65 or y > 635:
                self.ids.moveTo_btn.background_color = self.color_red
            else:
                self.ids.moveTo_btn.background_color = self.color_green


    def chooseXYvalue(self, axis):
        if axis == 0:
            self.axis = "x"
            self.ids.text_x_pos.text = "0"
            self.ids.text_x_pos.foreground_color = self.color_red
        else:
            self.axis = "y"
            self.ids.text_y_pos.text = "0"
            self.ids.text_y_pos.foreground_color = self.color_red

    def moveTo_btn(self):
        x = int(self.ids.text_x_pos.text)
        y = int(self.ids.text_y_pos.text)
        speed = 150

        if x < 70 or x > 620 or y < 65 or y > 650: # if outside of area
            popUpMoveTo() # show popup window
        else:
            serialCom.writeData(Commands.MOVE_TO, int(self.ids.text_x_pos.text), int(self.ids.text_y_pos.text), speed) # Move robot to choosen position


    def homing(self):
        global done_zeroing

        serialCom.writeData(Commands.ZERO) # start zeroing command
        self.doneHoming = False
        done_zeroing = False
        print("Test")


class SettingsWindow(Screen):
    pass


class WindowManager(ScreenManager):
    pass


class MoveToPopupWindow(FloatLayout):
    pass


class HelpPopupWindow(FloatLayout):
    pass


def testenoe(object):
    for _ in range(3): # for all in range 3
        a = input("Test: ")
        object.counter = int(a)
        object.ids.lbl.text = "{}".format(object.counter)


def popUpMoveTo():
    show = MoveToPopupWindow()
    popupWindow = Popup(title="WARNING", content=show, size_hint=(None,None), size=(400, 250))
    popupWindow.open()


def popUpHelp():
    show = HelpPopupWindow()
    popupWindow = Popup(title="HELP", content=show, size_hint=(None, None), size=(500, 250))
    popupWindow.open()


class MyApp(App):
    Window.clearcolor = (0.15, 0.15, 0.15, 1)








