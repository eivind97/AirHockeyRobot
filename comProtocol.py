import threading
import time

from globfile import *

import serial
from enum import Enum
import re

class Commands(Enum):
    NONE = 0
    CONNECTED = 1
    RECEIVED = 2
    ILLEGAL_COMMAND = 3
    ZERO = 4
    MOVE = 5
    MOVE_TO = 6
    CENTER = 7
    BOTPATH = 8
    SOLENOID = 9
    FAN = 10
    LIGHT = 11
    GET_BOTPOS = 12
    GET_STEPPERPOS = 13
    GET_LIMIT_SWITCHES = 14
    MOVE_OUT_OF_LIMITSWITCHES = 15


class SerialCom():
    def __init__(self, comport, baudrate):
        self.comport = comport
        self.baudrate = baudrate
        self.ser = None
        self.runThread = True
        self.runThreadLock = Lock() # create thread
        self.serLock = Lock() # create thread
        self.sendtCommandLock = Lock() # create thread
        self.sema = threading.Semaphore(7) # create 7 semaphore threads
        self.sendtCommands = [] # table
        self.lastArgsLock = Lock() # create thread
        self.lastReceivedArgs = [] # table
        self.lastMessageLock = Lock() # thread
        self.lastReceivedMessage = "" # string

    def connect(self):
        self.ser = serial.Serial(self.comport, self.baudrate, timeout = 1) # create a serialport connection
        time.sleep(2) # wait for 2 ms
        Thread(target=self.readingThread, daemon=True).start() # start thread
        self.emptyCom()
        self.writeData(Commands.CONNECTED) # write command connected using method writeData
        arduinoConnected = False
        arduinoConnected = self.waitForResponse(Commands.CONNECTED) # boolean value to indicate if raspberry pi is connected to arduino

        return arduinoConnected

    def waitForResponse(self, command):
        '''
        This method is blocking something
        '''

        receivedResponse = False
        while not receivedResponse:
            self.sendtCommandLock.acquire()
            if not command.value in self.sendtCommands:
                receivedResponse = True
            self.sendtCommandLock.release()

            time.sleep(0.01)
        return receivedResponse

    def readingThread(self):
        runThreadBuff = True
        newMessage = False
        line = ""

        while runThreadBuff: # while true
            self.runThreadLock.acquire() # start thread
            runThreadBuff = self.runThread
            self.runThreadLock.release() # stop thread

            self.serLock.acquire() # start thread

            if self.ser.inWaiting() > 0 and runThreadBuff: # if data is being received and runThreadBuff is true
                line = self.ser.readline().decode('utf-8').rstrip() # read data from port
                newMessage = True
                self.serLock.release() # release thread

            if newMessage: # if message received
                self.lastMessageLock.acquire()  # start thread
                self.lastReceivedMessage = ""
                command, arguments = self.extractData(line) # get command and arguments from received data
                if command != Commands.NONE: # if a command is received
                    if self.checkIfReturnValue(command):
                        self.lastArgsLock.acquire()
                        self.lastReceivedArgs = arguments # save arguments is static list
                        self.lastArgsLock.release()
                    self.sendtCommandLock.acquire()

                    try:
                        self.sendtCommands.remove(command.value) # remove number indicating command from commandlist
                    except:
                        print("received wrong command: ", command)
                        print(self.sendtCommands)

                    self.sendtCommandLock.release() # release thread
                    self.sema.release() # release thread

                else:
                    self.lastReceivedMessage = line # received command.NONE

                newMessage = False
                self.lastMessageLock.release()
            time.sleep(0.1)


    def getLastReceivedMessage(self): # retrive the last string received from arduino
        self.lastMessageLock.acquire() # start thread
        message = self.lastReceivedMessage
        self.lastMessageLock.release() # release thread
        return message


    def writeBotpath(self, botpathPoints, botpathSpeeds):
        data = f"<{Commands.BOTPATH}: "
        pointArgument = ""
        speedArgument = ""

        for i in range(len(botpathPoints)):
            pointArgument += f"{botpathPoints[i][0]}, {botpathPoints[i][1]}" # turn coordinates into text
            speedArgument += f"{botpathSpeeds[i]}," # turn speed into text

        data += pointArgument[:-1] + "|" + speedArgument[:-1] + ">" # create text with all positions and speeds

        return data

    def readData(self, command): # Bad naming. It does not read data but ask for data that should be read by another function
        # try to place code under writeData function
        args = []
        if self.checkIfReturnValue(command): # if one of specific commands is received
            data = f"<{command.value}>"  # create a text of the received command number
            self.sendtCommandLock.acquire() # start thread
            self.sendtCommands.append(command.value) # add command value to end of table
            self.sendtCommandLock.release() # release string

            self.sema.acquire()
            self.ser.write((data.encode('utf-8'))) # Sendt data to arduino

            self.waitForResponse(command) # check if arduino respond to command
            self.lastArgsLock.acquire()
            args = self.lastReceivedArgs
            self.lastArgsLock.release()

        else:
            print("Use writeData")

        return args

    def writeData(self, command, *args):
        if self.checkIfReturnValue(command): # if specific command is received
            print("Use readData")
        else:
            data = f"<{command.value}: " # create a text of the command value
            self.sendtCommandLock.acquire()
            self.sendtCommands.append(command.value)
            self.sendtCommandLock.release()

            for arg in args: # for each argument found in arguments list
                data += f"{arg}, " # create text with all arguments
            data = data[:-1] + ">" # remove last argument from text

            self.sema.acquire() # start thread
            self.ser.write((data.encode('utf-8'))) # encrypt code


    def extractData(self, line):
        command = Commands.NONE
        arguments = []
        data = re.search("<([0-9]+):?(-?[a-zA-Z0-9\.\, -]*)>", line) # split string into 2 groups by semicolon separation

        if data != None: # if data contains something
            command = Commands(int(data.group(1))) # find command based on command value
            arguments_string = data.group(2).split(',') # find arguments from decrypted code
            if self.checkIfReturnValue(command): # check if specific commands are received
                if command == Commands.GET_LIMIT_SWITCHES:
                    arguments = [int(x) for x in arguments_string] # make each argument an integer value
                else:
                    arguments = [float(x) for x in arguments_string] # make each argument a float value

            else: print("Incoming line: " + line)

            return command, arguments


    def checkIfReturnValue(self, command):
        if command == Commands.GET_BOTPOS or command == Commands.GET_STEPPERPOS or command == Commands.GET_LIMIT_SWITCHES:
            return True
        else:
            return False


    def emptyCom(self):
        self.serLock.acquire()
        self.lastArgsLock.acquire()
        self.ser.flushInput()
        self.ser.flushOutput()
        self.lastReceivedArgs = []
        self.sema = threading.Semaphore(7)
        time.sleep(1)
        self.lastArgsLock.release()
        self.serLock.release()









