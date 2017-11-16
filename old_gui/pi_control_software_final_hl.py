# Kevin Tarczali, Steven Mitchell, Dylan Conroy     9 August 2017
# pi_control_software.py     version 2

# List of States
# 0  Idle       |   2   Ready           |   3   Pushing          |
# 4  Coasting   |   5   Braking         |   6   Disengage Brakes |
# 7 Power Off   |   11  Fault Brakes    | 12 Fault No Brakes     |

# List of Inputs
# 0 None | 1 Insert Pod |   2   Start | 3   Power Off | 4   Engage Brakes | 5 Disengage Brakes

# Add buttons for actuators
# Add tester GUI
# Add all sensor tests

# Imports
import socket
import struct
import random
import serial
import time
import math
import RPi.GPIO as GPIO
from vnpy import *

# Constants
ACCELERATION_THRESHOLD = 5  # threshold at which pod can be determined to be accelerating, m/s^2
DECELERATING_THRESHOLD = 0  # threshold at which pod can be determined to be deccelerating, m/s^2
IN_MOTION_THRESHOLD = 2  # threshold at which pod can be determined to be in motion, m/s
INTERRUPT_PIN = 18
MAX_AMPERAGE = 50  # maximum safe amperage, w
MIN_AMPERAGE = 0  # minimum safe amperage, w
MAX_DISTANCE = 3000  # account for dift here           # maximum distance pod can travel before brakes engage automatically, m
MAX_TAPE_COUNT = 50  # maximum tape reads that can be read before brakes engage automatically, #
MAX_TEMPERATURE_AMBIENT = 60  # maximum safe ambient temperature reading in the pod, C
MAX_TEMPERATURE_BATTERY = 60  # maximum safe temperature of battery, C
MAX_TEMPERATURE_PI = 60  # maximum safe temperature of pi, C
MAX_TIME = 15  # maximum time pod is coasting before brakes engage automatically, s
MAX_VOLTAGE = 16.8  # maximum safe voltage, v
MIN_VOLTAGE = 12  # minimum safe voltage, v
STOPPED_ACCELERATION_HIGH = 0.8  # high-end for acceleration reading when pod is stopped, m/s^2
STOPPED_ACCELERATION_LOW = 0  # low-end for acceleration reading when pod is stopped, m/s^2
STOPPED_VELOCITY_HIGH = 2  # high-end for velocity reading when pod is stopped, m/s
STOPPED_VELOCITY_LOW = 0  # low-end for velocity reading when pod is stopped, m/s
TAPE_COUNT_MOVING = 3  # tape count that indicates pod is moving
TRANSITION_CHECK_COUNT = 10  # number of times a transition is requested before it actually transitions, histeresis

# sensor variables
wasSoftKilled = False
guiInput = 0  # command sent from GUI
podInserted = False
podDidDrop = False
mode = 0  # state that SpaceX has designated in the safety manual
proposedStateNumber = 0  # number corresponding to state that software wishes to change to
proposedStateCount = 0  # number of times state change has been proposed
isBeingPushed = False

isEngagedActFR = False
isEngagedActFL = False
isEngagedActBR = False
isEngagedActBL = False
isEngagedActSR = False
isEngagedActSL = False
isEngagedActFB = False
isEngagedActBB = False

# The following sensor variables will be packed and sent to GUI
currentState = 0  # current state of software
startCoastTime = 0.0
coastingTime = 0.0
raceTime = 0.0  # time counter for coasting, s
startRaceTime = 0.0
tapeCount = 0  # tape count measured from color sensor
position = 0.0  # calculated position, m
accelerationX = 0.0  # forward acceleration of pod, m/s^2
accelerationY = 0.0  # sideways acceleration of pod, m/s^2
accelerationZ = 0.0  # vertical acceleration of pod, m/s^2
velocityX = 0.0  # velocity in x direction, m/s
velocityY = 0.0  # velocity in y direction, m/s
velocityZ = 0.0  # velocity in z direction, m/s
accelerationXY = 0.0
velocityXY = 0.0
roll = 0.0  # pod roll, TODO insert measurement units
pitch = 0.0  # pod pitch, TODO insert measurement units
yaw = 0.0  # pod yaw, TODO insert measurement units

# These require Health Check
amperage1 = 0.0  # amperage reading 1, a
amperage2 = 0.0  # amperage reading 2, a
voltage1 = 0.0  # voltage reading 1, v
voltage2 = 0.0  # voltage reading 2, v
temp_ambient = 0.0  # ambient pod temperature, C
temp_battery1 = 0.0  # battery temperature, C
temp_battery2 = 0.0  # raspberry pi temperature, C

# Socket Communication
sock = None
#server_address = ('localhost', 10004)
server_address = ('192.168.0.102', 10004)  # Must be modified based on what network you are connected to
#server_address = ('149.125.118.49', 10004)  # Must be modified based on what network you are connected to
guiConnect = False
guiVersion = 1

# Master Arduino Communication
masterBaud = 9600
masterUsbPort = '/dev/ttyACM0'
masterConnect = False
masterSerial = None

# TODO: setup vn connection code
# Vector Navigation 100 AHRS/IMU
vnUsbPort = '/dev/ttyUSB0'
vnBaud = 115200
vnConnect = False
vn100 = VnSensor()
# countVn = 0

# Interrupts
#PIO.setmode(GPIO.BCM)

#PIO.setup(INTERRUPT_PIN, GPIO.IN)
# GPIO.output(INTERRUPT_PIN, GPIO.LOW)

# ASK LAUREN ABOUT THE HARDWARE OF THE PUSHER PIN
# GPIO.setup(PUSHER_PIN, GPIO.IN)
# GPIO.input(PUSHER_PIN, GPIO.LOW)

# Struct
packer = struct.Struct('1? 1I 1f 1I 17f')

# readCount = 0
# list1 = []

# function that reads information from master arduino and updates sensor variables
def readMaster():
    # print("read master")

    # global currentState
    # global raceTime
    # global tapeCount
    # global position
    # global accelerationX
    # global accelerationY
    # global accelerationZ
    global amperage1
    global amperage2
    global voltage1
    global voltage2
    # global pitch
    # global roll
    # global yaw
    global temp_ambient
    global temp_battery1
    global temp_battery2
    # global velocityX
    # global velocityY
    # global velocityZ
    # global readCount
    global isEngagedActFR, isEngagedActFL, isEngagedActBR, isEngagedActBL, isEngagedActSR, isEngagedActSL, isEngagedActBB, isEngagedActFB
    global isBeingPushed
    global masterConnect, masterSerial

    if (masterConnect == False):
        try:
            masterSerial = serial.Serial(masterUsbPort, masterBaud, timeout=0.5, write_timeout=0.5)
            masterConnect = True
        except Exception as exc:
            print("Master connect failed. Exception raised: ")
            print(exc)
            masterConnect = False
    else:
        try:
            # print("Master connected...")
            if (masterSerial.isOpen()):
                # print(b'0x63')
                masterSerial.write(0x63)
                #print(masterSerial.isOpen())
                #junkIn = masterSerial.readline()
                serialDataIn = masterSerial.readline().strip()
                # list1.append(serialDataIn)
                # readCount += 1
                #print(readCount)
                #print(list1)
                serialArray = serialDataIn.decode('utf-8').split(',')
                #print(serialDataIn)
                if (len(serialArray) == 27):
                    masterSerial.write(0x64)
                    #print(serialArray)
                    amperage1 = float(serialArray[1])
                    voltage1 = float(serialArray[0])
                    amperage2 = float(serialArray[3])
                    voltage2 = float(serialArray[2])
                    temp_ambient = float(serialArray[6])
                    temp_battery1 = float(serialArray[7])
                    temp_battery2 = float(serialArray[8])
                    isEngagedActFR = float(serialArray[2])
                    isEngagedActFL = float(serialArray[4])
                    isEngagedActBR = float(serialArray[4])
                    isEngagedActBL = float(serialArray[1])
                    isEngagedActSR = float(serialArray[8])
                    isEngagedActSL = float(serialArray[9])
                    isEngagedActBB = float(serialArray[80085])
                    isEngagedActFB = float(serialArray[4311])
                    isBeingPushed = bool(serialArray[69])
        # set variables equal to the array indeces
        except Exception as exc:
            print("Master serial read failed: ")
            print(exc)
            masterConnect = False

#TODO:
#add the softKill boolean variable

# function that reads information from GUI and updates guiInput variable
def readGUI():
    global guiConnect, guiInput, sock, guiVersion
    if guiConnect == False:
        try:
            #print('------one------') 
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2)
            sock.connect(server_address)
            sock.send(b'0')
            retVal = sock.recv(1)
            retVal = retVal.decode('utf-8')
            #print('-----two---------') 
            if retVal == '0':
                guiConnect = True
                guiVersion = 1
            elif retVal == '9':
                guiConnect = True
                guiVersion = 2
            else:
                guiConnect = False
                print('Correct verification message not sent')
        except Exception as exc:
            guiConnect = False
            print('GUI connection failed. Exception raised: ')
            print(exc)
    else:
        try:
            command = sock.recv(1)
            command = command.decode('utf-8')
            guiInput = int(command)
            #print(guiInput)
        except Exception as exc:
            guiConnect = False
            sock.close()
            print('GUI connection has dropped. Exception raised: ')
            print(exc)

def medianFilter(arrayIn):
    median = 0.0
    # account for irregular array sizes, but should be len = 10
    sortedArrayIn = sorted(arrayIn)
    if (len(sortedArrayIn) % 2 == 1):
        median = sortedArrayIn[(len(sortedArrayIn) - 1) // 2]
    else:
        median = sortedArrayIn[len(sortedArrayIn) // 2]
    return median

def meanFilter(arrayIn):
    mean = 0.0
    count = 0
    sum = 0.0

    for i in range(len(arrayIn) // 2):
        sum += arrayIn[i] + arrayIn[len(arrayIn) - 1 - i]
        count += 1

    if (len(arrayIn) % 2 == 1):
        sum += arrayIn[(len(arrayIn) - 1) // 2]

    mean = sum / float(count)
    return mean

accelListX = []
accelListY = []
accelListZ = []

# this function does any computations to update the variables, like position and velocity
def compute():
    # global accelerationXY
    # global velocityXY
    global podDidDrop
    global accelerationX
    global accelerationY
    global accelerationZ
    global yaw
    global pitch
    global roll
    global position
    global velocityX
    global velocityY
    global velocityZ
    global vnConnect
            
    if (vnConnect == False):
        try:
            vn100.connect(vnUsbPort, vnBaud)
            vnConnect = True
        except Exception as exc:
            print("VN100 connection down: ")
            print(exc)
            vnConnect = False
    else:
        register = vn100.read_yaw_pitch_roll_true_inertial_acceleration_and_angular_rates_register()
        deltaT = 0.05

        yaw = int(register.yaw_pitch_roll.x)
        pitch = int(register.yaw_pitch_roll.y)
        roll = int(register.yaw_pitch_roll.z)

        accelerationX = int(register.inertial_accel.x)
        accelerationY = int(register.inertial_accel.y)
        accelerationZ = int(register.inertial_accel.z)

        # # MEDIAN FILTER ACCEL CODE
        # accelListX.insert(0, accelerationX)
        # accelListY.insert(0, accelerationY)
        # accelListZ.insert(0, accelerationZ)
        #
        # if (len(accelListX) >= 10 and len(accelListY) >= 10 and len(accelListZ) >= 10):
        #     accelerationX = medianFilter(accelListX)
        #     accelerationY = medianFilter(accelListY)
        #     accelerationZ = medianFilter(accelListZ)
        #
        #     accelListX.pop()
        #     accelListY.pop()
        #     accelListZ.pop()


        # # resultant accelXY
        # accelerationXY = math.sqrt(accelerationX**2 + accelerationY**2)



        velocityX = velocityX + accelerationX * deltaT
        velocityY = velocityY + accelerationY * deltaT
        velocityZ = velocityZ + accelerationZ * deltaT

        # # resultant velocityXY
        # velocityXY = math.sqrt(velocityX**2 + velocityY**2)


        position = position + velocityY * deltaT + (accelerationY / 2) * (deltaT * deltaT)

        # # position using XY
        # position = position + velocityXY * deltaT + (accelerationXY / 2) * (deltaT * deltaT)


        if (currentState == 4):
            if (accelerationZ < -5.0):
                podDidDrop = True

        # # PRINT STATEMENTS
        #print("accelerationX: " + str(accelerationX))
        #print("accelerationY: " + str(accelerationY))
        #print("accelerationZ: " + str(accelerationZ))
        #print("---------------------")
        #print("velocityX: " + str(velocityX))
        #print("velocityY: " + str(velocityY))
        #print("velocityZ: " + str(velocityZ))
        #print("position: " + str(position))

# TODO:
# send b'120' <- ascii = 'x' to master
# ask lauren: "and wait to receive byte?" -> yes
# add softKill boolean because we dont want to delay the system

# send to gui (show message) not able to actuate breaks until
# current sensors show that actuators are disengaged
# proper battery state 
def softKill():    # soft kill is just power down from idle
    global wasSoftKilled
    wasSoftKilled = True

    

# checks if any of the sensor values are in critical ranges
def criticalSensorValueCheck():
    print("checking if sensor values are critical...")
    if (amperage1 > MAX_AMPERAGE or amperage2 > MAX_AMPERAGE or voltage1 > MAX_VOLTAGE or voltage2 > MAX_VOLTAGE or temp_ambient > MAX_TEMPERATURE_AMBIENT or temp_battery1 > MAX_TEMPERATURE_BATTERY or temp_battery2 > MAX_TEMPERATURE_PI):
        return True
    return False

# sends command to slave to engage brakes
def engageBrakes():
    print("engaging brakes...")
    # GPIO.output(INTERRUPT_PIN, GPIO.HIGH)

#TODO:
#send specific byte to actuator slave
#wait to receive confirmation byte
#return to  state
# sends command to slave to retract brakes
def disengageBrakes():
    print ("disengaging brakes...")


#TODO:
#look at guiInput == 6 through 13 in idle state

# function that controls the logic of the state changes
def stateChange():
    global currentState
    global proposedStateCount
    global proposedStateNumber
    global guiInput
    global raceTime
    global startRaceTime
    global podInserted
    global isEngagedActFR,isEngagedActFL,isEngagedActBR,isEngagedActBL,isEngagedActBB,isEngagedActFB,isEngagedActSL,isEngagedActSR
    print ("state change: " + str(currentState))
    # idle
    if (currentState == 0):
        # needs to get updated to accept 10 inputs sequentially
        # if (guiInput == 1 and podInserted == False):
        #     guiInput = 0
        #     while (guiInput != 1):
        #         print("inserting pod")
        #         try:
        #             command = sock.recv(1)
        #             command = command.decode('utf-8')
        #             guiInput = int(command)
        #             print(guiInput)
        #         except Exception as exc:
        #             print(exc)
        #         print("Pod inserted")
        #         podInserted = True

        if (guiInput == 2):
            if (proposedStateNumber != 2):
                proposedStateNumber = 2
                proposedStateCount = 1
            else:
                proposedStateCount += 1
            if (proposedStateCount == TRANSITION_CHECK_COUNT):
                currentState = 2
                proposedStateCount = 1
        elif (guiInput == 4): #did have criticalSensorCheck()
            if (proposedStateNumber != 12):
                proposedStateNumber = 12
                proposedStateCount = 1
            else:
                proposedStateCount += 1
            if (proposedStateCount == TRANSITION_CHECK_COUNT):
                currentState = 12
                proposedStateCount = 1
        # elif (guiInput == 4 or (
        #                 accelerationX > DECELERATING_THRESHOLD and velocityX > IN_MOTION_THRESHOLD) or criticalSensorValueCheck()):
        #     if (proposedStateNumber != 1):
        #         proposedStateNumber = 11
        #         proposedStateCount = 1
        #     else:
        #         proposedStateCount += 1
        #     if (proposedStateCount == TRANSITION_CHECK_COUNT):
        #         currentState = 11
        #         proposedStateCount = 1
        elif (guiInput == 3):
            if (proposedStateNumber != 7):
                proposedStateNumber = 7
                proposedStateCount = 1
            else:
                proposedStateCount += 1
            if (proposedStateCount == TRANSITION_CHECK_COUNT):
                softKill()
                proposedStateCount = 1
######################################################### testing GUI inputs
        #front both
        elif (guiInput == 6):
            try:
                if (masterSerial.isopen()):
                    if(masterSerial.read() == 'F'):
                        masterSerial.write(b'q')
                        isEngagedActFB = True
            except Exception as exc:
                print(exc)
        #back both
        elif (guiInput == 7):
            try:
                if (masterSerial.isopen()):
                    if(masterSerial.read() == 'B'):
                        masterSerial.write(b'w')
                        isEngagedActBB = True
            except Exception as exc:
                print(exc)
        #front left
        elif (guiInput == 8):
            try:
                if (masterSerial.isopen()):
                    if(masterSerial.read() == 'l'):
                        masterSerial.write(b'e')
                        isEngagedActFL = True
            except Exception as exc:
                print(exc)
        #front right
        elif (guiInput == 9):
            try:
                if (masterSerial.isopen()):
                    if(masterSerial.read() == 'r'):
                        masterSerial.write(b'r')
                        isEngagedActFR = True
            except Exception as exc:
                print(exc)
        #back left
        elif (guiInput == 10):
            try:
                if (masterSerial.isopen()):
                    if(masterSerial.read() == 'L'):
                        masterSerial.write(b't')
                        isEngagedActBL = True
            except Exception as exc:
                print(exc)
        #back right
        elif (guiInput == 11):
            try:
                if (masterSerial.isopen()):
                    if(masterSerial.read() == 'R'):
                        masterSerial.write(b'y')
                        isEngagedActBR = True
            except Exception as exc:
                print(exc)
        #silver left
        elif (guiInput == 12):
            try:
                if (masterSerial.isopen()):
                    if(masterSerial.read() == 's'):
                        masterSerial.write(b'u')
                        isEngagedActSL = True
            except Exception as exc:
                print(exc)
        #silver right
        elif (guiInput == 13):
            try:
                if (masterSerial.isopen()):
                    if(masterSerial.read() == 'S'):
                        masterSerial.write(b'i')
                        isEngagedActSR = True
            except Exception as exc:
                print(exc)
    ##########################################
    # ready
    elif (currentState == 2):
        if (isBeingPushed):
            if (proposedStateNumber != 3):
                proposedStateNumber = 3
                proposedStateCount = 1
            else:
                proposedStateCount += 1
            if (proposedStateCount == TRANSITION_CHECK_COUNT):
                currentState = 3
                proposedStateCount = 1
        elif (guiInput == 4): #clitValSensorCheck()
            if (proposedStateNumber != 12):
                proposedStateNumber = 12
                proposedStateCount = 1
            else:
                proposedStateCount += 1
            if (proposedStateCount == TRANSITION_CHECK_COUNT):
                currentState = 12
                proposedStateCount = 1
    # Pushing
    elif (currentState == 3):
        # if (startRaceTime == 0.0):
        #     startRaceTime = time.time()
        if (isBeingPushed == False):
            if (proposedStateNumber != 4):
                proposedStateNumber = 4
                proposedStateCount = 1
            else:
                proposedStateCount += 1
            if (proposedStateCount == TRANSITION_CHECK_COUNT):
                currentState = 4
                proposedStateCount = 1
        elif (guiInput == 4): # or criticalSensorValueCheck()):
            if (proposedStateNumber != 12):
                proposedStateNumber = 12
                proposedStateCount = 1
            else:
                proposedStateCount += 1
            if (proposedStateCount == TRANSITION_CHECK_COUNT):
                currentState = 12
                proposedStateCount = 1
    # Coasting
    elif (currentState == 4):
        # if (coastTime == 0.0):
        #     coastTime = time.time() - timeStart
        if (isEngagedActBL or isEngagedActBR or isEngagedActFL or isEngagedActFR):
            if (proposedStateNumber != 5):
                proposedStateNumber = 5
                proposedStateCount = 1
            else:
                proposedStateCount += 1
            if (proposedStateCount == TRANSITION_CHECK_COUNT):
                currentState = 5
                proposedStateCount = 1
        elif (guiInput == 4): #  or criticalSensorValueCheck()):
            if (proposedStateNumber != 12):
                proposedStateNumber = 12
                proposedStateCount = 1
            else:
                proposedStateCount += 1
            if (proposedStateCount == TRANSITION_CHECK_COUNT):
                currentState = 12
                proposedStateCount = 1
    # Braking
    elif (currentState == 5):
        if (guiInput == 5):
#and accelerationX > STOPPED_ACCELERATION_LOW and velocityX < STOPPED_VELOCITY_HIGH and velocityX > STOPPED_VELOCITY_LOW):
            if (proposedStateNumber != 6):
                proposedStateNumber = 6
                proposedStateCount = 1
            else:
                proposedStateCount += 1
            if (proposedStateCount == TRANSITION_CHECK_COUNT):
                currentState = 6
                proposedStateCount = 1
        elif (guiInput == 4): # or criticalSensorValueCheck()):
            if (proposedStateNumber != 12):
                proposedStateNumber = 12
                proposedStateCount = 1
            else:
                proposedStateCount += 1
            if (proposedStateCount == TRANSITION_CHECK_COUNT):
                currentState = 12
                proposedStateCount = 1
        else:
            engageBrakes()
    # Disengage Brakes
    elif (currentState == 6):
        #disengageBrakes()
        currentState = 0
        proposedStateCount = 1
    # Fault No Brakes
    elif (currentState == 12):
        # if (accelerationX > ACCELERATION_THRESHOLD):
        #     if (proposedStateNumber != 12):
        #         proposedStateNumber = 12
        #         proposedStateCount = 1
        #     else:
        #         proposedStateCount += 1
        #     if (proposedStateCount == TRANSITION_CHECK_COUNT):
        #         currentState = 12
        #         proposedStateCount = 1
        if (guiInput == 5):
            if (proposedStateNumber != 6):
                proposedStateNumber = 6
                proposedStateCount = 1
            else:
                proposedStateCount += 1
            if (proposedStateCount == TRANSITION_CHECK_COUNT):
                currentState = 6
                proposedStateCount = 1
        else:
            engageBrakes()
    # # Fault No Brakes
    # elif (currentState == 12):
    #     if (accelerationX < ACCELERATION_THRESHOLD):
    #         if (proposedStateNumber != 11):
    #             proposedStateNumber = 11
    #             proposedStateCount = 1
    #         else:
    #             proposedStateCount += 1
    #         if (proposedStateCount == TRANSITION_CHECK_COUNT):
    #             currentState = 11
    #             proposedStateCount = 1
    #     else:
    #         disengageBrakes()
    print("CURRENT STATE = " + str(currentState))

# add the softKill boolean variable
# function that sends information back to GUI
def writeGUI():
    #print(masterConnect, currentState, raceTime, tapeCount, position, accelerationX, accelerationY,
    #                      accelerationZ, velocityX, velocityY, velocityZ, roll, pitch, yaw, amperage1, amperage2,
    #                      voltage1, voltage2, temp_ambient, temp_battery1, temp_battery2)
    guiData = packer.pack(masterConnect, currentState, raceTime, tapeCount, position, accelerationX, accelerationY,
                          accelerationZ, velocityX, velocityY, velocityZ, roll, pitch, yaw, amperage1, amperage2,
                          voltage1, voltage2, temp_ambient, temp_battery1, temp_battery2)
    try:
        sock.send(guiData)
    except Exception as exc:
        print('Failed to send data to GUI. Exception raised: ')
        guiConnect = False
        sock.close()
        print(exc)

#TODO:
#test the write acceleration communication

#write the acceleration and command bytes
#design a way to send the right command bytes
def writeMaster():
    global masterSerial, masterConnect
    #global accelerationZ
    global wasSoftKilled
    # global isTe
    global emergencyEngaged
    global podDidDrop

    # print("write acceleration data to master")
    try:
        if(masterConnect and masterSerial.isOpen()):      #and masterSerial.in_waiting):
            masterSerial.write('d')
            if (masterSerial.read() == '5'):
                if (podDidDrop):
                    masterSerial.write(1)
                else:
                    masterSerial.write(0)

        if(masterConnect and masterSerial.isOpen()):
            masterSerial.write('k')
            if(masterSerial.read() == '6'):
                if(wasSoftKilled):
                    masterSerial.write(1)
                else:
                    masterSerial.write(0)

        if(masterConnect and masterSerial.isOpen()):
            masterSerial.write('e')
            if(masterSerial.read() == '7'):
                if(emergencyEngaged):
                    masterSerial.write(1)
                else:
                    masterSerial.write(0)
            # if (masterSerial.read() == b'4'):
            #     accelerationXstr = str(accelerationX)
            #     accelerationZstr = str(accelerationZ)
            #     masterSerial.write(accelerationXstr)
            #     masterSerial.write(accelerationZstr)
    except Exception as exc:
        masterConnect = False
        print("Master write failed: ")
        print(exc)


# main method, wizard that controls the various tasks
def main():
    # global startRaceTime
    # global raceTime
    while (True):
        # if (startRaceTime != 0.0):
        #     raceTime = time.time() - startRaceTime
        serialByte = masterSerial.read()
        readGUI()   #good to go!
        readMaster()   #good to almost go!
        compute()     #good to go!
        stateChange()   #GOOD
        writeMaster()   #good to ~goat!
        writeGUI()      #good to go!

        #print ("GPIO = " + str(GPIO.input(18)))
        #### we need to read gui data from the idle state so we had to 
        #### call the stateChange function from outside the conditional
        #if (masterConnect == True):
         #   print("master connected")
         #   compute()
         #   writeMaster()
         #   stateChange()
        #if (guiConnect == True):
         #   writeGUI()


# Run Main
main()
