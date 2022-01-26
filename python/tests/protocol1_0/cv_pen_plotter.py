#%% Get The Packages Going ^_^
import os
from pydub import AudioSegment
from pydub.playback import play
import csv
from time import sleep
import playsound

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_MX_TORQUE_ENABLE       = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION       = 30
ADDR_MX_PRESENT_POSITION    = 36
ADDR_MOVEMENT_SPEED         = 32

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 2                 # Dynamixel ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM6'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 450           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 550            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position
# 25 - 161 - 863
# 26 - 150 - 850


# CW

head_max = 600;
head_min = 250;


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 25, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 26, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)

if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")


def move_servo(servo, position):
# 25 - 161 - 863
# 26 - 150 - 850
    if (servo == 25):
        if (position >= 161 and position <= 863):
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo, ADDR_MOVEMENT_SPEED, 70)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo, ADDR_MX_GOAL_POSITION, position)
        else:
            print("out of bounds 25")

    if (servo == 26):
        if (position >= 150 and position <= 850):
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo, ADDR_MOVEMENT_SPEED, 70)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo, ADDR_MX_GOAL_POSITION, position)
        else:
            print("out of bounds 26")


def draw_point(servo, position):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo, ADDR_MOVEMENT_SPEED, 70)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo, ADDR_MX_GOAL_POSITION, position)


def move_wheel(servo,speed):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_MOVEMENT_SPEED, speed)

def get_position(servo):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, servo, ADDR_MX_PRESENT_POSITION)
    return dxl_present_position

#%% 
# Move servo (500 500 for mid position)
sleep(0.5)
draw_point(27,420)
move_servo(25,500)
move_servo(26,500)

#%%
#%%
import math 
import matplotlib.pyplot as plt
import numpy as np
import cv2

xrange = list(np.arange(-20,20+0.1))
yrange = list(np.arange(0,20+0.1))

img = cv2.imread("robot_signature8.png")
data = np.asarray( img, dtype="int32" )
data = np.argwhere(data[:,:,:] == 0)
x_vals = []
y_vals = []
# print(data)

for iterator in range(len(data)):
    x_vals.append(data[iterator][0])
    y_vals.append(data[iterator][1])

max_x = max(x_vals)
min_x = min(x_vals)
print(min_x, max_x)
max_y = max(y_vals)
min_y = min(y_vals)
print(min_y, max_y)


a1 = 20.5
a2 = 16.5
p = (a1+a2)/math.sqrt(2)
maxsize_armreach_width = 2*p
maxsize_armreach_height = 2*p
maxsize_armreach_width = 0.75*p
maxsize_armreach_height = 0.75*p
xyratio = max_x/max_y
yxratio = max_y/max_x
print(xyratio, 'xyratio')
print(yxratio, 'yxratio')
if max_y > max_x:
    adapted_y_ratio = round(maxsize_armreach_height-1,1)/max_y
    adapted_x_ratio = adapted_y_ratio*yxratio
if max_x > max_y:
    adapted_x_ratio = round(maxsize_armreach_width-1,1)/max_x
    adapted_y_ratio = adapted_x_ratio*yxratio
print(adapted_x_ratio, 'adapted_x_ratio')
print(adapted_y_ratio, 'adapted_y_ratio')


#%%
print(data)
sleep(2)
# for x in range(-20,20,1):
#     for y in range(-20,20,1):
a= 1

move_servo(25,500)
move_servo(26,500)
draw_point(27,420)
sleep(2)

for iterator in range(len(data)):
    x = data[iterator][0]*adapted_x_ratio-max_x*adapted_x_ratio/2
    y = data[iterator][1]*adapted_y_ratio-max_y*adapted_y_ratio/2+30
    alpha = math.acos(((a1**2) + (a2**2) - (x**2) - (y**2))/ (2*a1*a2))
    q2 = math.pi-alpha
    q1 = math.atan2(y,x)-math.atan2(a2*math.sin(q2),(a1+a2*math.cos(q2)))+0
    q2_degree = math.degrees(q2)
    q1_degree = math.degrees(q1)
    print('degrees')
    print(q1_degree,q2_degree)
    q1_degree_fixed = 150-90+q1_degree
    q2_degree_fixed = 150+q2_degree
    print('degrees fixed')
    print(q1_degree_fixed,q2_degree_fixed)
    q2_ax_motion = (q2_degree_fixed/300)*1023
    q1_ax_motion = (q1_degree_fixed/300)*1023

    print(int(q1_ax_motion),q2_ax_motion)



    move_servo(25,int(q1_ax_motion))
    move_servo(26,int(q2_ax_motion))


    plt.plot((0,a1*math.cos(q1)),(0,a1*math.sin(q1)),c='black')
    # plt.scatter(a1*math.cos(q1),a1*math.sin(q1),c='black')

    # plt.scatter(x12,y12,c='red')

    x11 = a1*math.cos(q1)
    x12 = x11+a2*math.cos(q2+q1)
    y11 = a1*math.sin(q1)
    y12 = y11+a2*math.sin(q2+q1)
    plt.plot((x11,x12),(y11,y12),c='blue')
    plt.gca().set_aspect('equal', adjustable='box')


    plt.scatter(x,y,c='green')
    plt.scatter(x12,y12,c='red')

    #print(x,y," are not part of it")
    #plt.scatter(x,y,c='red')
    plt.show()
    print(iterator)
    if (a == 1):
        sleep(2)
    else:
        sleep(0.5)
    draw_point(27,406)
    sleep(0.2)
    draw_point(27,420)

    a=2


#%%
# Disable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 2, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 3, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 5, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))


portHandler.closePort()

#%%
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 3, ADDR_MX_GOAL_POSITION, 500)

# %%
