#%%
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
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MOVEMENT_SPEED   = 60

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 2                 # Dynamixel ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM5'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 450           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 550            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 45                # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position


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
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 3, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 5, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)

# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 5, 6, 0)
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 5, 8, 0)
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 5, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")

#%%

def move_servo(servo, position):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo, ADDR_MOVEMENT_SPEED, 80)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo, ADDR_MX_GOAL_POSITION, position)

def move_wheel(servo,speed):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_MOVEMENT_SPEED, speed)

def get_position(servo):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, servo, ADDR_MX_PRESENT_POSITION)
    return dxl_present_position

#%%
move_servo(3,600)
#%%
x = 320 
w = 0
y = 240
h = 0
plays = 0
import cv2
# Load the cascade
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
# To capture video from webcam. 
cap = cv2.VideoCapture(1)
while True:
    # Read the frame
    _, img = cap.read()
    rotated_frame = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

    gray = cv2.cvtColor(rotated_frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 4)
    sleep(0.5)
    old_x = x
    old_y = y
    old_w = w
    old_h = h

    if (plays == 0):
        for (x, y, w, h) in faces:
            cv2.rectangle(rotated_frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            #print("x"+str(x) + "___w"+str(w) + "_____y"+str(y) + "___h"+str(h))

        if (x == old_x) and (y == old_y):
            move_servo(5,0)

        elif (x != old_x) and (y != old_y):

            print("position"+ str(get_position(5)))
            if (((x+x+w)/2)<270):
                move_servo(5,700)
                print("moving right")

            elif (((x+x+w)/2)>390):
                move_servo(5,200)
                print("moving left")

            else:
                move_servo(5,0) 

            height = get_position(3)

            if (((y+y+h)/2)<150):
                newheight = height+20
                if (newheight < head_max):
                    move_servo(3,300)
                    print("moving down")

            elif (((y+y+h)/2)>350):
                newheight = height-10
                if (newheight > head_min):
                    move_servo(3,600)
                    print("moving up")

            else:
                plays = 0

    # if (plays == 1):
    #     # song = AudioSegment.from_wav("test.wav")
    #     # play(song)
    #     playsound.playsound('test.wav', block = False)

    #     plays = 0



    # Display
    cv2.imshow('img', rotated_frame)
    # Stop if escape key is pressed
    k = cv2.waitKey(30) & 0xff
    if k==27:
        break

move_wheel(5,0)

# Release the VideoCapture object
cap.release()

# %%
packetHandler.write2ByteTxRx(portHandler, 5, ADDR_MOVEMENT_SPEED, 0)
#%%
newweight = 500
move_servo(3,550)
print(get_position(3))

# %%

print(dxl_present_position)
# %%
## 5
 #eft limit 0
# right limit 668

#%%

packetHandler.write2ByteTxRx(portHandler, 5, ADDR_MOVEMENT_SPEED, 0)

#%%

move_servo(3,350)
move_servo(2,400)
sleep(1)
song = AudioSegment.from_wav("test.wav")
play(song)

#%%
dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, 5, ADDR_MX_PRESENT_POSITION)
print(dxl_present_position)


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
