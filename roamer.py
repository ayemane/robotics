import serial
import time
import random

from Adafruit_MotorHAT import Adafruit_MotorHAT as amhat
from Adafruit_MotorHAT import Adafruit_DCMotor as adamo

'''
1 = left
2 = left
3 = right
4 = right
'''

# create motor objects
motHAT = amhat(addr=0x60)
mot1 = motHAT.getMotor(1) #left
mot2 = motHAT.getMotor(2) #left
mot3 = motHAT.getMotor(3) #right
mot4 = motHAT.getMotor(4) #right

# open serial port
ser = serial.Serial('/dev/ttyACM0', 9600)

# create variables
# sensors
distMid = 0.0
distLeft = 0.0
distRight = 0.0

# motor multipliers
m1Mult = 1.0
m2Mult = 1.0
m3Mult = 1.0
m4Mult = 1.0

# distance threshold
distThresh = 10.0
distCutOff = 30.0

# speeds
speedDef = 80
turnSpeedMultiplier = 2.0
leftSpeed = speedDef * turnSpeedMultiplier
rightSpeed = speedDef * turnSpeedMultiplier
backupSpeed = speedDef
speedMod = 20
turnTime = .15
defTime = 0.01
driveTime = defTime
backupDuration = .5

def snooze(msg,duration):
    print "==> "+msg
    #time.sleep(duration)

def driveMotors(leftChnl = speedDef, rightChnl = speedDef, duration = defTime):

    # determine the speed of each motor by multiplying
    # the channel by the motors multiplier
    m1Speed = leftChnl * m1Mult 
    m2Speed = leftChnl * m2Mult
    m3Speed = rightChnl * m3Mult
    m4Speed = rightChnl * m4Mult

    # set each motor speed. Since the speed can be a
    # negative number, we take the absolute value
    mot1.setSpeed(0*abs(int(m1Speed)))
    mot2.setSpeed(0*abs(int(m2Speed)))
    mot3.setSpeed(0*abs(int(m3Speed)))
    mot4.setSpeed(0*abs(int(m4Speed)))

    # run the motors. if the channel is negative, run
    # reverse. else run forward
    if(leftChnl < 0):
        mot1.run(amhat.BACKWARD)
        mot2.run(amhat.BACKWARD)
    else:
        mot1.run(amhat.FORWARD)
        mot2.run(amhat.FORWARD)

    if (rightChnl > 0):
        mot3.run(amhat.BACKWARD)
        mot4.run(amhat.BACKWARD)
    else:
        mot3.run(amhat.FORWARD)
        mot4.run(amhat.FORWARD)

    # wait for duration
    time.sleep(duration)

def shutItDown():
    mot1.run(amhat.RELEASE)
    mot2.run(amhat.RELEASE)
    mot3.run(amhat.RELEASE)
    mot4.run(amhat.RELEASE)

def scanDistance():
    # read the serial port
    val = ser.readline().decode('utf=8')
    left,mid,right = [0.0,0.0,0.0]
    
    # parse the serial string
    parsed = val.split(',')
    parsed = [x.rstrip() for x in parsed]

    if(len(parsed)>2):
        mid = float(parsed[0] + str(0))
        left = float(parsed[1] + str(0))
        right = float(parsed[2] + str(0))

    #print "{0:6}  {1}  {2}".format(distLeft, distMid, distRight)
    return left,mid, right;
        
def go():
    distLeft,distMid, distRight = scanDistance()
    print "{0}  {1}  {2}".format(distLeft, distMid, distRight)
    # apply cutoff distance
    if(distMid > distCutOff):
        distMid = distCutOff
    if(distLeft > distCutOff):
        distLeft = distCutOff
    if(distRight > distCutOff):
        distRight = distCutOff

    # reset driveTime
    driveTime = defTime

    # if obstacle to left, steer right by increasing
    # leftSpeed and running rightSpeed negative defSpeed
    # if obstacle to right, steer to left by increasing
    # rightSpeed and running leftSpeed negative
    if(distLeft <= distThresh):
        print("Steer Right")
        leftSpeed = speedDef * turnSpeedMultiplier
        rightSpeed = -speedDef * turnSpeedMultiplier
    elif (distRight <= distThresh):
        print("Steer Left")
        leftSpeed = -speedDef * turnSpeedMultiplier
        rightSpeed = speedDef * turnSpeedMultiplier
    else:
        print("Drive Straignt")
        leftSpeed = speedDef
        rightSpeed = speedDef

    # if obstacle dead ahead, stop then turn toward most
    # open direction. if both directions open, turn random
    if(distMid <= distThresh):
        snooze("blockage detected",5)
        
        # stop
        leftSpeed = 0
        rightSpeed = 0
        driveMotors(leftSpeed, rightSpeed, 1)
        time.sleep(.25)
        leftSpeed = -backupSpeed
        rightSpeed = -backupSpeed
        driveMotors(leftSpeed, rightSpeed, backupDuration)
        
        
        
        
        # determine preferred direction. if distLeft >
        # distRight, turn left. if distRight > distLeft,
        # turn right. if equal, turn random
        dirPref = distRight - distLeft
        if(dirPref == 0):
            #shutItDown();
            dirPref = random.random()
        if(dirPref < 0):
            snooze("prefer LEFT",5)
            leftSpeed = -speedDef * turnSpeedMultiplier
            rightSpeed = speedDef * turnSpeedMultiplier
        elif(dirPref > 0):
            snooze("prefer RIGHT",5)
            leftSpeed = speedDef * turnSpeedMultiplier
            rightSpeed = -speedDef * turnSpeedMultiplier
        driveTime = turnTime

    # drive the motors
    #print "{0:6}  {1:6}  {2:6}  {3:6}".format(leftSpeed, rightSpeed, driveTime, " FORWARD ")
    driveMotors(leftSpeed, rightSpeed, driveTime)

    ser.flushInput()


try:
    #driveMotors(-100,-100,.25)
    while 1:
        go()
        snooze("go",2)

except KeyboardInterrupt:
    shutItDown()
    
