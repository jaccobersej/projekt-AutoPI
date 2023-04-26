import numpy as np
import matplotlib.pyplot as plt
from smbus2 import SMBus
import time
import RPi.GPIO as GPIO

GPIO.cleanup()

##CONFIG##
gridsizeX = 5 #meter
gridsizeY = 13 #meter
timeStepLength = 1/40
gridFactor = 100 #cm
GPIO.setmode(GPIO.BCM)

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

Device_Address = 0x68   # MPU6050 device address //i2cdetect 1 in cmd
bus = SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards

#Distance sonsor pins
pin_trigger_left = 15
pin_echo_left = 18
GPIO.setup(pin_trigger_left, GPIO.OUT)
GPIO.setup(pin_echo_left, GPIO.IN)


pin_trigger_center = 23
pin_echo_center = 24
GPIO.setup(pin_trigger_center, GPIO.OUT)
GPIO.setup(pin_echo_center, GPIO.IN)


pin_trigger_right = 25
pin_echo_right = 8
GPIO.setup(pin_trigger_right, GPIO.OUT)
GPIO.setup(pin_echo_right, GPIO.IN)


def MPU_Init():
        #write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

def calibrate(addr, calibration_time=10):
      print("--"*25)
      print("Beginning calibration - DO NOT MOVE THE BOARD")
      offset = [0, 0, 0]
      num_of_points = 0
      end_loop_time = time.time() + calibration_time

      while(end_loop_time > time.time()):
            num_of_points += 1
            calX = read_raw_data(addr)
            calY = read_raw_data(addr+2)
            calZ = read_raw_data(addr+4)

            offset[0] += calX
            offset[1] += calY
            offset[2] += calZ

            if num_of_points % 100 == 0:
                print("Still calibrating... %d points so far" % num_of_points)
      print("Calibration complete! %d points total" % num_of_points)
      offset = [i/num_of_points for i in offset]

      return offset

MPU_Init()

offset_gyroscope = calibrate(GYRO_XOUT_H)
offset_accelometer = calibrate(ACCEL_XOUT_H)
print("Ready to read data of gyroscope and accelerometer")

def distance(pin_trigger, pin_echo):
    GPIO.output(pin_trigger, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(pin_trigger, GPIO.LOW)

    start_time = time.time()
    end_time = time.time()

    clock_time = time.time()
    while GPIO.input(pin_echo)==0:
        start_time = time.time()
        if time.time() - clock_time >= 0.025:
            return 0
    while GPIO.input(pin_echo)==1:
        end_time = time.time()

    pulse_duratioen = end_time - start_time
    distance = pulse_duratioen*34300/2

    return distance

estimatedMap = np.zeros((gridsizeX*gridFactor, gridsizeY*gridFactor))

movementQueue = []
chaseAngle = 0
running = True

def accelerometer(previousPos, previousAngle, speeds, dt):
    xAccel = (read_raw_data(ACCEL_XOUT_H) - offset_accelometer[0]) * 9.82 / 16384.0
    yAccel = (read_raw_data(ACCEL_YOUT_H) - offset_accelometer[1]) * 9.82 / 16384.0
    angularVel = ((read_raw_data(GYRO_ZOUT_H)- offset_gyroscope[2]) / 16.4) * (np.pi / 180)
    

    xSpeed = speeds[0]
    ySpeed = speeds[1]
    #print(xAccel, yAccel)

    newXPos = previousPos[0] + 0.5 * xAccel * dt**2 + xSpeed * dt
    newYPos = previousPos[1] + 0.5 * yAccel * dt**2 + ySpeed * dt
    newAngle = previousAngle + angularVel * dt 
    
    xSpeed = xSpeed + xAccel*dt
    ySpeed = ySpeed + yAccel*dt

    return [[newXPos, newYPos], newAngle, [xSpeed, ySpeed]]

def UltrasoundSensorReadings():
    distance1 = distance(pin_trigger_left, pin_echo_left)
    distance2 = distance(pin_trigger_center, pin_echo_center)
    distance3 = distance(pin_trigger_right, pin_echo_right)
    #print(distance1)
    return [distance1, distance2, distance3]

class Agent:
    def __init__ (self, timeLength):
        self.speed = [0, 0]
        self.angle = np.pi/2
        self.time = timeLength
        self.position = [125, 90]
        self.searching = False
        self.preservation = False


    class Sensor:
        def __init__(self, relX, relY, angle):
            self.relativePos = [relX*gridFactor, relY*gridFactor]
            self.relativeAngle = angle

        def raytraceAndUpdateEstimate(self, distance):
            gridDistance = distance
            for r in np.arange(0, gridDistance-1):
                xi = int(agent.position[0] + self.relativePos[0] + r * np.cos(agent.angle + self.relativeAngle))
                yi = int(agent.position[1] + self.relativePos[1] + r * np.sin(agent.angle + self.relativeAngle))
                if 0 <= xi < gridsizeX*gridFactor and 0 <= yi < gridsizeY*gridFactor:
                    estimatedMap[xi, yi] = max(0, estimatedMap[xi, yi]-0.4)
            
            xPos = int(agent.position[0] + self.relativePos[0] + distance * np.cos(agent.angle + self.relativeAngle))
            yPos = int(agent.position[1] + self.relativePos[1] + distance * np.sin(agent.angle + self.relativeAngle))
            if 0 <= xPos < gridsizeX*gridFactor and 0 <= yPos < gridsizeY*gridFactor:
                #print(xi, yi)
                estimatedMap[xPos, yPos] = min(1, estimatedMap[xPos, yPos] + 0.5)
            elif 0 <= xPos < gridsizeX*gridFactor:
                estimatedMap[xPos, gridsizeY*gridFactor-1] = min(1, estimatedMap[xPos, gridsizeY*gridFactor-1] + 0.5)
            elif 0 <= yPos < gridsizeY*gridFactor:
                estimatedMap[gridsizeX*gridFactor-1, yPos] = min(1, estimatedMap[gridsizeX*gridFactor-1, yPos] + 0.5)
            else:
                estimatedMap[gridsizeX*gridFactor-1, gridsizeY*gridFactor-1] = min(1, estimatedMap[gridsizeX*gridFactor-1, gridsizeY*gridFactor-1] + 0.5)



agent = Agent(timeStepLength)
sensors = [agent.Sensor(0.05, -0.075, 0), agent.Sensor(0.05, 0, 0), agent.Sensor(0.05, 0.075, 0)]

#while running:
st = time.time()
for i in range(1600):
    #Accelerometer:
    et = time.time()
    agent.position, agent.angle, agent.speed = accelerometer(agent.position, agent.angle, agent.speed, st-et)
    st = time.time()
    #print(agent.position)
    print(agent.angle * (180/np.pi))
    #UltralydsSensorer
    distances = UltrasoundSensorReadings()
    for index, sensor in enumerate(sensors):
        #if distances[index] <= 400:
        sensor.raytraceAndUpdateEstimate(distances[index])
    #plt.cla()
plt.imshow(estimatedMap)
    #plt.pause(0.0001)

plt.show()
GPIO.cleanup()