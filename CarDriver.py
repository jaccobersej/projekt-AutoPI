import numpy as np
import matplotlib.pyplot as plt
import random
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
import math
from smbus2 import SMBus
import RPi.GPIO as GPIO
import pigpio
import time

GPIO.cleanup()
pwm = pigpio.pi()

##CONFIG##
gridsizeX = 10 #meter
gridsizeY = 10 #meter
timeStepLength = 1/40
gridFactor = 100 #cm
measurementMaxDist = 400 #cm
graphCellFactor = 32 #cm
searching = True
target = [0, 1] #Intet mennekse, menneske
targetFound = False
GPIO.setmode(GPIO.BCM)


###SETUP###
servo = 12
pwm.set_mode(servo, pigpio.OUTPUT)
forward = 20
back = 21
GPIO.setup(forward, GPIO.OUT)
GPIO.setup(back, GPIO.OUT)
pwm.set_PWM_frequency(servo, 50)



#Funktion der ændrer bilens styringsmekanismes vinkel til at være den vinkel den modtager. Accepterer kun radianer, som input.
def servo_change(angle):
    #print("rads:", angle)
    angle = np.rad2deg(angle)+90
    #print("degrees", angle)
    angle_change = (angle*2000)/180
    if 500 > angle_change or angle_change > 1500:
        #print("wrong angle")
        return 1
    else:
        pwm.set_servo_pulsewidth(servo, 500+angle_change+40) 

    
#MPU6050 registre og deres adresser.
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

#MPU6050 device adress // i2cdetect 1 i commandline
Device_Address = 0x68 
bus = SMBus(1)


#Distance sensor pins
pin_trigger_left = 17
pin_echo_left = 10
GPIO.setup(pin_trigger_left, GPIO.OUT)
GPIO.setup(pin_echo_left, GPIO.IN)

#Distance sensor pins
pin_trigger_center = 27
pin_echo_center = 9
GPIO.setup(pin_trigger_center, GPIO.OUT)
GPIO.setup(pin_echo_center, GPIO.IN)

#Distance sensor pins
pin_trigger_right = 22
pin_echo_right = 11
GPIO.setup(pin_trigger_right, GPIO.OUT)
GPIO.setup(pin_echo_right, GPIO.IN)

#initialize gyroskop
def MPU_Init():
    #skriv til sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Skriv til power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Skriv til Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)

	#Skriv til Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

	#Skriv til interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)


def read_raw_data(addr):
	#Accelero og Gyro value er 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #Sæt den højere og lavere værdi sammen
        value = ((high << 8) | low)    
        #For at få en signed værdi fra mpu6050
        if(value > 32768):
                value = value - 65536
        return value

#Kalibrer gyroskopet
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

#Kald initialize
MPU_Init()

#Kald calibration
offset_gyroscope = calibrate(GYRO_XOUT_H)
#offset_accelometer = calibrate(ACCEL_XOUT_H)
print("Ready to read data of gyroscope and accelerometer")

#Mål afstand med ultrasonisk sensor
def distance(sensor):
    pin_trigger = sensor.trigger
    pin_echo = sensor.echo
    distance = 0
    for i in range(1):
        GPIO.output(pin_trigger, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(pin_trigger, GPIO.LOW)

        start_time = time.time()
        end_time = time.time()

        clock_time = time.time()
        while GPIO.input(pin_echo)==0:
            start_time = time.time()
            if (start_time - clock_time) >= 0.015:
                return 1000
        
        clock_time = time.time()
        while GPIO.input(pin_echo)==1:
            end_time = time.time()
            if (end_time - clock_time) >= 0.015:
                return 1000

        pulse_duratioen = end_time - start_time
        distance += pulse_duratioen*34300/2

    return distance/1

#Definer grids:
estimatedMap = np.zeros((gridsizeX*gridFactor, gridsizeY*gridFactor))
graphMap = np.ones((int(gridsizeX*(gridFactor/graphCellFactor)), int(gridsizeY*(gridFactor/graphCellFactor))))


movementQueue = []
    
#Lav målinger fra accelerometeret/gyroskopet og opdater bilens position og orientering.
def accelerometer(previousPos, previousAngle, dt):
    #angularVel = (change*30)/0.75
    angularVel = -((read_raw_data(GYRO_ZOUT_H)- offset_gyroscope[2]) / 16.4) * (np.pi / 180)
    newAngle = previousAngle + angularVel * dt 
    #print(newAngle) #agent.actualAngle + (random.randint(-1,1) * random.uniform(0,0.01)) #previousAngle + angularVel * timeStepLength 
    xSpeed = agent.actualSpeed * np.cos(newAngle)
    ySpeed = agent.actualSpeed * np.sin(newAngle)


    newXPos = previousPos[0] + (xSpeed * dt)
    newYPos = previousPos[1] + (ySpeed * dt)

    return [[newXPos, newYPos], newAngle, [xSpeed, ySpeed]]

#Saml afstandsmålinger. 
def UltrasoundSensorReadings(sensors):
    distance1 = distance(sensors[0])
    #print("Venster",distance1)
    distance2 = distance(sensors[1])
    distance3 = distance(sensors[2])
    #print("Højre",distance3)

    return [distance1, distance2, distance3]


#Agent klassen er tilsvarende til bilen. Vi har den bare sådan at vi nemmere/mere læsbart kan referere til nogle egenskaber/parametre den har
class Agent:
    def __init__ (self, timeLength):
        self.speed = [0, 0]
        self.angle = 0
        self.time = timeLength
        self.position = [(gridsizeX*gridFactor)/2, (gridsizeY*gridFactor)/2]
        self.actualSpeed = 0
        self.actualPosition = [(gridsizeX*gridFactor)/2, (gridsizeY*gridFactor)/2]
        self.actualAngle = 0
        self.searching = False
        self.preservation = False
        self.actualSpeed = 0 #m/s * gridFactor

    #Kør frem ad
    def forward(self):
        self.actualSpeed = 0.1 * gridFactor
        self.actualPosition[0] += self.actualSpeed * np.cos(self.actualAngle) * self.time
        self.actualPosition[1] += self.actualSpeed * np.sin(self.actualAngle) * self.time

    #Kør baglæns
    def back(self):
        self.actualSpeed = -0.1 * gridFactor
        self.actualPosition[0] += self.actualSpeed * np.cos(self.actualAngle) * self.time
        self.actualPosition[1] += self.actualSpeed * np.sin(self.actualAngle) * self.time

    #Kør baglæns
    def stop(self):
        self.actualSpeed = 0

    #Definerer subclassen Sensor, som tilhører Agent classen. Den er lavet på denne måde for samme årsager, som bilen/agent klassen. Her er det dog også mere brugbart eftersom vi arbejder med flere sensorer.
    class Sensor:
        def __init__(self, relX, relY, angle, pin_trigger, pin_echo):
            self.relativePos = [relX*gridFactor, relY*gridFactor]
            self.relativeAngle = angle
            self.trigger = pin_trigger
            self.echo = pin_echo

        #Opdater kortet ud fra afstands målingerne, som beskrevet i rapporten
        def raytraceAndUpdateEstimate(self, distance):
            gridDistance = distance
            for r in np.arange(0, gridDistance-1):
                xi = int(agent.position[0] + self.relativePos[0] + r * np.cos(agent.angle + self.relativeAngle))
                yi = int(agent.position[1] + self.relativePos[1] + r * np.sin(agent.angle + self.relativeAngle))
                if 0 <= xi < gridsizeX*gridFactor and 0 <= yi < gridsizeY*gridFactor:
                    estimatedMap[xi, yi] = max(0, estimatedMap[xi, yi]-0.5)
            
            xPos = int(agent.position[0] + self.relativePos[0] + gridDistance * np.cos(agent.angle + self.relativeAngle))
            yPos = int(agent.position[1] + self.relativePos[1] + gridDistance * np.sin(agent.angle + self.relativeAngle))

            if 0 <= xPos < gridsizeX*gridFactor and 0 <= yPos < gridsizeY*gridFactor:
                estimatedMap[xPos, yPos] = min(1, estimatedMap[xPos, yPos] + 0.4)
            elif 0 <= xPos < gridsizeX*gridFactor:
                estimatedMap[xPos, gridsizeY*gridFactor-1] = min(1, estimatedMap[xPos, gridsizeY*gridFactor-1] + 0.4)
            elif 0 <= yPos < gridsizeY*gridFactor:
                estimatedMap[gridsizeX*gridFactor-1, yPos] = min(1, estimatedMap[gridsizeX*gridFactor-1, yPos] + 0.4)
            else:
                estimatedMap[gridsizeX*gridFactor-1, gridsizeY*gridFactor-1] = min(1, estimatedMap[gridsizeX*gridFactor-1, gridsizeY*gridFactor-1] + 0.5)


#Initialize agenten sensorerne
agent = Agent(timeStepLength)
sensors = [agent.Sensor(0, 0, np.pi/3, pin_trigger_left, pin_echo_left), agent.Sensor(0, 0, 0, pin_trigger_center, pin_echo_center), agent.Sensor(0, 0, -np.pi/3, pin_trigger_right, pin_echo_right)]


#Funktion der bliver brugt til at bilen kan find vej hen til et givent punkt (brugbart når man skal køre hen til en person)
def pathfind(i, koordinate_x, koordinate_y):
    global path
    for y in range(0, graphMap.shape[1]):
        for x in range(0, graphMap.shape[0]):
            if np.sum(estimatedMap[x*graphCellFactor:x*graphCellFactor+graphCellFactor, y*graphCellFactor:y*graphCellFactor+graphCellFactor]) > 8:
                graphMap[x,y] = 0
            else:
                graphMap[x,y] = 1
    
    pfGrid = Grid(matrix=graphMap.T)
    if 0 < agent.position[0] <= gridsizeX*gridFactor and 0 < agent.position[0] <= gridsizeY*gridFactor:
        pathStart = pfGrid.node(int(agent.position[0]/graphCellFactor), int(agent.position[1]/graphCellFactor))
        pathEnd = pfGrid.node(int(koordinate_x/graphCellFactor), int(koordinate_y/graphCellFactor))
        pathFinder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path, runs = pathFinder.find_path(pathStart, pathEnd, pfGrid)
        print(print(pfGrid.grid_str(path=path, start=pathStart, end=pathEnd)))
        if len(path) > 2:
            xDist = (path[1][0]+path[2][0])/2 - int(agent.position[0]/graphCellFactor)
            yDist = (path[1][1]+path[2][1])/2 - int(agent.position[1]/graphCellFactor)
            pathAngle = math.atan2(yDist, xDist)
            if agent.angle < 0 and pathAngle > 0:
                pathAngle = -2*np.pi + pathAngle
            if agent.angle > 0 and pathAngle < 0:
                pathAngle = 2*np.pi + pathAngle
            #print(xDist, yDist, pathAngle) 
            for q in range(0, 1):
                movementQueue.insert(0, [1, pathAngle])
        if len(path) == 1:
            xDist = path[1][0] - int(agent.position[0]/graphCellFactor)
            yDist = path[1][1] - int(agent.position[1]/graphCellFactor)
            pathAngle = math.atan2(yDist, xDist)
            #print(xDist, yDist, pathAngle) 
            for q in range(0, 1):
                movementQueue.insert(0, [1, pathAngle])
    
    return path

#Kode der definerer det search pattern bilen kører i.
def search(i, distanceList):
    global et, st
    et = time.time()
    if i% 10 == 0:
        if distanceList[1] <= 20:
            for q in range (0, 20):
                movementQueue.insert(0, [-1, agent.angle])
        elif distanceList[0] <= 15:
            for j in range(0, 40):
                movementQueue.insert(0, [1, agent.angle - np.pi/2])
            for q in range(0, 40):
                movementQueue.insert(0, [-1, agent.angle])
        elif distanceList[2] <= 15:
            for j in range(0, 40):
                movementQueue.insert(0, [1, agent.angle + np.pi/2])
            for q in range(0, 40):
                movementQueue.insert(0, [-1, agent.angle])
        elif distanceList[1] <= 50:
            if et - st >= 1:
                print(agent.angle)
                for q in range(0, 40):
                    if agent.angle < 0:
                        if distanceList[2] > 35:
                            movementQueue.append([1, agent.angle-np.pi/2])
                        elif distanceList[2] <= 35:
                            movementQueue.append([1, agent.angle+np.pi/2])

                    elif agent.angle >= 0:
                        if distanceList[0] > 35:
                            movementQueue.append([1, agent.angle+np.pi/2])
                        elif distanceList[0] <= 35:
                            movementQueue.append([1, agent.angle-np.pi/2])
                        
                st = time.time()
        elif len(movementQueue) == 0:
            for j in range(0, 10):
                movementQueue.append([1, agent.angle])
    


xList = []
yList = []

currentAngle = agent.actualAngle
targetAngle = 0
change = 0
servo_change(0)
st = -40
et = 0
count = 0
pt = time.time()
ct = time.time()
#Main loop herunder. Det er her alting bliver kaldt fra og det, som får alle delene til at spille sammen.
try:
    while 1:
        #SENSOR DEL
        ct = time.time()
        agent.position, agent.angle, agent.speed = accelerometer(agent.position, agent.angle, ct-pt)
        pt = time.time()
        currentAngle = agent.angle
        xList.append(agent.position[0])
        yList.append(agent.position[1])
        
        distances = UltrasoundSensorReadings(sensors)
        for index, sensor in enumerate(sensors):
            if distances[index] < 250:
                sensor.raytraceAndUpdateEstimate(distances[index])
        #cm/grafcellfaktor
        file1 = open("Guess.txt", "r")
        guesses = file1.read()
        try:
            guess_1 = float(guesses.partition(",")[0])
            guess_2 = float(guesses.partition(",")[2])
        except:
            guess_1 = 0
            guess_2 = 0
            
            
        #PATHFINDING OG "DRIVERSEAT"
        if guess_2 >= 0.75:
            if targetFound != True:
                file2 = open("Vektor.txt", "r")
                vektor_koordinates = file2.read()
                vektor_x = float(vektor_koordinates.partition(",")[0]) + agent.position[0]
                vektor_y = float(vektor_koordinates.partition(",")[2]) + agent.position[1]
            targetFound = True
            
        
        if targetFound == True:
            print(vektor_x, vektor_y) 
            path = pathfind(count, vektor_x, vektor_y)
        if targetFound != True:
            search(count, distances)
        
        
        #KØRE DEL
        agent.actualAngle = currentAngle
        if round(targetAngle, 2) != round(currentAngle, 2):
            servo_change(change)
        if round(targetAngle, 2) == round(currentAngle, 2):
            change = 0


        if len(movementQueue) > 0:
            currentAction = movementQueue.pop(0)
            speedCommand = currentAction[0]

            if speedCommand > 0:
                agent.actualSpeed = 8.8
                GPIO.output(forward, GPIO.HIGH)
                GPIO.output(back, GPIO.LOW)
            elif speedCommand < 0:
                agent.actualSpeed = -8.8
                GPIO.output(forward, GPIO.LOW)
                GPIO.output(back, GPIO.HIGH)
            if speedCommand == 0:
                agent.actualSpeed = 0
                GPIO.output(forward, GPIO.LOW)
                GPIO.output(back, GPIO.LOW)

            if targetAngle != currentAction[1]:
                targetAngle = currentAction[1]
                change = (targetAngle - currentAngle)
                if change > (2*np.pi)/9:
                    change = (2*np.pi)/9
                elif change < -(2*np.pi)/9:
                    change = -(2*np.pi)/9
        
        else:
            agent.actualSpeed = 0
            GPIO.output(forward, GPIO.LOW)
            GPIO.output(back, GPIO.LOW)
        
        if round(targetAngle, 2) != round(currentAngle, 2) and agent.actualSpeed > 0:
            agent.actualSpeed = 6.9
        if round(targetAngle, 2) != round(currentAngle, 2) and agent.actualSpeed < 0:
            agent.actualSpeed = -6.9
        
        count += 1
        
#Hvis vi vælger at stoppe bilen fra computeren af:
except KeyboardInterrupt:
    GPIO.output(forward, GPIO.LOW)
    GPIO.output(back, GPIO.LOW)
    GPIO.cleanup()
    servo_change(0)
    time.sleep(0.5)
    pwm.set_PWM_dutycycle(servo, 0)
    pwm.set_PWM_frequency(servo, 0)
    
    #Plot kortet og bilens rute
    plt.plot(xList, yList, 'r')
    plt.imshow(estimatedMap.T, origin='lower')
    plt.show()


    #Plot vores lille graph version af plottet
    plt.cla()
    for y in range(0, graphMap.shape[1]):
            for x in range(0, graphMap.shape[0]):
                if np.sum(estimatedMap[x*graphCellFactor:x*graphCellFactor+graphCellFactor, y*graphCellFactor:y*graphCellFactor+graphCellFactor]) > 8:
                    graphMap[x,y] = 0
                else:
                    graphMap[x,y] = 1
    plt.imshow(graphMap.T, origin='lower')
    plt.show()
