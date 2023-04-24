import numpy as np
import matplotlib.pyplot as plt

##CONFIG##
gridsizeX = 10 #meter
gridsizeY = 10 #meter
timeStepLength = 1/40
gridFactor = 100 #cm
measurementMaxDist = 450*gridFactor/100 #cm

#Initialize groundTruth og predict maps:
groundTruth = np.zeros((gridsizeX*gridFactor, gridsizeY*gridFactor))
estimatedMap = np.zeros((gridsizeX*gridFactor, gridsizeY*gridFactor))

#Obstacles i groundTruth
groundTruth[0, 0:gridsizeX*gridFactor-1] = 1
groundTruth[0:gridsizeX*gridFactor-1, 0] = 1
groundTruth[gridsizeX*gridFactor-1, 0:gridsizeX*gridFactor-1] = 1
groundTruth[0:gridsizeX*gridFactor-1, gridsizeX*gridFactor-1] = 1

groundTruth[10:100, 10:100] = 1
groundTruth[300:400, 200:300] = 1
groundTruth[800:900, 700:800] = 1
groundTruth[690:780, 200:400] = 1

movementQueue = []
chaseAngle = 0

class Agent:
    def __init__ (self, timeLength):
        self.speed = 0
        self.angle = 0
        self.time = timeLength
        self.position = [(gridsizeX*gridFactor)/2, (gridsizeY*gridFactor)/2]
        self.searching = False
        self.preservation = False
        
    def move(self):
        if len(movementQueue) <= 0:
            #if self.test == True:
            self.speed = gridFactor
            angularVel = 0
            self.position[0] += self.speed * self.time * np.cos(self.angle + angularVel*self.time) 
            self.position[1] += self.speed * self.time * np.sin(self.angle + angularVel*self.time)
            self.angle += angularVel * self.time
            self.searching = False
        elif len(movementQueue) >= 1:
            firstElementOfMovementQueue = movementQueue.pop(0)
            self.speed = firstElementOfMovementQueue[0]
            angularVel = firstElementOfMovementQueue[1]
            #print(angularVel)
            self.position[0] += self.speed * self.time * np.cos(self.angle + angularVel*self.time) 
            self.position[1] += self.speed * self.time * np.sin(self.angle + angularVel*self.time)
            self.angle += angularVel * self.time
            self.searching = True
            #if len(movementQueue) == 1:
            #    self.test = False

    def search(self, distance):
        #Lav cirkel med tangent der er parrallel med væg, beregn arclength, diskretiser i 40 stykker for at beregne movement queue
        radius = distance - 30*(gridFactor/100)
        arcLength = 1/2 * np.pi * radius
        for i in range(1, 41):
            movementQueue.append([arcLength, -np.pi/2])
        print(movementQueue)

    def selfpreservation(self):
            for j in range(1, 21):
                movementQueue.insert(0, [-gridFactor, 0])
            for i in range(1, 11):
                movementQueue.insert(0, [0, 0])
            self.preservation = True
    
    def chaseInit(self, angle):
        chaseAngle = self.angle + angle
        for i in range(0, 120):
            movementQueue.insert(0, [gridFactor, 0])
        for j in range(0, 40):
            movementQueue.insert(0, [gridFactor, angle])



class Sensor:
    def __init__(self, givenAngle):
        self.relativeAngle = givenAngle

    def sense(self, agentPosition, agentAngle):

        for r in np.arange(0, measurementMaxDist):
            xi = int(agentPosition[0] + r * np.cos(agentAngle + self.relativeAngle))
            yi = int(agentPosition[1] + r * np.sin(agentAngle + self.relativeAngle))
            
            if r <= measurementMaxDist - 1:
                if 0 <= xi < gridsizeX*gridFactor and 0 <= yi < gridsizeY*gridFactor:
                    if groundTruth[xi, yi] > 0.5:
                        distance = r
                        return distance, xi, yi
                        break
        
        return measurementMaxDist, 0, 0
            

def update_estimate(distance, xPos, yPos, agentPos, agentAngle, sensorRelativeAngle):
    #Forøg den square som der blev detected noget på.
    estimatedMap[yPos, xPos] = min(1, estimatedMap[xPos, yPos] + 1)
    #Gør alle på vejen der hen mindre:
    for r in np.arange(0, distance-1):
        xi = int(agentPos[0] + r * np.cos(agentAngle + sensorRelativeAngle))
        yi = int(agentPos[1] + r * np.sin(agentAngle + sensorRelativeAngle))

        estimatedMap[xi, yi] = max(0, estimatedMap[xi, yi]-0.5)






agent = Agent(timeStepLength)
#sensor1 = Sensor(0)
sensors = [Sensor(0), Sensor(np.pi/3), Sensor(-np.pi/3)]
#plt.imshow(groundTruth)
for t in range(0, 2400):
    plt.cla()
    plt.plot(agent.position[0], agent.position[1], 'ro')
    for sensor in sensors:
        distance, xPos, yPos = sensor.sense(agent.position, agent.angle)
        if distance <= measurementMaxDist-1:
            update_estimate(distance, xPos, yPos, agent.position, agent.angle, sensor.relativeAngle)
        x = (agent.position[0] + measurementMaxDist*np.cos(agent.angle + sensor.relativeAngle))
        y = (agent.position[1] + measurementMaxDist*np.sin(agent.angle + sensor.relativeAngle))
        plt.plot([agent.position[0], x], [agent.position[1], y])
    distance, xPos, yPos = sensors[0].sense(agent.position, agent.angle)
    if distance <= (gridFactor*2)/np.pi+30 and agent.searching == False:
        agent.chaseInit(-np.pi)
        #agent.search(distance)
    #agent.search(distance)
    if distance <= 30*(gridFactor/100) and agent.preservation == False:
        agent.selfpreservation()
    if len(movementQueue) >= 1:
        print("distance:", distance,  "movementQueue:", movementQueue[0])
    agent.move()
    plt.imshow(estimatedMap)
    plt.pause(0.025)

count = np.count_nonzero(groundTruth == estimatedMap)
percentageMapped = count/((gridsizeX*gridFactor)*(gridsizeY*gridFactor))
print(percentageMapped)

plt.show()