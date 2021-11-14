import pybullet as p
import time
import pybullet_data
import math
import matplotlib.pyplot as plt
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version

p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])
robotId = p.loadURDF("olympian.urdf",startPos, startOrientation, 
                   # useMaximalCoordinates=1, ## New feature in Pybullet
                   flags=p.URDF_USE_INERTIA_FROM_FILE)

print("==========SIMULATION ENABLED================")

#GET JOINT INFO

print(p.getNumJoints(robotId))
for i in range(0, p.getNumJoints(robotId)):
    print(p.getJointInfo(robotId, i)[0:13])
    
listOfJointIndeces = []
for i in range(0, p.getNumJoints(robotId)):
    listOfJointIndeces.append(i)


def calcCOM():
    
    #CALCULATE COM
    masstimesxpossum = 0.0
    masstimesypossum = 0.0
    masstimeszpossum = 0.0
    masssum = 0.0
    for i in range(0, p.getNumJoints(robotId) -1):
        
        # if(i >= 0):
        #     print(p.getJointInfo(robotId, i)[0:13])
        
        wheight = p.getDynamicsInfo(robotId, i)[0]
        xpos = p.getLinkState(robotId, i)[0][0]
        ypos = p.getLinkState(robotId, i)[0][1]
        zpos = p.getLinkState(robotId, i)[0][2]
        
        masstimesxpossum += (wheight * xpos)
        masstimesypossum += (wheight * ypos)
        masstimeszpossum += (wheight * zpos)
        masssum += wheight
        
        # print(wheight)
        # print(xpos)
        # print(ypos)
        # print(zpos)
        # print("\n")
        p.stepSimulation()
    com = (masstimesxpossum/masssum, masstimesypossum/masssum, masstimeszpossum/masssum)
    
    print("mass: " + str(masssum))
    print("center of mass: " + str(com))
    print("\n")
    return com

newi = []

torque = 50
comPos = []
comPos2 = []
def squatDown():
        
    for i in range(0,30):
        comPos.append(calcCOM())
        newi.insert(0, i)
        positionsList = [0] * 23
        positionsList[11] = i/100 * 1 * math.pi #leg pitch 1
        positionsList[17] = i/100 * -1 * math.pi #leg pitch 2
    
        positionsList[14] = i/50 * math.pi #leg knee 1
        positionsList[20] = i/50 * -1  * math.pi #leg knee 2

        positionsList[16] = i/100 * 1  * math.pi#foot pitch 1
        positionsList[22] = i/100 * -1  * math.pi#foot pitch 2
        
        # positionsList[0] = math.pi / 4
        forceArray = [torque]*23
    
        p.setJointMotorControlArray(robotId, listOfJointIndeces, p.POSITION_CONTROL, targetPositions = positionsList, forces = forceArray)
        p.stepSimulation()


def squatUp():
    for i in newi:
        comPos2.append(calcCOM())
        positionsList = [0] * 23
        positionsList[11] = i/100 * 1 * math.pi #leg pitch 1
        positionsList[17] = i/100 * -1 * math.pi #leg pitch 2
    
        positionsList[14] = i/50 * math.pi #leg knee 1
        positionsList[20] = i/50 * -1  * math.pi #leg knee 2

        positionsList[16] = i/100 * 1  * math.pi#foot pitch 1
        positionsList[22] = i/100 * -1  * math.pi#foot pitch 2
    
        # positionsList[0] = math.pi / 4
        forceArray = [torque]*23
    
        p.setJointMotorControlArray(robotId, listOfJointIndeces, p.POSITION_CONTROL, targetPositions = positionsList, forces = forceArray)
        p.stepSimulation()

squatDown()
squatUp()


plt.plot(newi, comPos)
for i in newi[:-1]:
    newi[i] += 30
plt.plot(newi, comPos2)
plt.show()
    
# for i in range(0,1000000):
#     p.stepSimulation()
#     time.sleep(1./240.)
    
