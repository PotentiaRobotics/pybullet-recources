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
    

    return com

def error():
    robotCOM = calcCOM()
    realRobotCOM = robotCOM[1]
    footCOM = p.getLinkState(robotId, 15)
    realFootCOM = footCOM[0][1]
    error = realRobotCOM - realFootCOM
    print(realFootCOM, realRobotCOM)
    return error

Array = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
positions = [0]*23
forces = [1000]*23
indexes = list(range(0,23))
ka = 0
# p.setJointMotorControl2(robotId, 20, p.POSITION_CONTROL, targetPosition = 0, force = 10000)
# p.setJointMotorControl2(robotId, 14, p.POSITION_CONTROL, targetPosition = 0, force = 10000)

while(True):
    Kc = -0.05
    bias = 0
    initial_error = error()
    ka += Kc*initial_error + bias
    print(initial_error)
    positions[21] = ka
    positions[15] = ka
    positions[18] = ka
    positions[12] = ka
    p.setJointMotorControlArray(robotId, indexes, p.POSITION_CONTROL, targetPositions = positions, forces = forces)