import pybullet as p
import time
import pybullet_data
import math
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
for i in range(0, p.getNumJoints(robotId)-1):
    print(p.getJointInfo(robotId, i)[0:13])
    

#ACTUATE LINK 4  0.5 * pi RADIANS with 15NM of TORQUE
p.setJointMotorControl2(robotId, 4, controlMode = p.POSITION_CONTROL, targetPosition = 0.5 * math.pi, force = 15)



listOfJointIndeces = []
for i in range(0, p.getNumJoints(robotId)):
    listOfJointIndeces.append(i)

#INVERSE KINEMATICS 
#GET JOINT ANGLES NEEDED TO MOVE LINK 10 to (0. 0.6, 2)
#FORCE IS 25NM for every link bc im too lazy to custom set it for every link
#IT FALLS BECAUSE its a lot of torque exerted in a small amount of time

ikList = p.calculateInverseKinematics(robotId, 10, [0,0.6,2] )
p.setJointMotorControlArray(robotId, listOfJointIndeces, p.POSITION_CONTROL, 
                            targetPositions = ikList, forces = [25]*23)



#CALCULATE COM
masstimesxpossum = 0.0
masstimesypossum = 0.0
masstimeszpossum = 0.0
masssum = 0.0
for i in range(0, p.getNumJoints(robotId) -1):
    
    if(i >= 0):
        print(p.getJointInfo(robotId, i)[0:13])
    
    wheight = p.getDynamicsInfo(robotId, i)[0]
    xpos = p.getLinkState(robotId, i)[0][0]
    ypos = p.getLinkState(robotId, i)[0][1]
    zpos = p.getLinkState(robotId, i)[0][2]
    
    masstimesxpossum += (wheight * xpos)
    masstimesypossum += (wheight * ypos)
    masstimeszpossum += (wheight * zpos)
    masssum += wheight
    
    print(wheight)
    print(xpos)
    print(ypos)
    print(zpos)
    print("\n")

com = (masstimesxpossum/masssum, masstimesypossum/masssum, masstimeszpossum/masssum)
print("==========COM APROX EQUALS===========")
print(com)
print("\n")
#STEP SIMULATION
for i in range(0,10000):
    p.stepSimulation()
    time.sleep(1./240.)
    

print("========REALTIME SIMULATION DISABLED===============")
# p.calculateInverseKinematics(robotId, )
