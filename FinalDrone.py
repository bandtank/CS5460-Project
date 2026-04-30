# Make sure to have the add-on "ZMQ remote API" running in CoppeliaSim. Do not launch simulation, but run this script. Do not mannually close this script, stop the simulation so drawing is correctly deleted


# NOTE: I have not coded much in python, so I am not sure how to export the necessary packages with this script. Looking online said that the person with the script usually downloads required packages.
# I am not sure if this is correct at all, so please tell me to change this in the future if I need to. The package I used can be found with the command "pip install coppeliasim-zmqremoteapi-client".


import math
import time
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import array
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def moveDrone():
    # Vertical control:
    targetPos=sim.getObjectPosition(target)
    pos=sim.getObjectPosition(d)
    l,a=sim.getVelocity(robot)
    e=(targetPos[2]-pos[2])
    global cumul
    global lastE
    global pAlphaE
    global pBetaE
    global psp2
    global psp1
    global prevEuler
    cumul= cumul + e
    pv=pParam*e
    thrust=5.45+pv+(iParam*cumul)+(dParam*(e-lastE))+(l[2]*vParam)
    lastE=e
    
    # Horizontal control: 
    sp=sim.getObjectPosition(target,d)
    m=sim.getObjectMatrix(d)
    vx=[1,0,0]
    vx=sim.multiplyVector(m,vx)
    vy=[0,1,0]
    vy=sim.multiplyVector(m,vy)

    alphaE=(vy[2]-m[11])
    alphaCorr=(.5*alphaE)+(3.1*(alphaE-pAlphaE))
    betaE=(vx[2]-m[11])
    betaCorr=(-.5*betaE)-(3.1*(betaE-pBetaE))
    pAlphaE=alphaE
    pBetaE=betaE
    alphaCorr=alphaCorr+(sp[1]*0.005)+(1*(sp[1]-psp2))
    betaCorr=betaCorr-(sp[0]*0.005)-(1*(sp[0]-psp1))
    psp2=sp[1]
    psp1=sp[0]
    # Rotational control:
    euler=sim.getObjectOrientation(target, robot)
    rotCorr=(euler[2]*0.0005)+(2*(euler[2]-prevEuler))
    prevEuler=euler[2]
    
    # Decide of the motor velocities:
    handlePropeller(0,thrust*(1-alphaCorr+betaCorr+rotCorr))
    handlePropeller(1,thrust*(1-alphaCorr-betaCorr-rotCorr))
    handlePropeller(2,thrust*(1+alphaCorr-betaCorr+rotCorr))
    handlePropeller(3,thrust*(1+alphaCorr+betaCorr-rotCorr))




def handlePropeller(index,particleVelocity):
    propellerRespondable=propellerHandles[index]
    propellerJoint=jointHandles[index]
    propeller=sim.getObjectParent(propellerRespondable)
    particleObject=particleObjects[index]
    maxParticleDeviation=math.tan(particleScatteringAngle*0.5*math.pi/180)*particleVelocity
    notFullParticles=0

    t=sim.getSimulationTime()
    sim.setJointPosition(propellerJoint,t*10)
    ts=sim.getSimulationTimeStep()
    
    m=sim.getObjectMatrix(propeller)
    particleCnt=0
    pos=[0,0,0]
    dir=[0,0,1]
    
    requiredParticleCnt=particleCountPerSecond*ts+notFullParticles
    notFullParticles=requiredParticleCnt % 1
    requiredParticleCnt=math.floor(requiredParticleCnt)
    while (particleCnt<requiredParticleCnt):
        # we want a uniform distribution:
        x=(random.random()-0.5)*2
        y=(random.random()-0.5)*2
        if (x*x+y*y<=1):
            if (simulateParticles == True):
                pos[0]=x*0.08
                pos[1]=y*0.08
                pos[2]=-particleSize*0.6
                dir[0]=pos[0]+(random.random()-0.5)*maxParticleDeviation*2
                dir[1]=pos[1]+(random.random()-0.5)*maxParticleDeviation*2
                dir[2]=pos[2]-particleVelocity*(1+0.2*(random.random()-0.5))
                pos=sim.multiplyVector(m,pos)
                dir=sim.multiplyVector(m,dir)
                itemData=[pos[0],pos[1],pos[2],dir[0],dir[1],dir[2]]
                sim.addParticleObjectItem(particleObject,itemData)

            particleCnt=particleCnt+1


    # Apply a reactive force onto the body:
    totalExertedForce=particleCnt*particleDensity*particleVelocity*math.pi*particleSize*particleSize*particleSize/(6*ts)
    force=[0,0,totalExertedForce]
    m[3]=0
    m[7]=0
    m[11]=0
    force=sim.multiplyVector(m,force)
    rotDir=1-((index%2)*2)
    torque=[0,0,rotDir*0.002*particleVelocity]
    torque=sim.multiplyVector(m,torque)
    sim.addForceAndTorque(propellerRespondable,force,torque)



def dijkstraNavigation():
    currentPoint = [objectAbsolutePosition[0], objectAbsolutePosition[1]] 
    destinatiPoint = listOfPoints[newPointIndex]
    closestPoint = listOfPoints[0]
    closestDist = distance(currentPoint, listOfPoints[0])
    lineIntersctions = []
    for i in range(0, len(listOfPoints)):
        curDist = distance(currentPoint, listOfPoints[i])
        if curDist < closestDist:
            p1 = listOfPoints[i]
            p2 = currentPoint
            collisions = False

            for k in range(len(obstacles)):
                    p3 = [obstacles[k][0][0], obstacles[k][0][1]]
                    p4 = [obstacles[k][1][0], obstacles[k][1][1]]
                    if line_intersection(p1,p2,p3,p4) == True:
                        collisions = True
            if collisions != True:
                closestDist = curDist
                closestPoint = listOfPoints[i]
    
    for i in range(len(listOfPoints)):
        destinationNodes = []
        for j in range(len(listOfPoints)):
            if i != j:
                p1 = listOfPoints[i]
                p2 = listOfPoints[j]
                collisions = False

                for k in range(len(obstacles)):
                    p3 = [obstacles[k][0][0], obstacles[k][0][1]]
                    p4 = [obstacles[k][1][0], obstacles[k][1][1]]
                    if line_intersection(p1,p2,p3,p4) == True:
                        collisions = True
                for k in range(len(mazeBars)):
                    if (is_between_points(p1, p2, mazeBars[k], .45) == True and
                        (p1[0] < mazeBars[k][0] < p2[0] or p1[0] > mazeBars[k][0] > p2[0]) and
                        (p1[1] < mazeBars[k][1] < p2[1] or p1[1] > mazeBars[k][1] > p2[1])):
                        collisions = True

                
                if collisions != True:
                    destinationNodes.append([listOfPoints[i][0], listOfPoints[i][1], listOfPoints[j][0], listOfPoints[j][1]])
        if len(destinationNodes) > 0:
            lineIntersctions.append(destinationNodes)
    
    totalPoints = [[closestPoint[0], closestPoint[1], 0]]
    
    # Adding all nodes to a list with inf cost as intial value
    for i in range(len(lineIntersctions)):
        curPoint = lineIntersctions[i][0]
        pointAlreadyAdded = False
        for j in range(len(totalPoints)):
            p1 = [curPoint[0], curPoint[1]]
            p2 = [totalPoints[j][0], totalPoints[j][1]]
            if(p1[0] == p2[0] and p1[1] == p2[1]):
                pointAlreadyAdded = True
        if not pointAlreadyAdded:
            totalPoints.append([curPoint[0], curPoint[1], float('inf')])
    # Plotting viable pathes
    for i in range(len(lineIntersctions)):
        for j in range(len(lineIntersctions[i])):
            curLink = lineIntersctions[i][j]

            start_point = (curLink[0], curLink[1])  # (x1, y1)
            end_point = (curLink[2], curLink[3])  # (x2, y2)


            x_points = np.array([start_point[0], end_point[0]])
            y_points = np.array([start_point[1], end_point[1]])

            plt.plot(x_points, y_points, color='yellow')
    plt.show()
    curPoint = np.array(totalPoints[0])
    searching = True
    pointsToSearch = []
    searchedPoints = []

    # Finding the smallest cost for each node according to dijkstra's
    while searching:
        for i in range(len(lineIntersctions)):
            checkPoint = lineIntersctions[i][0]
            if(curPoint[0] == checkPoint[0] and curPoint[1] == checkPoint[1]):
                for j in range(len(lineIntersctions[i])):
                    curLink = lineIntersctions[i][j]
                    destinationPoint = np.array([curLink[2], curLink[3]])
                    pointsToSearch.append(destinationPoint)
                    dist = curPoint[2] + np.linalg.norm(destinationPoint-curPoint[:2])
                    for k in range(len(totalPoints)):
                        if destinationPoint[0] == totalPoints[k][0] and destinationPoint[1] == totalPoints[k][1] and (totalPoints[k][2] == float('inf') or totalPoints[k][2] > dist):
                            totalPoints[k][2] = float(dist)
                break

        i = 0
        while i < len(pointsToSearch) and len(pointsToSearch) > 0:
            for j in range(len(searchedPoints)):
                if(pointsToSearch[i][0] == searchedPoints[j][0] and pointsToSearch[i][1] == searchedPoints[j][1]):
                    pointsToSearch.pop(i)
                    i = i -1
                    break
            i = i + 1		

        if len(pointsToSearch) > 0:
            for j in range(len(totalPoints)):
                if(pointsToSearch[0][0] == totalPoints[j][0] and pointsToSearch[0][1] == totalPoints[j][1]):
                    curPoint = totalPoints[j]
                    searchedPoints.append(pointsToSearch[0])
                    pointsToSearch.pop(0)
                    break
        else:
            searching = False

    print("MIN")
    # From the goal point, need to trace back the minimum path
    curPoint = [destinatiPoint[0], destinatiPoint[1], .7]
    pointsToVisit = [curPoint]
    searching = True

    while searching:
        for i in range(len(lineIntersctions)):
            checkPoint = lineIntersctions[i][0]
            if(curPoint[0] == checkPoint[0] and curPoint[1] == checkPoint[1]):
                lowestDist = -1
                lowestDest = []
                for j in range(len(totalPoints)):
                    if(checkPoint[2] == totalPoints[j][0] and checkPoint[3] == totalPoints[j][1]):
                        lowestDist = totalPoints[j][2]
                        lowestDest = [totalPoints[j][0], totalPoints[j][1]]
                        break
                
                for j in range(1, len(lineIntersctions[i])):
                    curLink = lineIntersctions[i][j]
                    for k in range(len(totalPoints)):
                        if(curLink[2] == totalPoints[k][0] and curLink[3] == totalPoints[k][1] and totalPoints[k][2] < lowestDist):
                            lowestDist = totalPoints[k][2]
                            lowestDest = [totalPoints[k][0], totalPoints[k][1]]
                pointsToVisit.insert(0, [lowestDest[0], lowestDest[1], 0.7])
                curPoint = lowestDest
                if lowestDist == 0:
                    searching = False

                break			


    print(pointsToVisit)
    return pointsToVisit

def sysCall_sensing():
    m=sim.getObjectMatrix(sensor)
    sim.addDrawingObjectItem(sphereContainer,None)
    res,packet1,packet2=sim.handleVisionSensor(sensor)
    lineSegment = []
    newPoint = False
    coord = []
    coordLeft = []
    coordRight = []
    if res>=0 and len(packet2) > 0:
        blobCnt=int(packet2[0])
        valCnt=int(packet2[1])
        for i in range(blobCnt):
            blobSize=packet2[2+valCnt*i]
            blobOrientation=packet2[2+valCnt*i+1]
            blobPositionX=packet2[2+valCnt*i+2]
            blobPositionY=packet2[2+valCnt*i+3]
            blobWidth=packet2[2+valCnt*i+4]
            blobHeight=packet2[2+valCnt*i+5]
            toAdd = 0
            if blobPositionX > .5:
                toAdd = 1
            depth, test = sim.getVisionSensorDepth(sensor,1,[toAdd+math.floor(((blobPositionX))*(resX-0.99)),1+math.floor(blobPositionY*(resY-0.99))],[1,1])
            depth=sim.unpackFloatTable(depth)
            coord=[0,0,depth[0]]
            x=0.5-blobPositionX
            y=blobPositionY-0.5
            coord[0]=depth[0]*math.tan(xAngle*0.5)*(x)/0.5
            coord[1]=depth[0]*math.tan(yAngle*0.5)*(blobPositionY-0.5)/0.5
            coord=sim.multiplyVector(m,coord)
            sim.addDrawingObjectItem(sphereContainer,coord)
            depthLeft, test=sim.getVisionSensorDepth(sensor,1,[toAdd+math.floor(((max((blobSize*.5)-.015,0)+blobPositionX)*(resX-0.99))),1+math.floor(blobPositionY*(resY-0.99))],[1,1])
            depthLeft=sim.unpackFloatTable(depthLeft)
            coordLeft=[0,0,depthLeft[0]]
            x=0.5-blobPositionX - max((blobSize*.5),0)
            y=blobPositionY-0.5


            coordLeft[0]=depthLeft[0]*math.tan(xAngle*0.5)*(x)/0.5
            coordLeft[1]=depthLeft[0]*math.tan(yAngle*0.5)*(blobPositionY-0.5)/0.5
            coordLeft=sim.multiplyVector(m,coordLeft)
            sim.addDrawingObjectItem(sphereContainer,coordLeft)
           
            depthRight, test = sim.getVisionSensorDepth(sensor,1,[toAdd+math.floor((blobPositionX-max((blobSize*.5)-.015,0))*(resX-0.99)),1+math.floor(blobPositionY*(resY-0.99))],[1,1])
            depthRight=sim.unpackFloatTable(depthRight)
            coordRight=[0,0,depthRight[0]]
            x=0.5-blobPositionX + max((blobSize*.5),0)
            y=blobPositionY-0.5

            coordRight[0]=depthRight[0]*math.tan(xAngle*0.5)*(x)/0.5
            coordRight[1]=depthRight[0]*math.tan(yAngle*0.5)*(blobPositionY-0.5)/0.5
            coordRight=sim.multiplyVector(m,coordRight)
            sim.addDrawingObjectItem(sphereContainer,coordRight)
    return coord, coordLeft, coordRight

# Function for distance formula between two points
def distance(p1, p2):
    return math.sqrt(((p1[0] - p2[0]) ** 2) + ((p1[1] - p2[1]) ** 2))

# Function for if point is inbetween two points
def is_between_points_dist(A, B, C):
    AB = distance(A, B)
    AC = distance(A, C)
    CB = distance(C, B)
       
    return (AC + CB) - AB <= .1

# Function for if point is inbetween two points
def is_between_points(A, B, C, dif):
    C, A, B = np.array([C[0], C[1]]), np.array([A[0], A[1]]), np.array([B[0], B[1]])
    AB = B - A
    AC = C - A
    t = np.clip(np.dot(AC, AB) / (np.dot(AB, AB) + .001), 0, 1)
    closest_point = A + t * AB
    return np.linalg.norm(C - closest_point) <= dif

#Function for if two lines intersect
def line_intersection(p1, p2, p3, p4):
   
    D = ((p1[0] - p2[0]) * (p3[1] - p4[1])) -  ((p1[1] - p2[1]) * (p3[0] - p4[0]))+.001
    if D == 0:
        return False
    t = ((((p1[0] - p3[0]) * (p3[1] - p4[1])) - ((p1[1] - p3[1]) * (p3[0] - p4[0]))) / D)

    u = (((p1[0] - p3[0]) * (p1[1] - p2[1])) - ((p1[1] - p3[1]) * (p1[0] - p2[0]))) / D

    if 0 <= t <= 1 and 0 <= u <= 1:
        return True
    else:
        return False


def newRRTNode():
    # This will pull the tree towards a random point
    closestDist = -1
    closestLoc = [] 
    p = len(listOfPoints)-1
    while p >= 0:
        print(listOfPoints[p])
        for k in range(len(mazeNotSearched)):
            sim.step()
            moveDrone()
            if (distance(listOfPoints[p],[float(mazeNotSearched[k][0])+.125, float(mazeNotSearched[k][1])+.125]) < closestDist or closestDist == -1) and maze[k][2] != "V" and maze[k][2] != "B":
                col = False
                        # Need to make sure that the point isn't too close to the maze walls or that the edge connecting the new point and the closest is through a wall
                for i in range(len(obstacles)):
                    p1 = (obstacles[i][0][0], obstacles[i][0][1])  # (x1, y1)
                    p2 = (obstacles[i][1][0], obstacles[i][1][1])  # (x2, y2)
                    # If the edge between the points goes through a wall, it is forced back to the correct side of the wall
                    if line_intersection(p1,p2, listOfPoints[p], [float(mazeNotSearched[k][0])+.125, float(mazeNotSearched[k][1])+.125]) == True:
                        col = True

                    if is_between_points_dist(p1,p2, [float(mazeNotSearched[k][0])+.125, float(mazeNotSearched[k][1])+.125]) == True:
                        col = True

                    if (is_between_points(listOfPoints[p], [float(mazeNotSearched[k][0])+.125, float(mazeNotSearched[k][1])+.125], p1, .45) == True and
                        (listOfPoints[p][0]-.2 < p1[0] < float(mazeNotSearched[k][0])+.125+.2 or listOfPoints[p][0]+.2 > p1[0] > float(mazeNotSearched[k][0])+.125-.2) and
                        (listOfPoints[p][1]-.2 < p1[1] < float(mazeNotSearched[k][1])+.125+.2 or listOfPoints[p][1]+.2 > p1[1] > float(mazeNotSearched[k][1])+.125-.2)):
                        col = True
                    if (is_between_points(listOfPoints[p], [float(mazeNotSearched[k][0])+.125, float(mazeNotSearched[k][1])+.125], p2, .45) == True and
                        (listOfPoints[p][0]-.2 < p2[0] < float(mazeNotSearched[k][0])+.125+.2 or listOfPoints[p][0]+.2 > p2[0] > float(mazeNotSearched[k][0])+.125-.2) and
                        (listOfPoints[p][1]-.2 < p2[1] < float(mazeNotSearched[k][1])+.125+.2 or listOfPoints[p][1]+.2 > p2[1] > float(mazeNotSearched[k][1])+.125-.2)):
                        col = True
                if col == False:
                    closestLoc = [float(mazeNotSearched[k][0])+.125, float(mazeNotSearched[k][1])+.125]
                    closestDist = distance(listOfPoints[p],[float(mazeNotSearched[k][0])+.125, float(mazeNotSearched[k][1])+.125])#abs(listOfPoints[p][0] - float(mazeNotSearched[k][0])+.125) + abs(listOfPoints[p][1] - float(mazeNotSearched[k][1])+.125)
                    print(closestDist)
                    if(closestDist < .75 and closestDist != -1):
                        break
        if closestDist != -1:
            break
        p = p-1
        print(p)
        if p == -1:
            return -1,-1, False
                    
    xNew = np.array(closestLoc)
    targetAbsolutePosition=np.array(sim.getObjectPosition(target,sim.handle_world))
    targetAbsoluteRotation=np.array(sim.getObjectOrientation(target))

    rotation = math.atan2(xNew[1] - targetAbsolutePosition[1], xNew[0] - targetAbsolutePosition[0]) - 1.5708
    print(rotation)
    sim.setObjectOrientation(target, [0, 0, rotation])
    for i in range(100):
        sim.step()
        checkPoints()
        moveDrone()
    print(xNew)
    # Making sure the point isn't in a wall
    collisions = False
    if collisions != True:
        # Using the first stored point, the start point, as the intial base for the closest point
        smallestDist = distance(listOfPoints[0], xNew)
        closestPoint = listOfPoints[0]
        closestCost = listOfCosts[0]

        # Checking all other points for if there is a closer point
        for i in range(len(listOfPoints)):
            if distance(listOfPoints[i], xNew) < smallestDist:
                smallestDist = distance(listOfPoints[i], xNew)
                closestPoint = np.array(listOfPoints[i])
                closestCost = listOfCosts[i]

        # Once the closest point is found, a new point is made .75 distance from the closest point in the direction of the new point
        newPoint = xNew

        # Need to make sure that the point isn't too close to the maze walls or that the edge connecting the new point and the closest is through a wall
        for i in range(len(obstacles)):
            p1 = (obstacles[i][0][0], obstacles[i][0][1])  # (x1, y1)
            p2 = (obstacles[i][1][0], obstacles[i][1][1])  # (x2, y2)
            # If the edge between the points goes through a wall, it is forced back to the correct side of the wall
            if line_intersection(p1,p2, closestPoint, newPoint) == True:
                if (closestPoint[0] <= p1[0] and closestPoint[0] <= p2[0]):
                    newPoint[0] = p1[0]-.45
                elif (closestPoint[0] >= p1[0] and closestPoint[0] >= p2[0]):
                    newPoint[0] = p1[0] + .45
                if (closestPoint[1] <= p1[1] and closestPoint[1] <= p2[1]):
                    newPoint[1] = p1[1] -.45
                elif (closestPoint[1] >= p1[1] and closestPoint[1] >= p2[1]):
                    newPoint[1] = p1[1] +.45

            # Verifying that the point isn't too close to the wall to make navigation smoother
            if (newPoint[0] <= p1[0] and newPoint[0] <=  p2[0]) and abs(newPoint[0] -  p1[0]) < .45 and (p1[1] <= newPoint[1] <= p2[1] or p1[1] >= newPoint[1] >= p2[1]):
                newPoint[0] = p1[0]-.45
            elif (newPoint[0] >= p1[0] and newPoint[0] >= p2[0]) and abs(p1[0] - newPoint[0]) < .45 and (p1[1] <= newPoint[1] <= p2[1] or p1[1] >= newPoint[1] >= p2[1]):
                newPoint[0] = p1[0]+.45
           
            if (newPoint[1] <= p1[1] and newPoint[1] <= p2[1]) and abs(newPoint[1] - p1[1]) < .45 and (p1[0] <= newPoint[0] <= p2[0] or p1[0] >= newPoint[0] >= p2[0]):
                newPoint[1] = p1[1]-.45
            elif (newPoint[1] >= p1[1] and newPoint[1] >= p2[1]) and abs(p1[1] - newPoint[1]) < .45 and (p1[0] <= newPoint[0] <= p2[0] or p1[0] >= newPoint[0] >= p2[0]):
                newPoint[1] = p1[1]+.45

        # Storing the cost of the new node and checking for better nearby points
        curLowestCost = np.linalg.norm(((newPoint-closestPoint)/np.linalg.norm((newPoint-closestPoint)))) + closestCost
        
        for i in range(len(listOfPoints)):
            pointToComp = np.array(listOfPoints[i])
           
            intersection = False
            for k in range(len(obstacles)):
                p1 = (obstacles[k][0][0], obstacles[k][0][1])  # (x1, y1)
                p2 = (obstacles[k][1][0], obstacles[k][1][1])  # (x2, y2)
                if line_intersection(p1,p2, pointToComp, newPoint) == True:
                    intersection = True


            if 0 < abs(newPoint[0] - pointToComp[0]) <= expandRadius and 0 < abs(newPoint[1] - pointToComp[1]) <= expandRadius and not intersection:
                if np.linalg.norm(((newPoint-pointToComp)/np.linalg.norm((newPoint-pointToComp)))) + listOfCosts[i] < curLowestCost:
                    closestPoint = pointToComp
                    curLowestCost = np.linalg.norm(((newPoint-pointToComp)/np.linalg.norm((newPoint-pointToComp)))) + listOfCosts[i]
               
        
        # Storing the new point, edge, and cost of the node
        listOfPoints.append([newPoint[0], newPoint[1]])
        listOfCosts.append(curLowestCost)
        listOfEdges.append([closestPoint[0],closestPoint[1], newPoint[0], newPoint[1]])
        
        
        
    # Need to trace back from the final point to the start
    curPoint = listOfPoints[len(listOfPoints)-1]
   
    return len(listOfPoints)-1, len(listOfEdges)-1, True


def reevaluatePoint():
# Need to make sure that the point isn't too close to the maze walls or that the edge connecting the new point and the closest is through a wall
    global pointsToTraverse
    update = False
    objectAbsolutePosition=np.array(sim.getObjectPosition(robot,sim.handle_world ))
    targetAbsolutePosition=np.array(sim.getObjectPosition(target,sim.handle_world))
    for pointIndex in range(1, len(listOfPoints)):
        update = False
        edgeIndex = pointIndex - 1
        closestPoint = [listOfEdges[edgeIndex][0], listOfEdges[edgeIndex][1]]
        for i in range(len(obstacles)):
            p1 = (obstacles[i][0][0], obstacles[i][0][1])  # (x1, y1)
            p2 = (obstacles[i][1][0], obstacles[i][1][1])  # (x2, y2)
            # If the edge between the points goes through a wall, it is forced back to the correct side of the wall
            if line_intersection(p1,p2, closestPoint, listOfPoints[pointIndex]) == True:
                if (closestPoint[0] <= p1[0] and closestPoint[0] <= p2[0]):
                    update = True
                    listOfPoints[pointIndex][0] = p1[0] - .45
                elif (closestPoint[0] >= p1[0] and closestPoint[0] >= p2[0]):
                    update = True
                    listOfPoints[pointIndex][0] = p1[0] + .45
                if (closestPoint[1] <= p1[1] and closestPoint[1] <= p2[1]):
                    update = True
                    listOfPoints[pointIndex][1] = p1[1] - .45
                elif (closestPoint[1] >= p1[1] and closestPoint[1] >= p2[1]):
                    update = True
                    listOfPoints[pointIndex][1] = p1[1] + .45

            # Verifying that the point isn't too close to the wall to make navigation smoother
            if (listOfPoints[pointIndex][0] <= p1[0] and listOfPoints[pointIndex][0] <=  p2[0]) and (closestPoint[0] <= p1[0] and closestPoint[0] <= p2[0]) and abs(listOfPoints[pointIndex][0] -  p1[0]) < .45 and (p1[1] <= listOfPoints[pointIndex][1] <= p2[1] or p1[1] >= listOfPoints[pointIndex][1] >= p2[1]):
                update = True
                listOfPoints[pointIndex][0] = p1[0]-.45
            elif (listOfPoints[pointIndex][0] >= p1[0] and listOfPoints[pointIndex][0] >= p2[0]) and (closestPoint[0] >= p1[0] and closestPoint[0] >= p2[0]) and abs(p1[0] - listOfPoints[pointIndex][0]) < .45 and (p1[1] <= listOfPoints[pointIndex][1] <= p2[1] or p1[1] >= listOfPoints[pointIndex][1] >= p2[1]):
                update = True
                listOfPoints[pointIndex][0] = p1[0]+.45
        
            if (listOfPoints[pointIndex][1] <= p1[1] and listOfPoints[pointIndex][1] <= p2[1]) and (closestPoint[1] <= p1[1] and closestPoint[1] <= p2[1]) and abs(listOfPoints[pointIndex][1] - p1[1]) < .45 and (p1[0] <= listOfPoints[pointIndex][0] <= p2[0] or p1[0] >= listOfPoints[pointIndex][0] >= p2[0]):
                update = True
                listOfPoints[pointIndex][1] = p1[1]-.45
            elif (listOfPoints[pointIndex][1] >= p1[1] and listOfPoints[pointIndex][1] >= p2[1]) and (closestPoint[1] >= p1[1] and closestPoint[1] >= p2[1]) and abs(p1[1] - listOfPoints[pointIndex][1]) < .45 and (p1[0] <= listOfPoints[pointIndex][0] <= p2[0] or p1[0] >= listOfPoints[pointIndex][0] >= p2[0]):
                update = True
                listOfPoints[pointIndex][1] = p1[1]+.45
            


        if update == True and pointIndex == newPointIndex:
            update = False
            lastPoint = [listOfEdges[pointIndex-1][2], listOfEdges[pointIndex-1][3]]
            listOfEdges[pointIndex-1][2] = listOfPoints[pointIndex][0]
            listOfEdges[pointIndex-1][3] = listOfPoints[pointIndex][1]

            for i in range(len(listOfEdges)):
                if listOfEdges[i][0] == lastPoint[0] and listOfEdges[i][1] == lastPoint[1]:
                    listOfEdges[i][0] = listOfPoints[pointIndex][0]
                    listOfEdges[i][1] = listOfPoints[pointIndex][1]

            targetAbsolutePosition=np.array(sim.getObjectPosition(target,sim.handle_world))
            targetAbsoluteRotation=np.array(sim.getObjectOrientation(target))
    
            rotation = math.atan2(listOfPoints[newPointIndex][1] - objectAbsolutePosition[1], listOfPoints[newPointIndex][0] - objectAbsolutePosition[0]) - 1.5708
            print(rotation)
            sim.setObjectOrientation(target, [0, 0, rotation])

            i = 1
            while True:
                slightDifX = (listOfPoints[newPointIndex][0] - targetAbsolutePosition[0]) * (.0999 * (1/i))
                slightDifY = (listOfPoints[newPointIndex][1] - targetAbsolutePosition[1]) * (.0999 * (1/i))
                numCheck = 10*i
                if slightDifX < .05 and slightDifY < .05:
                    break
                i = i+1

            checkX = targetAbsolutePosition[0] + slightDifX
            checkY = targetAbsolutePosition[1] + slightDifY
            print("Set")
            needNewPoint = False
            
        if needDij == True:
            collisions = False
            for i in range(len(obstacles)):
                p1 = (obstacles[i][0][0], obstacles[i][0][1])  # (x1, y1)
                p2 = (obstacles[i][1][0], obstacles[i][1][1])  # (x2, y2)
                # If the edge between the points goes through a wall, it is forced back to the correct side of the wall
                if line_intersection(p1,p2, objectAbsolutePosition, pointsToTraverse[0]) == True:
                    collisions = True
                    break
            
            if collisions == True:
                pointsToTraverse = dijkstraNavigation()
                needNewPoint = False

            



def checkPoints():
    global lastPointFound
    coord, coordLeft, coordRight = sysCall_sensing()

    if len(lastPointFound) != 0 and len(coord) > 0 and ((abs(coord[1]-lastPointFound[1]) > .1) or (abs(coord[0]-lastPointFound[0]) > .1)) and coord[2] > .3:
        needReval = False
        for k in range(len(mazeNotSearched)):
            # If the edge between the points goes through a wall, it is forced back to the correct side of the wall
            if abs(coord[0] - float(mazeNotSearched[k][0])+.125) <= .2 and abs(coord[1] - float(mazeNotSearched[k][1])+.125) <= .2:
                maze[k][2] = "B"
                mazeBars.append([maze[k][0],  maze[k][1]])
                np.delete(mazeNotSearched, k)
                needReval = True
            if abs(coordLeft[0] - float(mazeNotSearched[k][0])+.125) <= .2 and abs(coordLeft[1] - float(mazeNotSearched[k][1])+.125) <= .2:
                maze[k][2] = "B"
                mazeBars.append([maze[k][0],  maze[k][1]])
                np.delete(mazeNotSearched, k)
                needReval = True
            if abs(coordRight[0] - float(mazeNotSearched[k][0])+.125) <= .2 and abs(coordRight[1] - float(mazeNotSearched[k][1])+.125) <= .2:
                maze[k][2] = "B"
                mazeBars.append([maze[k][0],  maze[k][1]])
                np.delete(mazeNotSearched, k)
                needReval = True

        if needReval:
            needReval = False
            lastPointFound = coord
            obstacles.append([coordLeft, coord])
            obstacles.append([coord, coordRight])
            reevaluatePoint()


    elif len(lastPointFound) == 0 and len(coord) != 0  and coord[2] > .3:
        needReval = False
        for k in range(len(mazeNotSearched)):
            # If the edge between the points goes through a wall, it is forced back to the correct side of the wall
            if abs(coord[0] - float(mazeNotSearched[k][0])+.125) <= .2 and abs(coord[1] - float(mazeNotSearched[k][1])+.125) <= .2:
                maze[k][2] = "B"
                mazeBars.append([maze[k][0],  maze[k][1]])
                np.delete(mazeNotSearched, k)
                needReval = True
            if abs(coordLeft[0] - float(mazeNotSearched[k][0])+.125) <= .2 and abs(coordLeft[1] - float(mazeNotSearched[k][1])+.125) <= .2:
                maze[k][2] = "B"
                mazeBars.append([maze[k][0],  maze[k][1]])
                np.delete(mazeNotSearched, k)
                needReval = True
            if abs(coordRight[0] - float(mazeNotSearched[k][0])+.125) <= .2 and abs(coordRight[1] - float(mazeNotSearched[k][1])+.125) <= .2:
                maze[k][2] = "B"
                mazeBars.append([maze[k][0],  maze[k][1]])
                np.delete(mazeNotSearched, k)
                needReval = True

        if needReval:
            needReval = False
            lastPointFound = coord
            obstacles.append([coordLeft, coordRight])
            reevaluatePoint()



print('Program started. Do not mannually close this script, stop the simulation so drawing is correctly deleted.')

# Using ZMQ remote API to connect to the simulation
client = RemoteAPIClient()
sim = client.require('sim')
simVision = client.require('simVision')

# Getting the objects from the scene that are needed for the script
robot = sim.getObject('/Quadcopter')
sensor = sim.getObject('/Quadcopter/blobTo3dPosition/sensor')
target = sim.getObject('/Quadcopter/target')

sim.setStepping(True)
# Starting simulation
sim.startSimulation()
startTime = sim.getSimulationTime()
finalTime = 0
TotalTime = 0
totalDist = 0
lastPos = []
particlesAreVisible=True
simulateParticles=False
fakeShadow=True
    
particleCountPerSecond=430
particleSize=0.005
particleDensity=8500
particleScatteringAngle=30
particleLifeTime=0.5
maxParticleCount=50

# Detatch the manipulation sphere:
sim.setObjectParent(target,-1,True)

# This control algo was quickly written and is dirty and not optimal. It just serves as a SIMPLE example
d=sim.getObject('/Quadcopter/base')

propellerHandles=[]
jointHandles=[]
particleObjects=[-1,-1,-1,-1]
ttype=sim.particle_roughspheres+sim.particle_cyclic+sim.particle_respondable1to4+sim.particle_respondable5to8+sim.particle_ignoresgravity
if not particlesAreVisible == True:
    ttype=ttype+sim.particle_invisible

for i in range(4):
    print(f"/Quadcopter/propeller[{i}]/respondable")
    propellerHandles.append(sim.getObject(f"/Quadcopter/propeller[{i}]/respondable"))
    jointHandles.append(sim.getObject(f"/Quadcopter/propeller[{i}]/joint"))
    if simulateParticles == True:
        particleObjects[i]=sim.addParticleObject(ttype,particleSize,particleDensity,[2,1,0.2,3,0.4],particleLifeTime,maxParticleCount,[0.3,0.7,1])

pParam=2
iParam=0
dParam=0
vParam=-2

cumul=0
lastE=0
pAlphaE=0
pBetaE=0
psp2=0
psp1=0

prevEuler=0
    


lastPointFound =[]
obstacles = []
sphereContainer=sim.addDrawingObject(sim.drawing_spherepts,0.03,0,-1,9999,[1,0,1])
xAngle=sim.getObjectFloatParam(sensor,sim.visionfloatparam_perspective_angle)
resX=sim.getObjectInt32Param(sensor,sim.visionintparam_resolution_x)
resY=sim.getObjectInt32Param(sensor,sim.visionintparam_resolution_y)
yAngle=xAngle
ratio=resX/resY
if resX>resY:
    yAngle=2*math.atan(math.tan(xAngle/2)/ratio)
else:
    xAngle=2*math.atan(math.tan(yAngle/2)/ratio)






#Some neccessary variables for later
objectAbsolutePosition=np.array(sim.getObjectPosition(robot,sim.handle_world))
listOfPoints = [[objectAbsolutePosition[0], objectAbsolutePosition[1]]]
listOfCosts = [0]
listOfEdges =[]
startPoint = [objectAbsolutePosition[0], objectAbsolutePosition[1]]
expandRadius = .75
needNewPoint = True
slightDifX = 0
slightDifY = 0
checkX = 0
checkY = 0
numCheck = 0
pointsToTraverse = []
maze = []
mazeBars = []
newPointIndex = -1



# Creating a grid to map obstacles to
x = -3
while x <= 3:
    y = -3
    while y <= 3:
        maze.append([x, y, "U"])
        y = y+.25
    x = x +.25


mazeNotSearched = np.copy(maze)




















while sim.getSimulationState()!=0:
    objectAbsolutePosition=np.array(sim.getObjectPosition(robot,sim.handle_world))
        # Calculating the distance from the last position for total distance traveled
    if len(lastPos) > 0:
        totalDist = totalDist + distance(lastPos, objectAbsolutePosition)
    checkPoints()
    moveDrone()
    toRemove = []
    for k in range(len(mazeNotSearched)):
        if abs(objectAbsolutePosition[0] - float(mazeNotSearched[k][0])+.125) < .45 and abs(objectAbsolutePosition[1] - float(mazeNotSearched[k][1])+.125) < .45 and maze[k][2] != "V" and maze[k][2] != "B":
            maze[k][2] = "V"
            toRemove.append(k)
    
    for k in range(len(toRemove)):
        print(len(mazeNotSearched))
        #mazeNotSearched= np.delete(mazeNotSearched, toRemove[k] - k,0)
            







    if needNewPoint == True:
        if sim.getSimulationState()!=0:
            reevaluatePoint()
            newPointIndex, newEdgeIndex, done = newRRTNode()
            if done == False:
                finalTime = sim.getSimulationTime()
                TotalTime = finalTime - startTime
                print(TotalTime)
                print(totalDist)
                break
            needDij = False
            for i in range(len(obstacles)):
                p1 = (obstacles[i][0][0], obstacles[i][0][1])  # (x1, y1)
                p2 = (obstacles[i][1][0], obstacles[i][1][1])  # (x2, y2)
                # If the edge between the points goes through a wall, it is forced back to the correct side of the wall
                if (line_intersection(p1,p2, [objectAbsolutePosition[0], objectAbsolutePosition[1]], listOfPoints[newPointIndex]) == True or
                        ((is_between_points(listOfPoints[newPointIndex], [objectAbsolutePosition[0], objectAbsolutePosition[1]], p1, .45) == True) and
                        (listOfPoints[newPointIndex][0] < p1[0] < objectAbsolutePosition[0] or listOfPoints[newPointIndex][0] > p1[0] > objectAbsolutePosition[0]) and
                        (listOfPoints[newPointIndex][1] < p1[1] < objectAbsolutePosition[1] or listOfPoints[newPointIndex][1] > p1[1] > objectAbsolutePosition[1])) or
                        ((is_between_points(listOfPoints[newPointIndex], [objectAbsolutePosition[0], objectAbsolutePosition[1]], p2, .45) == True) and
                        (listOfPoints[newPointIndex][0] < p2[0] < objectAbsolutePosition[0] or listOfPoints[newPointIndex][0] > p2[0] > objectAbsolutePosition[0]) and
                        (listOfPoints[newPointIndex][1] < p2[1] < objectAbsolutePosition[1] or listOfPoints[newPointIndex][1] > p2[1] > objectAbsolutePosition[1]))):
                    needDij = True
                    newPointIndex = -1
                    pointsToTraverse = dijkstraNavigation()
                    needNewPoint = False
                    break

            if needDij == False:
                targetAbsolutePosition=np.array(sim.getObjectPosition(target,sim.handle_world))
                targetAbsoluteRotation=np.array(sim.getObjectOrientation(target))

                rotation = math.atan2(listOfPoints[newPointIndex][1] - targetAbsolutePosition[1], listOfPoints[newPointIndex][0] - targetAbsolutePosition[0]) - 1.5708
                print(rotation)
                sim.setObjectOrientation(target, [0, 0, rotation])
                for i in range(200):
                    sim.step()
                    checkPoints()
                    moveDrone()
                    
            
                if sim.getSimulationState()!=0:
                    i = 1
                    while True:
                        slightDifX = (listOfPoints[newPointIndex][0] - targetAbsolutePosition[0]) * (.0999 * (1/i))
                        slightDifY = (listOfPoints[newPointIndex][1] - targetAbsolutePosition[1]) * (.0999 * (1/i))
                        numCheck = 10*i
                        if slightDifX < .05 and slightDifY < .05:
                            break
                        i = i+1
                    
                    checkX = targetAbsolutePosition[0] + slightDifX
                    checkY = targetAbsolutePosition[1] + slightDifY
                    sim.setObjectPosition(target, sim.handle_world, [checkX, checkY, .7])
                    needNewPoint = False



   
    if needNewPoint == False and needDij == False:
        if abs(objectAbsolutePosition[0] - listOfPoints[newPointIndex][0]) < .2 and abs(objectAbsolutePosition[1] - listOfPoints[newPointIndex][1]) < .2:
            needNewPoint = True
            for i in range(100):
                sim.step()
                checkPoints()
                moveDrone()
        elif abs(objectAbsolutePosition[0] - checkX) < .1 and abs(objectAbsolutePosition[1] -checkY) < .1:
                if sim.getSimulationState()!=0:
                    print("Check",numCheck)
                    if numCheck % 10 == 0:
                        for i in range(150):
                            sim.step()
                            checkPoints()
                            moveDrone()
                    if numCheck >= 0:
                        checkX = checkX + slightDifX
                        checkY = checkY + slightDifY
                        numCheck = numCheck - 1
                        sim.setObjectPosition(target, sim.handle_world, [checkX, checkY, .7])

    if needNewPoint == False and needDij == True:
        print("ENTER")
        if abs(objectAbsolutePosition[0] - pointsToTraverse[0][0]) < .2 and abs(objectAbsolutePosition[1] - pointsToTraverse[0][1]) < .2:
            if len(pointsToTraverse) > 1:
                pointsToTraverse.pop(0)
                print("POP")
                targetAbsolutePosition=np.array(sim.getObjectPosition(target,sim.handle_world))
                targetAbsoluteRotation=np.array(sim.getObjectOrientation(target))

                rotation = math.atan2(pointsToTraverse[0][1] - targetAbsolutePosition[1], pointsToTraverse[0][0] - targetAbsolutePosition[0]) - 1.5708
                print(pointsToTraverse[0])
                sim.setObjectOrientation(target, [0, 0, rotation])
                for i in range(200):
                    sim.step()
                    checkPoints()
                    moveDrone()
                    
            
                if sim.getSimulationState()!=0:
                    i = 1
                    while True:
                        slightDifX = (pointsToTraverse[0][0] - targetAbsolutePosition[0]) * (.0999 * (1/i))
                        slightDifY = (pointsToTraverse[0][1] - targetAbsolutePosition[1]) * (.0999 * (1/i))
                        numCheck = 10*i
                        if slightDifX < .05 and slightDifY < .05:
                            break
                        i = i+1
                    
                    checkX = targetAbsolutePosition[0] + slightDifX
                    checkY = targetAbsolutePosition[1] + slightDifY
                    sim.setObjectPosition(target, sim.handle_world, [checkX, checkY, .7])
                    needNewPoint = False
            if len(pointsToTraverse) == 1:
                pointsToTraverse.pop(0)
                needDij = False
                newPointIndex = len(listOfPoints)-1
        elif abs(objectAbsolutePosition[0] - checkX) < .1 and abs(objectAbsolutePosition[1] -checkY) < .1:
            if sim.getSimulationState()!=0:
                print(numCheck)
                if numCheck % 10 == 0:
                        for i in range(150):
                            sim.step()
                            checkPoints()
                            moveDrone()
                if numCheck >= 0:
                    checkX = checkX + slightDifX
                    checkY = checkY + slightDifY
                    numCheck = numCheck - 1
                    sim.setObjectPosition(target, sim.handle_world, [checkX, checkY, .7])
                    
    lastPos = objectAbsolutePosition
    sim.step()

   














print(len(obstacles))
for i in range(len(obstacles)):
    start_point = (obstacles[i][0][0], obstacles[i][0][1])
    end_point = (obstacles[i][1][0], obstacles[i][1][1])

    x_points = np.array([start_point[0], end_point[0]])
    y_points = np.array([start_point[1], end_point[1]])

    plt.plot(x_points, y_points, color='red')

    plt.scatter(x_points, y_points, color='red')

for i in range(len(listOfEdges)):
    start_point = (listOfEdges[i][0], listOfEdges[i][1])
    end_point = (listOfEdges[i][2], listOfEdges[i][3])

    x_points = np.array([start_point[0], end_point[0]])
    y_points = np.array([start_point[1], end_point[1]])

    plt.plot(x_points, y_points, color='blue')

    plt.scatter(x_points, y_points, color='black')

plt.scatter(x_points, y_points, color='black')
# Optional: Add labels and title for clarity
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.title("Line from Start to End Point")
plt.grid(True)

# Display the plot
plt.show()


fig, ax = plt.subplots()
for i in range(len(maze)):
    color = "White"
    if maze[i][2] == "B":
        color = "Black"
    elif maze[i][2] == "U":
        color = "Grey"
    square = patches.Rectangle((maze[i][0], maze[i][1]), .5, .5, linewidth=1, edgecolor='black', facecolor=color)
    ax.add_patch(square)	

ax.set_xlim(left=-10.5, right=10.5) 
ax.set_ylim(top=10.5, bottom=-10.5) 
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.title("Line from Start to End Point")
plt.grid(False)

# Display the plot
plt.show()
del client 
print('Program ended')