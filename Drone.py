import math
import time
import random
import numpy as np
import matplotlib.pyplot as plt
import array
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

print('Program started. Do not mannually close this script, stop the simulation so drawing is correctly deleted.')

# Using ZMQ remote API to connect to the simulation
client = RemoteAPIClient()
sim = client.require('sim')
simVision = client.require('simVision')

# Getting the objects from the scene that are needed for the script
robot = sim.getObject('/Quadcopter')
sensor = sim.getObject('/Quadcopter/blobTo3dPosition/sensor')
target = sim.getObject('/Quadcopter/target')


# Starting simulation
sim.startSimulation()
lineIntersctions = []
def dijkstraNavigation():
    currentPoint = [objectAbsolutePosition[0], objectAbsolutePosition[1]]
    destinatiPoint = listOfPoints[newPointIndex]

    closestPoint = listOfPoints[0]
    closestDist = abs(closestPoint[0] - currentPoint[0]) + abs(closestPoint[1] - currentPoint[1])

    for i in range(1, len(listOfPoints)):
        curDist = abs(listOfPoints[i][0] - currentPoint[0]) + abs(listOfPoints[i][1] - currentPoint[1])
        if curDist < closestDist:
            closestDist = curDist
            closestPoint = listOfPoints[i]

    lineIntersctions = listOfEdges
    # Plotting viable pathes
    for i in range(len(lineIntersctions)):
        for j in range(len(lineIntersctions[i])):
            curLink = lineIntersctions[i]

            start_point = (curLink[0], curLink[1])  # (x1, y1)
            end_point = (curLink[2], curLink[3])  # (x2, y2)


            x_points = np.array([start_point[0], end_point[0]])
            y_points = np.array([start_point[1], end_point[1]])

            plt.plot(x_points, y_points, color='yellow')


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
    plt.show()
    totalPoints = [[closestPoint[0], closestPoint[1], 0]]


    print(lineIntersctions)

    # Adding all nodes to a list with inf cost as intial value
    for i in range(len(lineIntersctions)):
        print(lineIntersctions[i][0])
        curPoint = lineIntersctions[i]
        pointAlreadyAdded = False
        for j in range(len(totalPoints)):
            if(curPoint[0] == totalPoints[j][0] and curPoint[1] == totalPoints[j][1]):
                pointAlreadyAdded = True
        if not pointAlreadyAdded:
            totalPoints.append([curPoint[0], curPoint[1], float('inf')])

    curPoint = np.array(totalPoints[0])
    searching = True
    pointsToSearch = []
    searchedPoints = []

    # Finding the smallest cost for each node according to dijkstra's
    while searching:
        for i in range(len(lineIntersctions)):
            checkPoint = lineIntersctions[i]
            if(curPoint[0] == checkPoint[0] and curPoint[1] == checkPoint[1]):
                for j in range(len(lineIntersctions[i])):
                    curLink = lineIntersctions[i]
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

    # From the goal point, need to trace back the minimum path
    curPoint = [destinatiPoint[0], destinatiPoint[0], .7]
    pointsToVisit = [curPoint]
    searching = True


    while searching:
        for i in range(len(lineIntersctions)):
            checkPoint = lineIntersctions[i]
            if(curPoint[0] == checkPoint[0] and curPoint[1] == checkPoint[1]):
                lowestDist = -1
                lowestDest = []
                for j in range(len(totalPoints)):
                    if(checkPoint[2] == totalPoints[j][0] and checkPoint[3] == totalPoints[j][1]):
                        lowestDist = totalPoints[j][2]
                        lowestDest = [totalPoints[j][0], totalPoints[j][1]]
                        break

                for j in range(1, len(lineIntersctions[i])):
                    curLink = lineIntersctions[i]
                    for k in range(len(totalPoints)):
                        if(curLink[2] == totalPoints[k][0] and curLink[3] == totalPoints[k][1] and totalPoints[k][2] < lowestDist):
                            lowestDist = totalPoints[k][2]
                            lowestDest = [totalPoints[k][0], totalPoints[k][1]]
                pointsToVisit.insert(0, [lowestDest[0], lowestDest[1], 0.43869])
                curPoint = lowestDest
                print(lowestDist)
                if lowestDist == 0:
                    searching = False

                break

    # Plotting minimum path
    for i in range(len(pointsToVisit) - 1):
        start_point = (pointsToVisit[i][0], pointsToVisit[i][1])  # (x1, y1)
        end_point = (pointsToVisit[i+1][0], pointsToVisit[i+1][1])  # (x2, y2)

        x_points = np.array([start_point[0], end_point[0]])
        y_points = np.array([start_point[1], end_point[1]])

        plt.plot(x_points, y_points, color='green')

    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.title("Line from Start to End Point")
    plt.grid(True)

    # Display the plot
    plt.show()






def sysCall_sensing(lastPointFound):
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

            depth, test = sim.getVisionSensorDepth(sensor,1,[1+math.floor(((blobPositionX)-.01)*(resX-0.99)),1+math.floor(blobPositionY*(resY-0.99))],[1,1])
            depth=sim.unpackFloatTable(depth)
            coord=[0,0,depth[0]]
            x=0.5-blobPositionX
            y=blobPositionY-0.5
            coord[0]=depth[0]*math.tan(xAngle*0.5)*(x)/0.5
            coord[1]=depth[0]*math.tan(yAngle*0.5)*(blobPositionY-0.5)/0.5
            coord=sim.multiplyVector(m,coord)
            sim.addDrawingObjectItem(sphereContainer,coord)

            depthLeft, test=sim.getVisionSensorDepth(sensor,1,[1+math.floor(((blobSize*.5+blobPositionX)-.01)*(resX-0.99)),1+math.floor(blobPositionY*(resY-0.99))],[1,1])
            depthLeft=sim.unpackFloatTable(depthLeft)
            coordLeft=[0,0,depthLeft[0]]
            x=0.5-blobPositionX - blobWidth/2
            y=blobPositionY-0.5


            coordLeft[0]=depthLeft[0]*math.tan(xAngle*0.5)*(x)/0.5
            coordLeft[1]=depthLeft[0]*math.tan(yAngle*0.5)*(blobPositionY-0.5)/0.5
            coordLeft=sim.multiplyVector(m,coordLeft)
            sim.addDrawingObjectItem(sphereContainer,coordLeft)


            depthRight, test = sim.getVisionSensorDepth(sensor,1,[1+math.floor((blobPositionX-blobSize*.5)*(resX-0.99)),1+math.floor(blobPositionY*(resY-0.99))],[1,1])
            depthRight=sim.unpackFloatTable(depthRight)
            coordRight=[0,0,depthRight[0]]
            x=0.5-blobPositionX + blobWidth/2
            y=blobPositionY-0.5

            coordRight[0]=depthRight[0]*math.tan(xAngle*0.5)*(x)/0.5
            coordRight[1]=depthRight[0]*math.tan(yAngle*0.5)*(blobPositionY-0.5)/0.5
            coordRight=sim.multiplyVector(m,coordRight)
            sim.addDrawingObjectItem(sphereContainer,coordRight)
    return coord, coordLeft, coordRight













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
listOfPoints = [[0, 0]]
listOfCosts = [0]
listOfEdges =[]
startPoint = [0, 0]
expandRadius = .75


# Function for distance formula between two points
def distance(p1, p2):
    return math.sqrt(((p1[0] - p2[0]) ** 2) + ((p1[1] - p2[1]) ** 2))

# Function for if point is inbetween two points
def is_between_points_dist(A, B, C):
    AB = distance(A, B)
    AC = distance(A, C)
    CB = distance(C, B)

    return (AC + CB) - AB <= .001

#Function for if two lines intersect
def line_intersection(p1, p2, p3, p4):

    D = ((p1[0] - p2[0]) * (p3[1] - p4[1])) -  ((p1[1] - p2[1]) * (p3[0] - p4[0]))
    if D == 0:
        return True
    t = ((((p1[0] - p3[0]) * (p3[1] - p4[1])) - ((p1[1] - p3[1]) * (p3[0] - p4[0]))) / D)

    u = (((p1[0] - p3[0]) * (p1[1] - p2[1])) - ((p1[1] - p3[1]) * (p1[0] - p2[0]))) / D

    if 0 <= t <= 1 and 0 <= u <= 1:
        return True
    else:
        return False


def newRRTNode():
    # This will pull the tree towards a random point
    xCoord = random.uniform(-10, 6)
    yCoord = random.uniform(-6, 10)
    xNew = np.array([xCoord, yCoord])

    # Making sure the point isn't in a wall
    collisions = False
    for i in range(len(obstacles)):
        p1 = (obstacles[i][0][0], obstacles[i][0][1])  # (x1, y1)
        p2 = (obstacles[i][1][0], obstacles[i][1][1])  # (x2, y2)
        if is_between_points_dist(p1,p2, xNew):
            collisions = True

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
        newPoint = np.array(closestPoint + (.5 * ((xNew-closestPoint)/np.linalg.norm((xNew-closestPoint)))))

        # Need to make sure that the point isn't too close to the maze walls or that the edge connecting the new point and the closest is through a wall
        for i in range(len(obstacles)):
            p1 = (obstacles[i][0][0], obstacles[i][0][1])  # (x1, y1)
            p2 = (obstacles[i][1][0], obstacles[i][1][1])  # (x2, y2)
            # If the edge between the points goes through a wall, it is forced back to the correct side of the wall
            if line_intersection(p1,p2, closestPoint, newPoint) == True:
                if (closestPoint[0] <= p1[0] and closestPoint[0] <= p2[0]):
                    newPoint[0] = p1[0]-.5
                elif (closestPoint[0] >= p1[0] and closestPoint[0] >= p2[0]):
                    newPoint[0] = p1[0] + .5
                if (closestPoint[1] <= p1[1] and closestPoint[1] <= p2[1]):
                    newPoint[1] = p1[1] -.5
                elif (closestPoint[1] >= p1[1] and closestPoint[1] >= p2[1]):
                    newPoint[1] = p1[1] +.5

            # Verifying that the point isn't too close to the wall to make navigation smoother
            if (newPoint[0] <= p1[0] and newPoint[0] <=  p2[0]) and abs(newPoint[0] -  p1[0]) < .5 and (p1[1] <= newPoint[1] <= p2[1] or p1[1] >= newPoint[1] >= p2[1]):
                newPoint[0] = p1[0]-.5
            elif (newPoint[0] >= p1[0] and newPoint[0] >= p2[0]) and abs(newPoint[0] - p1[0]) < .5 and (p1[1] <= newPoint[1] <= p2[1] or p1[1] >= newPoint[1] >= p2[1]):
                newPoint[0] = p1[0]+.5

            if (newPoint[1] <= p1[1] and newPoint[1] <= p2[1]) and abs(newPoint[1] - p1[1]) < .5 and (p1[0] <= newPoint[0] <= p2[0] or p1[0] >= newPoint[0] >= p2[0]):
                newPoint[1] = p1[1]-.5
            elif (newPoint[1] >= p1[1] and newPoint[1] >= p2[1]) and abs(p1[1] - newPoint[1]) < .5 and (p1[0] <= newPoint[0] <= p2[0] or p1[0] >= newPoint[0] >= p2[0]):
                newPoint[1] = p1[1]+.5

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

        # With the new point established, nearby points will be check to see if the cost from going from the new point to the nearby point is better
        for i in range(0, len(listOfPoints)-1):
            pointToComp = np.array(listOfPoints[i])

            intersection = False
            for k in range(len(obstacles)):
                p1 = (obstacles[k][0][0], obstacles[k][0][1])  # (x1, y1)
                p2 = (obstacles[k][1][0], obstacles[k][1][1])  # (x2, y2)
                if line_intersection(p1,p2, pointToComp, newPoint) == True:
                    intersection = True


            if 0 < abs(newPoint[0] - pointToComp[0]) <= expandRadius and 0 < abs(newPoint[1] - pointToComp[1]) <= expandRadius and not intersection:
                if np.linalg.norm(((newPoint-pointToComp)/np.linalg.norm((newPoint-pointToComp)))) + curLowestCost < listOfCosts[i]:
                    listOfEdges[i-1][0] = newPoint[0]
                    listOfEdges[i-1][1] = newPoint[1]
                    listOfCosts[i] = np.linalg.norm(((newPoint-pointToComp)/np.linalg.norm((newPoint-pointToComp)))) + curLowestCost

    # Need to trace back from the final point to the start
    curPoint = listOfPoints[len(listOfPoints)-1]

    return len(listOfPoints)-1, len(listOfEdges)-1


def reevaluatePoint():
# Need to make sure that the point isn't too close to the maze walls or that the edge connecting the new point and the closest is through a wall
    update = False
    objectAbsolutePosition=np.array(sim.getObjectPosition(robot,sim.handle_world ))
    targetAbsolutePosition=np.array(sim.getObjectPosition(target,sim.handle_world))
    for pointIndex in range(1, len(listOfPoints)):
        edgeIndex = pointIndex - 1
        closestPoint = [listOfEdges[edgeIndex][0], listOfEdges[edgeIndex][1]]
        for i in range(len(obstacles)):
            p1 = (obstacles[i][0][0], obstacles[i][0][1])  # (x1, y1)
            p2 = (obstacles[i][1][0], obstacles[i][1][1])  # (x2, y2)
            # If the edge between the points goes through a wall, it is forced back to the correct side of the wall
            if line_intersection(p1,p2, closestPoint, listOfPoints[pointIndex]) == True:
                print("RE")
                if (closestPoint[0] <= p1[0] and closestPoint[0] <= p2[0]):
                    update = True
                    listOfPoints[pointIndex][0] = p1[0] - .5
                elif (closestPoint[0] >= p1[0] and closestPoint[0] >= p2[0]):
                    update = True
                    listOfPoints[pointIndex][0] = p1[0] + .5
                if (closestPoint[1] <= p1[1] and closestPoint[1] <= p2[1]):
                    update = True
                    listOfPoints[pointIndex][1] = p1[1] - .5
                elif (closestPoint[1] >= p1[1] and closestPoint[1] >= p2[1]):
                    update = True
                    listOfPoints[pointIndex][1] = p1[1] + .5

            # Verifying that the point isn't too close to the wall to make navigation smoother
            if (listOfPoints[pointIndex][0] <= p1[0] and listOfPoints[pointIndex][0] <=  p2[0]) and (closestPoint[0] <= p1[0] and closestPoint[0] <= p2[0]) and abs(listOfPoints[pointIndex][0] -  p1[0]) < .5 and (p1[1] <= listOfPoints[pointIndex][1] <= p2[1] or p1[1] >= listOfPoints[pointIndex][1] >= p2[1]):
                update = True
                listOfPoints[pointIndex][0] = p1[0]-.5
            elif (listOfPoints[pointIndex][0] >= p1[0] and listOfPoints[pointIndex][0] >= p2[0]) and (closestPoint[0] >= p1[0] and closestPoint[0] >= p2[0]) and abs(p1[0] - listOfPoints[pointIndex][0]) < .5 and (p1[1] <= listOfPoints[pointIndex][1] <= p2[1] or p1[1] >= listOfPoints[pointIndex][1] >= p2[1]):
                update = True
                listOfPoints[pointIndex][0] = p1[0]+.5

            if (listOfPoints[pointIndex][1] <= p1[1] and listOfPoints[pointIndex][1] <= p2[1]) and (closestPoint[1] <= p1[1] and closestPoint[1] <= p2[1]) and abs(listOfPoints[pointIndex][1] - p1[1]) < .5 and (p1[0] <= listOfPoints[pointIndex][0] <= p2[0] or p1[0] >= listOfPoints[pointIndex][0] >= p2[0]):
                update = True
                listOfPoints[pointIndex][1] = p1[1]-.5
            elif (listOfPoints[pointIndex][1] >= p1[1] and listOfPoints[pointIndex][1] >= p2[1]) and (closestPoint[1] >= p1[1] and closestPoint[1] >= p2[1]) and abs(p1[1] - listOfPoints[pointIndex][1]) < .5 and (p1[0] <= listOfPoints[pointIndex][0] <= p2[0] or p1[0] >= listOfPoints[pointIndex][0] >= p2[0]):
                update = True
                listOfPoints[pointIndex][1] = p1[1]+.5

        if update == True:
            sim.setObjectPosition(target, sim.handle_world, [objectAbsolutePosition[0], objectAbsolutePosition[1], .7])
            print("update ",  listOfPoints[pointIndex])
            slightDifX = (listOfPoints[pointIndex][0] - targetAbsolutePosition[0]) * .25
            slightDifY = (listOfPoints[pointIndex][1] - targetAbsolutePosition[1]) * .25
            while slightDifX > .2 or slightDifX < -.2 :
                slightDifX = slightDifX*.5
                slightDifY = slightDifY*.5

            while slightDifY > .2 or slightDifY < -.2:
                slightDifX = slightDifX*.5
                slightDifY = slightDifY*.5
            checkX = targetAbsolutePosition[0] + slightDifX
            print("update ", checkX)
            checkY = targetAbsolutePosition[1] + slightDifY
            numCheck = 1
            sim.setObjectPosition(target, sim.handle_world, [checkX, checkY, .7])
            needNewPoint = False



needNewPoint = True
slightDifX = 0
slightDifY = 0

checkX = 0
checkY = 0
numCheck = 0

while sim.getSimulationState()!=0:
    objectAbsolutePosition=np.array(sim.getObjectPosition(robot,sim.handle_world ))

    coord, coordLeft, coordRight = sysCall_sensing(lastPointFound)
    if len(lastPointFound) != 0 and len(coord) > 0 and ((abs(coord[1]-lastPointFound[1]) > .1) or (abs(coord[0]-lastPointFound[0]) > .1)) and coord[2] > .3:
        print("NEW!")
        lastPointFound = coord
        obstacles.append([coordLeft, coordRight])
        reevaluatePoint()
    elif len(lastPointFound) == 0 and len(coord) != 0  and coord[2] > .1:
        print("NEW!")
        lastPointFound = coord
        obstacles.append([coordLeft, coordRight])
        reevaluatePoint()

    if needNewPoint:
        print("NEW")
        time.sleep(2)
        if sim.getSimulationState()!=0:
            reevaluatePoint()
            newPointIndex, newEdgeIndex = newRRTNode()
            needDij = False
            for i in range(len(obstacles)):
                p1 = (obstacles[i][0][0], obstacles[i][0][1])  # (x1, y1)
                p2 = (obstacles[i][1][0], obstacles[i][1][1])  # (x2, y2)
                # If the edge between the points goes through a wall, it is forced back to the correct side of the wall
                if line_intersection(p1,p2, [objectAbsolutePosition[0], objectAbsolutePosition[1]], listOfPoints[newPointIndex]) == True:
                    print("DIJ")
                    needDij = True

            if needDij == False:
                targetAbsolutePosition=np.array(sim.getObjectPosition(target,sim.handle_world))
                targetAbsoluteRotation=np.array(sim.getObjectOrientation(target))

                rotation = math.atan2(listOfPoints[newPointIndex][1] - objectAbsolutePosition[1], listOfPoints[newPointIndex][0] - objectAbsolutePosition[0]) - 1.5708
                sim.setObjectOrientation(robot, [0, 0, rotation])
                sim.setObjectOrientation(target, [0, 0, rotation])

                if sim.getSimulationState()!=0:
                    time.sleep(1)
                    slightDifX = (listOfPoints[newPointIndex][0] - targetAbsolutePosition[0]) * .25
                    slightDifY = (listOfPoints[newPointIndex][1] - targetAbsolutePosition[1]) * .25
                    while slightDifX > .2 or slightDifX < -.2:
                        slightDifX = slightDifX*.5
                        slightDifY = slightDifY*.5

                    while slightDifY > .2 or slightDifY < -.2:
                        slightDifX = slightDifX*.5
                        slightDifY = slightDifY*.5
                    checkX = targetAbsolutePosition[0] + slightDifX
                    checkY = targetAbsolutePosition[1] + slightDifY
                    numCheck = 1
                    sim.setObjectPosition(target, sim.handle_world, [checkX, checkY, .7])
                    needNewPoint = False
            else:
                dijkstraNavigation()


    if not needNewPoint:
        if abs(objectAbsolutePosition[0] - checkX) < .5 and abs(objectAbsolutePosition[1] -checkY) < .5:
                print("UP")
                time.sleep(.5)
                if sim.getSimulationState()!=0:
                 checkX = checkX + slightDifX
                 checkY = checkY + slightDifY
                 numCheck = numCheck + 1
                 sim.setObjectPosition(target, sim.handle_world, [checkX, checkY, .7])
        if abs(objectAbsolutePosition[0] - listOfPoints[newPointIndex][0]) < .5 and abs(objectAbsolutePosition[1] - listOfPoints[newPointIndex][1]) < .5:
            needNewPoint = True

    coord, coordLeft, coordRight = sysCall_sensing(lastPointFound)
    if len(lastPointFound) != 0 and len(coord) > 0 and ((abs(coord[1]-lastPointFound[1]) > .1) or (abs(coord[0]-lastPointFound[0]) > .1)) and coord[2] > .3:
        print("NEW!")
        lastPointFound = coord
        obstacles.append([coordLeft, coordRight])
        reevaluatePoint()
    elif len(lastPointFound) == 0 and len(coord) != 0  and coord[2] > .1:
        print("NEW!")
        lastPointFound = coord
        obstacles.append([coordLeft, coordRight])
        reevaluatePoint()

    rotation = math.atan2(listOfPoints[newPointIndex][1] - objectAbsolutePosition[1], listOfPoints[newPointIndex][0] - objectAbsolutePosition[0]) - 1.5708
    print(rotation)
    sim.setObjectOrientation(robot, [0, 0, rotation])
    sim.setObjectOrientation(target, [0, 0, rotation])
















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

print('Program ended')