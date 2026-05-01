import math
import time
import random
from datetime import datetime
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

from Geometry import Geometry
from Context import SimulationContext

# consts for sleeps
SLEEP_TARGET_STEP = 0.75
SLEEP_NEW_POINT = 1.5
SLEEP_TARGET_PROGRESS = 0.3

class DroneNavigator:
  def __init__(self, sim_ctx):
    self.ctx = sim_ctx
    self.sim = sim_ctx.sim
    self.robot = sim_ctx.robot
    self.target = sim_ctx.target

    self.lastPointFound = []
    self.obstacles = []

    # Some neccessary variables for later
    self.listOfPoints = [[0, 0]]
    self.listOfCosts = [0]
    self.listOfEdges = []
    self.startPoint = [0, 0]
    self.expandRadius = 0.75

    self.lineIntersctions = []

    self.needNewPoint = True
    self.slightDifX = 0
    self.slightDifY = 0

    self.checkX = 0
    self.checkY = 0
    self.numCheck = 0

    self.objectAbsolutePosition = np.array([0, 0, 0])
    self.newPointIndex = 0
    self.newEdgeIndex = 0

  def dijkstraNavigation(self):
    currentPoint = [self.objectAbsolutePosition[0], self.objectAbsolutePosition[1]]
    destinatiPoint = self.listOfPoints[self.newPointIndex]

    closestPoint = self.listOfPoints[0]
    closestDist = abs(closestPoint[0] - currentPoint[0]) + abs(closestPoint[1] - currentPoint[1])

    for i in range(1, len(self.listOfPoints)):
      curDist = abs(self.listOfPoints[i][0] - currentPoint[0]) + abs(self.listOfPoints[i][1] - currentPoint[1])
      if curDist < closestDist:
        closestDist = curDist
        closestPoint = self.listOfPoints[i]

    lineIntersctions = self.listOfEdges

    # Plotting viable pathes
    for i in range(len(lineIntersctions)):
      for j in range(len(lineIntersctions[i])):
        curLink = lineIntersctions[i]

        start_point = (curLink[0], curLink[1])  # (x1, y1)
        end_point = (curLink[2], curLink[3])  # (x2, y2)

        x_points = np.array([start_point[0], end_point[0]])
        y_points = np.array([start_point[1], end_point[1]])

        plt.plot(x_points, y_points, color='yellow')

    print(len(self.obstacles))
    for i in range(len(self.obstacles)):
      start_point = (self.obstacles[i][0][0], self.obstacles[i][0][1])
      end_point = (self.obstacles[i][1][0], self.obstacles[i][1][1])

      x_points = np.array([start_point[0], end_point[0]])
      y_points = np.array([start_point[1], end_point[1]])

      plt.plot(x_points, y_points, color='red')

      plt.scatter(x_points, y_points, color='red')

    for i in range(len(self.listOfEdges)):
      start_point = (self.listOfEdges[i][0], self.listOfEdges[i][1])
      end_point = (self.listOfEdges[i][2], self.listOfEdges[i][3])

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
        if curPoint[0] == totalPoints[j][0] and curPoint[1] == totalPoints[j][1]:
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
        if curPoint[0] == checkPoint[0] and curPoint[1] == checkPoint[1]:
          for j in range(len(lineIntersctions[i])):
            curLink = lineIntersctions[i]
            destinationPoint = np.array([curLink[2], curLink[3]])
            pointsToSearch.append(destinationPoint)
            dist = curPoint[2] + np.linalg.norm(destinationPoint - curPoint[:2])
            for k in range(len(totalPoints)):
              if (
                destinationPoint[0] == totalPoints[k][0]
                and destinationPoint[1] == totalPoints[k][1]
                and (totalPoints[k][2] == float('inf') or totalPoints[k][2] > dist)
              ):
                totalPoints[k][2] = float(dist)
          break

      i = 0
      while i < len(pointsToSearch) and len(pointsToSearch) > 0:
        for j in range(len(searchedPoints)):
          if pointsToSearch[i][0] == searchedPoints[j][0] and pointsToSearch[i][1] == searchedPoints[j][1]:
            pointsToSearch.pop(i)
            i = i - 1
            break
        i = i + 1

      if len(pointsToSearch) > 0:
        for j in range(len(totalPoints)):
          if pointsToSearch[0][0] == totalPoints[j][0] and pointsToSearch[0][1] == totalPoints[j][1]:
            curPoint = totalPoints[j]
            searchedPoints.append(pointsToSearch[0])
            pointsToSearch.pop(0)
            break
      else:
        searching = False

    # From the goal point, need to trace back the minimum path
    curPoint = [destinatiPoint[0], destinatiPoint[0], 0.7]
    pointsToVisit = [curPoint]
    searching = True

    while searching:
      for i in range(len(lineIntersctions)):
        checkPoint = lineIntersctions[i]
        if curPoint[0] == checkPoint[0] and curPoint[1] == checkPoint[1]:
          lowestDist = -1
          lowestDest = []
          for j in range(len(totalPoints)):
            if checkPoint[2] == totalPoints[j][0] and checkPoint[3] == totalPoints[j][1]:
              lowestDist = totalPoints[j][2]
              lowestDest = [totalPoints[j][0], totalPoints[j][1]]
              break

          for j in range(1, len(lineIntersctions[i])):
            curLink = lineIntersctions[i]
            for k in range(len(totalPoints)):
              if (
                curLink[2] == totalPoints[k][0]
                and curLink[3] == totalPoints[k][1]
                and totalPoints[k][2] < lowestDist
              ):
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
      end_point = (pointsToVisit[i + 1][0], pointsToVisit[i + 1][1])  # (x2, y2)

      x_points = np.array([start_point[0], end_point[0]])
      y_points = np.array([start_point[1], end_point[1]])

      plt.plot(x_points, y_points, color='green')

    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Line from Start to End Point')
    plt.grid(True)

    # Display the plot
    plt.show()

  def newRRTNode(self):
    # This will pull the tree towards a random point
    xCoord = random.uniform(-10, 6)
    yCoord = random.uniform(-6, 10)
    xNew = np.array([xCoord, yCoord])

    # Making sure the point isn't in a wall
    collisions = False
    for i in range(len(self.obstacles)):
      p1 = (self.obstacles[i][0][0], self.obstacles[i][0][1])  # (x1, y1)
      p2 = (self.obstacles[i][1][0], self.obstacles[i][1][1])  # (x2, y2)
      if Geometry.is_between_points_dist(p1, p2, xNew):
        collisions = True

    if collisions is not True:
      # Using the first stored point, the start point, as the intial base for the closest point
      smallestDist = Geometry.distance(self.listOfPoints[0], xNew)
      closestPoint = self.listOfPoints[0]
      closestCost = self.listOfCosts[0]

      # Checking all other points for if there is a closer point
      for i in range(len(self.listOfPoints)):
        if Geometry.distance(self.listOfPoints[i], xNew) < smallestDist:
          smallestDist = Geometry.distance(self.listOfPoints[i], xNew)
          closestPoint = np.array(self.listOfPoints[i])
          closestCost = self.listOfCosts[i]

      # Once the closest point is found, a new point is made .75 distance from the closest point in the direction of the new point
      newPoint = np.array(closestPoint + (0.5 * ((xNew - closestPoint) / np.linalg.norm((xNew - closestPoint)))))

      # Need to make sure that the point isn't too close to the maze walls or that the edge connecting the new point and the closest is through a wall
      for i in range(len(self.obstacles)):
        p1 = (self.obstacles[i][0][0], self.obstacles[i][0][1])  # (x1, y1)
        p2 = (self.obstacles[i][1][0], self.obstacles[i][1][1])  # (x2, y2)
        # If the edge between the points goes through a wall, it is forced back to the correct side of the wall
        if Geometry.line_intersection(p1, p2, closestPoint, newPoint) is True:
          if closestPoint[0] <= p1[0] and closestPoint[0] <= p2[0]:
            newPoint[0] = p1[0] - 0.5
          elif closestPoint[0] >= p1[0] and closestPoint[0] >= p2[0]:
            newPoint[0] = p1[0] + 0.5
          if closestPoint[1] <= p1[1] and closestPoint[1] <= p2[1]:
            newPoint[1] = p1[1] - 0.5
          elif closestPoint[1] >= p1[1] and closestPoint[1] >= p2[1]:
            newPoint[1] = p1[1] + 0.5

        # Verifying that the point isn't too close to the wall to make navigation smoother
        if (
          (newPoint[0] <= p1[0] and newPoint[0] <= p2[0])
          and abs(newPoint[0] - p1[0]) < 0.5
          and (p1[1] <= newPoint[1] <= p2[1] or p1[1] >= newPoint[1] >= p2[1])
        ):
          newPoint[0] = p1[0] - 0.5
        elif (
          (newPoint[0] >= p1[0] and newPoint[0] >= p2[0])
          and abs(newPoint[0] - p1[0]) < 0.5
          and (p1[1] <= newPoint[1] <= p2[1] or p1[1] >= newPoint[1] >= p2[1])
        ):
          newPoint[0] = p1[0] + 0.5

        if (
          (newPoint[1] <= p1[1] and newPoint[1] <= p2[1])
          and abs(newPoint[1] - p1[1]) < 0.5
          and (p1[0] <= newPoint[0] <= p2[0] or p1[0] >= newPoint[0] >= p2[0])
        ):
          newPoint[1] = p1[1] - 0.5
        elif (
          (newPoint[1] >= p1[1] and newPoint[1] >= p2[1])
          and abs(p1[1] - newPoint[1]) < 0.5
          and (p1[0] <= newPoint[0] <= p2[0] or p1[0] >= newPoint[0] >= p2[0])
        ):
          newPoint[1] = p1[1] + 0.5

      # Storing the cost of the new node and checking for better nearby points
      curLowestCost = np.linalg.norm(((newPoint - closestPoint) / np.linalg.norm((newPoint - closestPoint)))) + closestCost

      for i in range(len(self.listOfPoints)):
        pointToComp = np.array(self.listOfPoints[i])

        intersection = False
        for k in range(len(self.obstacles)):
          p1 = (self.obstacles[k][0][0], self.obstacles[k][0][1])  # (x1, y1)
          p2 = (self.obstacles[k][1][0], self.obstacles[k][1][1])  # (x2, y2)
          if Geometry.line_intersection(p1, p2, pointToComp, newPoint) is True:
            intersection = True

        if (
          0 < abs(newPoint[0] - pointToComp[0]) <= self.expandRadius
          and 0 < abs(newPoint[1] - pointToComp[1]) <= self.expandRadius
          and not intersection
        ):
          if (
            np.linalg.norm(((newPoint - pointToComp) / np.linalg.norm((newPoint - pointToComp)))) + self.listOfCosts[i]
            < curLowestCost
          ):
            closestPoint = pointToComp
            curLowestCost = (
              np.linalg.norm(((newPoint - pointToComp) / np.linalg.norm((newPoint - pointToComp))))
              + self.listOfCosts[i]
            )

      # Storing the new point, edge, and cost of the node
      self.listOfPoints.append([newPoint[0], newPoint[1]])
      self.listOfCosts.append(curLowestCost)
      self.listOfEdges.append([closestPoint[0], closestPoint[1], newPoint[0], newPoint[1]])

      # With the new point established, nearby points will be check to see if the cost from going from the new point to the nearby point is better
      for i in range(0, len(self.listOfPoints) - 1):
        pointToComp = np.array(self.listOfPoints[i])

        intersection = False
        for k in range(len(self.obstacles)):
          p1 = (self.obstacles[k][0][0], self.obstacles[k][0][1])  # (x1, y1)
          p2 = (self.obstacles[k][1][0], self.obstacles[k][1][1])  # (x2, y2)
          if Geometry.line_intersection(p1, p2, pointToComp, newPoint) is True:
            intersection = True

        if (
          0 < abs(newPoint[0] - pointToComp[0]) <= self.expandRadius
          and 0 < abs(newPoint[1] - pointToComp[1]) <= self.expandRadius
          and not intersection
        ):
          if np.linalg.norm(((newPoint - pointToComp) / np.linalg.norm((newPoint - pointToComp)))) + curLowestCost < self.listOfCosts[i]:
            self.listOfEdges[i - 1][0] = newPoint[0]
            self.listOfEdges[i - 1][1] = newPoint[1]
            self.listOfCosts[i] = (
              np.linalg.norm(((newPoint - pointToComp) / np.linalg.norm((newPoint - pointToComp)))) + curLowestCost
            )

    # Need to trace back from the final point to the start
    curPoint = self.listOfPoints[len(self.listOfPoints) - 1]

    return len(self.listOfPoints) - 1, len(self.listOfEdges) - 1

  def reevaluatePoint(self):
    # Need to make sure that the point isn't too close to the maze walls or that the edge connecting the new point and the closest is through a wall
    update = False
    objectAbsolutePosition = np.array(self.sim.getObjectPosition(self.robot, self.sim.handle_world))
    targetAbsolutePosition = np.array(self.sim.getObjectPosition(self.target, self.sim.handle_world))
    for pointIndex in range(1, len(self.listOfPoints)):
      edgeIndex = pointIndex - 1
      closestPoint = [self.listOfEdges[edgeIndex][0], self.listOfEdges[edgeIndex][1]]
      for i in range(len(self.obstacles)):
        p1 = (self.obstacles[i][0][0], self.obstacles[i][0][1])  # (x1, y1)
        p2 = (self.obstacles[i][1][0], self.obstacles[i][1][1])  # (x2, y2)
        # If the edge between the points goes through a wall, it is forced back to the correct side of the wall
        if Geometry.line_intersection(p1, p2, closestPoint, self.listOfPoints[pointIndex]) is True:
          print('RE')
          if closestPoint[0] <= p1[0] and closestPoint[0] <= p2[0]:
            update = True
            self.listOfPoints[pointIndex][0] = p1[0] - 0.5
          elif closestPoint[0] >= p1[0] and closestPoint[0] >= p2[0]:
            update = True
            self.listOfPoints[pointIndex][0] = p1[0] + 0.5
          if closestPoint[1] <= p1[1] and closestPoint[1] <= p2[1]:
            update = True
            self.listOfPoints[pointIndex][1] = p1[1] - 0.5
          elif closestPoint[1] >= p1[1] and closestPoint[1] >= p2[1]:
            update = True
            self.listOfPoints[pointIndex][1] = p1[1] + 0.5

        # Verifying that the point isn't too close to the wall to make navigation smoother
        if (
          (self.listOfPoints[pointIndex][0] <= p1[0] and self.listOfPoints[pointIndex][0] <= p2[0])
          and (closestPoint[0] <= p1[0] and closestPoint[0] <= p2[0])
          and abs(self.listOfPoints[pointIndex][0] - p1[0]) < 0.5
          and (p1[1] <= self.listOfPoints[pointIndex][1] <= p2[1] or p1[1] >= self.listOfPoints[pointIndex][1] >= p2[1])
        ):
          update = True
          self.listOfPoints[pointIndex][0] = p1[0] - 0.5
        elif (
          (self.listOfPoints[pointIndex][0] >= p1[0] and self.listOfPoints[pointIndex][0] >= p2[0])
          and (closestPoint[0] >= p1[0] and closestPoint[0] >= p2[0])
          and abs(p1[0] - self.listOfPoints[pointIndex][0]) < 0.5
          and (p1[1] <= self.listOfPoints[pointIndex][1] <= p2[1] or p1[1] >= self.listOfPoints[pointIndex][1] >= p2[1])
        ):
          update = True
          self.listOfPoints[pointIndex][0] = p1[0] + 0.5

        if (
          (self.listOfPoints[pointIndex][1] <= p1[1] and self.listOfPoints[pointIndex][1] <= p2[1])
          and (closestPoint[1] <= p1[1] and closestPoint[1] <= p2[1])
          and abs(self.listOfPoints[pointIndex][1] - p1[1]) < 0.5
          and (p1[0] <= self.listOfPoints[pointIndex][0] <= p2[0] or p1[0] >= self.listOfPoints[pointIndex][0] >= p2[0])
        ):
          update = True
          self.listOfPoints[pointIndex][1] = p1[1] - 0.5
        elif (
          (self.listOfPoints[pointIndex][1] >= p1[1] and self.listOfPoints[pointIndex][1] >= p2[1])
          and (closestPoint[1] >= p1[1] and closestPoint[1] >= p2[1])
          and abs(p1[1] - self.listOfPoints[pointIndex][1]) < 0.5
          and (p1[0] <= self.listOfPoints[pointIndex][0] <= p2[0] or p1[0] >= self.listOfPoints[pointIndex][0] >= p2[0])
        ):
          update = True
          self.listOfPoints[pointIndex][1] = p1[1] + 0.5

      if update is True:
        self.sim.setObjectPosition(self.target, self.sim.handle_world, [objectAbsolutePosition[0], objectAbsolutePosition[1], 0.7])
        print('update ', self.listOfPoints[pointIndex])
        slightDifX = (self.listOfPoints[pointIndex][0] - targetAbsolutePosition[0]) * 0.25
        slightDifY = (self.listOfPoints[pointIndex][1] - targetAbsolutePosition[1]) * 0.25
        while slightDifX > 0.2 or slightDifX < -0.2:
          slightDifX = slightDifX * 0.5
          slightDifY = slightDifY * 0.5

        while slightDifY > 0.2 or slightDifY < -0.2:
          slightDifX = slightDifX * 0.5
          slightDifY = slightDifY * 0.5
        checkX = targetAbsolutePosition[0] + slightDifX
        print('update ', checkX)
        checkY = targetAbsolutePosition[1] + slightDifY
        numCheck = 1
        self.sim.setObjectPosition(self.target, self.sim.handle_world, [checkX, checkY, 0.7])
        needNewPoint = False

  def plot_summary(self):
    print(len(self.obstacles))
    for i in range(len(self.obstacles)):
      start_point = (self.obstacles[i][0][0], self.obstacles[i][0][1])
      end_point = (self.obstacles[i][1][0], self.obstacles[i][1][1])

      x_points = np.array([start_point[0], end_point[0]])
      y_points = np.array([start_point[1], end_point[1]])

      plt.plot(x_points, y_points, color='red')

      plt.scatter(x_points, y_points, color='red')

    for i in range(len(self.listOfEdges)):
      start_point = (self.listOfEdges[i][0], self.listOfEdges[i][1])
      end_point = (self.listOfEdges[i][2], self.listOfEdges[i][3])

      x_points = np.array([start_point[0], end_point[0]])
      y_points = np.array([start_point[1], end_point[1]])

      plt.plot(x_points, y_points, color='blue')

      plt.scatter(x_points, y_points, color='black')

    plt.scatter(x_points, y_points, color='black')
    # Optional: Add labels and title for clarity
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Line from Start to End Point')
    plt.grid(True)

    output_dir = Path("output")
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().astimezone().isoformat(timespec="seconds")
    output_path = output_dir / f"{timestamp}.png"
    plt.savefig(output_path)
    plt.close()
    return output_path

  def _scan_and_update_obstacles(self):
    coord, coordLeft, coordRight = self.ctx.sysCall_sensing(self.lastPointFound)
    if (
      len(self.lastPointFound) != 0
      and len(coord) > 0
      and ((abs(coord[1] - self.lastPointFound[1]) > 0.1) or (abs(coord[0] - self.lastPointFound[0]) > 0.1))
      and coord[2] > 0.3
    ):
      print('NEW!')
      self.lastPointFound = coord
      self.obstacles.append([coordLeft, coordRight])
      self.reevaluatePoint()
    elif len(self.lastPointFound) == 0 and len(coord) != 0 and coord[2] > 0.1:
      print('NEW!')
      self.lastPointFound = coord
      self.obstacles.append([coordLeft, coordRight])
      self.reevaluatePoint()

  def _should_use_dijkstra(self):
    for i in range(len(self.obstacles)):
      p1 = (self.obstacles[i][0][0], self.obstacles[i][0][1])  # (x1, y1)
      p2 = (self.obstacles[i][1][0], self.obstacles[i][1][1])  # (x2, y2)
      # If the edge between the points goes through a wall, it is forced back to the correct side of the wall
      if Geometry.line_intersection(
        p1,
        p2,
        [self.objectAbsolutePosition[0], self.objectAbsolutePosition[1]],
        self.listOfPoints[self.newPointIndex],
      ) is True:
        print('DIJ')
        return True
    return False

  def _set_heading_to_new_point(self):
    rotation = (
      math.atan2(
        self.listOfPoints[self.newPointIndex][1] - self.objectAbsolutePosition[1],
        self.listOfPoints[self.newPointIndex][0] - self.objectAbsolutePosition[0],
      )
      - 1.5708
    )
    self.sim.setObjectOrientation(self.robot, [0, 0, rotation])
    self.sim.setObjectOrientation(self.target, [0, 0, rotation])

  def _initialize_target_step(self, targetAbsolutePosition):
    if self.sim.getSimulationState() != 0:
      time.sleep(SLEEP_TARGET_STEP)
      self.slightDifX = (self.listOfPoints[self.newPointIndex][0] - targetAbsolutePosition[0]) * 0.25
      self.slightDifY = (self.listOfPoints[self.newPointIndex][1] - targetAbsolutePosition[1]) * 0.25
      while self.slightDifX > 0.2 or self.slightDifX < -0.2:
        self.slightDifX = self.slightDifX * 0.5
        self.slightDifY = self.slightDifY * 0.5

      while self.slightDifY > 0.2 or self.slightDifY < -0.2:
        self.slightDifX = self.slightDifX * 0.5
        self.slightDifY = self.slightDifY * 0.5
      self.checkX = targetAbsolutePosition[0] + self.slightDifX
      self.checkY = targetAbsolutePosition[1] + self.slightDifY
      self.numCheck = 1
      self.sim.setObjectPosition(self.target, self.sim.handle_world, [self.checkX, self.checkY, 0.7])
      self.needNewPoint = False

  def _handle_new_point_request(self):
    if not self.needNewPoint:
      return

    print('NEW')
    time.sleep(SLEEP_NEW_POINT)
    if self.sim.getSimulationState() != 0:
      self.reevaluatePoint()
      self.newPointIndex, self.newEdgeIndex = self.newRRTNode()

      if self._should_use_dijkstra() is False:
        targetAbsolutePosition = np.array(self.sim.getObjectPosition(self.target, self.sim.handle_world))
        targetAbsoluteRotation = np.array(self.sim.getObjectOrientation(self.target))

        self._set_heading_to_new_point()
        self._initialize_target_step(targetAbsolutePosition)
      else:
        self.dijkstraNavigation()

  def _update_active_target_progress(self):
    if self.needNewPoint:
      return

    if abs(self.objectAbsolutePosition[0] - self.checkX) < 0.5 and abs(self.objectAbsolutePosition[1] - self.checkY) < 0.5:
      print('UP')
      time.sleep(SLEEP_TARGET_PROGRESS)
      if self.sim.getSimulationState() != 0:
        self.checkX = self.checkX + self.slightDifX
        self.checkY = self.checkY + self.slightDifY
        self.numCheck = self.numCheck + 1
        self.sim.setObjectPosition(self.target, self.sim.handle_world, [self.checkX, self.checkY, 0.7])
    if (
      abs(self.objectAbsolutePosition[0] - self.listOfPoints[self.newPointIndex][0]) < 0.5
      and abs(self.objectAbsolutePosition[1] - self.listOfPoints[self.newPointIndex][1]) < 0.5
    ):
      self.needNewPoint = True

  def _update_heading(self):
    rotation = (
      math.atan2(
        self.listOfPoints[self.newPointIndex][1] - self.objectAbsolutePosition[1],
        self.listOfPoints[self.newPointIndex][0] - self.objectAbsolutePosition[0],
      )
      - 1.5708
    )
    print(rotation)
    self.sim.setObjectOrientation(self.robot, [0, 0, rotation])
    self.sim.setObjectOrientation(self.target, [0, 0, rotation])

  def run(self):
    start_time = time.time()
    while self.sim.getSimulationState() != 0:
      if time.time() - start_time > 60:
        print("Time limit reached, stopping simulation...")
        break
      self.objectAbsolutePosition = np.array(self.sim.getObjectPosition(self.robot, self.sim.handle_world))
      self._scan_and_update_obstacles()
      self._handle_new_point_request()
      self._update_active_target_progress()
      self._scan_and_update_obstacles()
      self._update_heading()

    output_path = self.plot_summary()
    print(f"Saved plot to {output_path}")
    print('Program ended')
    return

if __name__ == "__main__":
  ctx = SimulationContext()
  ctx.start_simulation()
  ctx.configure_scene()

  navigator = DroneNavigator(ctx)
  try:
    navigator.run()
  except KeyboardInterrupt:
    print("SIGINT - stopping simulation")
  finally:
    ctx.stop_simulation()
