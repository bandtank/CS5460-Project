import math
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class SimulationContext:
  def __init__(self):
    self.client = RemoteAPIClient()
    self.sim = self.client.require('sim')
    self.simVision = self.client.require('simVision')

    self.robot = self.sim.getObject('/Quadcopter')
    self.sensor = self.sim.getObject('/Quadcopter/blobTo3dPosition/sensor')
    self.target = self.sim.getObject('/Quadcopter/target')

  def configure_scene(self):
    self.sphereContainer = self.sim.addDrawingObject(self.sim.drawing_spherepts, 0.03, 0, -1, 9999, [1, 0, 1])
    self.xAngle = self.sim.getObjectFloatParam(self.sensor, self.sim.visionfloatparam_perspective_angle)
    self.resX = self.sim.getObjectInt32Param(self.sensor, self.sim.visionintparam_resolution_x)
    self.resY = self.sim.getObjectInt32Param(self.sensor, self.sim.visionintparam_resolution_y)

    self.yAngle = self.xAngle
    ratio = self.resX / self.resY
    if self.resX > self.resY:
      self.yAngle = 2 * math.atan(math.tan(self.xAngle / 2) / ratio)
    else:
      self.xAngle = 2 * math.atan(math.tan(self.yAngle / 2) / ratio)

  def start_simulation(self):
    if self.sim.getSimulationState() == 0:
      self.sim.startSimulation()
    time.sleep(1) # Wait for CoppeliaSim

  def stop_simulation(self):
    if self.sim.getSimulationState() != 0:
      self.sim.stopSimulation()

  def sysCall_sensing(self, lastPointFound):
    m = self.sim.getObjectMatrix(self.sensor)
    self.sim.addDrawingObjectItem(self.sphereContainer, None)
    res, packet1, packet2 = self.sim.handleVisionSensor(self.sensor)

    coord = []
    coordLeft = []
    coordRight = []

    if res >= 0 and len(packet2) > 0:
      blobCnt = int(packet2[0])
      valCnt = int(packet2[1])
      for i in range(blobCnt):
        blobSize = packet2[2 + valCnt * i]
        blobOrientation = packet2[2 + valCnt * i + 1]
        blobPositionX = packet2[2 + valCnt * i + 2]
        blobPositionY = packet2[2 + valCnt * i + 3]
        blobWidth = packet2[2 + valCnt * i + 4]
        blobHeight = packet2[2 + valCnt * i + 5]

        depth, test = self.sim.getVisionSensorDepth(
          self.sensor,
          1,
          [
            1 + math.floor(((blobPositionX) - 0.01) * (self.resX - 0.99)),
            1 + math.floor(blobPositionY * (self.resY - 0.99)),
          ],
          [1, 1],
        )
        depth = self.sim.unpackFloatTable(depth)
        coord = [0, 0, depth[0]]
        x = 0.5 - blobPositionX
        y = blobPositionY - 0.5
        coord[0] = depth[0] * math.tan(self.xAngle * 0.5) * (x) / 0.5
        coord[1] = depth[0] * math.tan(self.yAngle * 0.5) * (blobPositionY - 0.5) / 0.5
        coord = self.sim.multiplyVector(m, coord)
        self.sim.addDrawingObjectItem(self.sphereContainer, coord)

        depthLeft, test = self.sim.getVisionSensorDepth(
          self.sensor,
          1,
          [
            1 + math.floor(((blobSize * 0.5 + blobPositionX) - 0.01) * (self.resX - 0.99)),
            1 + math.floor(blobPositionY * (self.resY - 0.99)),
          ],
          [1, 1],
        )
        depthLeft = self.sim.unpackFloatTable(depthLeft)
        coordLeft = [0, 0, depthLeft[0]]
        x = 0.5 - blobPositionX - blobWidth / 2
        y = blobPositionY - 0.5

        coordLeft[0] = depthLeft[0] * math.tan(self.xAngle * 0.5) * (x) / 0.5
        coordLeft[1] = depthLeft[0] * math.tan(self.yAngle * 0.5) * (blobPositionY - 0.5) / 0.5
        coordLeft = self.sim.multiplyVector(m, coordLeft)
        self.sim.addDrawingObjectItem(self.sphereContainer, coordLeft)

        depthRight, test = self.sim.getVisionSensorDepth(
          self.sensor,
          1,
          [
            1 + math.floor((blobPositionX - blobSize * 0.5) * (self.resX - 0.99)),
            1 + math.floor(blobPositionY * (self.resY - 0.99)),
          ],
          [1, 1],
        )
        depthRight = self.sim.unpackFloatTable(depthRight)
        coordRight = [0, 0, depthRight[0]]
        x = 0.5 - blobPositionX + blobWidth / 2
        y = blobPositionY - 0.5

        coordRight[0] = depthRight[0] * math.tan(self.xAngle * 0.5) * (x) / 0.5
        coordRight[1] = depthRight[0] * math.tan(self.yAngle * 0.5) * (blobPositionY - 0.5) / 0.5
        coordRight = self.sim.multiplyVector(m, coordRight)
        self.sim.addDrawingObjectItem(self.sphereContainer, coordRight)

    return coord, coordLeft, coordRight
