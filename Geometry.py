import math

class Geometry:
  @staticmethod
  def distance(p1, p2):
    return math.sqrt(((p1[0] - p2[0]) ** 2) + ((p1[1] - p2[1]) ** 2))

  @staticmethod
  def is_between_points_dist(A, B, C):
    AB = Geometry.distance(A, B)
    AC = Geometry.distance(A, C)
    CB = Geometry.distance(C, B)

    return (AC + CB) - AB <= 0.001

  @staticmethod
  def line_intersection(p1, p2, p3, p4):
    D = ((p1[0] - p2[0]) * (p3[1] - p4[1])) - ((p1[1] - p2[1]) * (p3[0] - p4[0]))
    if D == 0:
      return True

    t = ((((p1[0] - p3[0]) * (p3[1] - p4[1])) - ((p1[1] - p3[1]) * (p3[0] - p4[0]))) / D)
    u = (((p1[0] - p3[0]) * (p1[1] - p2[1])) - ((p1[1] - p3[1]) * (p1[0] - p2[0]))) / D

    return (0 <= t <= 1 and 0 <= u <= 1)
