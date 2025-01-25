import algos.utils as utils
import math
import bisect
from stonesoup.models.transition.linear import CombinedLinearGaussianTransitionModel, ConstantVelocity
import numpy as np
from stonesoup.models.measurement.linear import LinearGaussian
from stonesoup.predictor.kalman import KalmanPredictor
from stonesoup.updater.kalman import KalmanUpdater
from stonesoup.hypothesiser.distance import DistanceHypothesiser
from stonesoup.measures import Mahalanobis
from stonesoup.dataassociator.neighbour import NearestNeighbour
from stonesoup.types.state import GaussianState
from stonesoup.types.track import Track
from stonesoup.types.detection import Detection
from shapely.geometry import Polygon, Point, LineString
from shapely import buffer

class TrajImpMC:

    def __init__(self, DIR, timestep=1, negBuf=-0.01):
        pass

    def makeDetection(self, mean, heading, w, poly):
        pass

    def shrinkPolygon(self, poly, bufNeg):
        pass

    def solveGap(self, startT, gapSize, startLoc, endLoc, startSpeedX, startSpeedY):
        pass

class FindSequence:
    # start/end in format [x,y]
    def __init__(self, start, end):
        pass

    # snapshots [[timestampSTR, [pixelIDs]], ...]
    def searchCandidates(self, snapshots, startDist, id2ngbs, id2poly, endDist):
        pass


#if root: TYPE, point
#else: TYPE, coords, father, contactPoints, keyAngles
class DisjointConvexLSSPMnode:
    # coords are the boundary points of a convex polygon, without duplicates
    def __init__(self, coords, father=None):
        pass
    def fixMultiLinestring(self, ml):
        pass

    def initializeContactPoints(self):
        pass

    def initializeKeyAngles(self):
        pass

    def shortestPath(self, q):
        pass

    def shortestPathOneStep(self, q):
        pass

    def query(self, q):
        pass

class DisjointConvexLSSPM:
    def __init__(self, coordsS):
        pass

    def shortestPath(self, QUERY):
        pass

if __name__ == '__main__':
    pass