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
        self.timestep = timestep

        with open(DIR + 'trajsGuessFullsimplify.csv', 'r') as fIn:
            lines = fIn.readlines()[1:]
        self.polysByT = []
        for line in lines:
            parts = line.strip().split(',')
            self.polysByT.append([parts[0], [int(_) for _ in parts[1].split('@')]])
        self.T = [_[0] for _ in self.polysByT]

        indexName = 'trajsGuessFullindex.csv' if timestep == 1 else ('trajsGuessFullindex' + str(timestep) + '.csv')
        with open(DIR + indexName, 'r') as fIn:
            lines = fIn.readlines()[1:]
        self.polyID2ngbs = {}
        for line in lines:
            parts = line.strip().split(',')
            self.polyID2ngbs[int(parts[0])] = set([int(_) for _ in parts[1].split('@')])

        with open(DIR + 'trajsGuessFullDiffPolys.csv', 'r') as fIn:
            lines = fIn.readlines()[1:]
        self.id2poly = {}
        for line in lines:
            parts = line.strip().split(',')
            boundaryX = [float(_) for _ in parts[2].split('@')]
            boundaryY = [float(_) for _ in parts[3].split('@')]
            self.id2poly[int(parts[0])] = Polygon(list(zip(boundaryX, boundaryY)))


        self.id2polyShrinked = {}
        for _ in self.id2poly:
            self.id2polyShrinked[_] = self.shrinkPolygon(self.id2poly[_], negBuf)

        self.transition_model = CombinedLinearGaussianTransitionModel((ConstantVelocity(0.05), ConstantVelocity(0.05)))
        self.measurement_model = LinearGaussian(ndim_state=4, mapping=[0, 2], noise_covar=np.diag([15, 15]))
        self.predictor = KalmanPredictor(self.transition_model)
        self.updater = KalmanUpdater(self.measurement_model)
        self.measure = Mahalanobis()
        self.hypothesiser = DistanceHypothesiser(self.predictor, self.updater, self.measure)
        self.data_associator = NearestNeighbour(self.hypothesiser)

    def makeDetection(self, mean, heading, w, poly):
        if poly.contains(Point(mean)):
            return mean
        longAxis = math.sqrt(max(w))
        shortAxis = math.sqrt(min(w))
        coords = list(poly.exterior.coords)
        bestP = 0
        bestScore = 100000000
        for k in range(len(coords) - 1):
            p1 = coords[k]
            p2 = coords[k + 1]
            newP1 = utils.rotatePoint([p1[0] - mean[0], p1[1] - mean[1]], -heading)
            newP2 = utils.rotatePoint([p2[0] - mean[0], p2[1] - mean[1]], -heading)
            newP1 = [newP1[0] / longAxis, newP1[1] / shortAxis]
            newP2 = [newP2[0] / longAxis, newP2[1] / shortAxis]
            ans = utils.point2segment([0, 0], newP1, newP2)
            if ans[0] < bestScore:
                bestScore, bestP = ans
        bestP = bestP[0] * longAxis, bestP[1] * shortAxis
        bestP = utils.rotatePoint(bestP, heading)
        return bestP[0] + mean[0], bestP[1] + mean[1]

    def shrinkPolygon(self, poly, bufNeg):
        factor = 1
        while True:
            newPoly = buffer(poly, bufNeg * 1.0 / factor)
            if str(newPoly) == 'POLYGON EMPTY':
                factor *= 4
            else:
                if poly.contains(newPoly.convex_hull):
                    return newPoly.convex_hull
                curBuffer = bufNeg * 1.0 / factor
                step = curBuffer / 100.0
                while True:
                    curBuffer += step
                    newPoly = buffer(poly, curBuffer)
                    if str(newPoly) == 'POLYGON EMPTY':
                        raise Exception("No convex sub-polygon found!")
                    if poly.contains(newPoly.convex_hull):
                        return newPoly.convex_hull

    def solveGap(self, startT, gapSize, startLoc, endLoc, startSpeedX, startSpeedY):
        fs = FindSequence(startLoc, endLoc)
        pivot = bisect.bisect_left(self.T, startT)
        toi = list(range(pivot+self.timestep, pivot+gapSize, self.timestep))
        seconds = [self.polysByT[_] for _ in toi]
        fs.searchCandidates(seconds, 1.852*50/3.6 * self.timestep, self.polyID2ngbs, self.id2poly, 1.852*50*3.6*(pivot+gapSize-toi[-1]))
        seconds = fs.sequence




        selectedPolysID = []
        selectedPolysTimes = []
        stateStart = [startLoc[0], startSpeedX, startLoc[1], startSpeedY]
        tracks = []
        diag = np.diag([1.5, 0.5, 1.5, 0.5])
        tracks.append(Track([GaussianState(stateStart, diag, timestamp=utils.tStr2tObj(startT, utils.tFormat1))]))
        HH = np.eye(tracks[0].ndim)[[0, 2], :]
        for second in seconds:
            detectionT = utils.tStr2tObj(second[0], utils.tFormat1)
            polys_ = [[_, self.id2poly[_]] for _ in second[1] if len(selectedPolysID) == 0 or selectedPolysID[-1] in second[1][_]]

            prediction = self.predictor.predict(tracks[0].states[-1], detectionT)
            predictionMean = [prediction.state_vector[0], prediction.state_vector[2]]
            w, v = np.linalg.eig(HH @ prediction.covar @ HH.T)
            max_ind = np.argmax(w)
            orient = np.rad2deg(np.arctan2(v[1, max_ind], v[0, max_ind]))
            detectionS = {Detection(self.makeDetection(predictionMean, orient, w, poly[1]), timestamp=detectionT,
                                    measurement_model=self.measurement_model, metadata={'polyID':poly[0]}) for poly in polys_}

            hypotheses = self.data_associator.associate(tracks, detectionS, detectionT)
            for track in tracks:
                hypothesis = hypotheses[track]
                if hypothesis.measurement:
                    post = self.updater.update(hypothesis)
                    track.append(post)
                    polyID = hypothesis.measurement.metadata['polyID']
                    if len(selectedPolysID) == 0:
                        selectedPolysID.append(polyID)
                        selectedPolysTimes.append(1)
                    else:
                        if polyID == selectedPolysID[-1]:
                            selectedPolysTimes[-1] += 1
                        else:
                            selectedPolysID.append(polyID)
                            selectedPolysTimes.append(1)
                else:
                    raise Exception("Error in stonesoup: no hypothesis found!")




        coordsS = []
        coordsS.append([startLoc])
        for polyID in selectedPolysID:
            coordsS.append(self.id2polyShrinked[polyID].exterior.coords[:-1])
        lsspm = DisjointConvexLSSPM(coordsS)
        sp = lsspm.shortestPath(endLoc)




        start = sp[0]
        end = sp[-1]
        path = sp[1:-1]
        newPoints = []
        for idx in range(len(path)):
            if idx == 0 and path[0] == start:
                newPoints += utils.evenPartition(start, path[1], selectedPolysTimes[idx])
            else:
                newPoints.append(path[idx])
                if selectedPolysTimes[idx] > 1:
                    newPoints += utils.evenPartition(path[idx], (path[idx+1] if idx + 1 < len(path) else end), selectedPolysTimes[idx]-1)
        assert len(newPoints) == len(toi)


        return  [start] + newPoints + [end]

class FindSequence:
    # start/end in format [x,y]
    def __init__(self, start, end):
        self.start = Point(start)
        self.end = Point(end)

    # snapshots [[timestampSTR, [pixelIDs]], ...]
    def searchCandidates(self, snapshots, startDist, id2ngbs, id2poly, endDist):
        self.sequence = []
        for snapshot in snapshots:
            tmp = []
            timestamp, polyIDs = snapshot
            tmp.append(timestamp)
            tmp.append({})

            if len(self.sequence) == 0:
                for polyID in polyIDs:
                    if id2poly[polyID].distance(self.start) <= startDist:
                        tmp[-1][polyID] = []
            else:
                fathers = list(self.sequence[-1][1].keys())
                for father in fathers:
                    ngbs = id2ngbs[father]
                    for polyID in polyIDs:
                        if polyID in ngbs:
                            if polyID not in tmp[-1]:
                                tmp[-1][polyID] = []
                            tmp[-1][polyID].append(father)
            if not bool(tmp[-1]):
                raise Exception("No solution found")
            self.sequence.append(tmp)

        # remove useless paths
        for idx in range(len(self.sequence) - 1, -1, -1):
            node = self.sequence[idx][1]
            if idx == len(self.sequence) - 1:
                pixels = list(node.keys())
                for pixel in pixels:
                    if id2poly[pixel].distance(self.end) > endDist:
                        node.pop(pixel)
                if not bool(node):
                    raise Exception("No solution found")
            else:
                nextNode = self.sequence[idx + 1][1]
                nextNodeLabels = []
                for pixel in nextNode:
                    nextNodeLabels += nextNode[pixel]
                pixels = list(node.keys())
                for pixel in pixels:
                    if pixel not in nextNodeLabels:
                        node.pop(pixel)


#if root: TYPE, point
#else: TYPE, coords, father, contactPoints, keyAngles
class DisjointConvexLSSPMnode:
    # coords are the boundary points of a convex polygon, without duplicates
    def __init__(self, coords, father=None):
        if len(coords) == 1:
            self.TYPE = "root"
            self.point = coords[0]
        else:
            self.TYPE = "child"
            self.father = father

            # make it counter-clockwise
            self.coords = coords
            if utils.whichSide(self.coords[0], self.coords[1], self.coords[2]) == "right":
                self.coords = self.coords[::-1]

            if self.father.TYPE == "root" and Polygon(self.coords).intersects(Point(self.father.point)):
                self.dummy = True
            else:
                self.dummy = False

            if not self.dummy:
                self.initializeContactPoints()
                self.initializeKeyAngles()

    def fixMultiLinestring(self, ml):
        geoms = list(ml.geoms)
        geoms = [list(ls.coords) for ls in geoms]
        for k in range(len(geoms) - 1):
            if geoms[k][1] != geoms[k + 1][0]:
                raise Exception("MultiLinestrings not connected!")
        return [geoms[0][0], geoms[-1][1]]

    def initializeContactPoints(self):
        partBefore = []
        partAfter = []
        poly = Polygon(self.coords)
        flagBefore = True
        for coord in self.coords:
            if LineString([self.father.query(coord), coord]).crosses(poly):
                flagBefore = False
            else:
                which = partBefore if flagBefore else partAfter
                which.append(coord)
        contactPoints = partAfter + partBefore
        if len(contactPoints) == 2:
            self.contactPoints = contactPoints
            return

        maxIntersectionLength = -1000000
        maxIntersectionIndex = None
        for idx,cp in enumerate(contactPoints):
            nextIdx = (idx + 1) if idx + 1 < len(contactPoints) else 0
            midPoint = [(contactPoints[idx][0] + contactPoints[nextIdx][0]) / 2.0, (contactPoints[idx][1] + contactPoints[nextIdx][1]) / 2.0]
            intersection = LineString([self.father.query(midPoint), midPoint]).intersection(poly)
            if str(intersection) != "LINESTRING Z EMPTY":
                if str(intersection).startswith("MULTILINESTRING"):
                    fix = self.fixMultiLinestring(intersection)
                    length = math.dist(*fix)
                elif str(intersection).startswith("LINESTRING"):
                    length = math.dist(*list(intersection.coords))
                else:
                    length = 0
                if length > maxIntersectionLength:
                    maxIntersectionLength = length
                    maxIntersectionIndex = [idx,nextIdx]
        if maxIntersectionIndex[0] < maxIntersectionIndex[1]:
            contactPoints = contactPoints[maxIntersectionIndex[1]:] + contactPoints[:maxIntersectionIndex[1]]
        self.contactPoints = contactPoints

    def initializeKeyAngles(self):
        self.keyAngles = []
        for idx,coord in enumerate(self.contactPoints):
            previous = self.father.query(coord)
            self.keyAngles.append([])

            if idx == 0:
                targetDeg = utils.angle(previous, coord)
            else:
                deg1 = utils.angle(previous, coord)
                deg2 = utils.angle(coord, self.contactPoints[idx - 1])
                if utils.angleDiff(deg1, deg2) > 90:
                    deg2 = deg2 + 180.0
                targetDeg = (2 * deg2 - deg1)%360.0
            self.keyAngles[-1].append(targetDeg)

            if idx == len(self.contactPoints) - 1:
                targetDeg = utils.angle(previous, coord)
            else:
                deg1 = utils.angle(previous, coord)
                deg2 = utils.angle(coord, self.contactPoints[idx + 1])
                if utils.angleDiff(deg1, deg2) > 90:
                    deg2 = deg2 + 180.0
                targetDeg = (2 * deg2 - deg1)%360.0
            self.keyAngles[-1].append(targetDeg)

    def shortestPath(self, q):
        keyPoints = []
        step = self.shortestPathOneStep(q)
        if len(step) == 5:
            keyPoints.append(step[3:])
        else:
            keyPoints.append(step[-1])
        while step[0] is not None:
            step = step[0].shortestPathOneStep(step[1])
            if len(step) == 5:
                keyPoints.append(step[3:])
            else:
                keyPoints.append(step[-1])
        keyPoints_ = []
        for point in keyPoints[::-1]:
            if type(point[0]) != float:
                intersection = LineString([point[0], keyPoints_[-1]]).intersection(point[1])
                if str(intersection) == "LINESTRING Z EMPTY":
                    coords = point[1].exterior.coords
                    distances = [math.dist(coord, point[0]) + math.dist(coord, keyPoints_[-1]) for coord in coords]
                    keyPoints_.append(coords[distances.index(min(distances))])
                elif str(intersection).startswith("MULTILINESTRING"):
                    fix = self.fixMultiLinestring(intersection)
                    keyPoints_.append(fix[1])
                else:
                    keyPoints_.append(intersection.coords[1])
            else:
                keyPoints_.append(point)
        return keyPoints_

    def shortestPathOneStep(self, q):
        if self.TYPE == "root":
            return [None, None, self.point]
        if self.dummy:
            return [self.father, None, self.father.point]
        for idx, cp in enumerate(self.contactPoints):
            deg = utils.angle(cp, q)
            if deg == "SAME" or (utils.angleInSector(deg, self.keyAngles[idx][0], self.keyAngles[idx][1])):
                return [self.father, cp, cp]
        for idx, cp in enumerate(self.contactPoints):
            if idx < len(self.contactPoints) - 1:
                nextCP = self.contactPoints[idx + 1]
                deg1 = utils.angle(cp, q)
                deg2 = utils.angle(nextCP, q)
                bool1 = utils.angleInSector(deg1, self.keyAngles[idx][1], utils.angle(cp, nextCP))
                bool2 = utils.angleInSector(deg2, utils.angle(nextCP, cp), self.keyAngles[idx + 1][0])
                if bool1 and bool2:
                    image = utils.flipPoint(cp, nextCP, q)
                    previousPoint = self.father.query(image)
                    intersection = LineString([cp, nextCP]).intersection(LineString([image, previousPoint]))
                    if str(intersection) == "LINESTRING Z EMPTY":
                        tmpANS = utils.segmentsIntersection(image, previousPoint, cp, nextCP)
                        minX,maxX = min(cp[0], nextCP[0]),max(cp[0], nextCP[0])
                        minY,maxY = min(cp[1], nextCP[1]),max(cp[1], nextCP[1])
                        if tmpANS[0] <= maxX and tmpANS[1] <= maxY and tmpANS[0] >= minX and tmpANS[1] >= minY:
                            return [self.father, image, tmpANS]
                        return [self.father, image, (cp if math.dist(cp, tmpANS) <= math.dist(nextCP, tmpANS) else nextCP)]
                    else:
                        return [self.father, image, intersection.coords[0]]
        return [self.father, q, None, q, Polygon(self.coords)]

    def query(self, q):
        if self.TYPE == "root":
            return self.point
        if self.dummy:
            return self.father.point
        # cones inclusive; three-sided open regions exclusive
        for idx, cp in enumerate(self.contactPoints):
            deg = utils.angle(cp, q)
            if deg == "SAME" or (utils.angleInSector(deg, self.keyAngles[idx][0], self.keyAngles[idx][1])):
                return cp
        for idx, cp in enumerate(self.contactPoints):
            if idx < len(self.contactPoints) - 1:
                nextCP = self.contactPoints[idx + 1]

                deg1 = utils.angle(cp, q)
                deg2 = utils.angle(nextCP, q)
                bool1 = utils.angleInSector(deg1, self.keyAngles[idx][1], utils.angle(cp, nextCP))
                bool2 = utils.angleInSector(deg2, utils.angle(nextCP, cp), self.keyAngles[idx + 1][0])
                if bool1 and bool2:
                    image = utils.flipPoint(cp, nextCP, q)
                    intersection = LineString([cp, nextCP]).intersection(LineString([image, self.father.query(image)]))
                    if str(intersection) == "LINESTRING Z EMPTY":
                        tmpANS = utils.segmentsIntersection(image, self.father.query(image), cp, nextCP)
                        minX, maxX = min(cp[0], nextCP[0]), max(cp[0], nextCP[0])
                        minY, maxY = min(cp[1], nextCP[1]), max(cp[1], nextCP[1])
                        if tmpANS[0] <= maxX and tmpANS[1] <= maxY and tmpANS[0] >= minX and tmpANS[1] >= minY:
                            return tmpANS
                        return cp if math.dist(cp, tmpANS) <= math.dist(nextCP, tmpANS) else nextCP
                    return intersection.coords[0]
        return self.father.query(q)

class DisjointConvexLSSPM:
    def __init__(self, coordsS):
        self.nodes = []
        self.nodes.append(DisjointConvexLSSPMnode(coordsS[0]))
        for idx,coords in enumerate(coordsS[1:]):
            self.nodes.append(DisjointConvexLSSPMnode(coords, self.nodes[-1]))

    def shortestPath(self, QUERY):
        return self.nodes[-1].shortestPath(QUERY) + [QUERY]

if __name__ == '__main__':
    # point = [[0,0]]
    # poly1 = [[1.0,1.0], [1.0,2.0], [2.0,2.0], [2.0,1.0]]
    # poly2 = [[2.2,0.0], [2.2,0.2], [2.4,0.2], [2.4,0.0]]
    # QUERY = [2.2,0.4]
    # lsspm = DisjointConvexLSSPM([point, poly1, poly2])
    # print(lsspm.shortestPath(QUERY))

    pass