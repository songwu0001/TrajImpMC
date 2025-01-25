import math
import geopandas as gpd
from datetime import datetime,timedelta

#if p1 equals p2, return "SAME"
#otherwise, return angle in [0.0,360.0)
#anti-clockwise, due east is zero degrees
def angle(p1, p2):
    if p1 == p2:
        return "SAME"
    if p1[0] == p2[0]:
        if p1[1] < p2[1]:
            return 90
        else:
            return 270
    ans = math.atan(1.0 * (p2[1] - p1[1]) / (p2[0] - p1[0]))
    if p1[0] > p2[0]:
        ans += math.pi
    if ans < 0:
        ans += 2 * math.pi
        
    return math.degrees(ans)

#if p1 equals p2, return "SAME"
#otherwise, return angle in [0.0,360.0)
#clockwise, due north is zero degrees
def heading(p1, p2):
    if p1 == p2:
        return "SAME"
    return angle(p1[::-1], p2[::-1])

#[0, 180]
#a1 and a2 are in degrees
def angleDiff(a1, a2):
    diff = (1.0 * max(a1, a2) - min(a1, a2)) % 360.0
    return diff if diff <= 180 else 360 - diff

#[0, 180]
#a1 and a2 are in degrees
def headingDiff(a1, a2):
    return angleDiff(a1, a2)

def groupBy(vals, fun):
    prev = 'None'
    answer = []
    for str in vals:
        if fun(str) != prev:
            prev = fun(str)
            answer.append([])
        answer[-1].append(str)
    return answer

#角度在扇形内, 两边inclusive
#start and end cannot be the same
def angleInSector(deg, start, end, colckWise=False):
    deg %= 360
    start %= 360
    end %= 360
    if not colckWise:
        if start < end:
            return deg >= start and deg <= end
        if start > end:
            return deg >= start or deg <= end
    if colckWise:
        if start > end:
            return deg <= start and deg >= end
        if start < end:
            return deg <= start or deg >= end

#inFiles have the same header
def unionFiles(outFile, inFiles):
    with open(outFile, 'w') as fOut:
        with open(inFiles[0], "r") as fIn:
            fOut.write("".join(fIn.readlines()))
        for inFile in inFiles[1:]:
            with open(inFile, "r") as fIn:
                fOut.write("".join(fIn.readlines()[1:]))

# print(keypointAlongLine([[0,0], [0,0]], [[1,1],[0.1,1]]))
# print(point2segment([0,0], [1,1],[-1,1]))

#angle is in degrees
#anti-clockwise, due east is zero degrees
def rotatePoint(p, angle):
    x1,y1 = p
    angleNew = math.radians(angle)
    x2,y2 = math.cos(angleNew),math.sin(angleNew)
    return x1*x2-y1*y2,x2*y1+x1*y2

def point2segment(p, s1, s2):
    pX,pY = p
    s1X,s1Y = s1
    s2X,s2Y = s2
    prod = (pX - s1X) * (s2X - s1X) + (pY - s1Y) * (s2Y - s1Y)
    if prod <= 0:
        return math.dist(p, s1), s1
    if prod >= ((s1X - s2X)**2 + (s1Y - s2Y)**2):
        return math.dist(p, s2), s2
    r = prod / ((s1X - s2X)**2 + (s1Y - s2Y)**2)
    sX = s1X + r * (s2X - s1X)
    sY = s1Y + r * (s2Y - s1Y)
    return math.dist(p, (sX, sY)), (sX, sY)

#https://blog.51cto.com/u_16175471/8483338
def segmentsIntersection(p1, p2, p3, p4):
    x1,y1 = p1
    x2,y2 = p2
    x3,y3 = p3
    x4,y4 = p4
    A1 = y2 - y1
    B1 = x1 - x2
    C1 = x2 * y1 - x1 * y2
    A2 = y4 - y3
    B2 = x3 - x4
    C2 = x4 * y3 - x3 * y4
    ansX = (B1 * C2 - B2 * C1) / (A1 * B2 - A2 * B1)
    ansY = (A2 * C1 - A1 * C2) / (A1 * B2 - A2 * B1)
    return ansX,ansY

def flipPoint(p1,p2,p):
    deg1 = angle(p1, p2)
    deg2 = angle(p1, p)
    targetDeg = (2 * deg1 - deg2)%360.0
    dist = math.dist(p1, p)
    return [p1[0] + dist * math.cos(math.radians(targetDeg)), p1[1] + dist * math.sin(math.radians(targetDeg))]
def whichSide(p1,p2,p):
    x1,y1 = p1
    x2,y2 = p2
    x,y = p
    cross_product = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)
    if cross_product == 0:
        return "online"
    elif cross_product < 0:
        return "right"
    else:
        return "left"

tFormat1 = "%Y-%m-%d %H:%M:%S"
tFormat2 = "%d/%m/%Y %H:%M:%S"
def tObj2tStr(tObj, format):
    return tObj.strftime(format)
def tStr2tObj(tStr, format):
    return datetime.strptime(tStr, format)
def tStr2tStr(tStr, fromFormat, toFormat):
    return tObj2tStr(tStr2tObj(tStr, fromFormat), toFormat)
def tStrDiff(tStr1, tStr2, format1, format2):
    return int((tStr2tObj(tStr1, format1) - tStr2tObj(tStr2, format2)).total_seconds())
def tObjDiff(tObj1, tObj2):
    return int((tObj1 - tObj2).total_seconds())
def tStrAdd2Str(tStr, format, addedSeconds):
    return tObj2tStr(tStr2tObj(tStr, format) + timedelta(seconds=addedSeconds), format)
def tStrAdd2Obj(tStr, format, addedSeconds):
    return tStr2tObj(tStr, format) + timedelta(seconds=addedSeconds)
def tObjAdd2Str(tObj, format, addedSeconds):
    return tObj2tStr(tObj + timedelta(seconds=addedSeconds), format)
def tObjAdd2Obj(tObj, addedSeconds):
    return tObj + timedelta(seconds=addedSeconds)

#input and output are of type Point
def point4326to25832(point):
    return gpd.GeoSeries([point], crs=4326).to_crs(25832)[0]
def points4326to25832(points):
    return list(gpd.GeoSeries(points, crs=4326).to_crs(25832))
#input and output are of type Point
def point25832to4326(point):
    return gpd.GeoSeries([point], crs=25832).to_crs(4326)[0]
def points25832to4326(points):
    return list(gpd.GeoSeries(points, crs=25832).to_crs(4326))
def polygon4326to25832(poly):
    return gpd.GeoSeries([poly], crs=4326).to_crs(25832)[0]
def polygon25832to4326(poly):
    return gpd.GeoSeries([poly], crs=25832).to_crs(4326)[0]
def polygons4326to25832(polys):
    return list(gpd.GeoSeries(polys, crs=4326).to_crs(25832))
def polygons25832to4326(polys):
    return list(gpd.GeoSeries(polys, crs=25832).to_crs(4326))

def polygon2text(polygon, withOrder=True):
    answer = []
    for index,coord in enumerate(list(polygon.exterior.coords)):
        answer.append((str(index)+"," if withOrder else "") + str(coord[0]) + "," + str(coord[1]))
    return answer
def point2text(p, withOrder=True):
    return [("0," if withOrder else "") + str(p.coords[0][0]) + "," + str(p.coords[0][1])]
def line2text(line, withOrder=True):
    answer = []
    for index,coord in enumerate(list(line.coords)):
        answer.append((str(index)+"," if withOrder else "") + str(coord[0]) + "," + str(coord[1]))
    return answer

def evenPartition(fromP, toP, k):
    diffX = toP[0] - fromP[0]
    diffY = toP[1] - fromP[1]
    coefs = [1.0 / (k+1) * (_+1) for _ in range(k)]
    return [(fromP[0] + diffX * coef, fromP[1] + diffY * coef) for coef in coefs]

if __name__ == '__main__':
    print(angleDiff(120,-20))