import numpy as np

def averagePoints(pointArray):
    averagePoint = np.array([0.0,0.0,0.0])
    index = 0
    for point in pointArray:
        averagePoint += point
        index += 1
    averagePoint = averagePoint/index
    return averagePoint
        

def solvePlane(p1, p2, p3):
    #Define two vectors that describe the plane
    vectorA = p3 - p1
    vectorB = p2 - p1
    
    #Calculate the cross product to find normal vector
    planeNormal = np.cross(vectorA, vectorB)
    
    #Average the points to get the origin as the center
    planePoint = averagePoints([p1, p2, p3])

    return [planeNormal,planePoint]

def intersect(point, plane):
    ndotu = np.dot(plane[0],point)
    w = -plane[1]
    si = -(np.dot(plane[0],w))/ndotu
    Psi = w + plane[1] + (si * point)
    return Psi

def findOrigin(plane, scale): 
    pass


# Find plane from 3 points
# https://kitchingroup.cheme.cmu.edu/blog/2015/01/18/Equation-of-a-plane-through-three-points/
#
# Plane and line interesection
# https://stackoverflow.com/questions/5666222/3d-line-plane-intersection
# https://web.archive.org/web/20210421132230/http://geomalgorithms.com/a05-_intersect-1.html
#
# Normalize the point relative to the plane
# https://math.stackexchange.com/questions/108123/convert-a-world-point-to-a-local-point-relative-to-a-plane


#0,0,0 -> 0,0,camera height
#planePoint -> scale, 0, 0
#X,0,Y -> origin