import vectorMath as vm
import numpy as np

#Group 1
g1p1 = np.array([2, 1, -2])

#Group 2
g2p1 = np.array([3, 1.5, 0])
g2p2 = np.array([2, 2.5, 0])

#Group 3
g3p1 = np.array([0, 2, -1])
g3p2 = np.array([0, 2, 0])
g3p3 = np.array([0, 2, 4])

#Calulate 3 Average Points
g1a = vm.averagePoints([g1p1])
g2a = vm.averagePoints([g2p1,g2p2])
g3a = vm.averagePoints([g3p1,g3p2,g3p3])

plane = vm.solvePlane(g1a, g2a, g3a)

arucoPoint = np.array([3, 3, -2])

intPoint = vm.intersect(arucoPoint, plane)
print (intPoint)



#print(g1a)
#print(g2a)
#print(g3a)
#print(plane)