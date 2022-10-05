#   Eric Sobczak
#   8/26/2022
#   SEED Lab Python Assignment 1, Excercise 1


# This program reads throught the values in " datafile.txt" and outputs 
# information on values within. The program lists both the largest and 
# smallest values in the datafile. The program also finds all the 
# locations of a particular values and gives you their index within the 
# list. It is currently set to look for 38 in "indexToFind" variable. 
# Then, reapeated values are listed. The vzlues that are listed can be
# adjusted with the "repeatedBreak" vraiable. Lastly, the lists are sorted
# and also seperated to just even. All these taks are completled using
# Numpy. The datafile.txt can be changed to a different filetype by changing
# the "datafile" vraible.


##########   IMPORT AND CONSTANTS   ##########
# Import the numpy package as np
import numpy as np

#Constants
#Name of file containing integars
datafile = "datafile.txt"
#What value to find the index of within the array
indexToFind = 38
#Determine mininum number of repeats to be considered relevant
repeatedBreak = 3

##########   FUNCTIONS   ##########
#Code for finding Max
def findMax(np_array):
    if np_array.size > 0:
        return np.amax(np_array)
    else:
        Exception("Cant find max of empty array")

#Code for finding Min
def findMin(np_array):
    if np_array.size > 0:
        return np.amin(np_array)
    else:
        Exception("Cant find min of empty array")

#Code for finding index of a value
def findValue(np_array, value):
    tempLocal =  np.where(np_array == value)[0]
    #Return string with info, but format depending on count
    if tempLocal.size > 1:
        return ("can be found at the indexes " + str(tempLocal))
    elif tempLocal.size == 1:
        return ("can be found at the index " + str(tempLocal[0]))
    else:
        return ("cannot be found")

#Repeated numbers view
def repeatedValues(np_array):
    #Sort original list so preview looks nice
    np_sorted = np.sort(np_array)
    #Use Numpy Unique to find which value is most common
    np_unique = np.unique(np_sorted, return_counts=True)
    np_unique_values = np_unique[0]
    np_unique_counts = np_unique[1]
    #Calculate sort permutation on unique counts
    np_counts_perm = np_unique_counts.argsort()
    #Apply sort prmutation and flip
    np_values_sorted = np.flip(np_unique_values[np_counts_perm])
    np_counts_sorted = np.flip(np_unique_counts[np_counts_perm])
    #Return two new arrays for printing
    return([np_values_sorted, np_counts_sorted])

#Sorted List
def sortList(np_array):
    return np.sort(np_array)

#All even numbers, in order,
def sortEven(np_array):
    #Use list comprehension to limit to only even values.
    np_even_values =[value for value in np_array if value %2 == 0]
    return np.sort(np_even_values)

##########   INITIALIZATION   ##########
#Open the datafile containg integars
try:
    with open(datafile,'r') as f:
        integarList = eval(f.read())
    #Convert the list into a Numpy List
    np_integarList = np.array(integarList)
except:
    #Raise exception if file is bad
    raise Exception("Invalid file!")

#Print Max, Min, and Index
print("The Maximum value within the list of integars is", findMax(np_integarList))
print("The Mininum value within the list of integars is", findMin(np_integarList))
print("The value", indexToFind, findValue(np_integarList, indexToFind))

#Print repeated values
np_repeated_info = repeatedValues(np_integarList)
if (np_repeated_info[1][0] > 1):
    print("\nThis list contains repeated value, these are all the ones with more than two occurrences")
    print("Value \t|  # of repeats")
    for index, x in enumerate(np_repeated_info[1]):
        #Only prints values above the threshold
        if x >= repeatedBreak:
            print(" "  + str(np_repeated_info[0][index]) + "\t|   " + str(x))
else:
    print("\nThis list contains no reaptead values")
    
#Print sorted array:
print("\nList of integars sorted using Numpy:", end =" ")
np_sorted_list = sortList(np_integarList)
for index, x in enumerate(np_sorted_list):
    #Create a new line after 10 values
    if index % 10 == 0:
        print()
    print(x, end ="\t ")

#Print even numbers in order
print("\n\nSorted even numbers from list using list comprehension", end =" ")
np_sorted_even = sortEven(np_integarList)
for index, x in enumerate(np_sorted_even):
    #Create a new line after 10 values
    if index % 10 == 0:
        print()
    print(x, end ="\t ")

##########   SOURCES USED   ##########
#Numpy Max, Min, Size, Where, Flip, Sort, Argsort:
# https://numpy.org/doc/stable/reference/generated/numpy.amax.html
# https://numpy.org/doc/stable/reference/generated/numpy.amin.html
# https://numpy.org/doc/stable/reference/generated/numpy.ndarray.size.html
# https://numpy.org/doc/stable/reference/generated/numpy.where.html
# https://numpy.org/doc/stable/reference/generated/numpy.flip.html
# https://numpy.org/doc/stable/reference/generated/numpy.sort.html
# https://numpy.org/doc/stable/reference/generated/numpy.argsort.html
#
#How to calculate and apply sort permutation
# https://exchangetuts.com/how-can-i-zip-sort-parallel-numpy-arrays-1639552927939566
#
#Tabs in python
# https://www.delftstack.com/howto/python/python-print-tab/
#
#Print without newline
# https://www.geeksforgeeks.org/print-without-newline-python/
#
#Exception handeling tips
# https://www.w3schools.com/python/python_try_except.asp