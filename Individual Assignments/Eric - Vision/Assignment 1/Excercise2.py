#   Eric Sobczak
#   8/28/2022
#   SEED Lab Python Assignment 1, Excercise 2

# This program uses a Finite State Machine to detect the sequence 
# "abcd" within a user entered string. WHen the program runs, the 
# user is asked to input a string and then hit ebter. Then the 
# code goes letter by letter through the entered string and passes 
# them to a state machine. If the FSM machine detects "abcd" it will 
# list out its occurences, otherwise it will say none were found.

##########   FSM STATES   ##########
def state0(nextLetter):
    #If A was found go to A
    if nextLetter == "a":
        return stateA
    #Stay at zero becuase irrelevent next letter
    else:
        return state0
    
def stateA(nextLetter):
    #A was found, go back to start
    if nextLetter == "a":
        return stateA
    #If B was found go to B
    elif nextLetter == "b":
        return stateB
    #Go back to zero becuase irrelevent next letter
    else:
        return state0
    
def stateB(nextLetter):
    #A was found, go back to start
    if nextLetter == "a":
        return stateA
    #If C was found go to C
    elif nextLetter == "c":
        return stateC
    #Go back to zero becuase irrelevent next letter
    else:
        return state0
    
def stateC(nextLetter):
    #A was found, go back to start
    if nextLetter == "a":
        return stateA
    #If D was found go to D
    elif nextLetter == "d":
        return stateD
    #Go back to zero becuase irrelevent next letter
    else:
        return state0
    
def stateD(nextLetter):
    #A was found, go back to start
    if nextLetter == "a":
        return stateA
    #Return to 0 becuase no string
    else:
        return state0    

#Dictionary containg all states
state_dictionary = {
    state0 : "Searching",
    stateA : "A Found",
    stateB : "B Found",
    stateC : "C Found",
    stateD : "D Found"
    }

##########   INITIALIZATION   ##########
state = state0 #initial state
foundLocations = [] #Holder for all found locations

print("Enter a string of text, and the FSM will look for 'abcd'")
#Collect input from user
inputString = input("input: ")

#Loop through all characters in string
for index, x in enumerate(inputString):
    new_state = state(x)
    state = new_state
    #If the FSM has reached state D, the string was found
    if (state == stateD):
        #Markdown the index of where the string occured
        foundLocations.append((index-3))
        
#Loop through found locations and print
if (len(foundLocations) == 0):
    #If none where found, print none
    print("No 'abcd' was found")
else:
    #Print found with index of 'abcd'
    for i in foundLocations:
        print("'abcd' found at the index", i)

##########   SOURCES USED   ##########
#I forgot how to iterate through a string in python, forogt for loop doesnt require anything additional
# https://www.geeksforgeeks.org/iterate-over-characters-of-a-string-in-python/