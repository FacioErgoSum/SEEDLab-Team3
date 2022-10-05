# The following code demonstrates a very simple idiom for a state machine.
# Each of the state functions below performs some action and then implements
# logic to choose next state.

# define state variable, and initialize state 0
state = 0

# create a dictionary to describe the states
state_dictionary = {
    0: "locked",
    1: "unlocked"
    }

# function for state 0
def state0(activity):
    # you can put any code here that should run in state 0
    # athough this example doesn’t have any
    #
    # At the end, we decide what the next state should be
    if activity == "push":
        print("turnstile was pushed, turnstile remaining locked")
        return 0
    elif activity == "coin":
        print("coin was entered, turnstile becoming unlocked")
        return 1
    else:
        return None

# function for state 1
def state1(activity):
    if activity == "push":
        print("turstile was pushed, turnstile becoming locked")
        return 0
    elif activity == "coin":
        print("coin was entered, turnstile remaining unlocked")
        return 1
    else:
        return None
    
# initalization
state = 0  # initial state
print("Type coin or push to run state machine")
print("Anything else will end the program")
# main loop
while state is not None:  # Run until state is None
    print("Current state is "+state_dictionary[state])
    # Get current activity
    activity = input("input: ")
    if state == 0:
        state = state0(activity)
    elif state == 1:
        state = state1(activity)
    else:
        Exception("Invalid state has been entered")
print("Done with state machine")
