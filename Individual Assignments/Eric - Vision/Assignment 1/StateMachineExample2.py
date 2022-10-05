# The following code demonstrates a very simple idiom for a state machine
# Each of the state functions below performs some action and then implements
# logic to choose next state. Each state function returns the next state.

def state0(activity):
    if activity == "push":
        print("turnstile was pushed, turnstile remaining locked")
        return state0
    elif activity == "coin":
        print("coin was entered, turnstile becoming unlocked")
        return state1
    else:
        return None
    
def state1(activity):
    if activity == "push":
        print("turstile was pushed, turnstile becoming locked")
        return state0
    elif activity == "coin":
        print("coin was entered, turnstile remaining unlocked")
        return state1
    else:
        return None
    
# create a dictionary to describe the states
state_dictionary = {
    state0 : "locked",
    state1 : "unlocked"
    }

# initalization
state = state0 # initial state as pointer to state0 function
print("Type coin or push to run state machine")
print("Anything else will end the program")
while state is not None: # Run until state is None
    print("Current state is "+state_dictionary[state])
    # Get current activity
    activity = input("input: ")
    new_state = state(activity) # launch state machine
    state = new_state # update the next state
print("Done with state machine")