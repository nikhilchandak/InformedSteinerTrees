
class Node:
    def __init__(self, state, index, dim=2):
        self.state = state
        self.ndim = dim 
        self.index = index 
        self.vector = [-1 for i in range(dim)]
        self.setVector(state)
        
    def setVector(self, state):
        for i in range(self.ndim):
            self.vector[i] = state[i] 

    def setState(self, newState):
        self.state = newState 
        self.setVector(newState)

    def setStateVector(self, newStateVector):
        for i in range(self.ndim):
            self.state[i] = newStateVector[i]
        self.setVector(self.state)

    def getState(self):
        return self.state.get()

    def getVector(self):
        return self.vector 

    def __repr__(self):
        return str(self.vector)
