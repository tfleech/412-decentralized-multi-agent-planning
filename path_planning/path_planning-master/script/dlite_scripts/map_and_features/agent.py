import numpy as np

# Bobby Holden - April 2018

class Agent:
    # Only carries info that the 'local' copy would need about others
    # Things like range & dynamics should be captured in a bigger self-class
    def __init__(self, a_id, a_type, pos, theta, v, ip, plan, synced):   
        self.id = a_id
        self.type = a_type
        self.state = (pos,theta,v)  # pos is an (x,y) tuple, by default
        #TODO: might want to distinguish discretized map-position from estimated state (which should be continuous)
        self.ip = ip
        self.plan = plan
        self.planStep = 0
        self.synced = synced
        self.movementHistory = []
        self.movementHistory.append(pos)
        self.drawn = False

    def __str__(self):
        return "Agent {} @ [{}, {}]".format(self.id,self.state[0][0],self.state[0][1])

    def __eq__(self,other):
        return (self.id == other.id)

    def __ne__(self,other): # Not necessary for python 3
        return not(self==other)

    @classmethod
    def min_agent_init(cls, a_id, pos):   # minimal
        return cls(a_id, 'FA', pos, 0,0, '', [], True)

    @classmethod
    def min_agents_initFromList(cls, list): #expect list of id's and positions
        agents_list = []
        for rawAgent in list:
            agents_list.append(cls.min_agent_init(rawAgent[0],rawAgent[1]))
        return agents_list

    def getPos(self):           
        return self.state[0]

    def setPos(self, pos):
        self.state = (pos,self.state[1],self.state[2])
        self.movementHistory.append(pos)

    def getOrientation(self):
        return self.state[1]

    def setOrientation(self, theta):
        self.state[1] = theta

    def getVelocity(self):
        return self.state[2]

    def setVelocity(self, v):
        self.state[2] = v

    # implies no velocity info (true for now)
    def ingestPath(self, path): 
        plan_constVel = []
        for p in path:
            plan_constVel.append( (p,0,1) ) #pos,theta,v
        self.plan = plan_constVel
        self.planStep = 0;

    def getPath(self):
        path = []
        for p in self.plan:
            path.append(p[0])
        return path

    def getPath_NoDelays(self):
        path = []
        prevPos = (-1,-1) #presumably an invalid point
        for p in self.plan:
            if(p[0] != prevPos):
                path.append(p[0])
                prevPos = p[0]
        return path

    def peekNextStep(self): #which in itself is just a goal
        if(len(self.plan) > 0):
            if((len(self.plan)-1) > self.planStep):
                return self.plan[self.planStep+1][0]
            else: #Done with path, no problemo
                return False
        else:
            print("No Plan Available.")
            return False

    def popNextStep(self): #which in itself is just a goal
        if(len(self.plan) > 0):
            if((len(self.plan)-1) > self.planStep):
                self.plan.pop(0)
                return self.plan[0][0]
            else: #Done with path, no problemo
                return []
        else:
            print("No Plan Available.")
            return False



    # propagate self by time passed and update self variables and map state
    # While setters & getters will be used to update according to input from slam
    # This will be be used to propogate for sim & estimation (could be overriden by slam estimation as well)
    #def propogateSelf(self, dt, map)  

