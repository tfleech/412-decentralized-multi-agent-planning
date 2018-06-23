try:
    import Queue as Q  # ver. < 3.0
except ImportError:
    import queue as Q
import numpy as np
import sys
sys.path.insert(0, 'map_and_features/') #Where I'm keeping stuff right now
from map_and_features.maII_map import MAII_map 
from PriorityQueue import PriorityQueue
from KeyedVertex import KeyedVertex
import copy

# This implementation is derived from Koenig & Likhachev (2002), see http://idm-lab.org/publications.html
# Bobby Holden - April 2018

inf = float("inf")

class DStar_planner(object):
    def __init__(self, start, goal, known_map, r):  #The non-pset version
    # def __init__(self, start, goal, known_map, r, _uvu, _cspu): #PSET verison
        self.s_start = KeyedVertex(start)
        self.s_last = self.s_start
        self.s_goal = KeyedVertex(goal)
        self.k_m = 0
        self.U = PriorityQueue()                        # Koenig-22
        self.map_known = known_map
        self.robRange = r
        self.encounteredVertices = []

        #further initialization
        self.s_goal.rhs = 0
        self.s_goal.key = self.s_goal.CalculateKey(self.s_start,self.map_known,self.k_m)
        self.U.put(self.s_goal)
        self.encounteredVertices.append(self.s_goal);         #This one is my addition, to help track initialized vertices
        self.encounteredVertices.append(self.s_start);

        #configurable pset functions
        # self.uvu = _uvu
        # self.cspu = _cspu

    def pathExists(self):
        if(self.s_start.g == inf):
            print("No path exists by virtue of the start node having infinite g cost.")
            return False
        else:
            return True

    # For each of the update functions, check to see if we're tracking one already
    def updateStart(self, pos):
        self.s_start = next((s for s in self.encounteredVertices if (s.pos == pos)), KeyedVertex(pos))

    # This goal refers to overall current target, not next waypoint
    def updateGoal(self, pos):
        self.s_goal = next((s for s in self.encounteredVertices if (s.pos == pos)), False)
        if(not self.s_goal): #<Burn it to the ground> This means the above got false back, and the whole thing needs to be reset for the next pathfinding goal
            self.encounteredVertices = [] #abandon all the collected values
            self.s_start = KeyedVertex(self.s_start.pos) #creating a new copy, forget about the old
            self.s_goal = KeyedVertex(pos) # as originally fed in
            self.s_last = self.s_start
            self.k_m = 0
            self.U = PriorityQueue()   
            # cont'd
            self.s_goal.rhs = 0
            self.s_goal.key = self.s_goal.CalculateKey(self.s_start,self.map_known,self.k_m)
            self.U.put(self.s_goal)
            self.encounteredVertices.append(self.s_goal);         #This one is my addition, to help track initialized vertices
            self.encounteredVertices.append(self.s_start);


    def checkAndUpdate(self, path, planStep):
        # Starts at line 28; prior lines taken care of via main & stepping
        # print("C&U -> Plan step: {}".format(self.s_start)) #Plan step should always be 0 with this config
        changedEdges = []
        edgeFrom = self.s_start #s
        for pos in path[1:]:                                                                    # Koenig-28
            edgeTo = next((s for s in self.encounteredVertices if (s.pos == pos)), False)       # get corresponding vertex
            if(self.map_known.c(edgeFrom,edgeTo) == inf):                                       # Koenig-29 obstruction appeared
                changedEdges.append(edgeFrom) #s
            edgeFrom = edgeTo

        if(len(changedEdges) > 0):
            # print("Replanning due to changed edges..."),
            # self.k_m = self.k_m + self.s_last.h(self.s_start,self.map_known)
            prvKm = self.k_m
            prvH = self.s_last.h(self.s_start,self.map_known)
            self.k_m = prvKm + prvH
            self.s_last = copy.deepcopy(self.s_start) #so it doesn't link w/ and change when s_start changes. could just record pos, but then the nifty h function would need to be reworked

            for s in changedEdges:
                # print("Changed edges:{}".format(s))
                self.UpdateVertex(s)

            path = self.computeShortestPath()
            # path = retrieveShortestPath(map_known, agent1.getPos()) # Koenig-26a   (overkill, only need first element, but we want to record the whole map)
            if(not path):
                print("Path not retrieved successfully. Final (known) state:")
                self.map_known.display_terminal()
#                 input("Exits on enter...")
                raise Exception("Path not found. Self destruct!")

            # input("Exits on enter...")
            return True, path
        else: 
            return False, path
        #     # print("No new obstructions.")
        


    def computeShortestPath(self):
        # return self.cspu(self.map_known, self) #relic of PSET
        while( (self.U.key_peek() < self.s_start.CalculateKey(self.s_start,self.map_known,self.k_m)) \
                or (self.s_start.rhs != self.s_start.g) ): # Koenig-10")
            
            k_old = self.U.key_peek()            # Koenig-11
            u = self.U.get()                     # Koenig-12 
            if(k_old < u.CalculateKey(self.s_start,self.map_known,self.k_m)):              # Koenig-13
                u.key = u.CalculateKey(self.s_start,self.map_known,self.k_m)         # Koenig-14 (start)
                self.U.put(u)                    # Koenig-14 (end)
            elif(u.g > u.rhs):                      # Koenig-15
                u.g = u.rhs                         # Koenig-16
                                                    # Koenig-17 (start)
                for s in self.potentialPredeccessorPositions(u):
                    self.UpdateVertex(s)         # Koenig-17 (end)
            else:                                   # Koenig-18
                u.g = inf                           # Koenig-19
                                                    # Koenig-20 (start)
                for s in self.potentialPredeccessorPositions(u):
                    self.UpdateVertex(s)
                self.UpdateVertex(u)             # Koenig-20 (end)
        return self.retrieveShortestPath()

    # get list of potential movements
    # r dictates range (per dynamics, etc)
    # TODO: Should we consider dynamics to determine where you can actually get - immediately left /right may be in "range" of a ground robot, but it can't strafe
    # inits and adds to encountered list if not previously on it
    # Note: Do not consider obsticles here, that's should be in calculating the cost to go
    def potentialSuccessorPositions(self, slf): #Up down left right? requires some dynamics knowledge in future
        intRange = int(self.robRange) #can round down for what range to iterate over
        xDim, yDim = self.map_known.dimensions()
        xRange = range( max(slf.pos[0]-intRange-1,0), min(slf.pos[0]+intRange+1,xDim-1) ) #center around s.position, and limit to Map bounds
        #For y, visibly up is numerically down (origin is top-left) ; split up so can break on dist easier
        yRangeDown = range(slf.pos[1] , min(slf.pos[1]+intRange+1,yDim-1) ) #center around s.position, and limit to Map bounds
        yRangeUp = range( (slf.pos[1]-1) , max(slf.pos[1]-intRange-1,0),-1 ) #center around s.position, and limit to Map bounds

        successors = []
        for x in xRange:
            for y in yRangeDown:
                if(slf.pos == (x,y)):
                    continue

                if(MAII_map.distance(slf.pos,(x,y)) > self.robRange):
                    break
                else:
                    s = next((s for s in self.encounteredVertices if (s.pos == (x,y))), False)
                    if not s:
                        s = KeyedVertex((x,y))
                        self.encounteredVertices.append(s)  
                    successors.append(s)  #get from list of existing; if doesn't exist, init

            for y in yRangeUp:
                if(MAII_map.distance(slf.pos,(x,y)) > self.robRange):
                    break
                else:
                    s = next((s for s in self.encounteredVertices if (s.pos == (x,y))), False)
                    if not s:
                        s = KeyedVertex((x,y))
                        self.encounteredVertices.append(s)
                    successors.append(s)
        return successors

    # For now, disregarding dynamics, successors=
    # See above commentary on self.potentialSuccessorPositions
    def potentialPredeccessorPositions(self, s):
        return self.potentialSuccessorPositions(s)


    def UpdateVertex(self, u):
        # self.uvu(self.map_known, self, u) # PSET relic
        if not (u == self.s_goal):                       # Koenig-7 (start)
            successors = self.potentialSuccessorPositions(u) 
            min_cost = inf

            for s in successors: #over successors
                # self.map_known.setObs((1,3),False)
                cg = s.g + self.map_known.c(u,s);
                if(cg < min_cost):
                    min_cost = cg

            u.rhs = min_cost                            # Koenig-7 (end)
        self.U.remove(u)                             # Koenig-8
        u.key = u.CalculateKey(self.s_start,self.map_known,self.k_m)                     # Koenig-9 (start)

        if not (u.g==u.rhs):
            self.U.put(u)

    # Go from start to fin by following vertexes which minimize c(s,s')+g(s')
    def retrieveShortestPath(self):#, known_map, cur):
        path = []
        curPos = next((s for s in self.encounteredVertices if (s.pos == self.s_start.pos)), False)
        path.append(curPos.pos)
        nextPos = curPos
        iteration = 0
        while not (curPos.pos == self.s_goal.pos):                       # Koenig-7 (start)
            succs = self.potentialSuccessorPositions(curPos) 
            min_cost = inf
            for s_suc in succs: #over successors
                cg = s_suc.g+self.map_known.c(curPos,s_suc);
                if(cg < min_cost):
                    min_cost = cg
                    nextPos = s_suc
            if(min_cost == inf):
                return False
            elif(iteration == 50):
                return False

            curPos = nextPos  
            path.append(curPos.pos)
            iteration += 1
        return path
