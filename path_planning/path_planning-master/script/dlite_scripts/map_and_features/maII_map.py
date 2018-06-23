#!/usr/bin/env python3
import numpy as np
import sys #so i don't have to switch everything to py3 print to not use \n or space at end
import re
import copy
from agent import Agent
from goal import Goal

# Bobby Holden - April 2018

inf = float("inf")

class MAII_map:
    # ----- Internal Types ----- #
    # position as type so it can be expanded to 3D w/ minimal impact below 
    position_T = np.dtype({'names': ['x','y'], 
        'formats': [int, int]}) 

    #cell, with ID-pointers to other complex objects 
    cell_T = np.dtype({'names': ['obs','occ-id','goal-id'], 
        'formats': [bool, int, int]})
    
    def __init__(self, xDim, yDim, numAgents, maxGoals):
        # ----- Fields ----- #
        self.xDim = xDim
        self.yDim = yDim
        self._map_ = np.zeros([xDim,yDim], dtype=MAII_map.cell_T)
        self.agents = [] 
        self.goals = [] 

        # Populate w/ defaults
        self._map_.fill((False, -1, -1))                         #default empty & unoccupied

    #assumes a well-formed map
        # rectangular
        # Obstruction: X
        # Agents: A-W  (X+ disallowed for use as obstructions)
        # Goals: a-z
        # Open space without issues: ' '
    @classmethod
    def ingestMap(cls, path_to_mapfile): # this should take in whatever format we expect to receive and assemble the map accordingly
        with open(path_to_mapfile, 'r') as mapfile:
            lump = mapfile.read()
            lines = lump.splitlines()     #To avoid \n per line

            yDim = len(lines)
            xDim = len(lines[0]) 
            reg_Agents = '[A-W]'
            reg_Goals = '[a-z]'

            numAgents = len(re.findall(reg_Agents,lump))
            numGoals = len(re.findall(reg_Goals,lump))

            newMap = cls(xDim,yDim,numAgents,numGoals)
            agent_list = newMap.agents #[]
            goal_list = newMap.goals

            for y, line in enumerate(lines): # For each y row
                for x, val in enumerate(line):  # For each x pos in y row
                    if (bool(re.match(reg_Agents,val))):
                        agent_index = (ord(val)-ord('A'))
                        newMap.placeAgent(Agent.min_agent_init(agent_index,(x,y)),(x,y)) #create an agent & pass it in (constructor takes id & position)
                        # agent_list.append()##(agent_index,(x,y)))  #uses tuples instead of Agent so we can continue to ignore anything about agents, and just throw this up to the next level
                    elif (bool(re.match(reg_Goals,val))):
                        goal_index = (ord(val)-ord('a'))
                        newMap.placeGoal(goal_index,(x,y))
                    elif (val=='X'):
                        newMap.setObs((x,y),True)
        
        return newMap, agent_list, goal_list

    @classmethod
    def init_from_gMap(cls, g_map, xDim, yDim): # this should take in whatever format we expect to receive and assemble the map accordingly
        newMap = cls(xDim,yDim,1,1) # Only considering this agent & corresponding
        newMap.update_gMap(g_map, xDim, yDim)

        return newMap

    def update_gMap(self, g_map, xDim, yDim): # this should take in whatever format we expect to receive and assemble the map accordingly
        if((self.xDim != xDim) or (self.yDim != yDim)):
            print("WARNING: Assumed map dimension stays the same through all updates.")

        for j in range(0,yDim):
            for i in range(0,xDim):
                if(g_map[j*xDim + i] == 100):
                    self.setObs((i,j),True)
                else:
                    self.setObs((i,j),False)

        #Safegaurding for my stuff because the boarder is bad
        for i in range(0,xDim):
            self.setObs((i,0),True)
            self.setObs((i,(yDim-1)),True)
        for j in range(0,yDim):
            self.setObs((0,j),True)
            self.setObs(((xDim-1),j),True)

    @classmethod
    def distance(cls,a,b): #euclidian, for now
        return np.sqrt((np.square(a[0]-b[0]))+(np.square(a[1]-b[1])))

    # Should include dynamics with this (esp for ground robot that has limitation in DOF)
    # move this over to map class
    # cost pos->s1
    def c(self, pos1, pos2):
        # How is this diff from h? Actual vs estimated? 
        # This limited by what it can actually reach? (e.g., range & obstructions) #prob not obs, since that would be taken care of by rhs
        # Maybe this is dictated by dyn, but that's just distance

        # Removed occupation constraint, as negotiation of speed should handle
        if (self.checkObs(pos2.pos)): 
            return inf                                        
        else:
            return self.distance(pos1.pos, pos2.pos) 


    def dimensions(self):
        return self.xDim, self.yDim

    # Makes a copy (true, not reference) and removes non-boundary upstructions
    def copyHideObstructions(self):
        newMap = copy.deepcopy(self)
        for x in range(1,newMap.xDim-1):
            for y in range(1,newMap.yDim-1):
                if (newMap.checkObs((x,y)) and (newMap.get_occupiedByAt((x,y)) == -1)): #if obstructed by something other than a known agent
                    newMap.setObs((x,y),False)
        return newMap

    def copy(self):
        return copy.deepcopy(self)

    # Uses write to avoid print adding a final character, so format is as intended
    def display_terminal(self):
        # Take all paths from all agents and lay out on  map for display
        paths = np.zeros(self.dimensions(),dtype='U')
        paths.fill(' ') # now the basis for our blank map
        for aID,agent in enumerate(self.agents):
            for p in agent.getPath():
                paths[p[0]][p[1]] = '.';#chr(ord('a')+aID)

        for x in range(0,self.xDim):
            for y in range(0,self.yDim): 
                if self.get_occupiedByAt((x,y)) != -1:
                    sys.stdout.write(chr(ord('A')+self.get_occupiedByAt((x,y))))
                elif self.get_goalAt((x,y)) != -1:
                    sys.stdout.write(chr(ord('a')+self.get_goalAt((x,y))))
                elif self.checkObs((x,y)):
                    sys.stdout.write('X')
                else:
                    sys.stdout.write(paths[x][y])
            sys.stdout.write('\n')   #next row

    # Displays side-by-side maps for comparison
    @classmethod
    def display_multiple_maps_terminal(cls, list_of_maps):
        # Take all paths from all agents and lay out on  map for display
        paths = np.zeros(list_of_maps[0].dimensions(),dtype='U')
        paths.fill(' ') # now the basis for our blank map
        for aID,agent in enumerate(list_of_maps[0].agents):
            for p in agent.getPath():
                paths[p[0]][p[1]] = '.';#chr(ord('a')+aID)


        for y in range(0,list_of_maps[0].yDim):         #row by row
            for m in range(0,len(list_of_maps)):             #for each map
                sys.stdout.write('\t')
                for x in range(0,list_of_maps[m].xDim):                    #step through columns
                    if list_of_maps[m].get_occupiedByAt((x,y)) != -1:
                        sys.stdout.write(chr(ord('A')+list_of_maps[m].get_occupiedByAt((x,y))))
                    elif list_of_maps[m].get_goalAt((x,y)) != -1:
                        sys.stdout.write(chr(ord('a')+list_of_maps[m].get_goalAt((x,y))))
                    elif list_of_maps[m].checkObs((x,y)):
                        sys.stdout.write('X')
                    else:
                        sys.stdout.write(paths[x][y])
                if(m==(len(list_of_maps)-1)):
                    sys.stdout.write('\n')   #next row

    def simRevealVisibleObstructions(self, truth_map, agentPos, agentRange): #For simulation...else we'd get this from SLAM
        intRange = int(agentRange) #can round down for what range to iterate over

        xRange = range( max(agentPos[0]-intRange-1,0), min(agentPos[0]+intRange+1,self.xDim-1) ) #center around agentPosition, and limit to Map bounds
        #For y, visibly up is numerically down (origin is top-left) ; split up so can break on dist easier
        yRangeDown = range(agentPos[1] , min(agentPos[1]+intRange+1,self.yDim-1) ) #center around agentPosition, and limit to Map bounds
        yRangeUp = range( (agentPos[1]-1) , max(agentPos[1]-intRange-1,0),-1 ) #center around agentPosition, and limit to Map bounds

        for x in xRange:
            for y in yRangeDown:
                if(MAII_map.distance(agentPos,(x,y)) > agentRange):
                    break
                else:   #TODO: break if there's an obstruction already in the line of sight (think off-angle) - This is fine for r=1, though
                    self.setObs((x,y),truth_map.checkObs((x,y)))

            for y in yRangeUp:
                if(MAII_map.distance(agentPos,(x,y)) > agentRange):
                    break
                else:   #TODO: break if there's an obstruction already in the line of sight (think off-angle) - This is fine for r=1, though
                    self.setObs((x,y),truth_map.checkObs((x,y)))

    def listMapFeaturesByCell(self):
        for y in range(0,self.yDim): 
            for x in range(0,self.xDim):
                if self.checkObs((x,y)):
                    print("Obstr | "),
                else:
                    print("Clear | "),

                if self.get_occupiedByAt((x,y)) != -1:
                    print("{} {} occupied: ".format(x,y)), 
                    print(chr(ord('A')+self.get_occupiedByAt((x,y)))),
                else:
                    print("{} {} not occupied".format(x,y)),

                if self.get_goalAt((x,y)) != -1:
                    print("\tGoal: "),
                    print(chr(ord('a')+self.get_goalAt((x,y)))),
                
                print("") #just for newline newline

    # Indicates to place obstruction on map, whether map feature, or other agent
    def setObs(self, pos, t_o_f): 
        self._map_[pos[0]][pos[1]]['obs'] = t_o_f; 

    def checkObs(self, pos):
        return self._map_[pos[0]][pos[1]]['obs']

    def getObstacles(self):
        obs = []
        for x in range(0,self.xDim):
            for y in range(0,self.yDim):
                if(self.checkObs((x,y))):
                    obs.append((x,y))
        return obs

    def checkOcc(self, pos):
        return self._map_[pos[0]][pos[1]]['occ-id']

    def get_occupiedByAt(self, pos):   #Using for display, doubt this will be useful for alg
        return self._map_[pos[0]][pos[1]]['occ-id'];

    def get_goalAt(self, pos):   #Using for display, doubt this will be useful for alg
        return self._map_[pos[0]][pos[1]]['goal-id'];

    # Particularly useful on init
    def placeAgent(self, agent, pos):
        agentExists = False
        for a in self.agents:
            if a.id == agent.id:
                self.moveAgent(agent, pos)
                agentExists = True;
                return True
        if not agentExists:
            if self.withinBounds(pos):
                # self.setObs(pos, True)
                self._map_[pos[0]][pos[1]]['occ-id'] = agent.id
                self.agents.append(agent)
                return True
            else:
                return False

        #TODO/Option: Set surrounding cells as occupied up to certain distance...eh, rather leave that to a dist_func 

    # Display path as list of (x,y)
    # def setPath(self, path):
    #     for p in path:

    def withinBounds(self, pos):
        if(pos[0] > 0 and pos[0] < self.xDim and pos[1] > 0 and pos[1] < self.yDim):
            return True
        else:
            return False

    # Useful throughout operation
    def moveAgent(self, agent, pos_to):
        #occupy new pos
        if(self.withinBounds(pos_to)):
            oldPos = agent.getPos()
            # self.setObs(pos_to, True)
            self._map_[pos_to[0]][pos_to[1]]['occ-id'] = agent.id

            if not (oldPos == pos_to):            # else: duplicate action (occurs when mirroring moves on both maps, for instance)
                agent.setPos(pos_to)
                #clear old pos - do this second to maintain as mutex (in case of multi threading or simultaneous map-sharing)
                # self.setPosUnobstructed(oldPos) #should not be necessary
                self._map_[oldPos[0]][oldPos[1]]['occ-id'] = -1

            return True
        else: #ensure the robot didn't actually move
            return False

    # Any time a target is generated, add it here
    def placeGoal(self, goal_id, pos):
        self._map_[pos[0]][pos[1]]['goal-id'] = goal_id
        self.goals.append(Goal.min_goal_init(goal_id,pos))    #uses tuples instead of Goal so we can continue to ignore anything about agents, and just throw this up to the next level

    # Goal achieved? no longer care?
    def clearGoal(self, goal_id):
        self._map_[self.goal_positions[goal_id][0]][self.goal_positions[goal_id][1]]['goal-id'] = -1      #clear on map
        self.goal_positions[goal_id] = (-1,-1)   #invalidate goal existence

    # set unobstructed implies clear to enter, but could still have target or something unobstructive
    def setPosUnobstructed(self, pos):
        # self._map_[pos[0]][pos[1]]['obs'] = False;
        self.setObs(pos, False)

    def receivePath(self, agentID, path):
        for a in self.agents:
            if(a.id == agentID):
                a.ingestPath(path)
                return True
        return False

    # def receiveDeconflictedPaths(self, cost, paths);







