import numpy as np

# Bobby Holden - April 2018

class Goal: 
    def __init__(self, a_id, a_type, pos, assigned_agent_id):   
        self.id = a_id
        self.type = a_type
        self.pos = pos      # pos is an (x,y) tuple, by default
        self.assigned_agent_id = assigned_agent_id
        self.drawn = False

    def __str__(self):
        return "Goal {} @ [{}, {}]".format(self.id,self.pos[0],self.pos[1])

    @classmethod
    def min_goal_init(cls, a_id, pos):   # minimal
        return cls(a_id, 'T', pos, -1)

    @classmethod
    def min_goals_initFromList(cls, list): #expect list of id's and positions
        goals_list = []
        for rawGoal in list:
            goals_list.append(cls.min_goal_init(rawGoal[0],rawGoal[1]))
        return goals_list

    def getPos(self):           
        return self.pos

    def setPos(self, pos):
        self.pos = pos

    #getters/setters for type & assigned agent TBD