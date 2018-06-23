import numpy as np


class Config(object):
    def __init__(self, summary=True):
        self.robot_radius = 1.0        # Robot radius
        self.vel_max      = 1.0        # Max velocities  (m/s)
        self.step         = 0.1        # Simulation step (s)
        self.goal_th      = 0.10       # Threshold for reaching a goal

        if summary:
            self._summary()

    def _summary(self):
        print('=========================================================')
        print('[ ENV-INFO ] Robot radius (m)         :', self.robot_radius) 
        print('[ ENV-INFO ] Maximum velocities (m/s) :', self.vel_max) 
        print('[ ENV-INFO ] Simulation step time  (s):', self.step) 
        print('=========================================================')
