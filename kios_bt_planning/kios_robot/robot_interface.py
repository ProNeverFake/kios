'''
a robot interface for robot behavior control.
some functionaltiy is tailored for mios.
'''

from kios_utils.task import *

class RobotInterface:

    def __init__(self, rci:str = None):
        
        if rci is None:
            self.rci = "mios"
        else:
            self.rci = rci

    
    
    

    

