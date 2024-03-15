import math
import numpy as np
import os
import matplotlib.pyplot as plt
import matplotlib.font_manager as font_manager

def angle_of_line(x1, y1, x2, y2):
    return math.atan2(y2-y1, x2-x1)

def make_square(x, y, width):
    square = np.array([[x-int(width/2),i] for i in range(y-int(width/2),y+int(width/2))] +\
                      [[x+int(width/2),i] for i in range(y-int(width/2),y+int(width/2))] +\
                      [[i,y-int(width/2)] for i in range(x-int(width/2),x+int(width/2))] +\
                      [[i,y+int(width/2)] for i in range(x-int(width/2),x+int(width/2))]) 
    return square

