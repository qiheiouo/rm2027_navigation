"""Authored static SDF map only; dynamic truth is never admitted here."""
import numpy as np

def grid_for(scenario):
    grid=np.zeros((120,160),np.int8)
    grid[[0,-1],:]=100;grid[:,[0,-1]]=100
    if scenario=='course':
        for y in (.525,-.525):
            for iy in range(120):
                for ix in range(160):
                    x0,y0=-1+ix*.05,-3+iy*.05
                    if x0<4 and x0+.05>2 and y0<y+.125 and y0+.05>y-.125:grid[iy,ix]=100
    return grid
