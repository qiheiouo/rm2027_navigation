"""Authored static SDF map only; dynamic truth is never admitted here."""
import numpy as np

def grid_for(scenario, width=160, height=120, resolution=.05, origin=(-1.,-3.)):
    grid=np.zeros((height,width),np.int8)
    grid[[0,-1],:]=100;grid[:,[0,-1]]=100
    if scenario=='course':
        for y in (.525,-.525):
            for iy in range(height):
                for ix in range(width):
                    x0,y0=origin[0]+ix*resolution,origin[1]+iy*resolution
                    if x0<4 and x0+resolution>2 and y0<y+.125 and y0+resolution>y-.125:grid[iy,ix]=100
    return grid
