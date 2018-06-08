# Pybullet-ANYmal Test

## Issues

1. ANYmal torque control fails when the number of the robots > 16

    - source code: [here](https://github.com/eastskykang/pybullet-benchmark/blob/master/ANYmalPDcontrol.py)
    - control mode: direct torque control
    - [Demo video](https://www.youtube.com/watch?v=S7MpEqYPYLg) is here. 

2. ~~Elastic collision failure~~
    - elastic collision works when the objects are loaded from URDF. 
    - if multibody is created by [createMultiBody()](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.jgm6tud6blu3), simulation is much slower and elastic collision fails.  
    - [Demo video](https://www.youtube.com/watch?v=Q2jNi1bzpe4)
    
3. Request for multibody energy getter (kinetic and potential) API.