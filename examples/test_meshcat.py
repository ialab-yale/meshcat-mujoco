import mujoco
from meshcat_mujoco import MeshCatVisualizer
import numpy as np

xml_path = './a1/xml/a1.xml'
model    = mujoco.MjModel.from_xml_path(xml_path)
data     = mujoco.MjData(model)

viewer = MeshCatVisualizer('./a1/xml/a1.xml', model, data)


# PD control parameters
kp = 80.
kd = 10.
# simulate and render
i = 0 # <-- counter 
for _ in range(10000): 
    with open("mocap.txt","r") as filestream: # < -- grabs the mocap data
        for line in filestream:
            # the following lines convert the text to target joint values
            currentline = line.split(",")
            frame = currentline[0]
            t = currentline[1]
            target_joints = np.array([float(_q) for _q in currentline[2:14]])
            
            # make sure that you can still see the environment 
            if True:#viewer.is_alive:
                # pull the joint position and velocities
                q = np.array(data.qpos[7:]) # <-- notice that the first 7 values are the base body position (3) and orientation in quaternains (4)
                v = np.array(data.qvel[6:]) # <-- here the joint velocities start after the base body rotations (only 3, not 4)
                # basic PD controller 
                tau = kp * (target_joints-q) + kd * (-v)
                # to apply a control input to mujoco, you need to set the inputs to the data structure's ctrl variable
                data.ctrl = tau
                # the data and model sets passed to the step function to step the simulation. New poses, vel, cont, etc, get recorded into data
                mujoco.mj_step(model, data)
                
                # every 30 steps render the environment 
                # if i % 30 == 0:
                viewer.render()
                i += 1
            else:
                break
viewer.close()
# close

