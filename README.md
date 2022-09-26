## The Mechcat Mujoco viewer 

This library is a WebGL renderer for mujoco that is baed on mechcat-python
and the mjcf_viewer packages on git. 

To install you must have the following dependencies 
```bash
pip install dm_control
pip install meshcat
```

once these packages are successfully installed, from this package directory install it via pip
```bash
pip install -e . 
```

This will install the package as an importable python library

## How to use 

You must first have an instantiated mujoco MjModel and MjData class 

```python
import mujoco 
from meshcat_mujoco import MeshCatVisualizer

xml_path = './a1/xml/a1/a1.xml'
model   = mujoco.MjModel.from_xml_path(xml_path)
data    = mujoco.MjData(model)


viewer = MeshCatVisualizer(xml_path, model, data)

i = 0 # <-- counter 
for _ in range(10000): 

    # use mujoco as normal with stepping function 
    mujoco.mj_step(model, data)
    # make sure you call the render function 
    viewer.render()

viewer.close()
# close
```
