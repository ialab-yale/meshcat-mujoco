import mujoco 

class Sim(object):
    """
    Simple object that acts as a wrapper for the 
    Mujoco model and data classes. 
    """
    def __init__(self, fname) -> None:
        self.xml_path = fname
        self.model   = mujoco.MjModel.from_xml_path(fname)
        self.data    = mujoco.MjData(self.model)
