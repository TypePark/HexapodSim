import time
import pybullet as p
import pybullet_data
import Parser
import Data_extractor
import Gaits


p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
hexapod = p.loadURDF("hexapod_urdf_2/hexapod.urdf", [0, 0, 0.29], globalScaling=0.01, useFixedBase=False)
gait_type = "wave" # Select gait

class HexapodEnv:
    def __init__(self, hexapod_in, gait_type_in):
        self.parser = Parser.HexapodParser(hexapod_in)
        self.data_extractor  = Data_extractor.DataExtraction(hexapod_in, self.parser, gait_type_in)
        self.gait = Gaits.Gaits(hexapod_in, self.parser, gait_type_in)

env = HexapodEnv(hexapod, gait_type)
start_time = time.time()

while 1:
    p.stepSimulation()
    updated_time = time.time() - start_time
    env.gait.hexapod_wave_gait(updated_time) # Can change this
    time.sleep(1/240)