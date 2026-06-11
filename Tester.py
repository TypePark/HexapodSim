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

for frame in range(2000):
    p.stepSimulation()
    updated_time = time.time() - start_time

    foot_contact = env.data_extractor.foot_contact()
    foot_target = env.data_extractor.foot_com_target(foot_contact)
    print(foot_target)
    env.gait.hexapod_wave_gait(updated_time) # Change this for the selected gait. (TO-DO: Remove the need for changing this line)
    env.data_extractor.center_of_mass()

    time.sleep(1/240)