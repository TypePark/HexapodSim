import time
import pybullet as p
import parser
import data_extractor
import gaits
import debug_camera
import terrain_generator
import pybullet_data

p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
planhe = p.loadURDF("plane.urdf")
hexapod = p.loadURDF("hexapod_urdf_2/hexapod.urdf", [0, 0, 0.725], globalScaling=0.01, useFixedBase=False)
gait_type = "ripple" # Select gait

class HexapodEnv:
    def __init__(self, hexapod_in, gait_type_in):
        self.parser = parser.HexapodParser(hexapod_in)
        self.data_extractor  = data_extractor.DataExtraction(hexapod_in, self.parser, gait_type_in)
        self.gait = gaits.Gaits(hexapod_in, self.parser, gait_type_in)
        self.debug_camera = debug_camera.DebugCamera(hexapod_in)

env = HexapodEnv(hexapod, gait_type)
gait = getattr(env.gait, f"hexapod_{gait_type}_gait")

start_time = time.time()

while 1:
#for frame in range(2000):
    p.stepSimulation()

    updated_time = time.time() - start_time
    foot_contact = env.data_extractor.foot_contact()
    foot_target = env.data_extractor.foot_com_target(foot_contact)
    print(foot_target)
    env.data_extractor.center_of_mass()

    gait(updated_time)

    time.sleep(1/240)