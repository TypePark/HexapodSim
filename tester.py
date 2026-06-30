import time
import pybullet as p
import parser
import data_extractor
import gaits
import debug_camera
import terrain_generator
import pybullet_data
import data_logger

p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

#p.setAdditionalSearchPath(pybullet_data.getDataPath())
#plane = p.loadURDF("plane.urdf")

terrain = terrain_generator.Terrain()
hexapod = p.loadURDF("hexapod_urdf_2/hexapod.urdf", [0, 0, terrain.hexapod_spawn], globalScaling=0.01, useFixedBase=False)


gait_type = "tripod" # Select gait
file_name = "test_1.csv"

class HexapodEnv:
    def __init__(self, hexapod_in, gait_type_in):
        self.parser = parser.HexapodParser(hexapod_in)
        self.data_extractor  = data_extractor.DataExtraction(hexapod_in, self.parser, gait_type_in)
        self.gait = gaits.Gaits(hexapod_in, self.parser, gait_type_in)
        self.debug_camera = debug_camera.DebugCamera(hexapod_in)
        self.gait_speed = self.gait.gait_speed
        self.data_logger = data_logger.DataLogger(file_name)

env = HexapodEnv(hexapod, gait_type)
gait = getattr(env.gait, f"hexapod_{gait_type}_gait")

start_time = time.time()

#while 1:
for frame in range(2000):
    p.stepSimulation()

    updated_time = time.time() - start_time
    foot_contact = env.data_extractor.foot_contact()
    foot_target = env.data_extractor.foot_com_target(foot_contact)
    com = env.data_extractor.center_of_mass()
    slip = env.data_extractor.slip_detection(foot_contact, env.gait_speed)
    fall = env.data_extractor.fall_detection(foot_contact)
    gait(updated_time, env.data_extractor.total_mass)
    env.data_logger.log(gait_type, -9.81, fall, slip, foot_target )

    time.sleep(1/240)

env.data_logger.close()