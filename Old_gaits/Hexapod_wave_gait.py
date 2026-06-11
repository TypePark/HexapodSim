import pybullet as p
import math

class HexapodWaveGait:
    def __init__(self, hexapod, parser):
        self.hexapod = hexapod
        self.move_order = parser.leg_move_order()
        self.leg_data_dict = parser.leg_data_dict

    def hexapod_wave_gait(self, current_time):
        leg_keys = self.leg_data_dict.keys()
        total_leg_num = len(leg_keys)
        gait_speed = 10.0
        total_cycle = 2 * math.pi
        step_window = total_cycle / total_leg_num


        for index, leg_id in enumerate(self.move_order):
            cycle =  (current_time * gait_speed + index * step_window) % total_cycle
            polarity = -1.0 if "l_" in leg_id else 1.0
            position_leg = self.leg_data_dict[leg_id]["coxa"]["joint_center"] + polarity * math.sin(cycle) * 0.3
            position_foot = self.leg_data_dict[leg_id]["foot"]["joint_upper_limit"] - math.cos(cycle)

            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["coxa"]["index"], p.POSITION_CONTROL, position_leg)
            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["foot"]["index"], p.POSITION_CONTROL, position_foot)