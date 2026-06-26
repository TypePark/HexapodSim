import math
import pybullet as p

class Gaits:
    def __init__(self, hexapod, parser, gait_type):
        self.hexapod = hexapod
        self.leg_data_dict = parser.leg_data_dict
        self.leg_move_order = parser.gait_selector(gait_type)
        self.gait_speed = 1
        self.total_cycle = math.pi * 2
        self.num_legs = len(self.leg_move_order)

    def hexapod_tripod_gait(self, current_time, mass):
        swing_time = self.total_cycle / 2
        stance_time = self.total_cycle - swing_time

        for index, leg_id in enumerate(self.leg_move_order):
            legs_used_first_cycle = math.pi if index >= 3 else 0.0 # Divides cycle to 2, and decides how many legs should be used in first cycle rest goes to the second cycle
            each_cycle = (current_time * self.gait_speed + legs_used_first_cycle) % self.total_cycle
            polarity = -1.0 if "l_" in leg_id else 1.0
            force = mass * 9.81
            if each_cycle < swing_time:
                leg_target = self.leg_data_dict[leg_id]["coxa"]["center"] - ((each_cycle / swing_time) * 2 - 1) * polarity / 5
                foot_target = self.leg_data_dict[leg_id]["foot"]["upper_limit"] + (math.cos(each_cycle / swing_time * math.pi) + 1) / 2
            else:
                leg_target = self.leg_data_dict[leg_id]["coxa"]["center"] - (1 - ((each_cycle - swing_time) / stance_time) * 2) * polarity / 5
                foot_target = self.leg_data_dict[leg_id]["foot"]["upper_limit"]

            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["coxa"]["index"], p.POSITION_CONTROL, leg_target)
            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["foot"]["index"], p.POSITION_CONTROL, foot_target, force= force)

    def hexapod_ripple_gait(self, current_time, mass):
        num_leg_pairs = self.num_legs // 2
        swing_time = self.total_cycle / num_leg_pairs
        stance_time = self.total_cycle - swing_time
        force = mass * 9.81
        for index, leg_id in enumerate(self.leg_move_order):
            paired_index = index // 2
            each_cycle = (current_time * self.gait_speed + (swing_time * paired_index)) % self.total_cycle
            polarity = -1.0 if "l_" in leg_id else 1.0
            if each_cycle < swing_time:
                leg_target = self.leg_data_dict[leg_id]["coxa"]["center"] - ((each_cycle / swing_time) * 2 - 1) * polarity / 5
                foot_target = self.leg_data_dict[leg_id]["foot"]["upper_limit"] + (math.cos(each_cycle / swing_time * math.pi) + 1) / 2
            else:
                leg_target = self.leg_data_dict[leg_id]["coxa"]["center"] - (1 - ((each_cycle - swing_time) / stance_time) * 2) * polarity / 5
                foot_target = self.leg_data_dict[leg_id]["foot"]["upper_limit"]

            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["coxa"]["index"], p.POSITION_CONTROL, leg_target)
            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["foot"]["index"], p.POSITION_CONTROL, foot_target, force= force)

    def hexapod_wave_gait(self, current_time, mass):
        swing_time = self.total_cycle / self.num_legs
        stance_time = self.total_cycle - swing_time
        force = mass * 9.81
        for index, leg_id in enumerate(self.leg_move_order):
            each_cycle = (current_time * self.gait_speed + (swing_time * index)) % self.total_cycle
            polarity = -1.0 if "l_" in leg_id else 1.0
            if each_cycle < swing_time:
                leg_target = self.leg_data_dict[leg_id]["coxa"]["center"] - ((each_cycle / swing_time) * 2 - 1) * polarity / 5
                foot_target = self.leg_data_dict[leg_id]["foot"]["upper_limit"] + (math.cos(each_cycle / swing_time * math.pi) + 1) / 2
            else:
                leg_target = self.leg_data_dict[leg_id]["coxa"]["center"] - (1 - ((each_cycle - swing_time) / stance_time) * 2) * polarity / 5
                foot_target = self.leg_data_dict[leg_id]["foot"]["upper_limit"]

            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["coxa"]["index"], p.POSITION_CONTROL, leg_target)
            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["foot"]["index"], p.POSITION_CONTROL, foot_target, force= force)

