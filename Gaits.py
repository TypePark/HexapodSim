import math
import pybullet as p

class Gaits:
    def __init__(self, hexapod, parser, gait_type):
        self.hexapod = hexapod
        self.leg_data_dict = parser.leg_data_dict
        self.leg_move_order = parser.gait_selector(gait_type)
        self.gait_speed = 2.0
        self.total_cycle = math.pi * 2
        self.num_legs = len(self.leg_move_order)

    def hexapod_tripod_gait(self, current_time):
        for index, leg_id in enumerate(self.leg_move_order):
            legs_used_first_cycle = math.pi if index >= 3 else 0.0
            each_cycle = (current_time * self.gait_speed + legs_used_first_cycle) % self.total_cycle
            polarity = -1.0 if "l_" in leg_id else 1.0
            if each_cycle < math.pi:
                leg_target = self.leg_data_dict[leg_id]["coxa"]["center"] + (math.sin(each_cycle) * polarity) / 5
                foot_target = self.leg_data_dict[leg_id]["foot"]["upper_limit"] - math.cos(each_cycle)

            else:
                leg_target = self.leg_data_dict[leg_id]["coxa"]["center"] + (math.sin(each_cycle) * polarity) / 5
                foot_target = self.leg_data_dict[leg_id]["foot"]["upper_limit"] - math.cos(each_cycle)

            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["coxa"]["index"], p.POSITION_CONTROL, leg_target)
            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["foot"]["index"], p.POSITION_CONTROL, foot_target)

    def hexapod_ripple_gait(self, current_time):
        for index, leg_id in enumerate(self.leg_move_order):
            phase_time = (self.total_cycle / self.num_legs) * index
            each_cycle = (current_time * self.gait_speed + phase_time) % self.total_cycle
            polarity = -1.0 if "l_" in leg_id else 1.0
            if each_cycle < phase_time:
                leg_target = self.leg_data_dict[leg_id]["coxa"]["center"] + math.sin(each_cycle) * polarity / 5
                foot_target = self.leg_data_dict[leg_id]["foot"]["upper_limit"] - math.cos(each_cycle)
            else:
                leg_target = self.leg_data_dict[leg_id]["coxa"]["center"] + math.sin(each_cycle) * polarity / 5
                foot_target = self.leg_data_dict[leg_id]["foot"]["upper_limit"] - math.cos(each_cycle)

            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["coxa"]["index"], p.POSITION_CONTROL, targetPosition=leg_target)
            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["foot"]["index"], p.POSITION_CONTROL, targetPosition=foot_target)

    def hexapod_wave_gait(self, current_time):
        phase_time = self.total_cycle / self.num_legs
        stance_time = self.total_cycle - phase_time

        for index, leg_id in enumerate(self.leg_move_order):
            each_cycle = (current_time * self.gait_speed + (phase_time * index)) % self.total_cycle
            polarity = -1.0 if "l_" in leg_id else 1.0

            if each_cycle < phase_time:
                leg_target = self.leg_data_dict[leg_id]["coxa"]["center"] - ((each_cycle / phase_time) * 2 - 1) * polarity / 5
                foot_target = self.leg_data_dict[leg_id]["foot"]["upper_limit"] + math.cos(each_cycle)
            else:
                leg_target = self.leg_data_dict[leg_id]["coxa"]["center"] - (1 - ((each_cycle - phase_time) / stance_time) * 2) * polarity / 5
                foot_target = self.leg_data_dict[leg_id]["foot"]["upper_limit"]

            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["coxa"]["index"], p.POSITION_CONTROL, leg_target)
            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["foot"]["index"], p.POSITION_CONTROL, foot_target)


