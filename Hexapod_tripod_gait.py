import math
import pybullet as p

class TripodGait:
    def __init__(self, hexapod, parser):
        self.hexapod = hexapod
        self.leg_data_dict = parser.leg_data_dict
        self.leg_move_order = parser.leg_move_order()


    def calculate_balance(self, foot_com_target, current_time):
        u = foot_com_target["contact_position_0"]
        v = foot_com_target["contact_position_1"]
        w = foot_com_target["contact_position_2"]
        threshold = 0.03
        min_weight = max(0.0, (min(u, v, w)))
        move_potential = max(0.0, min(1.0, min_weight / threshold))
        gait_speed = 2.0
        total_cycle = 2 * math.pi


        for index, leg_id in enumerate(self.leg_move_order):
            phase_offset = math.pi if index >= 3 else 0.0
            cycle = (current_time * gait_speed + phase_offset) % total_cycle
            amplitude = (self.leg_data_dict[leg_id]["coxa"]["range"] / 2) * move_potential
            polarity = -1.0 if "l_" in leg_id else 1.0
            if cycle < math.pi:
                target = self.leg_data_dict[leg_id]["coxa"]["center"] + (amplitude * math.sin(cycle) * polarity)
                position_foot = self.leg_data_dict[leg_id]["foot"]["upper_limit"] - math.cos(cycle)
                force = 20
            else:
                target = self.leg_data_dict[leg_id]["coxa"]["center"] + (amplitude * math.sin(cycle) * polarity)
                position_foot = self.leg_data_dict[leg_id]["foot"]["upper_limit"] - math.cos(cycle)
                force = 10

            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["coxa"]["index"], p.POSITION_CONTROL, target, force = force)
            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["foot"]["index"], p.POSITION_CONTROL, position_foot)
