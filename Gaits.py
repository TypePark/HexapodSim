import math
import pybullet as p

class TripodGait:
    def __init__(self, hexapod, parser, gait_type):
        self.hexapod = hexapod
        self.leg_data_dict = parser.leg_data_dict
        self.leg_move_order = parser.gait_selector(gait_type)


    def hexapod_tripod_gait(self, current_time):
        gait_speed = 2.0
        total_cycle = 2 * math.pi

        for index, leg_id in enumerate(self.leg_move_order):
            phase_offset = math.pi if index >= 3 else 0.0
            cycle = (current_time * gait_speed + phase_offset) % total_cycle
            polarity = -1.0 if "l_" in leg_id else 1.0
            if cycle < math.pi:
                target = self.leg_data_dict[leg_id]["coxa"]["center"] + ( math.sin(cycle) * polarity) / 5
                position_foot = self.leg_data_dict[leg_id]["foot"]["upper_limit"] - math.cos(cycle)

            else:
                target = self.leg_data_dict[leg_id]["coxa"]["center"] + ( math.sin(cycle) * polarity) / 5
                position_foot = self.leg_data_dict[leg_id]["foot"]["upper_limit"] - math.cos(cycle)

            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["coxa"]["index"], p.POSITION_CONTROL, target)
            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["foot"]["index"], p.POSITION_CONTROL, position_foot)

class HexapodRippleGait:
    def __init__(self, hexapod, parser):
        self.ripple_order = parser.leg_ripple_orderer()
        self.hexapod = hexapod
        self.leg_indexes = parser.leg_indexes


    def hexapod_ripple_gait(self, current_time):
        num_legs = len(self.ripple_order)
        gait_speed = 8

        for i, leg_id in enumerate(self.ripple_order):
            cycle =  2 * 3.1415
            step_window = cycle / num_legs
            leg_offset = i * step_window
            phase = (current_time * gait_speed + leg_offset) % cycle
            print(current_time, phase)
            current_leg = self.leg_indexes[leg_id]
            coxa = current_leg["coxa"]
            foot = current_leg["foot"]
            center = current_leg["coxa"]["center"]
            multiplier = -1.0 if "l_" in leg_id else 1.0

            if phase < step_window:
                smallerphase = phase/step_window
                if smallerphase < 0.25:
                    foot_pos = 0.1
                    coxa_pos = center - (0.1 * multiplier)

                elif smallerphase < 0.75:
                    foot_pos = 0.1
                    coxa_pos = center + (0.5 * multiplier)

                else:
                    foot_pos = 1
                    coxa_pos = center - (0.1 * multiplier)

            else:
                foot_pos = 1
                coxa_pos = center - (0.1 * multiplier)

            p.setJointMotorControl2(self.hexapod, coxa["index"], p.POSITION_CONTROL, targetPosition=coxa_pos)
            p.setJointMotorControl2(self.hexapod, foot["index"], p.POSITION_CONTROL, targetPosition=foot_pos)

class HexapodWaveGait:
    def __init__(self, hexapod, parser, gait_type):
        self.hexapod = hexapod
        self.move_order = parser.gait_selector(gait_type)
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
            position_leg = self.leg_data_dict[leg_id]["coxa"]["center"] + polarity * math.sin(cycle) * 0.3
            position_foot = self.leg_data_dict[leg_id]["foot"]["upper_limit"] - math.cos(cycle)

            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["coxa"]["index"], p.POSITION_CONTROL, position_leg)
            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["foot"]["index"], p.POSITION_CONTROL, position_foot)