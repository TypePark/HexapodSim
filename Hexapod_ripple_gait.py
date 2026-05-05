import pybullet as p

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

