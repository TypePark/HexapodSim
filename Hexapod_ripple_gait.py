import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.81)
hexapod = p.loadURDF("", [0, 0, 0.2], globalScaling=0.01, useFixedBase=False)

class HexapodParser:
    def __init__(self):
        self.hexapod = hexapod
        self.leg_indexes = {}
        self._parser()


    def _parser(self):
        joint_data = {}

        for num_joints in range(p.getNumJoints(self.hexapod)):
            joint_info = p.getJointInfo(self.hexapod, num_joints)
            joint_name = joint_info[1].decode("utf-8")
            joint_lower_limit = joint_info[8]
            joint_upper_limit = joint_info[9]

            if "leg_l_" in joint_name or "leg_r_" in joint_name:
                parts = joint_name.split("_")
                leg_id = parts[-2] + "_" + parts[-1]


                if leg_id not in joint_data:
                    joint_data[leg_id] = {}


                data = {
                        "index": num_joints,
                        "lower": joint_lower_limit,
                        "upper": joint_upper_limit,
                        "range": joint_upper_limit - joint_lower_limit,
                        "center": (joint_lower_limit + joint_upper_limit) / 2
                    }

                if "base_to" in joint_name:
                    joint_data[leg_id]["coxa"] = data

                elif "to_foot" in joint_name:
                    joint_data[leg_id]["foot"] = data

        self.leg_indexes = joint_data

    def leg_ripple_orderer(self):
        leg_keys = self.leg_indexes.keys()
        ripple_order = []
        left_legs = []
        right_legs = []
        for leg_id in leg_keys:
            if "l_" in leg_id:
                left_legs.append(leg_id)
            elif "r_" in leg_id:
                right_legs.append(leg_id)

        num_pairs = len(left_legs)
        for pair in range(num_pairs):
            left_legs_index = (pair * 2) % num_pairs
            ripple_order.append(left_legs[left_legs_index])
            right_legs_index = (pair * 2 + 1 ) % num_pairs
            ripple_order.append(right_legs[right_legs_index])
        return ripple_order

class HexapodRippleGait:
    def __init__(self, parser):
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


HParser = HexapodParser()
leg_name_index = list(HParser.leg_indexes.keys())
gait =HexapodRippleGait(HParser)
start_time = time.time()

while True:
    current_time = time.time() - start_time
    gait.hexapod_ripple_gait(current_time)

    p.stepSimulation()
    time.sleep(1. / 240.)