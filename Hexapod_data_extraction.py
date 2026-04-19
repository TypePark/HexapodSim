import pybullet as p
import pybullet_data
import time
import math
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
p.setGravity(0,0, -9.81)
hexapod = p.loadURDF("", [0, 0, 0.2], globalScaling=0.01, useFixedBase=False)

class HexapodParser:
    def __init__(self):
        self.hexapod = hexapod
        self.leg_data_dict = {}
        self._parser()


    def _parser(self):
        joint_data_dict = {}

        for num_joints in range(p.getNumJoints(self.hexapod)):
            joint_info = p.getJointInfo(self.hexapod, num_joints)
            joint_name = joint_info[1].decode("utf-8")
            joint_upper_limit = joint_info[8]
            joint_lower_limit = joint_info[9]

            split = joint_name.split("_")
            leg_id = split[-2] + "_" + split[-1]

            if leg_id not in joint_data_dict:
                joint_data_dict[leg_id] = {}

            data = {
                "index": num_joints,
                "joint_upper_limit": joint_upper_limit,
                "joint_lower_limit": joint_lower_limit,
                "joint_range": joint_upper_limit - joint_lower_limit,
                "joint_center": (joint_upper_limit + joint_lower_limit) / 2
            }

            if "base_to" in joint_name:
                joint_data_dict[leg_id]["coxa"] = data
            elif "to_foot" in joint_name:
                joint_data_dict[leg_id]["foot"] = data
        self.leg_data_dict = joint_data_dict

    def leg_move_order(self):
        leg_keys = self.leg_data_dict.keys()
        left_legs = []
        right_legs = []
        move_order = []

        for leg_ids in leg_keys:
            if "l_" in leg_ids:
                left_legs.append(leg_ids)
            elif "r_" in leg_ids:
                right_legs.append(leg_ids)

        leg_num_of_each_side = len(left_legs) if len(left_legs) == len(right_legs) else print("error lol")
        for leg_num in range(leg_num_of_each_side):
            move_order.append(left_legs[leg_num])
            move_order.append(right_legs[leg_num])
        return move_order

    def foot_data_extract(self, leg_id):

        foot_index =self.leg_data_dict[leg_id]["foot"]["index"]
        contact_points = p.getContactPoints(self.hexapod, linkIndexA = foot_index)
        if contact_points:
            contact_distance = contact_points[0][8]
            force = contact_points[0][9]
            return contact_distance, force
        return 0.0, 0.0

    def record_foot_data(self, move_order, current_time):

        foot_data = []

        for leg_id in move_order:
                contact_distance, force = self.foot_data_extract(leg_id)
                foot_data.append({
                    "time": current_time,
                    "contact_distance": contact_distance,
                    "force": force
                })

                status = "CONTACT" if force > 0 else "SWING"
                print(f"{leg_id} | Distance: {(100000 * contact_distance):.2f} | Force: {force:.4f} | Time Elapsed: {current_time:.2f} | {status}")

        return foot_data

class HexapodSlideGait:
    def __init__(self, parser):
        self.hexapod = hexapod
        self.plane = plane
        self.leg_data_dict = parser.leg_data_dict
        self.move_order = parser.leg_move_order()

    def gait(self, current_time):
        leg_keys = self.leg_data_dict.keys()
        total_leg_num = len(leg_keys)
        gait_speed = 1.0
        total_cycle = 2 * math.pi
        step_window = total_cycle / total_leg_num



        for index, leg_id in enumerate(self.move_order):
            pairs = index // 6
            cycle = (current_time * gait_speed + pairs * step_window) % total_cycle
            polarity = -1.0 if "l_" in leg_id else 1.0
            position_leg = self.leg_data_dict[leg_id]["coxa"]["joint_center"] + polarity * math.sin(cycle) * 0.3
            position_foot = self.leg_data_dict[leg_id]["foot"]["joint_upper_limit"] - math.cos(cycle)

            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["coxa"]["index"], p.POSITION_CONTROL, position_leg)
            p.setJointMotorControl2(self.hexapod, self.leg_data_dict[leg_id]["foot"]["index"], p.POSITION_CONTROL, position_foot)


HParser = HexapodParser()
HParser.leg_move_order()
start_time = time.time()
HexapodGait = HexapodSlideGait(HParser)


while 1:
    p.stepSimulation()
    updated_time = time.time() - start_time
    HexapodGait.gait(updated_time)
    HParser.record_foot_data(HexapodGait.move_order, updated_time)

    time.sleep(1./240.)