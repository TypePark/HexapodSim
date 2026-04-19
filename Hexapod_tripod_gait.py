import pybullet as p
import pybullet_data
import time
import math

connect = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
gravity = p.setGravity(0,0, -9.81)
hexapod = p.loadURDF("", [0, 0, 0.3],  globalScaling=0.01, useFixedBase=False)





class HexapodParser:
    def __init__(self):
        self.hexapod = hexapod
        self.leg_data_dict = {}
        self._parser()

    def _parser(self):
        joint_data_dict = {}
        for joint_num in range(p.getNumJoints(self.hexapod)):
            joint_info = p.getJointInfo(self.hexapod, joint_num)
            joint_name = joint_info[1].decode("utf-8")
            joint_upper_limit = joint_info[8]
            joint_lower_limit = joint_info[9]

            split = joint_name.split("_")
            leg_id = split[-2] + "_" + split[-1]

            if leg_id not in joint_data_dict: joint_data_dict[leg_id] = {}

            data = {
                "index": joint_num,
                "upper_limit": joint_upper_limit,
                "lower_limit": joint_lower_limit,
                "range": joint_upper_limit - joint_lower_limit,
                "center": (joint_upper_limit + joint_lower_limit) / 2
            }

            if "base_to" in joint_name:
                joint_data_dict[leg_id]["coxa"] = data
            elif "to_foot" in joint_name:
                joint_data_dict[leg_id]["foot"] = data
        self.leg_data_dict = joint_data_dict

    def leg_move_order(self):
        leg_keys = self.leg_data_dict.keys()
        left_side = []
        right_side = []
        move_order = []

        for leg_id in leg_keys:
            if "l_" in leg_id:
                left_side.append(leg_id)
            elif "r_" in leg_id:
                right_side.append(leg_id)

        if len(left_side) != len(right_side):
            raise ValueError("Leg numbers are different!")

        leg_num_of_each_side = len(left_side)
        for leg_num in range(leg_num_of_each_side):
            left_tripod_order = (leg_num * 2 ) % leg_num_of_each_side
            move_order.append(left_side[left_tripod_order])

            right_tripod_order = (leg_num * 2 + 1) % leg_num_of_each_side
            move_order.append(right_side[right_tripod_order])
        return  move_order

class DataExtraction:
    def __init__(self, hparser, current_leg_id):
        self.hexapod = hexapod
        self.leg_data_dict = hparser.leg_data_dict
        self.leg_move_order = hparser.leg_move_order
        self.mass_list = []
        self.total_mass = 0.0

    def mass_calculation(self):
        mass_list = []
        total_mass = 0.0

        for i in range(-1, p.getNumJoints(self.hexapod)):
            dynamic_info = p.getDynamicsInfo(self.hexapod, i)
            mass_list.append(dynamic_info[0])

        for mass_values in mass_list:
            total_mass += mass_values

        self.total_mass = total_mass
        self.mass_list = mass_list


    def center_of_mass(self):
        total_mass = self.total_mass
        local_x_com = 0.0
        local_y_com = 0.0
        local_z_com = 0.0

        for part_num in range(-1, p.getNumJoints(self.hexapod)):
            mass_list = part_num + 1
            mass = self.mass_list[mass_list]
            if part_num != -1: position = p.getLinkState(self.hexapod, part_num)[0]
            else: position = p.getBasePositionAndOrientation(self.hexapod)[0]

            local_x_com += mass * position[0]
            local_y_com += mass * position[1]
            local_z_com += mass * position[2]

        return local_x_com / total_mass, local_y_com / total_mass, local_z_com / total_mass


    def foot_data_prep(self, current_leg_id):
        foot_index = self.leg_data_dict[current_leg_id]["foot"]["index"]
        contact_points = p.getContactPoints(self.hexapod, linkIndexA= foot_index)

        if contact_points:
            contact_distance = contact_points[0][8]
            contact_force = contact_points[0][9]
            ground_contact = True if contact_force > 0 else False


            return contact_distance, contact_force, ground_contact
        return 0.0, 0.0, False

    def foot_com_target(self):
        pass





b = HexapodParser()
c = DataExtraction(b, "l_0")
c.mass_calculation()

while 1:
    p.stepSimulation()
    time.sleep(1./240.)
    print(c.center_of_mass())
