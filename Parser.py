import pybullet as p

class HexapodParser:
    def __init__(self, hexapod):
        self.hexapod = hexapod
        self._parser()
        self.leg_data_dict = {}


    def _parser(self):
        joint_data_dict = {}
        for joint_num in range(p.getNumJoints(self.hexapod)):
            joint_info = p.getJointInfo(self.hexapod, joint_num)
            joint_name = joint_info[1].decode("utf-8")
            joint_upper_limit = joint_info[8]
            joint_lower_limit = joint_info[9]

            split = joint_name.split("_")
            leg_id = split[-2] + "_" + split[-1] # Ex: l_0

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

    def gait_type(self, gait_type):
        gait_types = ["tripod", "wave", "ripple"]

        if gait_type not in gait_types:
            raise KeyError("gait type does not exist.")

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

        if gait_type == gait_types[0] or gait_type == gait_types[2]:
            for leg_num in range(leg_num_of_each_side):
                left_tripod_order = (leg_num * 2) % leg_num_of_each_side
                move_order.append(left_side[left_tripod_order])

                right_tripod_order = (leg_num * 2 + 1) % leg_num_of_each_side
                move_order.append(right_side[right_tripod_order])

        elif gait_type == gait_types[1]:
            for leg_num in range(leg_num_of_each_side):
                move_order.append(left_side[leg_num])
                move_order.append(right_side[leg_num])

        return move_order


