import pybullet as p
import numpy as np

class DataExtraction:
    def __init__(self, hexapod, hparser, gait_type):
        self.hexapod = hexapod
        self.leg_data_dict = hparser.leg_data_dict
        self.leg_move_order = hparser.gait_selector(gait_type)
        self.mass_list = []
        self.total_mass = 0.0
        self._mass_calculation()

    def _mass_calculation(self):
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
        base_pos = p.getBasePositionAndOrientation(self.hexapod)[0]

        local_x_com = 0.0
        local_y_com = 0.0
        local_z_com = 0.0

        for part_num in range(-1, p.getNumJoints(self.hexapod)):
            mass = self.mass_list[part_num + 1]
            if part_num != -1:
                position = p.getLinkState(self.hexapod, part_num)[0]
            else:
                position = base_pos

            local_x_com += mass * (position[0] - base_pos[0])
            local_y_com += mass * (position[1] - base_pos[1])
            local_z_com += mass * (position[2] - base_pos[2])

        com = {"localx": local_x_com / total_mass,
                "localy": local_y_com / total_mass,
                "localz": local_z_com / total_mass}

        return com

    def foot_contact(self):
        foot_data_result = {}
        for leg_ids in self.leg_move_order:
            foot_index = self.leg_data_dict[leg_ids]["foot"]["index"]
            contact_points = p.getContactPoints(self.hexapod, linkIndexA=foot_index)
            contact_position = [0.0, 0.0, 0.0]
            contact_distance = 0.0
            contact_force = 0.0
            ground_contact = False

            if contact_points:
                contp = contact_points[0]
                contact_position = contp[5]
                contact_distance = contp[8]
                contact_force = contp[9]
                ground_contact  = True if contact_force > -0.01 else False

            foot_data_result[leg_ids] = {"position": contact_position,
                                         "contact": ground_contact,
                                         "force": contact_force,
                                         "distance": contact_distance}
        return foot_data_result


    def foot_com_target(self, data_result):
        com = self.center_of_mass()
        (x_com, y_com) = com["localx"], com["localy"]
        base_pos = np.array(p.getBasePositionAndOrientation(self.hexapod)[0])

        epsilon = np.finfo(float).eps
        inverse_weights = {}
        the_ones_not_helping = []

        for leg_ids, data in data_result.items():
            if data["contact"]:
                    position = np.array(data["position"])
                    (x_curr, y_curr, _) = (position - base_pos)
                    com_distance = np.sqrt((x_curr - x_com)**2 + (y_curr - y_com)**2)
                    com_distance_inverse_weights = 1.0 / (com_distance + epsilon)
                    inverse_weights[leg_ids] = com_distance_inverse_weights
            elif not data["contact"]:
                the_ones_not_helping.append(leg_ids)

        inverse_weights_sum = sum(inverse_weights.values())
        normalized_weights = {leg_id: float(weigths / inverse_weights_sum) for leg_id, weigths in inverse_weights.items()}

        return normalized_weights, the_ones_not_helping

