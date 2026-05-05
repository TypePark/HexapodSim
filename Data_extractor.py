import pybullet as p

class DataExtraction:
    def __init__(self, hexapod, hparser, gait_type):
        self.hexapod = hexapod
        self.leg_data_dict = hparser.leg_data_dict
        self.leg_move_order = hparser.gait_type(gait_type)
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
        local_x_com = 0.0
        local_y_com = 0.0
        local_z_com = 0.0

        for part_num in range(-1, p.getNumJoints(self.hexapod)):
            mass_list = part_num + 1
            mass = self.mass_list[mass_list]
            if part_num != -1:
                position = p.getLinkState(self.hexapod, part_num)[0]
            else:
                position = p.getBasePositionAndOrientation(self.hexapod)[0]

            local_x_com += mass * position[0]
            local_y_com += mass * position[1]
            local_z_com += mass * position[2]

        return {"localx": local_x_com / total_mass,
                "localy": local_y_com / total_mass,
                "localz": local_z_com / total_mass}

    def foot_data_prep(self, leg_ids_from_move_order):
        foot_data_result = {}
        for leg_id in leg_ids_from_move_order:
            foot_index = self.leg_data_dict[leg_id]["foot"]["index"]
            contact_points = p.getContactPoints(self.hexapod, linkIndexA=foot_index)
            contact_position = [0.0, 0.0, 0.0]
            contact_distance = [0.0]
            contact_force = [0.0]
            ground_contact = [False]

            if contact_points:
                contp = contact_points[0]
                contact_position = contp[5]
                contact_distance = contp[8]
                contact_force = contp[9]
                ground_contact = contact_force > 0

            foot_data_result[leg_id] = {"position": contact_position,
                                         "contact": ground_contact,
                                         "force": contact_force,
                                         "distance": contact_distance}
        return foot_data_result

    @staticmethod
    def foot_com_target(contact_positions, com):
        if len(contact_positions) < 3:
            raise ValueError("Contact list is lacking enough coordinates!")

        (cx1, cy1) = contact_positions[0]
        (cx2, cy2) = contact_positions[1]
        (cx3, cy3) = contact_positions[2]
        (comx, comy) = com

        triangle_center_x = (cx1 + cx2 + cx3) / 3
        triangle_center_y = (cy1 + cy2 + cy3) / 3

        denominator = (cy2 - cy3) * (cx1 - cx3) + (cx3 - cx2) * (cy1 - cy3)
        if denominator == 0:
            u, v, w = 0, 0, 0
        else:
            u = ((cy2 - cy3) * (comx - cx3) + (cx3 - cx2) * (comy - cy3)) / denominator
            v = ((cy3 - cy1) * (comx - cx3) + (cx1 - cx3) * (comy - cy3)) / denominator
            w = 1 - u - v

        return {"contact_position_0": u,
                "contact_position_1": v,
                "contact_position_2": w,
                "triangle_center_x": triangle_center_x,
                "triangle_center_y": triangle_center_y,
                "center_of_mass": com}
