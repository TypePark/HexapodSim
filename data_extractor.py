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

        self.previous_velocity = {}
        self.frame_counter = 0

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

    def center_of_mass(self): # Only keeps track of center of mass coordinates.
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

        com = {"local_x": local_x_com / total_mass,
                "local_y": local_y_com / total_mass,
                "local_z": local_z_com / total_mass}
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
        (x_com, y_com) = com["local_x"], com["local_y"]
        base_pos = np.array(p.getBasePositionAndOrientation(self.hexapod)[0])

        epsilon = np.finfo(float).eps
        inverse_weights = {}
        normalized_weights = {}

        for leg_ids, data in data_result.items():
            inverse_weights[leg_ids] = 0.0
            if data["contact"]:
                    position = np.array(data["position"])
                    (x_curr, y_curr, _) = (position - base_pos)
                    com_distance = np.sqrt((x_curr - x_com)**2 + (y_curr - y_com)**2)
                    com_distance_inverse_weights = 1.0 / (com_distance + epsilon)
                    inverse_weights[leg_ids] = com_distance_inverse_weights

        inverse_weights_sum = sum(inverse_weights.values())
        if inverse_weights_sum == 0:
            for leg_ids in inverse_weights.keys():
                normalized_weights[leg_ids] = 0.0
        else:
            for leg_ids, weights in inverse_weights.items():
                normalized_weights[leg_ids] = float(weights / inverse_weights_sum)

        return normalized_weights

    def slip_detection(self, data_result, gait_speed):
        slip = {}

        for leg_ids, data in data_result.items():

            if data["contact"]:
                index = self.leg_data_dict[leg_ids]["foot"]["index"]
                link_state = p.getLinkState(self.hexapod, index, 1)
                velocity = link_state[6]
                magnitude_velocity = np.linalg.norm(velocity)
                previous_velocity = self.previous_velocity.get(leg_ids, magnitude_velocity)

                velocity_spike = magnitude_velocity - previous_velocity

                if velocity_spike > 1.6 * gait_speed:
                    slip[leg_ids] = {"slip": True, "slip_velocity": magnitude_velocity}
                else:
                    slip[leg_ids] = {"slip": False, "slip_velocity": magnitude_velocity}
                self.previous_velocity[leg_ids] = magnitude_velocity

            else:
                slip[leg_ids] = {"slip": False, "slip_velocity": 0.0}
        return slip

    def fall_detection(self, foot_contact):
        foot_contact_count = sum(1 for data in foot_contact.values() if data["contact"])

        _, orientation = p.getBasePositionAndOrientation(self.hexapod)
        euler_angles = p.getEulerFromQuaternion(orientation)
        angle = abs(euler_angles[0]) + abs(euler_angles[1])

        tilt_fall = angle > 0.5
        lack_contact_fall = foot_contact_count < 3

        if tilt_fall or lack_contact_fall:
            self.frame_counter += 1
        else:
            self.frame_counter = 0

        return self.frame_counter >= 12 # frames