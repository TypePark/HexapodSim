import csv

class DataLogger:
    def __init__(self, file_name):
        self.file_name = file_name
        self.file = open(file_name, "w", newline="")
        self.writer = csv.writer(self.file)
        self.frame = 0
        self.writer.writerow([
            "frame",
            "gait_type",
            "gravity",
            "fall",
            "l_0_slip", "l_0_vel", "l_0_weights",
            "l_1_slip", "l_1_vel", "l_1_weights",
            "l_2_slip", "l_2_vel", "l_2_weights",
            "r_0_slip", "r_0_vel", "r_0_weights",
            "r_1_slip", "r_1_vel", "r_1_weights",
            "r_2_slip", "r_2_vel", "r_2_weights",
        ])

    def log(self, gait_type, gravity, fall, slip, foot_target):
        row= ([
            self.frame,
            gait_type,
            gravity,
            fall,
        ])

        for leg_ids in ["l_0", "l_1", "l_2", "r_0", "r_1", "r_2"]:
            leg_slip = slip.get(leg_ids, {"slip": False, "slip_velocity": 0.0})
            row.append(leg_slip["slip"])
            row.append(leg_slip["slip_velocity"])
            row.append(foot_target[leg_ids])
        self.writer.writerow(row)
        self.frame += 1

    def close(self):
        self.file.close()


