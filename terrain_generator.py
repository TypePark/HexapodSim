import pybullet as p
import numpy as np

class Terrain:
    def __init__(self):
        self.rows = 700
        self.columns = 700 # Matrices bigger than 700x700 refuses to render, instead increase the meshScale at self.terrain_shape (Note: Increasing the meshScale sometimes results in clipping through ground).

        height_matrix = np.zeros((self.rows, self.columns))
        r_peak = np.random.randint(0,self.rows)
        c_peak = np.random.randint(0,self.columns)
        peak_height = 20

        r_grid, c_grid= np.meshgrid(np.arange(self.rows), np.arange(self.columns), indexing="ij")

        peak_distance = np.sqrt((r_grid - r_peak)**2 + (c_grid - c_peak)**2)
        decay = np.exp(-peak_distance / 200)
        height_matrix = decay * peak_height

        pybullet_shift = (height_matrix.max() + height_matrix.min()) / 2 # Pybullet divides peak_height by 2 and put half of it negative and other half to positive axis.
        self.hexapod_spawn = (height_matrix[self.rows // 2, self.columns // 2]) - pybullet_shift + (peak_height / 2) + 0.25 # Note: Divide operator always returns float.
        self.heights = height_matrix.flatten()


        self.terrain_shape = p.createCollisionShape(
            shapeType=p.GEOM_HEIGHTFIELD,
            meshScale=[1.0, 1.0, 1.0],
            numHeightfieldRows=self.rows,
            numHeightfieldColumns=self.columns,
            heightfieldData=self.heights
        )

        self.terrain_body = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=self.terrain_shape,
            basePosition=[0,0, peak_height / 2]
        )

        p.changeVisualShape(self.terrain_body, -1, rgbaColor=[0.75, 0.75, 0.75, 1.0])

