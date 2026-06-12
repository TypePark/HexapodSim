import pybullet as p
import numpy as np

class Terrain:
    def __init__(self):
        self.rows = 400
        self.columns = 400

        self.heights = np.random.rand(self.rows, self.columns).flatten()

        self.terrain_shape = p.createCollisionShape(
            shapeType=p.GEOM_HEIGHTFIELD,
            meshScale=[1.0, 1.0, 1.0],
            numHeightfieldRows=self.rows,
            numHeightfieldColumns=self.columns,
            heightfieldData=self.heights
        )


        self.terrain_body = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=self.terrain_shape
        )

        p.changeVisualShape(self.terrain_body, -1, rgbaColor=[0.75, 0.75, 0.75, 1.0])