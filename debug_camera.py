import pybullet as p
import threading


class DebugCamera:
    def __init__(self, hexapod):
        self.hexapod = hexapod
        self.hexapod_cam, _ = p.getBasePositionAndOrientation(self.hexapod)

        debug_camera = p.getDebugVisualizerCamera()
        self.cam_yaw = debug_camera[8]
        self.cam_pitch = debug_camera[9]
        self.cam_dist = debug_camera[10]
        self.cam_target = list(debug_camera[11])

        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._camera_loop, daemon=True)
        self._thread.start()

    def _camera_loop(self):
        while self._running:
            updated_pos, _ = p.getBasePositionAndOrientation(self.hexapod)
            with self._lock:
                self.cam_target[0] += updated_pos[0] - self.hexapod_cam[0]
                self.cam_target[1] += updated_pos[1] - self.hexapod_cam[1]
                self.cam_target[2] += updated_pos[2] - self.hexapod_cam[2]
                self.hexapod_cam = updated_pos

            debug_camera = p.getDebugVisualizerCamera()
            self.cam_yaw = debug_camera[8]
            self.cam_pitch = debug_camera[9]
            self.cam_dist = debug_camera[10]
            p.resetDebugVisualizerCamera(
                self.cam_dist,
                self.cam_yaw,
                self.cam_pitch,
                self.cam_target
            )

    def stop(self):
        self._running = False
        self._thread.join()
