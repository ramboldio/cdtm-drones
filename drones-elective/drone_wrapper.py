from libardrone.libardrone import ARDrone2, ARDrone, at_pcmd

import constants


class ARDroneWrapper(ARDrone2):

    def __init__(self):
        ARDrone2.__init__(self, False)
        self.should_hold_altitude = False
        self.camera_down_active = False

    def setup(self):
        self.reset()
        self.set_speed(constants.DRONE_DEFAULT_SPEED)

    def apply_velocity(self, speed_x, speed_y, speed_z):
        if any(abs(speed) > 0.1 for speed in (speed_x, speed_y, speed_z)):
            self.at(at_pcmd, True, speed_x * 0.2, speed_y, speed_z, speed_x)
        else:
            self.hover()

    def apply_z_velocity(self, u):
        if abs(u) < 30:
            self.hover()
        else:
            self.at(at_pcmd, True, 0, 0, min(max(u / 100.0, -1.0), 1.0), 0)

    def apply_active_cam(self, cam_down_dest):
        if not self.cam_down_active == cam_down_dest:
            self.cam_down_active = cam_down_dest
            self.set_camera_view(self.cam_down_active)

    @property
    def ctrl_state(self):
        return self.get_curr_navdata('ctrl_state')

    @property
    def phi(self):
        return self.get_curr_navdata('phi')

    @property
    def psi(self):
        return self.get_curr_navdata('psi')

    @property
    def theta(self):
        return self.get_curr_navdata('theta')

    @property
    def altitude(self):
        return self.get_curr_navdata('altitude')

    @property
    def vx(self):
        return self.get_curr_navdata('vx')

    @property
    def vy(self):
        return self.get_curr_navdata('vy')

    @property
    def vz(self):
        return self.get_curr_navdata('vz')

    @property
    def battery(self):
        return self.get_curr_navdata('battery')

    @property
    def num_frames(self):
        return self.get_curr_navdata('num_frames')

    def get_curr_navdata(self, key):
        return self.get_navdata()[0][key]
