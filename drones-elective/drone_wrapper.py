from libardrone.libardrone import ARDrone2, ARDrone

import constants


class ARDroneWrapper(ARDrone2):
    def __init__(self):
        ARDrone2.__init__(self, False)

        self.should_hold_altitude = False

    def setup(self):
        self.reset()
        self.set_speed(constants.DRONE_DEFAULT_SPEED)
        self.set_camera_view(True)

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
