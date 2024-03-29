import logging
import axis_ptz_utilities

class Camera:
    def __init__ (self,
            tripod_longitude = 0,
            tripod_latitude = 0,
            tripod_altitude = 0,
            tripod_yaw = 0,
            tripod_pitch = 0,
            tripod_roll = 0,
    ):
        
        """ 
        Camera class to store the camera's tripod position and orientation.
        
        Parameters:
        tripod_longitude (float): The longitude of the camera tripod.
        tripod_latitude (float): The latitude of the camera tripod.
        tripod_yaw (float): The yaw of the camera tripod.
        tripod_pitch (float): The pitch of the camera tripod.
        tripod_roll (float): The roll of the camera tripod.
        """

        self.tripod_longitude = tripod_longitude
        self.tripod_latitude = tripod_latitude
        self.tripod_altitude = tripod_altitude
        self.tripod_yaw = tripod_yaw
        self.tripod_pitch = tripod_pitch
        self.tripod_roll = tripod_roll

        self.xyz_point = axis_ptz_utilities.compute_r_XYZ(
            self.tripod_longitude, self.tripod_latitude, self.tripod_altitude
        )

        # Compute orthogonal transformation matrix from geocentric
        # (XYZ) to topocentric (ENz) coordinates
        (
            self.E_XYZ_to_ENz,
            self.e_E_XYZ,
            self.e_N_XYZ,
            self.e_z_XYZ,
        ) = axis_ptz_utilities.compute_E_XYZ_to_ENz(self.lambda_t, self.varphi_t)

        # Compute the rotations from the geocentric (XYZ) coordinate
        # system to the camera housing fixed (uvw) coordinate system
        (
            self.q_alpha,
            self.q_beta,
            self.q_gamma,
            self.E_XYZ_to_uvw,
            _,
            _,
            _,
        ) = axis_ptz_utilities.compute_camera_rotations(
            self.e_E_XYZ,
            self.e_N_XYZ,
            self.e_z_XYZ,
            self.tripod_yaw,
            self.tripod_pitch,
            self.tripod_roll,
            0, #rho
            0, #tau
        )

    def get_yaw_pitch_roll(self):
        """
        Get the camera's yaw, pitch, and roll.
        """
        return self.tripod_yaw, self.tripod_pitch, self.tripod_roll
    
    def get_e_n_z(self):
        """
        Get the camera's e, n, and z vectors.
        """
        return self.e_E_XYZ, self.e_N_XYZ, self.e_z_XYZ


    def get_xyz_point(self):
        """
        Get the camera's xyz point.
        """
        return self.xyz_point
    
    def get_xyz_to_enz_transformation_matrix(self):
        """
        Get the camera's xyz to enz transformation matrix.
        """
        return self.E_XYZ_to_ENz

    def get_xyz_to_uvw_transformation_matrix(self):
        """
        Get the camera's xyz to uvw transformation matrix.
        """
        return self.E_XYZ_to_uvw

    def get_xyz_to_rst_transformation_matrix(self, object_rho, object_tau):
        """
        Get the camera's xyz to rst transformation matrix.
        """

        # Compute position and velocity in the camera fixed (rst)
        # coordinate system of the object relative to the tripod at
        # time zero after pointing the camera at the object
        (
            _,
            _,
            _,
            _,
            _,
            _,
            self.E_XYZ_to_rst,
        ) = axis_ptz_utilities.compute_camera_rotations(
            self.e_E_XYZ,
            self.e_N_XYZ,
            self.e_z_XYZ,
            self.tripod_yaw,
            self.tripod_pitch,
            self.tripod_roll,
            object_rho,
            object_tau,
        )

        return self.E_XYZ_to_rst

    def _log_config():
        """
        Log the camera configuration.
        """
        
        config = {
            'tripod_longitude': self.tripod_longitude,
            'tripod_latitude': self.tripod_latitude,
            'tripod_altitude': self.tripod_altitude,
            'tripod_yaw': self.tripod_yaw,
            'tripod_pitch': self.tripod_pitch,
            'tripod_roll': self.tripod_roll,
        }

        logging.info(f'Camera configuration: {config}')