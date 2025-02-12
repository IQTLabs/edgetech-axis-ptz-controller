import logging
from time import sleep
import math
from typing import Tuple

import numpy as np
import numpy.typing as npt


import axis_ptz_utilities
from camera_configuration import CameraConfiguration
from camera_control import CameraControl


class Camera:
    """Camera class for the Axis PTZ camera. This tracks the camera's configuration
    and controls the camera's movement.
    """

    def __init__(
        self,
        camera_ip: str,
        camera_user: str,
        camera_password: str,
        use_camera: bool = True,
        auto_focus: bool = False,
        tripod_longitude: float = 0,
        tripod_latitude: float = 0,
        tripod_altitude: float = 0,
        tripod_yaw: float = 0,
        tripod_pitch: float = 0,
        tripod_roll: float = 0,
        pan_rate_max: float = 150.0,
        tilt_rate_max: float = 150.0,
        zoom: int = 6000,
        focus: int = 8749,
        focus_min: int = 7499,
        focus_max: int = 9999,
        hyperfocal_distance: float = 22500.0,
        is_dome: bool = True,
    ) -> None:
        """Initializes the instance of the Camera class.

        Args:
        tripod_longitude (float): The longitude of the camera tripod.
        tripod_latitude (float): The latitude of the camera tripod.
        tripod_altitude (float): The altitude of the camera tripod.
        tripod_yaw (float): The yaw of the camera tripod.
        tripod_pitch (float): The pitch of the camera tripod.
        tripod_roll (float): The roll of the camera tripod.
        pan_rate_max (float): The maximum pan rate of the camera.
                            Find this on the Axis Spec Sheet.
        tilt_rate_max (float): The maximum tilt rate of the camera.
                            Find this on the Axis Spec Sheet.
        zoom (int): The zoom of the camera.
        focus (int): The focus of the camera, between 0 and 9999.
        focus_min (int): The minimum focus of the camera.
        focus_max (int): The maximum focus of the camera.
        hyperfocal_distance (float): The hyperfocal distance of the camera.
        """

        self.camera_ip = camera_ip
        self.camera_user = camera_user
        self.camera_password = camera_password
        self.tripod_longitude = tripod_longitude
        self.tripod_latitude = tripod_latitude
        self.tripod_altitude = tripod_altitude
        self.tripod_yaw = tripod_yaw
        self.tripod_pitch = tripod_pitch
        self.tripod_roll = tripod_roll
        self.pan_rate_max = pan_rate_max
        self.tilt_rate_max = tilt_rate_max
        self.zoom = zoom
        self.focus = focus
        self.focus_min = focus_min
        self.focus_max = focus_max
        self.hyperfocal_distance = hyperfocal_distance
        self.use_camera = use_camera
        self.auto_focus = auto_focus
        self.is_dome = is_dome

        self.rho = 0.0
        self.tau = 0.0
        self.zoom_c = 0
        self.focus_c = 0

        # Initialize pan and tilt rate indices, which are between -100
        # and 100
        self.pan_rate_index = 0.0
        self.tilt_rate_index = 0.0

        # Always construct camera configuration and control since
        # instantiation only assigns arguments
        logging.info("Constructing camera configuration and control")
        self.camera_configuration = CameraConfiguration(
            self.camera_ip, self.camera_user, self.camera_password
        )
        self.camera_control = CameraControl(
            self.camera_ip, self.camera_user, self.camera_password
        )

        # Camera focus parameters. Note that the focus setting is
        # minimum at and greater than the hyperfocal distance, and the
        # focus setting is maximum when the distance is zero.
        self.focus_slope = (self.focus_min - self.focus_max) / self.hyperfocal_distance
        self.focus_intercept = self.focus_max

        # Initialize tripod position in the geocentric (XYZ)
        # coordinate system, orthogonal transformation matrix from
        # geocentric (XYZ) to topocentric (ENz) coordinates, and East,
        # North, and zenith unit vectors

        # Compute tripod position in the geocentric (XYZ) coordinate
        # system
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
        ) = axis_ptz_utilities.compute_E_XYZ_to_ENz(
            self.tripod_longitude, self.tripod_latitude
        )

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
            0,  # rho
            0,  # tau
        )

        self._config_log()

    def _config_log(self) -> None:
        """Outputs the camera configuration to the logging."""

        config = {
            "tripod_longitude": self.tripod_longitude,
            "tripod_latitude": self.tripod_latitude,
            "tripod_altitude": self.tripod_altitude,
            "tripod_yaw": self.tripod_yaw,
            "tripod_pitch": self.tripod_pitch,
            "tripod_roll": self.tripod_roll,
            "E_XYZ_to_ENz": self.E_XYZ_to_ENz,
            "E_XYZ_to_uvw": self.E_XYZ_to_uvw,
            "xyz_point": self.xyz_point,
            "pan_rate_max": self.pan_rate_max,
            "tilt_rate_max": self.tilt_rate_max,
        }

        logging.info(f"Camera configuration: {config}")

    def update_tripod_position(
        self, tripod_longitude: float, tripod_latitude: float, tripod_altitude: float
    ) -> None:
        """
        Updates the camera's tripod position. The transformations will
        be recalculated based on the updated information.

        Args:
        tripod_longitude (float): The longitude of the camera tripod.
        tripod_latitude (float): The latitude of the camera tripod.
        tripod_altitude (float): The altitude of the camera tripod.
        """
        self.tripod_longitude = tripod_longitude
        self.tripod_latitude = tripod_latitude
        self.tripod_altitude = tripod_altitude

        # Compute tripod position in the geocentric (XYZ) coordinate
        # system
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
        ) = axis_ptz_utilities.compute_E_XYZ_to_ENz(
            self.tripod_longitude, self.tripod_latitude
        )
    def recalculate_focus(self) -> None:
        self.focus_slope = (self.focus_min - self.focus_max) / self.hyperfocal_distance
        self.focus_intercept = self.focus_max

    def update_tripod_orientation(
        self, tripod_yaw: float, tripod_pitch: float, tripod_roll: float
    ) -> None:
        """
        Update the camera's tripod orientation. The camera's rotations will
        be recalculated based on the updated information.

        Args:
        tripod_yaw (float): The yaw of the camera tripod.
        tripod_pitch (float): The pitch of the camera tripod.
        tripod_roll (float): The roll of the camera tripod.
        """
        self.tripod_yaw = tripod_yaw
        self.tripod_pitch = tripod_pitch
        self.tripod_roll = tripod_roll

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
            0,  # rho
            0,  # tau
        )

    def get_ptz(self) -> Tuple[float, float, int, int]:
        """Queries the camera for its current pan, tilt, zoom, and focus settings.

        Returns:
        rho (float): The camera's pan setting.
        tau (float): The camera's tilt setting.
        zoom (int): The camera's zoom setting.
        focus (int): The camera's focus setting.
        """
        # Get camera pan and tilt
        self.rho, self.tau, self.zoom_c, self.focus_c = self.camera_control.get_ptz()
        return self.rho, self.tau, self.zoom_c, self.focus_c

    def update_focus(self, distance: float) -> None:
        """Update the camera's focus based on the distance to the object."""

        if not self.auto_focus:
            # Note that focus cannot be negative, since distance_to_tripod3d
            # is non-negative
            self.focus = int(
                max(
                    self.focus_min,
                    self.focus_slope * distance + self.focus_intercept,
                )
            )
            logging.debug(f"Commanding focus: {self.focus} for distance: {distance}, slope: {self.focus_slope}, intercept: {self.focus_intercept}")
            try:
                self.camera_control.set_focus(self.focus)
            except Exception as e:
                logging.error(f"Error: {e}")

    def stop_move(self) -> None:
        """Stops the camera's movement."""

        if self.use_camera:
            try:
                self.camera_control.stop_move()
            except Exception as e:
                logging.error(f"Error: {e}")

    def update_pan_tilt_rates(self, pan_rate: float, tilt_rate: float) -> None:
        """Update the camera's pan and tilt rates. The rates are in
        degrees per second which will be converted to an index
        between -100 and 100 based on the camera's pan and tilt
        rate maximums.

        Args:
        pan_rate (float): pan rate in degrees per second.
        tilt_rate (float): tilt rate in degrees per second.
        """
        self._compute_pan_rate_index(pan_rate)
        self._compute_tilt_rate_index(tilt_rate)

        try:
            logging.disable(logging.INFO)
            self.camera_control.continuous_move(
                self.pan_rate_index,
                self.tilt_rate_index
            )
            logging.disable(logging.NOTSET)

        except Exception as e:
            logging.error(f"Error: {e}")

    def update_zoom(self, zoom: int) -> None:
        """Update the camera's zoom setting."""
        self.zoom = zoom
        self.camera_control.absolute_move(None, None, self.zoom, None)

    def move_to_azimuth_elevation(
        self, azimuth: float, elevation: float, zoom: int
    ) -> None:
        """Move the camera to the specified azimuth and elevation angles.
        The camera will also zoom to the specified zoom level.
        The Yaw, Pitch, and Roll of the camera will be used to ensure that
        the camera is pointing correctly.

        Args:
        azimuth (float): The azimuth angle to move the camera to.
        elevation (float): The elevation angle to move the camera to.
        zoom (int): The zoom level to move the camera to.
        """

        self.zoom = zoom
        if self.use_camera:
            self.rho, self.tau, _zoom, _focus = self.camera_control.get_ptz()
            logging.info(
                f"Current Camera pan and tilt: {self.rho}, {self.tau} [deg]"
            )

            # Compute position of aircraft relative to tripod in ENz, then XYZ,
            # then uvw coordinates
            r_ENz_a_t = np.array(
                [
                    math.sin(math.radians(azimuth)) * math.cos(math.radians(elevation)),
                    math.cos(math.radians(azimuth)) * math.cos(math.radians(elevation)),
                    math.sin(math.radians(elevation)),
                ]
            )
            r_XYZ_a_t = np.matmul(self.E_XYZ_to_ENz.transpose(), r_ENz_a_t)
            r_uvw_a_t = np.matmul(self.E_XYZ_to_uvw, r_XYZ_a_t)

            # Compute pan an tilt
            camera_pan = math.degrees(math.atan2(r_uvw_a_t[0], r_uvw_a_t[1]))  # [deg]
            camera_tilt = math.degrees(
                math.atan2(r_uvw_a_t[2], axis_ptz_utilities.norm(r_uvw_a_t[0:2]))
            )  # [deg]

            logging.info(
                f"Absolute Move - Target pan: {azimuth} tilt: {elevation} Corrected pan: {camera_pan}, and tilt: {camera_tilt}, with zoom: {self.zoom}"
            )
            if camera_tilt >= 0 and camera_pan >= -180 and camera_pan <= 180:

                self.slew_camera(camera_pan, camera_tilt)
            else:
                logging.error(
                    f"Invalid azimuth and elevation angles: {camera_pan}, {camera_tilt} "

                )   

    def slew_camera(self, rho_target: float, tau_target: float) -> None:
        """Slew the camera to the specified pan and tilt angles. It
        is assumed that the yaw, pitch, and roll corrections have
        already been applied.

        Args:
        rho_target (float): The target pan angle.
        tau_target (float): The target tilt angle.
        """

        # Get camera pan and tilt
        self.rho, self.tau, self.zoom_c, self.focus_c = self.camera_control.get_ptz()
        logging.info(
            f"Camera pan, tilt, zoom, and focus: {self.rho} [deg], {self.tau} [deg], {self.zoom_c}, {self.focus_c}"
        )

        logging.info(
            f"Absolute move to pan: {rho_target}, and tilt: {tau_target}, with zoom: {self.zoom}"
        )
        try:
            self.camera_control.absolute_move(rho_target, tau_target, self.zoom, 99)
        except Exception as e:
            logging.error(f"Error: {e}")
        tilt_delta = math.fabs(
                axis_ptz_utilities.compute_angle_delta(
                    theta_c=self.tau, theta_o=tau_target
                )
            )
        pan_delta = math.fabs(
                axis_ptz_utilities.compute_angle_delta(
                    theta_c=self.rho, theta_o=rho_target
                )
            )
        duration = max(
            pan_delta
            / (self.pan_rate_max),
            tilt_delta
            / (self.tilt_rate_max),
        )
        duration = duration + 0.5
        logging.info(f"Panning {pan_delta} degrees, Tilting {tilt_delta} Sleeping: {duration} [s] = Pan time {pan_delta / self.pan_rate_max} or  Tilt time {tilt_delta / self.tilt_rate_max} + 0.5")
        sleep(duration)

    def _compute_pan_rate_index(self, rho_dot: float) -> None:
        """Compute pan rate index between -100 and 100 using rates in
        deg/s, limiting the results to the specified minimum and
        maximum. Note that the dead zone from -1.8 to 1.8 deg/s is ignored.

        Parameters
        ----------
        rho_dot : float
            Pan rate [deg/s]

        Returns
        -------
        """
        if self.is_dome:  # Dome style cameras have linear behavior
            self.pan_rate_index = rho_dot/self.pan_rate_max*100.0
        else:  # Articulating style cameras have exponential behavior, gives more precise control at low numbers
            self.pan_rate_index = (abs(rho_dot)/0.002)**(1/2.38824)*(rho_dot/abs(rho_dot))
        
        if self.pan_rate_index<-100:
            self.pan_rate_index=-100
        
        if self.pan_rate_index>100:
            self.pan_rate_index=100

        # logging.info(f'{self.pan_rate_index} | {rho_dot}')
        # Even though the VAPIX API says it only supports INT, it seems to handle floats just fine

    def _compute_tilt_rate_index(self, tau_dot: float) -> None:
        """Compute tilt rate index between -100 and 100 using rates in
        deg/s, limiting the results to the specified minimum and
        maximum. Note that the dead zone from -1.8 to 1.8 deg/s is ignored.

        Parameters
        ----------
        tau_dot : float
            Tilt rate [deg/s]

        Returns
        -------
        """
        if self.is_dome:  # Dome style cameras have linear behavior
            self.tilt_rate_index = tau_dot/self.tilt_rate_max*100.0
        else:  # Articulating style cameras have exponential behavior
            self.tilt_rate_index = (abs(tau_dot)/0.002)**(1/2.38824)*(tau_dot/abs(tau_dot))
        
        if self.tilt_rate_index<-100:
            self.tilt_rate_index=-100
        
        if self.tilt_rate_index>100:
            self.tilt_rate_index=100

        # logging.info(f'{self.tilt_rate_index} | {tau_dot}')
        # Even though the VAPIX API says it only supports INT, it seems to handle floats just fine

    def get_yaw_pitch_roll(self) -> Tuple[float, float, float]:
        """
        Get the camera's yaw, pitch, and roll.

        Returns:
        tripod_yaw (float): The camera's yaw in degrees.
        tripod_pitch (float): The camera's pitch in degrees.
        tripod_roll (float): The camera's roll in degrees.
        """
        return self.tripod_yaw, self.tripod_pitch, self.tripod_roll

    def get_e_n_z(
        self,
    ) -> Tuple[
        npt.NDArray[np.float64], npt.NDArray[np.float64], npt.NDArray[np.float64]
    ]:
        """
        Get the camera's topocentric (ENz) East,
        North, and zenith unit vectors, for conversion from XYZ to ENz.

        Returns:
        e_E_XYZ (np.array): The camera's East unit vector.
        e_N_XYZ (np.array): The camera's North unit vector.
        e_z_XYZ (np.array): The camera's zenith unit vector.
        """
        return self.e_E_XYZ, self.e_N_XYZ, self.e_z_XYZ

    def get_xyz_point(self) -> npt.NDArray[np.float64]:
        """
        Gets the camera's location in the geocentric (XYZ) coordinate system.

        Returns:
        xyz_point (np.array): The camera's location in the geocentric (XYZ) coordinate system.
        """
        return self.xyz_point

    def get_xyz_to_enz_transformation_matrix(self) -> npt.NDArray[np.float64]:
        """
        Get the camera's transformation matrix from geocentric (XYZ) to
        topocentric (ENz) coordinates.

        Returns:
        E_XYZ_to_ENz (np.array): The camera's transformation matrix from
                            geocentric (XYZ) to topocentric (ENz)
                            coordinates.
        """
        return self.E_XYZ_to_ENz

    def get_xyz_to_uvw_transformation_matrix(self) -> npt.NDArray[np.float64]:
        """
        Get the camera's transformation matrix from geocentric (XYZ) to
        camera housing fixed (uvw) coordinates.

        Returns:
        E_XYZ_to_uvw (np.array): The camera's transformation matrix from
                            geocentric (XYZ) to camera housing fixed (uvw)
                            coordinates.
        """
        return self.E_XYZ_to_uvw
