import logging
import math
from time import time
import threading
from typing import Any, Dict
import numpy as np
import axis_ptz_utilities
from camera import Camera


class Object:

    def __init__(
        self,
        object_id: str,
        camera: Camera,
        include_age: bool = True,
        lead_time: float = 0.0,
    ) -> None:
        """
        Initializes the Object instance.

        Args:
            object_id (str): Object identifier, expected to be IACO hex code
            camera (Camera): Camera instance
            include_age (bool): Flag to include age of object message
            lead_time (float): Time in seconds to lead object message
        """
        self.object_id = object_id
        self.msg_longitude = 0
        self.msg_latitude = 0
        self.msg_altitude = 0
        self.msg_timestamp = 0
        self.msg_track = 0
        self.msg_horizontal_velocity = 0
        self.msg_vertical_velocity = 0
        self.camera = camera
        self.lead_time = lead_time
        self.include_age = include_age

        self._config_log()

        # Object pan and tilt relative to the tripod
        self.rho = 0.0  # [deg] currently set to rho_lead
        self.tau = 0.0  # [deg]
        self.rho_now = 0.0  # [deg] where the object is now
        self.tau_now = 0.0  # [deg]
        self.rho_lead = 0.0  # [deg] where the object will be with lead time
        self.tau_lead = 0.0  # [deg]
        self.rho_rate = 0.0  # [deg/s]
        self.tau_rate = 0.0  # [deg/s]
        self.azm = 0.0  # [deg] azimuth of object relative to tripod,
        # without camera housing corrections
        self.elv = 0.0  # [deg] elevation of object relative to tripod,
        # without camera housing corrections

        # Position in the geocentric (XYZ) coordinate system of the
        # object relative to the tripod at time zero, the observation
        # time
        self.xyz_point_msg = np.zeros((3,))

        # Orthogonal transformation matrix from camera housing (uvw)
        # to camera fixed (rst) coordinates
        self.E_XYZ_to_rst = np.zeros((3, 3))

        # Distance between the object and the tripod at time one
        self.distance_to_tripod3d = 0.0  # [m]
        self.rho_derivative = 0.0  # [deg/s]
        self.tau_derivative = 0.0  # [deg/s]

        # Position and velocity in the geocentric (XYZ) coordinate
        # system of the object relative to the tripod at time zero
        self.xyz_point_msg_relative_to_tripod = np.zeros((3,))
        self.xyz_point_now_relative_to_tripod = np.zeros((3,))
        self.xyz_point_lead_relative_to_tripod = np.zeros((3,))
        self.xyz_velocity_msg_relative_to_tripod = np.zeros((3,))

        # Position and velocity in the topocentric (ENz) coordinate
        # system of the object relative to the tripod at time zero
        self.enz_point_msg_relative_to_tripod = np.zeros((3,))
        self.enz_velocity_msg_relative_to_tripod = np.zeros((3,))
        self.enz_point_now_relative_to_tripod = np.zeros((3,))
        self.enz_point_lead_relative_to_tripod = np.zeros((3,))

        # Object id, timestamp of object message and corresponding
        # object longitude, latitude, and altitude, and position and
        # velocity relative to the tripod in the camera fixed (rst)
        # coordinate system
        self.rst_point_now_relative_to_tripod = np.zeros((3,))  # [m/s]
        self.rst_point_lead_relative_to_tripod = np.zeros((3,))  # [m/s]
        self.rst_point_msg_relative_to_tripod = np.zeros((3,))  # [m/s]
        self.rst_velocity_msg_relative_to_tripod = np.zeros((3,))  # [m/s]

        self.uvw_point_now_relative_to_tripod = np.zeros((3,))
        self.uvw_point_lead_relative_to_tripod = np.zeros((3,))
        self.location_update_time = time()
        self.no_derivative = True
        
        # Lock to make sure object info is not used while being updated
        self.object_lock = threading.Lock()

    def _config_log(self) -> None:
        """Print to Logging the object configuration"""

        config = {
            "object_id": self.object_id,
            "msg_longitude": self.msg_longitude,
            "msg_latitude": self.msg_latitude,
            "msg_altitude": self.msg_altitude,
            "msg_timestamp": self.msg_timestamp,
            "lead_time": self.lead_time,
            "msg_track": self.msg_track,
            "msg_horizontal_velocity": self.msg_horizontal_velocity,
            "msg_vertical_velocity": self.msg_vertical_velocity,
            "include_age": self.include_age,
        }
        logging.info(f"Object configuration: {config}")

    def update_from_msg(self, msg: Dict[str, Any]) -> None:
        """
        Update the object's position and timestamp from a message.

        Args:
            msg (dict): Message containing object position and timestamp

        """
        self.object_lock.acquire()
        self.msg_longitude = msg["longitude"]
        self.msg_latitude = msg["latitude"]
        self.msg_altitude = msg["altitude"]
        self.msg_timestamp = msg["timestamp"]
        self.msg_track = msg["track"]
        self.msg_horizontal_velocity = msg["horizontal_velocity"]
        self.msg_vertical_velocity = msg["vertical_velocity"]

        # Compute position in the geocentric (XYZ) coordinate system
        # of the object relative to the tripod at time zero, the
        # observation time

        self.xyz_point_msg = axis_ptz_utilities.compute_r_XYZ(
            self.msg_longitude, self.msg_latitude, self.msg_altitude
        )

        self.xyz_point_msg_relative_to_tripod = (
            self.xyz_point_msg - self.camera.get_xyz_point()
        )
        self.enz_point_msg_relative_to_tripod = np.matmul(
            self.camera.get_xyz_to_enz_transformation_matrix(),
            self.xyz_point_msg_relative_to_tripod,
        )

        radian_track = math.radians(self.msg_track)

        self.enz_velocity_msg_relative_to_tripod = np.array(
            [
                self.msg_horizontal_velocity * math.sin(radian_track),
                self.msg_horizontal_velocity * math.cos(radian_track),
                self.msg_vertical_velocity,
            ]
        )

        # Compute position, at time zero,
        # in the geocentric (XYZ) coordinate system of the object
        # relative to the tripod

        self.xyz_velocity_msg_relative_to_tripod = np.matmul(
            self.camera.get_xyz_to_enz_transformation_matrix().transpose(),
            self.enz_velocity_msg_relative_to_tripod,
        )

        self.no_derivative = True # things get weird when you update the object and then recompute the derivative
        self.object_lock.release()

    def recompute_location(self) -> None:
        """Recompute the object's current position and velocity relative to the tripod.
        The amount of time since the last position message is used to calculate
         where the object is."""

        self.object_lock.acquire()

        # Assign lead time, computing and adding age of object
        # message, if enabled
        time_delta = self.lead_time  # [s] time_since_last_update
        msg_age = 0.0
        if self.include_age:
            msg_age = (
                time() - self.msg_timestamp
            )  # datetime.utcnow().timestamp() - self.timestamp_o  # [s]
            logging.debug(f"Object msg age: {msg_age} [s]")
            time_delta += msg_age



        # Compute position and velocity in the topocentric (ENz)
        # coordinate system of the object relative to the tripod at
        # time zero, and position at slightly later time one

        # self.enz_point_now_relative_to_tripod = (
        #     self.enz_point_msg_relative_to_tripod
        #     + msg_age * self.enz_velocity_msg_relative_to_tripod
        # )
        self.enz_point_lead_relative_to_tripod = (
            self.enz_point_msg_relative_to_tripod
            + time_delta * self.enz_velocity_msg_relative_to_tripod
        )
        
        # Compute position, at time one, and velocity, at time zero,
        # in the geocentric (XYZ) coordinate system of the object
        # relative to the tripod

        # self.xyz_point_now_relative_to_tripod = np.matmul(
        #     self.camera.get_xyz_to_enz_transformation_matrix().transpose(),
        #     self.enz_point_now_relative_to_tripod,
        # )
        self.xyz_point_lead_relative_to_tripod = np.matmul(
            self.camera.get_xyz_to_enz_transformation_matrix().transpose(),
            self.enz_point_lead_relative_to_tripod,
        )

        # Compute the distance between the object and the tripod at
        # time one
        self.distance_to_tripod3d = axis_ptz_utilities.norm(
            self.enz_point_lead_relative_to_tripod
        )

        # Compute the object azimuth and elevation relative to the
        # tripod
        self.azm = math.degrees(
            math.atan2(
                self.enz_point_lead_relative_to_tripod[0],
                self.enz_point_lead_relative_to_tripod[1],
            )
        )  # [deg]
        self.elv = math.degrees(
            math.atan2(
                self.enz_point_lead_relative_to_tripod[2],
                axis_ptz_utilities.norm(self.enz_point_lead_relative_to_tripod[0:2]),
            )
        )  # [deg]

        # Compute pan and tilt to point the camera at the object
        # self.uvw_point_now_relative_to_tripod = np.matmul(
        #     self.camera.get_xyz_to_uvw_transformation_matrix(),
        #     self.xyz_point_now_relative_to_tripod,
        # )
        self.uvw_point_lead_relative_to_tripod = np.matmul(
            self.camera.get_xyz_to_uvw_transformation_matrix(),
            self.xyz_point_lead_relative_to_tripod,
        )

        # self.rho_now = math.degrees(
        #     math.atan2(
        #         self.uvw_point_now_relative_to_tripod[0],
        #         self.uvw_point_now_relative_to_tripod[1],
        #     )
        # )  # [deg]
        # self.tau_now = math.degrees(
        #     math.atan2(
        #         self.uvw_point_now_relative_to_tripod[2],
        #         axis_ptz_utilities.norm(self.uvw_point_now_relative_to_tripod[0:2]),
        #     )
        # )

        self.rho_lead = math.degrees(
            math.atan2(
                self.uvw_point_lead_relative_to_tripod[0],
                self.uvw_point_lead_relative_to_tripod[1],
            )
        )  # [deg]
        self.tau_lead = math.degrees(
            math.atan2(
                self.uvw_point_lead_relative_to_tripod[2],
                axis_ptz_utilities.norm(self.uvw_point_lead_relative_to_tripod[0:2]),
            )
        )
        self.location_update_period = time() - self.location_update_time
        self.location_update_time = time()

        if not self.no_derivative:
            self.rho_derivative = (self.rho - self.rho_lead) / self.location_update_period
            self.tau_derivative = (self.tau - self.tau_lead) / self.location_update_period


        self.rho = self.rho_lead
        self.tau = self.tau_lead

        camera_yaw, camera_pitch, camera_roll = self.camera.get_yaw_pitch_roll()
        camera_E_XYZ, camera_N_XYZ, camera_z_XYZ = self.camera.get_e_n_z()
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
            camera_E_XYZ,
            camera_N_XYZ,
            camera_z_XYZ,
            camera_yaw,
            camera_pitch,
            camera_roll,
            self.rho,
            self.tau,
        )

        # self.rst_point_now_relative_to_tripod = np.matmul(
        #     self.E_XYZ_to_rst, self.xyz_point_now_relative_to_tripod
        # )
        self.rst_point_lead_relative_to_tripod = np.matmul(
            self.E_XYZ_to_rst, self.xyz_point_lead_relative_to_tripod
        )
        self.rst_point_msg_relative_to_tripod = np.matmul(
            self.E_XYZ_to_rst, self.xyz_point_msg_relative_to_tripod
        )
        self.rst_velocity_msg_relative_to_tripod = np.matmul(
            self.E_XYZ_to_rst, self.xyz_velocity_msg_relative_to_tripod
        )

        
        omega = (
            axis_ptz_utilities.cross(
                self.rst_point_lead_relative_to_tripod,
                self.rst_velocity_msg_relative_to_tripod,
            )
            / axis_ptz_utilities.norm(self.rst_point_lead_relative_to_tripod) ** 2
        )

        # The original code use the position when the last MSG was received, not the position with leading, giving this a try
        # omega = (
        #     axis_ptz_utilities.cross(
        #         self.rst_point_msg_relative_to_tripod,
        #         self.rst_velocity_msg_relative_to_tripod,
        #     )
        #     / axis_ptz_utilities.norm(self.rst_point_msg_relative_to_tripod) ** 2
        # )
        
        self.rho_rate = math.degrees(-omega[2])
        self.tau_rate = math.degrees(omega[0])
        self.no_derivative = False

        self.object_lock.release()

        # Compute object slew rate
        # omega = (
        #     axis_ptz_utilities.cross(self.r_rst_o_0_t, self.v_rst_o_0_t)
        #     / axis_ptz_utilities.norm(self.r_rst_o_0_t) ** 2
        # )
