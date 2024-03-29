import logging
import axis_ptz_utilities
import camera
import numpy as np
import math
import quaternion

class Object:
    def __init__(self,
                 object_id,
                 camera: camera,
                 include_age=True):
        self.object_id = object_id
        self.msg_longitude = 0
        self.msg_latitude = 0
        self.msg_altitude = 0
        self.msg_timestamp = 0
        self.lead_time = 0
        self.msg_track = 0
        self.msg_ground_speed = 0
        self.msg_vertical_rate = 0
        self.camera = camera
        self.include_age = include_age
        self.E_XYZ_to_ENz = np.zeros((3, 3))

    def update_from_msg(self, msg):
        """
        Update the object's position and timestamp from a message.
        """
        self.msg_longitude = msg.longitude
        self.msg_latitude = msg.latitude
        self.msg_altitude = msg.altitude
        self.msg_timestamp = msg.timestamp
        self.msg_track = msg.track
        self.msg_ground_speed = msg.ground_speed
        self.msg_vertical_rate = msg.vertical_rate


        self.xyz_point_msg = axis_ptz_utilities.compute_r_XYZ(
            self.msg_longitude, self.msg_latitude, self.msg_altitude
        )

        # Compute the xyz point relative to the camera tripod.
        self.xyz_point_msg_relative_to_tripod = self.xyz_point_msg - self.camera.get_xyz_point()


        self.enz_point_msg_relative_to_tripod = np.matmul(self.camera.get_xyz_to_enz_transformation_matrix(), self.xyz_point_msg_relative_to_tripod)
        radian_track = math.radians(self.msg_track)
        self.enz_velocity_msg_relative_to_tripod = np.array([
            self.msg_ground_speed * math.sin(radian_track),
            self.msg_ground_speed * math.cos(radian_track),
            self.msg_vertical_rate,
        ])

        # Compute position, at time zero,
        # in the geocentric (XYZ) coordinate system of the object
        # relative to the tripod

        self.xyz_velocity_msg_relative_to_tripod = np.matmul(self.camera.get_xyz_to_enz_transformation_matrix(), self.enz_velocity_msg_relative_to_tripod)

    def recompute_location(self):
        # Assign lead time, computing and adding age of object
        # message, if enabled
        time_delta = self.lead_time  # [s] time_since_last_update 
        msg_age = 0
        if self.include_age:
            msg_age = time() - self.timestamp_o      #datetime.utcnow().timestamp() - self.timestamp_o  # [s]
            logging.debug(f"Object msg age: {msg_age} [s]")
            time_delta += msg_age


        # Compute position and velocity in the topocentric (ENz)
        # coordinate system of the object relative to the tripod at
        # time zero, and position at slightly later time one
            
        self.enz_point_now_relative_to_tripod = self.enz_point_msg_relative_to_tripod + msg_age * self.enz_velocity_msg_relative_to_tripod
        self.enz_point_lead_relative_to_tripod = self.enz_point_msg_relative_to_tripod + time_delta * self.enz_velocity_msg_relative_to_tripod

        # Compute position, at time one, and velocity, at time zero,
        # in the geocentric (XYZ) coordinate system of the object
        # relative to the tripod

        self.xyz_point_now_relative_to_tripod = np.matmul(self.camera.get_xyz_to_enz_transformation_matrix(), self.enz_point_now_relative_to_tripod)
        self.xyz_point_lead_relative_to_tripod = np.matmul(self.camera.get_xyz_to_enz_transformation_matrix(), self.enz_point_lead_relative_to_tripod)

        # Compute the distance between the object and the tripod at
        # time one
        self.distance_to_tripod3d = axis_ptz_utilities.norm(self.xyz_point_lead_relative_to_tripod)

        # Compute the object azimuth and elevation relative to the
        # tripod
        self.azm = math.degrees(math.atan2(self.xyz_point_lead_relative_to_tripod[0], self.xyz_point_lead_relative_to_tripod[1]))  # [deg]
        self.elv = math.degrees(
            math.atan2(self.xyz_point_lead_relative_to_tripod[2], axis_ptz_utilities.norm(self.xyz_point_lead_relative_to_tripod[0:2]))
        )  # [deg]

       # Compute pan and tilt to point the camera at the object
        self.uvw_point_now_relative_to_tripod = np.matmul(self.camera.get_xyz_to_uvw_transformation_matrix(), self.xyz_point_now_relative_to_tripod)
        self.uvw_point_lead_relative_to_tripod = np.matmul(self.camera.get_xyz_to_uvw_transformation_matrix(), self.xyz_point_lead_relative_to_tripod)

        self.rho_now = math.degrees(math.atan2(self.uvw_point_now_relative_to_tripod[0], self.uvw_point_now_relative_to_tripod[1]))  # [deg]
        self.tau_now = math.degrees(
            math.atan2(self.uvw_point_now_relative_to_tripod[2], axis_ptz_utilities.norm(self.uvw_point_now_relative_to_tripod[0:2]))
        )

        self.rho_lead = math.degrees(math.atan2(self.uvw_point_lead_relative_to_tripod[0], self.uvw_point_lead_relative_to_tripod[1]))  # [deg]
        self.tau_lead = math.degrees(
            math.atan2(self.uvw_point_lead_relative_to_tripod[2], axis_ptz_utilities.norm(self.uvw_point_lead_relative_to_tripod[0:2]))
        )

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
            self.rho_lead,
            self.tau_lead,
        )

        self.rst_point_now_relative_to_tripod = np.matmul(self.E_XYZ_to_rst, self.xyz_point_now_relative_to_tripod)
        self.rst_point_lead_relative_to_tripod = np.matmul(self.E_XYZ_to_rst, self.xyz_point_lead_relative_to_tripod)
        self.rst_point_msg_relative_to_tripod = np.matmul(self.E_XYZ_to_rst, self.xyz_point_msg_relative_to_tripod)
        self.rst_velocity_msg_relative_to_tripod = np.matmul(self.E_XYZ_to_rst, self.xyz_velocity_msg_relative_to_tripod)

        # Compute object slew rate relative to the camera
        # omega = (
        #     axis_ptz_utilities.cross(self.rst_point_lead_relative_to_tripod, self.rst_velocity_msg_relative_to_tripod)
        #     / axis_ptz_utilities.norm(self.rst_point_lead_relative_to_tripod) ** 2
        # )

        omega = (
            axis_ptz_utilities.cross(self.rst_point_now_relative_to_tripod, self.rst_velocity_msg_relative_to_tripod)
            / axis_ptz_utilities.norm(self.rst_point_now_relative_to_tripod) ** 2
        )

        self.rho_rate = math.degrees(-omega[2])
        self.tau_rate = math.degrees(omega[0])

        # Compute object slew rate
        # omega = (
        #     axis_ptz_utilities.cross(self.r_rst_o_0_t, self.v_rst_o_0_t)
        #     / axis_ptz_utilities.norm(self.r_rst_o_0_t) ** 2
        # )

