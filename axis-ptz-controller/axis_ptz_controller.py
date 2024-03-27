"""Defines the AxisPtzController child class of BaseMQTTPubSub, and a
method for making AxisPtzController instances. Instatiates an
AxisPtzController, and executes its main() method when run as a
module.
"""

import ast
from datetime import datetime
import json
import logging
import threading
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
from time import sleep, time
import traceback
from types import FrameType
from typing import Any, Dict, Optional, Union
from enum import Enum
import threading

import numpy as np
import quaternion
import paho.mqtt.client as mqtt
import schedule

from base_mqtt_pub_sub import BaseMQTTPubSub
import axis_ptz_utilities
from camera_configuration import CameraConfiguration
from camera_control import CameraControl

class Status(Enum):
    SLEEPING = 0
    SLEWING = 1
    TRACKING = 2

class AxisPtzController(BaseMQTTPubSub):
    """Point the camera at an object using a proportional rate
    controller, and capture images while in track."""

    def __init__(
        self,
        hostname: str,
        camera_ip: str,
        camera_user: str,
        camera_password: str,
        config_topic: str,
        orientation_topic: str,
        object_topic: str,
        image_filename_topic: str,
        image_capture_topic: str,
        manual_control_topic: str,
        logger_topic: str,
        lambda_t: float = 0.0,
        varphi_t: float = 0.0,
        h_t: float = 0.0,
        heartbeat_interval: int = 10,
        loop_interval: float = 0.1,
        capture_interval: int = 2,
        capture_dir: str = ".",
        tracking_interval: float = 1.0,
        tripod_yaw: float = 0.0,
        tripod_pitch: float = 0.0,
        tripod_roll: float = 0.0,
        pan_gain: float = 0.2,
        pan_rate_max: float = 150.0,
        tilt_gain: float = 0.2,
        tilt_rate_max: float = 150.0,
        zoom: int = 6000,
        focus: int = 8749,
        focus_min: int = 7499,
        focus_max: int = 9999,
        hyperfocal_distance: float = 22500.0,
        jpeg_resolution: str = "1920x1080",
        jpeg_compression: int = 5,
        use_mqtt: bool = True,
        use_camera: bool = True,
        auto_focus: bool = False,
        include_age: bool = True,
        log_to_mqtt: bool = False,
        log_level: str = "INFO",
        continue_on_exception: bool = False,
        **kwargs: Any,
    ):
        """Instantiate the PTZ controller by connecting to the camera
        and message broker, and initializing data attributes.

        Parameters
        ----------
        hostname (str): Name of host
        camera_ip: str
            Camera IP address
        camera_user: str
            Camera user name
        camera_password: str
            Camera user password
        config_topic: str
            MQTT topic for subscribing to configuration messages
        orientation_topic: str
            MQTT topic for subscribing to orientation messages
        object_topic: str
            MQTT topic for subscribing to object messages
        image_filename_topic: str
            MQTT topic for publising image filenames
        image_capture_topic: str
            MQTT topic for publising image metadata
        manual_control_topic: str
            MQTT topic for subscribing to manual control messages
        logger_topic: str
            MQTT topic for publishing logger messages
        lambda_t: float
            Tripod geodetic longitude [deg]
        varphi_t: float = 0.0,
            Tripod geodetic latitude [deg]
        h_t: float = 0.0,
            Tripod geodetic altitude [deg]
        heartbeat_interval: int
            Interval at which heartbeat message is to be published [s]
        loop_interval: float
            Interval at which pointing of the camera is computed [s]
        capture_interval: int
            Interval at which the camera image is captured [s]
        capture_dir: str
            Directory in which to place captured images
        tracking_interval: float
            Lead time used when computing camera pointing to the
            object [s]
        tripod_yaw: float
            Yaw angle of camera tripod from level North [degrees]
        tripod_pitch: float
            Pitch angle of camera tripod from level North [degrees]
        tripod_roll: float
            Roll angle of camera tripod from level North [degrees]
        pan_gain: float
            Proportional control gain for pan error [1/s]
        pan_rate_max: float
            Camera pan rate maximum [deg/s]
        tilt_gain: float
            Proportional control gain for tilt error [1/s]
        tilt_rate_max: float
            Camera tilt rate maximum [deg/s]
        zoom: int
            Camera zoom level [0-9999]
        focus: int
            Camera focus level [7499-9999]
        focus_min: int
            Focus minimum from settings
        focus_max: int
             Focus maximum from settings
        hyperfocal_distance: float
             Distance at or above which focus setting remains minimum [m]
        jpeg_resolution: str
            Image capture resolution, for example, "1920x1080"
        jpeg_compression: int
            Image compression: 0 to 100
        use_mqtt: bool
            Flag to use MQTT, or not
        use_camera: bool
            Flag to use camera configuration and control, or not
        auto_focus: bool
            Flag to auto focus, or not
        include_age: bool
            Flag to include object message age in lead time, or not
        log_to_mqtt: bool
            Flag to publish logger messages to MQTT, or not
        log_level (str): One of 'NOTSET', 'DEBUG', 'INFO', 'WARN',
            'WARNING', 'ERROR', 'FATAL', 'CRITICAL'
        continue_on_exception: bool
            Continue on unhandled exceptions if True, raise exception
            if False (the default)

        Returns
        -------
        AxisPtzController
        """
        # Parent class handles kwargs, including MQTT IP
        super().__init__(**kwargs)
        self.hostname = hostname
        self.camera_ip = camera_ip
        self.camera_user = camera_user
        self.camera_password = camera_password
        self.config_topic = config_topic
        self.orientation_topic = orientation_topic
        self.object_topic = object_topic
        self.image_filename_topic = image_filename_topic
        self.image_capture_topic = image_capture_topic
        self.manual_control_topic = manual_control_topic
        self.logger_topic = logger_topic
        self.lambda_t = lambda_t
        self.varphi_t = varphi_t
        self.h_t = h_t
        self.heartbeat_interval = heartbeat_interval
        self.loop_interval = loop_interval
        self.capture_interval = capture_interval
        self.capture_dir = capture_dir
        self.tracking_interval = tracking_interval
        self.alpha = tripod_yaw
        self.beta = tripod_pitch
        self.gamma = tripod_roll
        self.pan_gain = pan_gain
        self.pan_rate_max = pan_rate_max
        self.tilt_gain = tilt_gain
        self.tilt_rate_max = tilt_rate_max
        self.zoom = zoom
        self.focus = focus
        self.focus_min = focus_min
        self.focus_max = focus_max
        self.hyperfocal_distance = hyperfocal_distance
        self.jpeg_resolution = jpeg_resolution
        self.jpeg_compression = jpeg_compression
        self.use_mqtt = use_mqtt
        self.use_camera = use_camera
        self.auto_focus = auto_focus
        self.include_age = include_age
        self.log_to_mqtt = log_to_mqtt
        self.log_level = log_level
        self.continue_on_exception = continue_on_exception

        # Always construct camera configuration and control since
        # instantiation only assigns arguments
        logging.info("Constructing camera configuration and control")
        self.camera_configuration = CameraConfiguration(
            self.camera_ip, self.camera_user, self.camera_password
        )
        self.camera_control = CameraControl(
            self.camera_ip, self.camera_user, self.camera_password
        )

        # Connect MQTT client
        if self.use_mqtt:
            logging.info("Connecting MQTT client")
            self.connect_client()
            sleep(1)
            self.publish_registration("PTZ Controller Module Registration")

        # Object id, timestamp of object message and corresponding
        # object longitude, latitude, and altitude, and position and
        # velocity relative to the tripod in the camera fixed (rst)
        # coordinate system
        self.object_id = "NA"
        self.timestamp_o = 0.0  # [s]
        self.lambda_o = 0.0  # [deg]
        self.varphi_o = 0.0  # [deg]
        self.h_o = 0.0  # [m]
        self.r_rst_o_0_t = np.zeros((3,))  # [m/s]
        self.r_rst_o_1_t = np.zeros((3,))  # [m/s]
        self.v_rst_o_0_t = np.zeros((3,))  # [m/s]

        # Tripod yaw, pitch, and roll rotation quaternions
        self.q_alpha = quaternion.quaternion()
        self.q_beta = quaternion.quaternion()
        self.q_gamma = quaternion.quaternion()

        # Tripod position in the geocentric (XYZ) coordinate system
        self.r_XYZ_t = np.zeros((3,))

        # Compute orthogonal transformation matrix from geocentric
        # (XYZ) to topocentric (ENz) coordinates
        self.E_XYZ_to_ENz = np.zeros((3, 3))
        self.e_E_XYZ = np.zeros((3,))
        self.e_N_XYZ = np.zeros((3,))
        self.e_z_XYZ = np.zeros((3,))

        # Orthogonal transformation matrix from geocentric (XYZ) to
        # camera housing fixed (uvw) coordinates
        self.E_XYZ_to_uvw = np.zeros((3, 3))

        # Position and velocity in the topocentric (ENz) coordinate
        # system of the object relative to the tripod at time zero
        self.r_ENz_o_0_t = np.zeros((3,))
        self.v_ENz_o_0_t = np.zeros((3,))

        # Position and velocity in the geocentric (XYZ) coordinate
        # system of the object relative to the tripod at time zero
        self.r_XYZ_o_0_t = np.zeros((3,))
        self.r_XYZ_o_1_t = np.zeros((3,))
        self.v_XYZ_o_0_t = np.zeros((3,))

        # Distance between the object and the tripod at time one
        self.distance3d = 0.0  # [m]

        # Object azimuth and elevation angles
        self.azm_o = 0.0  # [deg]
        self.elv_o = 0.0  # [deg]

        # Object pan and tilt angles
        self.rho_o = 0.0  # [deg]
        self.tau_o = 0.0  # [deg]

        # Pan, and tilt rotation quaternions
        self.q_row = quaternion.quaternion()
        self.q_tau = quaternion.quaternion()

        # Orthogonal transformation matrix from camera housing (uvw)
        # to camera fixed (rst) coordinates
        self.E_XYZ_to_rst = np.zeros((3, 3))

        # Object pan and tilt rates
        self.rho_dot_o = 0.0  # [deg/s]
        self.tau_dot_o = 0.0  # [deg/s]

        # Time of pointing update, camera pan and tilt angles
        self.timestamp_c = 0.0  # [s]
        self.rho_c = 0.0  # [deg]
        self.tau_c = 0.0  # [deg]

        # Delta between camera and object pan and tilt angles
        self.delta_rho = 0.0  # [deg]
        self.delta_tau = 0.0  # [deg]

        # Camera pan and tilt rates
        self.rho_dot_c = 0.0  # [deg/s]
        self.tau_dot_c = 0.0  # [deg/s]

        # Camera pan and tilt rate differences
        self.delta_rho_dot_c = 0.0  # [deg/s]
        self.delta_tau_dot_c = 0.0  # [deg/s]
        self.pan_rate_index = 0
        self.tilt_rate_index = 0

        # Set the status for the controller
        self.status = Status.SLEEPING

        # Object track, ground speed, and vertical rate
        self.track_o = 0.0  # [deg]
        self.ground_speed_o = 0.0  # [m/s]
        self.vertical_rate_o = 0.0  # [m/s]

        # Lock to make sure object info is not used while being updated
        self.object_lock = threading.Lock()

        # Camera focus parameters. Note that the focus setting is
        # minimum at and greater than the hyperfocal distance, and the
        # focus setting is maximum when the distance is zero.
        self.focus_slope = (self.focus_min - self.focus_max) / self.hyperfocal_distance
        self.focus_intercept = self.focus_max

        # Capture boolean and last capture time
        self.do_capture = False
        self.capture_time = 0.0

        # Initialize tripod position in the geocentric (XYZ)
        # coordinate system, orthogonal transformation matrix from
        # geocentric (XYZ) to topocentric (ENz) coordinates, and East,
        # North, and zenith unit vectors
        config_msg = self.generate_payload_json(
            push_timestamp=int(datetime.utcnow().timestamp()),
            device_type=os.environ.get("DEVICE_TYPE", "Collector"),
            id_=self.hostname,
            deployment_id=os.environ.get(
                "DEPLOYMENT_ID", f"Unknown-Location-{self.hostname}"
            ),
            current_location=os.environ.get("CURRENT_LOCATION", "-90, -180"),
            status="Debug",
            message_type="Event",
            model_version="null",
            firmware_version="v0.0.0",
            data_payload_type="Configuration",
            data_payload=json.dumps(
                {
                    "axis-ptz-controller": {
                        "tripod_longitude": self.lambda_t,
                        "tripod_latitude": self.varphi_t,
                        "tripod_altitude": self.h_t,
                    }
                }
            ),
        )
        self._config_callback(None, None, config_msg)

        os.makedirs(self.capture_dir, exist_ok=True)
        # Initialize the rotations from the geocentric (XYZ)
        # coordinate system to the camera housing fixed (uvw)
        # coordinate system
        orientation_msg = self.generate_payload_json(
            push_timestamp=int(datetime.utcnow().timestamp()),
            device_type=os.environ.get("DEVICE_TYPE", "Collector"),
            id_=self.hostname,
            deployment_id=os.environ.get(
                "DEPLOYMENT_ID", f"Unknown-Location-{self.hostname}"
            ),
            current_location=os.environ.get("CURRENT_LOCATION", "-90, -180"),
            status="Debug",
            message_type="Event",
            model_version="null",
            firmware_version="v0.0.0",
            data_payload_type="Orientation",
            data_payload=json.dumps(
                {
                    "tripod_yaw": self.alpha,
                    "tripod_pitch": self.beta,
                    "tripod_roll": self.gamma,
                }
            ),
        )
        self._orientation_callback(None, None, orientation_msg)

        # Initialize camera pointing
        if self.use_camera:
            if self.auto_focus:
                logging.info(
                    f"Absolute move to pan: {self.rho_c}, and tilt: {self.tau_c}, with zoom: {self.zoom}"
                )
                try:
                    self.camera_control.absolute_move(
                        self.rho_c, self.tau_c, self.zoom, 99
                    )
                except Exception as e:
                    logging.error(f"Error: {e}")
            else:
                logging.info(
                    f"Absolute move to pan: {self.rho_c}, and tilt: {self.tau_c}, with zoom: {self.zoom}, and focus: {self.focus}"
                )
                try:
                    self.camera_control.absolute_move(
                        self.rho_c, self.tau_c, self.zoom, 99, self.focus
                    )
                except Exception as e:
                    logging.error(f"Error: {e}")

        # Log configuration parameters
        self._log_config()

    def decode_payload(
        self, msg: Union[mqtt.MQTTMessage, str], data_payload_type: str
    ) -> Dict[Any, Any]:
        """
        Decode the payload carried by a message.

        Parameters
        ----------
        payload: mqtt.MQTTMessage
            The MQTT message
        data_payload_type: str
            The data payload type

        Returns
        -------
        data : Dict[Any, Any]
            The data payload of the message payload
        """
        if type(msg) == mqtt.MQTTMessage:
            payload = msg.payload.decode()
        else:
            payload = msg
        try:
            json_payload = json.loads(payload)
            data_payload = json_payload[data_payload_type]
        except (KeyError, TypeError) as e:
            logging.error(f"Error: {e}")
            logging.error(json_payload)
            logging.error(
                f"Data payload type: {data_payload_type} not found in payload: {data_payload}"
            )
            return {}
        return json.loads(data_payload)

    def _config_callback(
        self,
        _client: Union[mqtt.Client, None],
        _userdata: Union[Dict[Any, Any], None],
        msg: Union[mqtt.MQTTMessage, str],
    ) -> None:
        """
        Process configuration message.

        Parameters
        ----------
        _client: Union[mqtt.Client, None]
            MQTT client
        _userdata: Union[Dict[Any, Any], None]
            Any required user data
        msg: Union[mqtt.MQTTMessage, Dict[Any, Any]]
            An MQTT message, or dictionary

        Returns
        -------
        None
        """
        # Assign data attributes allowed to change during operation,
        # ignoring config message data without a "axis-ptz-controller"
        # key
        data = self.decode_payload(msg, "Configuration")
        if "axis-ptz-controller" not in data:
            logging.info(
                f"Configuration message data missing axis-ptz-controller: {data}"
            )
            return
        logging.info(f"Processing config msg data: {data}")
        config = data["axis-ptz-controller"]
        self.camera_ip = config.get("camera_ip", self.camera_ip)
        self.camera_user = config.get("camera_user", self.camera_user)
        self.camera_password = config.get("camera_password", self.camera_password)
        self.config_topic = config.get("config_topic", self.config_topic)
        self.orientation_topic = config.get("orientation_topic", self.orientation_topic)
        self.object_topic = config.get("object_topic", self.object_topic)
        self.image_filename_topic = config.get(
            "image_filename_topic", self.image_filename_topic
        )
        self.image_capture_topic = config.get(
            "image_capture_topic", self.image_capture_topic
        )
        self.logger_topic = config.get("logger_topic", self.logger_topic)
        self.lambda_t = config.get("tripod_longitude", self.lambda_t)  # [deg]
        self.varphi_t = config.get("tripod_latitude", self.varphi_t)  # [deg]
        self.h_t = config.get("tripod_altitude", self.h_t)  # [m]
        self.heartbeat_interval = config.get(
            "heartbeat_interval", self.heartbeat_interval
        )
        self.loop_interval = config.get("loop_interval", self.loop_interval)  # [s]
        self.capture_interval = config.get(
            "capture_interval", self.capture_interval
        )  # [s]
        self.capture_dir = config.get("capture_dir", self.capture_dir)
        self.tracking_interval = config.get("tracking_interval", self.tracking_interval)  # [s]
        # Cannot set tripod yaw, pitch, and roll because of side effects
        self.pan_gain = config.get("pan_gain", self.pan_gain)  # [1/s]
        self.pan_rate_max = config.get("pan_rate_max", self.pan_rate_max)
        self.tilt_gain = config.get("tilt_gain", self.tilt_gain)  # [1/s]
        self.tilt_rate_max = config.get("tilt_rate_max", self.tilt_rate_max)
        self.zoom = config.get("zoom", self.zoom)  # [0-9999]
        self.focus = config.get("focus", self.focus)  # [7499-9999]
        self.focus_min = config.get("focus_min", self.focus_min)
        self.focus_max = config.get("focus_max", self.focus_max)
        self.hyperfocal_distance = config.get(
            "hyperfocal_distance", self.hyperfocal_distance
        )
        self.jpeg_resolution = config.get("jpeg_resolution", self.jpeg_resolution)
        self.jpeg_compression = config.get("jpeg_compression", self.jpeg_compression)
        self.use_mqtt = config.get("use_mqtt", self.use_mqtt)
        self.use_camera = config.get("use_camera", self.use_camera)
        self.auto_focus = config.get("auto_focus", self.auto_focus)
        self.include_age = config.get("include_age", self.include_age)
        self.log_to_mqtt = config.get("log_to_mqtt", self.log_to_mqtt)
        self.log_level = config.get("log_level", self.log_level)
        self.continue_on_exception = config.get(
            "continue_on_exception", self.continue_on_exception
        )
        self.camera_control.absolute_move(None, None, self.zoom, None)

        # Log configuration parameters
        self._log_config()

        # Compute tripod position in the geocentric (XYZ) coordinate
        # system
        self.r_XYZ_t = axis_ptz_utilities.compute_r_XYZ(
            self.lambda_t, self.varphi_t, self.h_t
        )

        # Compute orthogonal transformation matrix from geocentric
        # (XYZ) to topocentric (ENz) coordinates
        (
            self.E_XYZ_to_ENz,
            self.e_E_XYZ,
            self.e_N_XYZ,
            self.e_z_XYZ,
        ) = axis_ptz_utilities.compute_E_XYZ_to_ENz(self.lambda_t, self.varphi_t)

    def _log_config(self) -> None:
        """Logs all paramters that can be set on construction."""
        config = {
            "hostname": self.hostname,
            "camera_ip": self.camera_ip,
            "camera_user": self.camera_user,
            "camera_password": self.camera_password,
            "config_topic": self.config_topic,
            "orientation_topic": self.orientation_topic,
            "object_topic": self.object_topic,
            "image_filename_topic": self.image_filename_topic,
            "image_capture_topic": self.image_capture_topic,
            "logger_topic": self.logger_topic,
            "lambda_t": self.lambda_t,
            "varphi_t": self.varphi_t,
            "h_t": self.h_t,
            "heartbeat_interval": self.heartbeat_interval,
            "loop_interval": self.loop_interval,
            "capture_interval": self.capture_interval,
            "capture_dir": self.capture_dir,
            "tracking_interval": self.tracking_interval,
            "tripod_yaw": self.alpha,
            "tripod_pitch": self.beta,
            "tripod_roll": self.gamma,
            "pan_gain": self.pan_gain,
            "pan_rate_max": self.pan_rate_max,
            "tilt_gain": self.tilt_gain,
            "tilt_rate_max": self.tilt_rate_max,
            "zoom": self.zoom,
            "focus": self.focus,
            "focus_min": self.focus_min,
            "focus_max": self.focus_max,
            "hyperfocal_distance": self.hyperfocal_distance,
            "jpeg_resolution": self.jpeg_resolution,
            "jpeg_compression": self.jpeg_compression,
            "use_mqtt": self.use_mqtt,
            "use_camera": self.use_camera,
            "auto_focus": self.auto_focus,
            "include_age": self.include_age,
            "log_to_mqtt": self.log_to_mqtt,
            "log_level": self.log_level,
            "continue_on_exception": self.continue_on_exception,
        }
        logging.info(
            f"AxisPtzController configuration:\n{json.dumps(config, indent=4)}"
        )



    def _compute_object_pointing(self, time_since_last_update=0) -> None:
        # Compute position in the geocentric (XYZ) coordinate system
        # of the object relative to the tripod at time zero, the
        # observation time
        r_XYZ_o_0 = axis_ptz_utilities.compute_r_XYZ(
            self.lambda_o, self.varphi_o, self.h_o
        )
        self.r_XYZ_o_0_t = r_XYZ_o_0 - self.r_XYZ_t

        # Assign lead time, computing and adding age of object
        # message, if enabled
        lead_time = self.tracking_interval  # [s] time_since_last_update 
        if self.include_age:
            object_msg_age = time() - self.timestamp_o      #datetime.utcnow().timestamp() - self.timestamp_o  # [s]
            logging.debug(f"Object msg age: {object_msg_age} [s]")
            lead_time += object_msg_age
        

        # Compute position and velocity in the topocentric (ENz)
        # coordinate system of the object relative to the tripod at
        # time zero, and position at slightly later time one
        self.r_ENz_o_0_t = np.matmul(self.E_XYZ_to_ENz, self.r_XYZ_o_0_t)
        radian_track_o = math.radians(self.track_o)
        self.v_ENz_o_0_t = np.array(
            [
                self.ground_speed_o * math.sin(radian_track_o),
                self.ground_speed_o * math.cos(radian_track_o),
                self.vertical_rate_o,
            ]
        )
        r_ENz_o_1_t = self.r_ENz_o_0_t + self.v_ENz_o_0_t * lead_time

        # Compute position, at time one, and velocity, at time zero,
        # in the geocentric (XYZ) coordinate system of the object
        # relative to the tripod
        self.r_XYZ_o_1_t = np.matmul(self.E_XYZ_to_ENz.transpose(), r_ENz_o_1_t)
        self.v_XYZ_o_0_t = np.matmul(self.E_XYZ_to_ENz.transpose(), self.v_ENz_o_0_t)

        # Compute the distance between the object and the tripod at
        # time one
        self.distance3d = axis_ptz_utilities.norm(r_ENz_o_1_t)

        # TODO: Restore?
        # Compute the distance between the object and the tripod
        # along the surface of a spherical Earth
        # distance2d = axis_ptz_utilities.compute_great_circle_distance(
        #     self.self.lambda_t,
        #     self.varphi_t,
        #     self.lambda_o,
        #     self.varphi_o,
        # )  # [m]

        # Compute the object azimuth and elevation relative to the
        # tripod
        self.azm_o = math.degrees(math.atan2(r_ENz_o_1_t[0], r_ENz_o_1_t[1]))  # [deg]
        self.elv_o = math.degrees(
            math.atan2(r_ENz_o_1_t[2], axis_ptz_utilities.norm(r_ENz_o_1_t[0:2]))
        )  # [deg]
        logging.debug(f"Object azimuth and elevation: {self.azm_o}, {self.elv_o} [deg]")

        # Compute pan and tilt to point the camera at the object
        r_uvw_o_1_t = np.matmul(self.E_XYZ_to_uvw, self.r_XYZ_o_1_t)
        self.rho_o = math.degrees(math.atan2(r_uvw_o_1_t[0], r_uvw_o_1_t[1]))  # [deg]
        self.tau_o = math.degrees(
            math.atan2(r_uvw_o_1_t[2], axis_ptz_utilities.norm(r_uvw_o_1_t[0:2]))
        )  # [deg]
        logging.debug(f"Object pan and tilt: {self.rho_o}, {self.tau_o} [deg]")
        logging.debug(f"\t🔭\tObject - Pan: {self.rho_o}\t Obj ENz 1t: {r_ENz_o_1_t}\t Obj ENz 0t: {self.r_ENz_o_0_t}\t Obj Velo 0t: {self.v_ENz_o_0_t}\t Lead time: {lead_time}")

    def _track_object(self, time_since_last_update) -> None:

        if self.status != Status.TRACKING:
            return
        
        # Make sure Object info is not updated while pointing is being computed
        self.object_lock.acquire()

        start_time = time()
        self._compute_object_pointing(time_since_last_update)

        if self.use_camera:
            # Get camera pan and tilt
            self.rho_c, self.tau_c, _zoom, _focus = self.camera_control.get_ptz()
            # logging.info(
            #     f"Camera pan, tilt, zoom, and focus: {self.rho_c} [deg], {self.tau_c} [deg], {_zoom}, {_focus}"
            # )
        else:
            logging.debug(f"Controller pan and tilt: {self.rho_c}, {self.tau_c} [deg]")


        # Compute angle delta between camera and object pan and tilt
        self.delta_rho = self._compute_angle_delta(self.rho_c, self.rho_o)
        self.delta_tau = self._compute_angle_delta(self.tau_c, self.tau_o)

        # Compute slew rate differences
        self.delta_rho_dot_c = self.pan_gain * self.delta_rho
        self.delta_tau_dot_c = self.tilt_gain * self.delta_tau

        logging.debug(
            f"Delta pan and tilt rates: {self.delta_rho_dot_c}, {self.delta_tau_dot_c} [deg/s]"
        )
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
            self.alpha,
            self.beta,
            self.gamma,
            self.rho_o,
            self.tau_o,
        )
        self.r_rst_o_1_t = np.matmul(self.E_XYZ_to_rst, self.r_XYZ_o_1_t)
        self.r_rst_o_0_t = np.matmul(self.E_XYZ_to_rst, self.r_XYZ_o_0_t)
        self.v_rst_o_0_t = np.matmul(self.E_XYZ_to_rst, self.v_XYZ_o_0_t)

        # Compute object slew rate
        omega = (
            axis_ptz_utilities.cross(self.r_rst_o_0_t, self.v_rst_o_0_t)
            / axis_ptz_utilities.norm(self.r_rst_o_0_t) ** 2
        )
        # omega = (
        #     axis_ptz_utilities.cross(self.r_rst_o_1_t, self.v_rst_o_0_t)
        #     / axis_ptz_utilities.norm(self.r_rst_o_1_t) ** 2
        # )
        self.rho_dot_o = math.degrees(-omega[2])
        self.tau_dot_o = math.degrees(omega[0])
        logging.debug(
            f"Object pan and tilt rates: {self.rho_dot_o}, {self.tau_dot_o} [deg/s]"
        )

        # Update camera pan and tilt rate
        self.rho_dot_c = self.rho_dot_o + self.delta_rho_dot_c
        self.tau_dot_c = self.tau_dot_o + self.delta_tau_dot_c
        # logging.info(
        #     f"Camera pan and tilt rates: {self.rho_dot_c}, {self.tau_dot_c} [deg/s]"
        # )

        # Get, or compute and set focus, command camera pan and tilt
        # rates, and begin capturing images, if needed
        if self.use_camera:
            if not self.auto_focus:
                # Note that focus cannot be negative, since distance3d
                # is non-negative
                self.focus = int(
                    max(
                        self.focus_min,
                        self.focus_slope * self.distance3d + self.focus_intercept,
                    )
                )
                logging.debug(f"Commanding focus: {self.focus}")
                try:
                    self.camera_control.set_focus(self.focus)
                except Exception as e:
                    logging.error(f"Error: {e}")

            self._compute_pan_rate_index(self.rho_dot_c)
            self._compute_tilt_rate_index(self.tau_dot_c)
            # logging.info(
            #     f"Commanding pan and tilt rate indexes: {self.pan_rate_index}, {self.tilt_rate_index}"
            # )

            # All done with object info
            self.object_lock.release()

            try:
                logging.disable(logging.INFO)
                self.camera_control.continuous_move(
                    self.pan_rate_index,
                    self.tilt_rate_index,
                    0.0,
                )
                logging.disable(logging.NOTSET)

            except Exception as e:
                logging.error(f"Error: {e}")

            if not self.do_capture:
                logging.info(f"Starting image capture of object: {self.object_id}")
                self.do_capture = True
                self.capture_time = time()

            if ( self.do_capture
                and time() - self.capture_time >  self.capture_interval
            ):
                capture_thread = threading.Thread(target=self._capture_image)
                capture_thread.daemon = True
                capture_thread.start()


        #logging.info(f"\t⏱️\tRATES - 🎥 Pan: {self.rho_dot_c}\tTilt: {self.tau_dot_c}\t🛩️  Pan: {self.rho_dot_o}\tTilt: {self.tau_dot_o} ANGLES: 🎥  Pan: {self.rho_c}\tTilt: {self.tau_c}\t🛩️  Pan: {self.rho_o}\tTilt: {self.tau_o} ")
        elapsed_time = time() - start_time
        # Log camera pointing using MQTT
        if self.log_to_mqtt:
            logger_msg = self.generate_payload_json(
                push_timestamp=int(datetime.utcnow().timestamp()),
                device_type=os.environ.get("DEVICE_TYPE", "Collector"),
                id_=self.hostname,
                deployment_id=os.environ.get(
                    "DEPLOYMENT_ID", f"Unknown-Location-{self.hostname}"
                ),
                current_location=os.environ.get("CURRENT_LOCATION", "-90, -180"),
                status="Debug",
                message_type="Event",
                model_version="null",
                firmware_version="v0.0.0",
                data_payload_type="Logger",
                data_payload=json.dumps(
                    {
                        "camera-pointing": {
                            "timestamp_c": self.timestamp_c,
                            "rho_o": self.rho_o,
                            "tau_o": self.tau_o,
                            "rho_dot_o": self.rho_dot_o,
                            "tau_dot_o": self.tau_dot_o,
                            "rho_c": self.rho_c,
                            "tau_c": self.tau_c,
                            "rho_dot_c": self.rho_dot_c,
                            "tau_dot_c": self.tau_dot_c,
                            "pan_rate_index": self.pan_rate_index,
                            "tilt_rate_index": self.tilt_rate_index,
                            "delta_rho_dot_c": self.delta_rho_dot_c,
                            "delta_tau_dot_c": self.delta_tau_dot_c,
                            "delta_rho": self.delta_rho,
                            "delta_tau": self.delta_tau,
                            "tracking_loop_time": elapsed_time,
                            "time_since_last_update": time_since_last_update,
                            "object_id": self.object_id,
                        }
                    }
                ),
            )
            logging.debug(f"Publishing logger msg: {logger_msg}")
            self.publish_to_topic(self.logger_topic, logger_msg)




    def _slew_camera(self) -> None:

        if self.status == Status.SLEWING:
            logging.error("Camera is already slewing")
            return
        
        self.status = Status.SLEWING

        # Get camera pan and tilt
        self.rho_c, self.tau_c, _zoom, _focus = self.camera_control.get_ptz()
        logging.info(
            f"Camera pan, tilt, zoom, and focus: {self.rho_c} [deg], {self.tau_c} [deg], {_zoom}, {_focus}"
        )

        if self.auto_focus:
            logging.info(
                f"Absolute move to pan: {self.rho_o}, and tilt: {self.tau_o}, with zoom: {self.zoom}"
            )
            try:
                self.camera_control.absolute_move(
                    self.rho_o, self.tau_o, self.zoom, 99
                )
            except Exception as e:
                logging.error(f"Error: {e}")
        else:
            logging.info(
                f"Absolute move to pan: {self.rho_o}, and tilt: {self.tau_o}, with zoom: {self.zoom}, and focus: {self.focus}"
            )
            try:
                self.camera_control.absolute_move(
                    self.rho_o, self.tau_o, self.zoom, 99, self.focus
                )
            except Exception as e:
                logging.error(f"Error: {e}")

        duration = max(
            math.fabs(self._compute_angle_delta(self.rho_c, self.rho_o))
            / (self.pan_rate_max),
            math.fabs(self._compute_angle_delta(self.tau_c, self.tau_o))
            / (self.tilt_rate_max),
        )
        logging.info(f"Sleeping: {duration} [s]")
        sleep(duration)

        # Start Tracking
        self.status = Status.TRACKING


    def _orientation_callback(
        self,
        _client: Union[mqtt.Client, None],
        _userdata: Union[Dict[Any, Any], None],
        msg: Union[mqtt.MQTTMessage, str],
    ) -> None:
        """
        Process orientation message.

        Parameters
        ----------
        _client: Union[mqtt.Client, None]
            MQTT client
        _userdata: Union[Dict[Any, Any], None]
            Any required user data
        msg: Union[mqtt.MQTTMessage, Dict[Any, Any]]
            An MQTT message, or dictionary

        Returns
        -------
        None
        """
        # Assign camera housing rotation angles
        data = self.decode_payload(msg, "Orientation")
        logging.info(f"\t🌎\tProcessing orientation msg data: {data}")
        self.alpha = data["tripod_yaw"]  # [deg]
        self.beta = data["tripod_pitch"]  # [deg]
        self.gamma = data["tripod_roll"]  # [deg]

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
            self.alpha,
            self.beta,
            self.gamma,
            self.rho_c,
            self.tau_c,
        )

    def _reset_stop_timer(self) -> None:
        if hasattr(self, "_timer"):
            self._timer.cancel()  # type: ignore
        self._timer = threading.Timer(3, self._stop_timer_callback)
        self._timer.start()

    def _stop_timer_callback(self) -> None:
        # Call your function here
        print("Timer callback called")

        logging.info(f"Stopping image capture of object, updates timed out")
        self.do_capture = False
        self.status = Status.SLEEPING
        logging.info("Stopping continuous pan and tilt - updates timed out")
        try:
            self.camera_control.stop_move()
        except Exception as e:
            logging.error(f"Error: {e}")

            # ... existing code ...

    def _object_callback(
        self,
        _client: Union[mqtt.Client, None],
        _userdata: Union[Dict[Any, Any], None],
        msg: Union[mqtt.MQTTMessage, str],
    ) -> None:
        """
        Process object message.

        Parameters
        ----------
        _client: Union[mqtt.Client, None]
            MQTT client
        _userdata: Union[Dict[Any, Any], None]
            Any required user data
        msg: Union[mqtt.MQTTMessage, Dict[Any, Any]]
            An MQTT message, or dictionary

        Returns
        -------
        None
        """
        # Assign identifier, time, position, and velocity of the
        # object
        data = self.decode_payload(msg, "Selected Object")
        if not set(
            [
                "object_id",
                "object_type",
                "timestamp",
                "latitude",
                "longitude",
                "altitude",
                "track",
                "horizontal_velocity",
                "vertical_velocity",
            ]
        ) <= set(data.keys()):
            logging.info(f"Required keys missing from object message data: {data}")
            return
        logging.info(f"\t🗒️\tProcessing object msg data: {data['object_id']} \t {data['latitude']} \t {data['longitude']} \t {data['altitude']}")
        self.timestamp_o = float(data["timestamp"])  # [s]
        self.timestamp_c = self.timestamp_o
        self.lambda_o = data["longitude"]  # [deg]
        self.varphi_o = data["latitude"]  # [deg]
        self.h_o = data["altitude"]  # [m]
        self.track_o = data["track"]  # [deg]
        self.ground_speed_o = data["horizontal_velocity"]  # [m/s]
        self.vertical_rate_o = data["vertical_velocity"]  # [m/s]
        object_id = data["object_id"]

        if self.use_camera and self.tau_o < 0:
            logging.info(f"Stopping image capture of object: {object_id}")
            self.do_capture = False
            self.status = Status.SLEEPING
            logging.info(
                "Stopping continuous pan and tilt - Object is below the horizon"
            )
            try:
                self.camera_control.stop_move()
            except Exception as e:
                logging.error(f"Error: {e}")
        
        # Reset the stop timer because we received an object message
        self._reset_stop_timer()
        
        if self.use_camera:
            # Point the camera at any new object directly
            if self.object_id != object_id:
                self.object_id = object_id
                # Compute object pointing
                self._compute_object_pointing()
                self._slew_camera()
        else:
            logging.debug(f"Controller pan and tilt: {self.rho_c}, {self.tau_c} [deg]")

    def _manual_control_callback(
        self,
        _client: Union[mqtt.Client, None],
        _userdata: Union[Dict[Any, Any], None],
        msg: Union[mqtt.MQTTMessage, str],
    ) -> None:
        """Process manual control message.

        Parameters
        ----------
        data : Dict[str, str]
            Dictionary that maps keys to type and payload

        Returns
        -------
        None
        """
        data = self.decode_payload(msg, "Manual Control")
        if not set(["azimuth", "elevation", "zoom"]) <= set(data.keys()):
            logging.info(
                f"Required keys missing from manual control message data: {data}"
            )
            return

        azimuth = data["azimuth"]
        elevation = data["elevation"]
        self.zoom = data["zoom"]
        logging.info(f"Setting zoom to: {self.zoom}")
        # Get camera azimuth and elevation
        self.rho_c, self.tau_c, _zoom, _focus = self.camera_control.get_ptz()
        logging.info(f"Current Camera pan and tilt: {self.rho_c}, {self.tau_c} [deg]")
        logging.info(
            f"Absolute move to pan: {self.rho_o}, and tilt: {self.tau_o}, with zoom: {self.zoom}"
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

        try:
            self.camera_control.absolute_move(camera_pan, camera_tilt, self.zoom, 99)
        except Exception as e:
            logging.error(f"Error: {e}")
        logging.info(
            f"Current Camera Reading - Pan: {self.rho_c},  Tilt: {self.tau_c} \t Target - Azimuth: {azimuth} Elevation: {elevation}  \t Corrected Target - Pan: {camera_pan}, Tilt: {camera_tilt}  [deg]"
        )
        if elevation > 0 and azimuth > 0:
            duration = max(
                math.fabs(self._compute_angle_delta(self.rho_c, azimuth))
                / (self.pan_rate_max),
                math.fabs(self._compute_angle_delta(self.tau_c, elevation))
                / (self.tilt_rate_max),
            )
            logging.info(f"Sleeping: {duration} [s]")
            sleep(duration)

    def _send_data(self, data: Dict[str, str]) -> bool:
        """Leverages edgetech-core functionality to publish a JSON
        payload to the MQTT broker on the topic specified by type.

        Parameters
        ----------
        data : Dict[str, str]
            Dictionary that maps keys to type and payload

        Returns
        -------
        success : bool
            Returns True if successful publish, else False
        """
        # Generate payload as JSON
        payload = self.generate_payload_json(
            push_timestamp=int(datetime.utcnow().timestamp()),
            device_type=os.environ.get("DEVICE_TYPE", "Collector"),
            id_=self.hostname,
            deployment_id=os.environ.get(
                "DEPLOYMENT_ID", f"Unknown-Location-{self.hostname}"
            ),
            current_location=os.environ.get("CURRENT_LOCATION", "-90, -180"),
            status="Debug",
            message_type="Event",
            model_version="null",
            firmware_version="v0.0.0",
            data_payload_type=data["type"],
            data_payload=data["payload"],
        )

        # Publish payload to the topic selected by type
        if data["type"] == "ImageFileName":
            topic = self.image_filename_topic

        elif data["type"] == "ImageMetadata":
            topic = self.image_capture_topic

        success = self.publish_to_topic(topic, payload)
        if not success:
            logging.error(f"Failed to publish data on channel {topic}: {data}")

        return success

    def _compute_angle_delta(self, theta_c: float, theta_o: float) -> float:
        """Given the angle of the camera and object in a domain of
        width 360 degrees, determine the angle delta, that is the
        smallest difference in angle, signed according to the sign of
        the angular rate required to bring the angle of the camera
        toward the angle of the object.

        Parameters
        ----------
        theta_c : float
            Pan or tilt of the camera [deg]
        theta_o : float
            Pan or tilt of the object [deg]

        Returns
        -------
        float
            Angle delta [deg]
        """
        theta_c = math.radians(theta_c)
        theta_o = math.radians(theta_o)
        d = math.cos(theta_c) * math.cos(theta_o) + math.sin(theta_c) * math.sin(
            theta_o
        )
        c = math.cos(theta_c) * math.sin(theta_o) - math.sin(theta_c) * math.cos(
            theta_o
        )
        if math.fabs(c) == 0:
            logging.info(
                f"theta_c: {theta_c}, theta_o: {theta_o}, d: {d}, c: {c}, math.fabs(c): {math.fabs(c)}"
            )
            return 0
        return math.degrees(math.acos(d)) * c / math.fabs(c)

    def _compute_pan_rate_index(self, rho_dot: float):
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
        if rho_dot < -self.pan_rate_max:
            self.pan_rate_index = -100

        elif self.pan_rate_max < rho_dot:
            self.pan_rate_index = +100

        else:
            self.pan_rate_index = round((100 / self.pan_rate_max) * rho_dot)


    def _compute_tilt_rate_index(self, tau_dot: float):
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
        if tau_dot < -self.tilt_rate_max:
            self.tilt_rate_index = -100

        elif self.tilt_rate_max < tau_dot:
            self.tilt_rate_index = 100

        else:
            self.tilt_rate_index = round((100 / self.tilt_rate_max) * tau_dot)

    def _capture_image(self) -> None:
        """When enabled, capture an image in JPEG format, and publish
        corresponding image metadata.

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        if self.do_capture:
            # Capture an image in JPEG format and publish it's
            # filename
            self.capture_time = time()
            datetime_c = datetime.utcnow()
            timestr = datetime_c.strftime("%Y-%m-%d-%H-%M-%S")
            image_filepath = Path(self.capture_dir) / "{}_{}_{}_{}_{}.jpg".format(
                self.object_id,
                int(self.azm_o) % 360,
                int(self.elv_o),
                int(self.distance3d),
                timestr,
            )
            logging.info(
                f"\t📸\tCapturing image of object: {self.object_id}, at: {self.capture_time}, in: {image_filepath}"
            )
            # TODO: Update camera configuration to make renaming the
            # file unnecessary
            with tempfile.TemporaryDirectory() as d:
                with axis_ptz_utilities.pushd(d):
                    try:
                        text = self.camera_configuration.get_jpeg_request(
                            resolution=self.jpeg_resolution,
                            compression=self.jpeg_compression,
                        )
                        logging.debug(f"Camera configuration response: {text}")
                        shutil.move(list(Path(d).glob("*.jpg"))[0], image_filepath)
                    except Exception as e:
                        logging.error(f"Could not capture image to directory: {d}: {e}")
                        return
            logging.debug(
                f"Publishing filename: {image_filepath}, for object: {self.object_id}, at: {self.capture_time}"
            )
            self._send_data(
                {
                    "type": "ImageFileName",
                    "payload": str(image_filepath),
                }
            )

            # Populate and publish image metadata, getting current pan
            # and tilt, and accounting for object message age relative
            # to the image capture
            try:
                rho_c, tau_c, _zoom, _focus = self.camera_control.get_ptz()
            except Exception as e:
                logging.error(f"Error: {e}")
                return

            object_msg_age = datetime_c.timestamp() - self.timestamp_o  # [s]
            image_metadata = {
                "timestamp": timestr,
                "imagefile": str(image_filepath),
                "camera": {
                    "rho_c": rho_c,
                    "tau_c": tau_c,
                    "lambda_t": self.lambda_t,
                    "varphi_t": self.varphi_t,
                    "zoom": self.zoom,
                },
                "object": {
                    "object_msg_age": object_msg_age,
                    "r_ENz_o_0_t": self.r_ENz_o_0_t.tolist(),
                    "v_ENz_o_0_t": self.v_ENz_o_0_t.tolist(),
                    "delta_rho": self.delta_rho,
                    "delta_tau": self.delta_tau,
                },
            }
            logging.debug(
                f"Publishing metadata: {image_metadata}, for object: {self.object_id}, at: {self.capture_time}"
            )
            self._send_data(
                {
                    "type": "ImageMetadata",
                    "payload": json.dumps(image_metadata),
                }
            )

    def _update_pointing(self) -> None:
        """Update values of camera pan and tilt using current pan and
        tilt rate. Note that these value likely differ slightly from
        the actual camera pan and tilt angles, and will be overwritten
        when processing a object message. The values are used for
        development and testing.

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        self.timestamp_c += self.loop_interval
        self.rho_c += self.rho_dot_c * self.loop_interval
        self.tau_c += self.tau_dot_c * self.loop_interval

    def _exit_handler(self, signum: int, frame: Optional[FrameType]) -> None:
        """Exit the controller gracefully by stopping continuous pan
        and tilt.

        Parameters
        ----------
        signum : int
            The signal number

        frame : Optional[FrameType]
            The current stack frame (None or a frame object)

        Returns
        -------
        None
        """
        if self.use_camera:
            logging.info("Stopping continuous pan and tilt")
            try:
                self.camera_control.stop_move()
            except Exception as e:
                logging.error(f"Error: {e}")
        logging.info("Exiting")
        sys.exit()

    def main(self) -> None:
        """Schedule module heartbeat and image capture, subscribe to
        all required topics, then loop forever. Update pointing for
        logging, and command zero camera pan and tilt rates and stop
        capturing images after twice the capture interval has elapsed.
        """
        if self.use_mqtt:
            # Schedule module heartbeat
            heartbeat_job = schedule.every(self.heartbeat_interval).seconds.do(
                self.publish_heartbeat, payload="PTZ Controller Module Heartbeat"
            )

            # Subscribe to required topics
            self.add_subscribe_topic(self.config_topic, self._config_callback)
            self.add_subscribe_topic(self.orientation_topic, self._orientation_callback)
            self.add_subscribe_topic(self.object_topic, self._object_callback)
            self.add_subscribe_topic(
                self.manual_control_topic, self._manual_control_callback
            )

    
        update_tracking_time = time()

        # Enter the main loop
        while True:
            try:
                # Run pending scheduled messages
                if self.use_mqtt:
                    schedule.run_pending()

                # Update camera pointing
                sleep(self.loop_interval)
                if not self.use_camera:
                    self._update_pointing()


                # Track object
                if ( self.use_camera and time() - update_tracking_time > self.tracking_interval ):
                    time_since_last_update = time() - update_tracking_time
                    update_tracking_time = time()
                    self._track_object(time_since_last_update)

                # Command zero camera pan and tilt rates, and stop
                # capturing images if a object message has not been
                # received in twice the capture interval
                if (
                    self.do_capture
                    and time() - self.capture_time > 2.0 * self.capture_interval
                ):
                    logging.info(f"Stopping image capture of object: {self.object_id}")
                    self.do_capture = False
                    if self.use_camera:
                        logging.info("Stopping continuous pan and tilt")
                        try:
                            self.camera_control.stop_move()
                        except Exception as e:
                            logging.error(f"Error: {e}")

            except KeyboardInterrupt as exception:
                # If keyboard interrupt, fail gracefully
                logging.warning("Received keyboard interrupt")
                if self.use_camera:
                    logging.info("Stopping continuous pan and tilt")
                    try:
                        self.camera_control.stop_move()
                    except Exception as e:
                        logging.error(f"Error: {e}")
                logging.warning("Exiting")
                sys.exit()

            except Exception as e:
                # Optionally continue on exception
                if self.continue_on_exception:
                    traceback.print_exc()
                else:
                    raise


def _check_required_env_vars() -> None:
    """Check that all required environment variables are set."""
    required_env_vars = [
        "MQTT_IP",
        "CONFIG_TOPIC",
        "ORIENTATION_TOPIC",
        "OBJECT_TOPIC",
        "IMAGE_FILENAME_TOPIC",
        "IMAGE_CAPTURE_TOPIC",
        "LOGGER_TOPIC",
        "MANUAL_CONTROL_TOPIC",
        "TRIPOD_LONGITUDE",
        "TRIPOD_LATITUDE",
    ]
    for env_var in required_env_vars:
        if env_var not in os.environ:
            raise ValueError(f"Environment variable {env_var} is not set")


def make_controller() -> AxisPtzController:
    """Instantiate an AxisPtzController."""

    _check_required_env_vars()

    return AxisPtzController(
        hostname=str(os.environ.get("HOSTNAME")),
        camera_ip=os.environ.get("CAMERA_IP", ""),
        camera_user=os.environ.get("CAMERA_USER", ""),
        camera_password=os.environ.get("CAMERA_PASSWORD", ""),
        mqtt_ip=os.environ.get("MQTT_IP"),
        config_topic=os.environ.get("CONFIG_TOPIC", ""),
        orientation_topic=os.environ.get("ORIENTATION_TOPIC", ""),
        object_topic=os.environ.get("OBJECT_TOPIC", ""),
        image_filename_topic=str(os.environ.get("IMAGE_FILENAME_TOPIC")),
        image_capture_topic=os.environ.get("IMAGE_CAPTURE_TOPIC", ""),
        manual_control_topic=os.environ.get("MANUAL_CONTROL_TOPIC", ""),
        logger_topic=os.environ.get("LOGGER_TOPIC", ""),
        lambda_t=float(os.environ.get("TRIPOD_LONGITUDE", 0.0)),
        varphi_t=float(os.environ.get("TRIPOD_LATITUDE", 0.0)),
        h_t=float(os.environ.get("TRIPOD_ALTITUDE", 0.0)),
        heartbeat_interval=int(os.environ.get("HEARTBEAT_INTERVAL", 10)),
        loop_interval=float(os.environ.get("LOOP_INTERVAL", 0.1)),
        capture_interval=int(os.environ.get("CAPTURE_INTERVAL", 2)),
        capture_dir=os.environ.get("CAPTURE_DIR", "."),
        tracking_interval=float(os.environ.get("TRACKING_INTERVAL", 1.0)),
        tripod_yaw=float(os.environ.get("TRIPOD_YAW", 0.0)),
        tripod_pitch=float(os.environ.get("TRIPOD_PITCH", 0.0)),
        tripod_roll=float(os.environ.get("TRIPOD_ROLL", 0.0)),
        pan_gain=float(os.environ.get("PAN_GAIN", 0.2)),
        pan_rate_max=float(os.environ.get("PAN_RATE_MAX", 150.0)),
        tilt_gain=float(os.environ.get("TILT_GAIN", 0.2)),
        tilt_rate_max=float(os.environ.get("TILT_RATE_MAX", 150.0)),
        zoom=int(os.environ.get("ZOOM", 6000)),
        focus=int(os.environ.get("FOCUS", 8749)),
        focus_min=int(os.environ.get("FOCUS_MIN", 7499)),
        focus_max=int(os.environ.get("FOCUS_MAX", 9999)),
        hyperfocal_distance=float(os.environ.get("HYPERFOCAL_DISTANCE", 22500.0)),
        jpeg_resolution=os.environ.get("JPEG_RESOLUTION", "1920x1080"),
        jpeg_compression=int(os.environ.get("JPEG_COMPRESSION", 5)),
        use_mqtt=ast.literal_eval(os.environ.get("USE_MQTT", "True")),
        use_camera=ast.literal_eval(os.environ.get("USE_CAMERA", "True")),
        auto_focus=ast.literal_eval(os.environ.get("AUTO_FOCUS", "True")),
        include_age=ast.literal_eval(os.environ.get("INCLUDE_AGE", "True")),
        log_to_mqtt=ast.literal_eval(os.environ.get("LOG_TO_MQTT", "False")),
        log_level=os.environ.get("LOG_LEVEL", "False"),
        continue_on_exception=ast.literal_eval(
            os.environ.get("CONTINUE_ON_EXCEPTION", "False")
        ),
    )


if __name__ == "__main__":
    # Instantiate controller and execute
    controller = make_controller()
    controller.main()
