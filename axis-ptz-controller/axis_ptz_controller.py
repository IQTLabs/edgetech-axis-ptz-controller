"""Defines the AxisPtzController child class of BaseMQTTPubSub, and a
method for making AxisPtzController instances. Instatiates an
AxisPtzController, and executes its main() method when run as a
module.
"""

import ast
from datetime import datetime, timezone
import json
from json import JSONDecodeError

import math
import logging
import threading
import os
from pathlib import Path
import shutil
import sys
import tempfile
from time import sleep, time
import traceback
from typing import Any, Dict, Union
from enum import Enum
import paho.mqtt.client as mqtt
import schedule
import numpy as np

from base_mqtt_pub_sub import BaseMQTTPubSub
import axis_ptz_utilities
from camera_configuration import CameraConfiguration
from camera_control import CameraControl
from camera import Camera
from object import Object



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
        tripod_longitude: float = 0.0,
        tripod_latitude: float = 0.0,
        tripod_altitude: float = 0.0,
        heartbeat_interval: int = 10,
        loop_interval: float = 0.1,
        capture_interval: int = 2,
        capture_dir: str = ".",
        tracking_interval: float = 1.0,
        tripod_yaw: float = 0.0,
        tripod_pitch: float = 0.0,
        tripod_roll: float = 0.0,
        pan_gain: float = 0.2,
        pan_derivative_gain_max: float = 10.0,
        pan_rate_max: float = 150.0,
        tilt_gain: float = 0.2,
        tilt_derivative_gain_max: float = 10.0,
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
        lead_time: float = 0.5,
        capture_lead_time: float = 0.25,
        log_to_mqtt: bool = False,
        log_level: str = "INFO",
        continue_on_exception: bool = False,
        is_dome: bool = True,
        min_camera_tilt = 0,
        max_camera_tilt = 90,
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
        tripod_longitude: float
            Tripod geodetic longitude [deg]
        tripod_latitude: float = 0.0,
            Tripod geodetic latitude [deg]
        tripod_altitude: float = 0.0,
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
        pan_derivative_gain_max: float
            Maximum gain level from the object's pan derivative
        pan_rate_max: float
            Camera pan rate maximum [deg/s]
        tilt_gain: float
            Proportional control gain for tilt error [1/s]
        tilt_derivative_gain_max: float
            Maximum gain level from the object's tilt derivative
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
        lead_time: float
            Lead time used when computing camera pointing
        capture_lead_time: float
            Lead time for the capture position
        log_to_mqtt: bool
            Flag to publish logger messages to MQTT, or not
        log_level (str): One of 'NOTSET', 'DEBUG', 'INFO', 'WARN',
            'WARNING', 'ERROR', 'FATAL', 'CRITICAL'
        continue_on_exception: bool
            Continue on unhandled exceptions if True, raise exception
            if False (the default)
        is_dome: bool
            flag for if this is a Dome type PTZ camera or a fully articulating camera
        min_camera_tilt: float
            minimum physical tilt level of camera
        max_camera_tilt: float
            maximum physical tilt level of camera

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
        self.heartbeat_interval = heartbeat_interval
        self.loop_interval = loop_interval
        self.capture_interval = capture_interval
        self.capture_dir = capture_dir
        self.tracking_interval = tracking_interval
        self.focus_interval = 1.0
        self.pan_gain = pan_gain
        self.pan_derivative_gain_max = pan_derivative_gain_max
        self.tilt_gain = tilt_gain
        self.tilt_derivative_gain_max = tilt_derivative_gain_max
        self.jpeg_resolution = jpeg_resolution
        self.jpeg_compression = jpeg_compression
        self.use_mqtt = use_mqtt
        self.use_camera = use_camera
        self.include_age = include_age
        self.lead_time = lead_time
        self.capture_lead_time = capture_lead_time
        self.log_to_mqtt = log_to_mqtt
        self.log_level = log_level
        self.continue_on_exception = continue_on_exception
        self.is_dome = is_dome
        self.min_camera_tilt = min_camera_tilt
        self.max_camera_tilt = max_camera_tilt

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

        # Set the status for the controller
        self.status = Status.SLEEPING

        # Lock to make sure object info is not used while being updated
        self.object_lock = threading.Lock()

        # Capture boolean and last capture time
        self.do_capture = False
        self.capture_time = 0.0
        self.object_update_time = 0.0

        # Initialize camera class, which manages the PTZ camera and attributes
        self.camera = Camera(
            camera_ip=camera_ip,
            camera_user=camera_user,
            camera_password=camera_password,
            tripod_longitude=tripod_longitude,
            tripod_latitude=tripod_latitude,
            tripod_altitude=tripod_altitude,
            tripod_yaw=tripod_yaw,
            tripod_pitch=tripod_pitch,
            tripod_roll=tripod_roll,
            pan_rate_max=pan_rate_max,
            tilt_rate_max=tilt_rate_max,
            zoom=zoom,
            focus=focus,
            focus_min=focus_min,
            focus_max=focus_max,
            hyperfocal_distance=hyperfocal_distance,
            use_camera=use_camera,
            auto_focus=auto_focus,
            is_dome=is_dome,
        )

        # Object to track
        self.object: Object = Object("fake", self.camera) # Initialize with a fake object for Typing

        config_msg = self.generate_payload_json(
            push_timestamp=int(datetime.now(timezone.utc).timestamp()),
            device_type=os.environ.get("DEVICE_TYPE", "Collector"),
            id_=self.hostname,
            deployment_id=os.environ.get(
                "DEPLOYMENT_ID", f"Unknown-Location-{self.hostname}"
            ),
            current_location=os.environ.get(
                "CURRENT_LOCATION", f"{tripod_latitude}, {tripod_longitude}"
            ),
            status="Debug",
            message_type="Event",
            model_version="null",
            firmware_version="v0.0.0",
            data_payload_type="Configuration",
            data_payload=json.dumps(
                {
                    "axis-ptz-controller": {
                        "tripod_longitude": self.camera.tripod_longitude,
                        "tripod_latitude": self.camera.tripod_latitude,
                        "tripod_altitude": self.camera.tripod_altitude,
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
            push_timestamp=int(datetime.now(timezone.utc).timestamp()),
            device_type=os.environ.get("DEVICE_TYPE", "Collector"),
            id_=self.hostname,
            deployment_id=os.environ.get(
                "DEPLOYMENT_ID", f"Unknown-Location-{self.hostname}"
            ),
            current_location=os.environ.get(
                "CURRENT_LOCATION", f"{tripod_latitude}, {tripod_longitude}"
            ),
            status="Debug",
            message_type="Event",
            model_version="null",
            firmware_version="v0.0.0",
            data_payload_type="Orientation",
            data_payload=json.dumps(
                {
                    "tripod_yaw": self.camera.tripod_yaw,
                    "tripod_pitch": self.camera.tripod_pitch,
                    "tripod_roll": self.camera.tripod_roll,
                }
            ),
        )
        self._orientation_callback(None, None, orientation_msg)

        # Initialize camera pointing
        self.camera.move_to_azimuth_elevation(0, 0, zoom)

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
        if isinstance(msg,mqtt.MQTTMessage):
            payload = msg.payload.decode()
        else:
            payload = msg
        try:
            json_payload = json.loads(payload)
            data_payload = json_payload[data_payload_type]
        except (KeyError, TypeError, JSONDecodeError, json.JSONDecodeError) as e:
            logging.error(f"Error: {e}")
            logging.error(payload)
            logging.error(
                f"Data payload type: {data_payload_type} not found in payload: {payload}"
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
        if data == {}:  # If data is empty, return
            return
        if "axis-ptz-controller" not in data:
            logging.info(
                f"Configuration message data missing axis-ptz-controller: {data}"
            )
            return
        logging.info(f"Processing config msg data: {data}")
        config = data["axis-ptz-controller"]
        if (
            "camera_ip" in config
            or "camera_user" in config
            or "camera_password" in config
        ):
            logging.error("Camera IP, user, and password cannot be changed via MQTT")

        if (
            "config_topic" in config
            or "orientation_topic" in config
            or "object_topic" in config
            or "image_filename_topic" in config
            or "image_capture_topic" in config
            or "logger_topic" in config
        ):
            logging.error("MQTT topics cannot be changed via MQTT")

        if (
            "tripod_longitude" in config
            or "tripod_latitude" in config
            or "tripod_altitude" in config
        ):
            if (
                "tripod_longitude" in config
                and "tripod_latitude" in config
                and "tripod_altitude" in config
            ):
                self.camera.update_tripod_position(
                    config["tripod_longitude"],
                    config["tripod_latitude"],
                    config["tripod_altitude"],
                )
                
            else:
                logging.error(
                    "Tripod longitude, latitude, and altitude must be set together"
                )

        self.heartbeat_interval = config.get(
            "heartbeat_interval", self.heartbeat_interval
        )
        self.loop_interval = config.get("loop_interval", self.loop_interval)  # [s]
        self.capture_interval = config.get(
            "capture_interval", self.capture_interval
        )  # [s]
        self.capture_dir = config.get("capture_dir", self.capture_dir)
        self.tracking_interval = config.get(
            "tracking_interval", self.tracking_interval
        )  # [s]
        # Cannot set tripod yaw, pitch, and roll because of side effects

        if "pan_rate_max" in config:
            self.camera.pan_rate_max = config["pan_rate_max"]
        if "tilt_rate_max" in config:
            self.camera.tilt_rate_max = config["tilt_rate_max"]

        self.pan_gain = config.get("pan_gain", self.pan_gain)  # [1/s]
        self.tilt_gain = config.get("tilt_gain", self.tilt_gain)  # [1/s]
        self.pan_derivative_gain_max = config.get(
            "pan_derivative_gain_max", self.pan_derivative_gain_max
        )
        self.tilt_derivative_gain_max = config.get(
            "tilt_derivative_gain_max", self.tilt_derivative_gain_max
        )

        if "zoom" in config:
            self.camera.update_zoom(config["zoom"])
        if "focus" in config:
            self.camera.focus = config["focus"]
        if "focus_min" in config or "focus_max" in config or "hyperfocal_distance" in config:
            if "focus_min" in config:
                self.camera.focus_min = float(config["focus_min"])
            if "focus_max" in config:
                self.camera.focus_max = float(config["focus_max"])
            if "hyperfocal_distance" in config:
                self.camera.hyperfocal_distance = float(config["hyperfocal_distance"])
            self.camera.recalculate_focus()
        if "auto_focus" in config:
            self.camera.auto_focus = config["auto_focus"]
        if "use_camera" in config:
            self.use_camera = config["use_camera"]
            self.camera.use_camera = config["use_camera"]
            if not config["use_camera"]:
                self.camera.stop_move()
        if "include_age" in config:
            self.include_age = config[
                "include_age"
            ]  # we need to still keep track of include_age because we will be creating new Objects later
            self.object.include_age = config["include_age"]
        if "lead_time" in config:
            self.lead_time = config[
                "lead_time"
            ]  # we need to still keep track of lead_time because we will be creating new Objects later
            self.object.lead_time = config["lead_time"]

        self.capture_lead_time = config.get(
            "capture_lead_time", self.capture_lead_time
        )
        self.jpeg_resolution = config.get("jpeg_resolution", self.jpeg_resolution)
        self.jpeg_compression = config.get("jpeg_compression", self.jpeg_compression)
        self.use_mqtt = config.get("use_mqtt", self.use_mqtt)
        self.log_to_mqtt = config.get("log_to_mqtt", self.log_to_mqtt)
        self.log_level = config.get("log_level", self.log_level)
        self.continue_on_exception = config.get(
            "continue_on_exception", self.continue_on_exception
        )

        # Log configuration parameters
        self._log_config()
        self._publish_config()

    def _config_object(self) -> Dict[str, Any]:
        config = {
            "hostname": self.hostname,
            "camera_ip": self.camera_ip,
            "config_topic": self.config_topic,
            "orientation_topic": self.orientation_topic,
            "object_topic": self.object_topic,
            "image_filename_topic": self.image_filename_topic,
            "image_capture_topic": self.image_capture_topic,
            "logger_topic": self.logger_topic,
            "manual_control_topic": self.manual_control_topic,
            "tripod_longitude": self.camera.tripod_longitude,
            "tripod_latitude": self.camera.tripod_latitude,
            "tripod_altitude": self.camera.tripod_altitude,
            "heartbeat_interval": self.heartbeat_interval,
            "loop_interval": self.loop_interval,
            "capture_interval": self.capture_interval,
            "capture_dir": self.capture_dir,
            "tracking_interval": self.tracking_interval,
            "tripod_yaw": self.camera.tripod_yaw,
            "tripod_pitch": self.camera.tripod_pitch,
            "tripod_roll": self.camera.tripod_roll,
            "pan_gain": self.pan_gain,
            "pan_derivative_gain_max": self.pan_derivative_gain_max,
            "tilt_gain": self.tilt_gain,
            "tilt_derivative_gain_max": self.tilt_derivative_gain_max,
            "pan_rate_max": self.camera.pan_rate_max,
            "tilt_rate_max": self.camera.tilt_rate_max,
            "zoom": self.camera.zoom,
            "focus": self.camera.focus,
            "focus_min": self.camera.focus_min,
            "focus_max": self.camera.focus_max,
            "hyperfocal_distance": self.camera.hyperfocal_distance,
            "jpeg_resolution": self.jpeg_resolution,
            "jpeg_compression": self.jpeg_compression,
            "use_mqtt": self.use_mqtt,
            "use_camera": self.camera.use_camera,
            "auto_focus": self.camera.auto_focus,
            "include_age": self.include_age,
            "lead_time": self.lead_time,
            "capture_lead_time": self.capture_lead_time,
            "log_to_mqtt": self.log_to_mqtt,
            "log_level": self.log_level,
            "continue_on_exception": self.continue_on_exception,
        }

        return config

    def _publish_config(self) -> None:
        config = self._config_object()
        config_msg = self.generate_payload_json(
            push_timestamp=int(datetime.now(timezone.utc).timestamp()),
            device_type=os.environ.get("DEVICE_TYPE", "Collector"),
            id_=self.hostname,
            deployment_id=os.environ.get(
                "DEPLOYMENT_ID", f"Unknown-Location-{self.hostname}"
            ),
            current_location=os.environ.get(
                "CURRENT_LOCATION", f"{self.camera.tripod_latitude}, {self.camera.tripod_longitude}"
            ),
            status="Debug",
            message_type="Event",
            model_version="null",
            firmware_version="v0.0.0",
            data_payload_type="Current Configuration",
            data_payload=json.dumps(
                {
                    "axis-ptz-controller": config
                }
            ),
        )
        self.publish_to_topic(self.config_topic, config_msg)

    def _log_config(self) -> None:
        """Logs all paramters that can be set on construction."""
        config = self._config_object()
        logging.info(
            f"AxisPtzController configuration:\n{json.dumps(config, indent=4)}"
        )


    def _track_object(self, time_since_last_update: float) -> None:
        if self.status != Status.TRACKING:
            return

        if self.object is None:
            logging.error(f"Object is None, not tracking")
            self.camera.stop_move()
            return

        # Make sure Object info is not updated while pointing is being computed
        # self.object_lock.acquire()

        start_time = time()

        try:

            if self.use_camera:
                # Get camera pan and tilt
                self.rho_c, self.tau_c, _zoom, _focus = self.camera.get_ptz()
                # logging.info(
                #     f"Camera pan, tilt, zoom, and focus: {self.rho_c} [deg], {self.tau_c} [deg], {_zoom}, {_focus}"
                # )
            else:
                logging.debug(f"Controller pan and tilt: {self.rho_c}, {self.tau_c} [deg]")


            if self.object is None:
                logging.error(f"Not sure why it is None here, but not earlier")
                self.camera.stop_move()
                return

            # recompute the object's current location
            # we want to do this after getting the camera's current location because that is a network call
            # and it there is latency and jitter in how long it takes.
            self.object.recompute_location()

            # Compute angle delta between camera and object pan and tilt
            self.delta_rho = axis_ptz_utilities.compute_angle_delta(
                self.camera.rho, self.object.rho
            )
            self.delta_tau = axis_ptz_utilities.compute_angle_delta(
                self.camera.tau, self.object.tau
            )

            # Compute slew rate differences

            # tracking the rate of change for the object's pan and tilt allows us to amplify the gain
            # when the object is moving quickly past the camera
            object_rho_derivative = abs(self.object.rho_derivative)
            object_tau_derivative = abs(self.object.tau_derivative)

            # we want to make sure the object derivative does not have a dampening effect on the gain
            if object_rho_derivative < 1:
                object_rho_derivative = 1
            if object_tau_derivative < 1:
                object_tau_derivative = 1
            if object_rho_derivative > self.pan_derivative_gain_max:
                object_rho_derivative = self.pan_derivative_gain_max
            if object_tau_derivative > self.tilt_derivative_gain_max:
                object_tau_derivative = self.tilt_derivative_gain_max

            self.rho_c_gain = self.pan_gain * self.delta_rho * object_rho_derivative
            self.tau_c_gain = self.tilt_gain * self.delta_tau * object_tau_derivative

            # Compute position and velocity in the camera fixed (rst)
            # coordinate system of the object relative to the tripod at
            # time zero after pointing the camera at the object

            # Update camera pan and tilt rate
            self.rho_dot_c = self.object.rho_rate + self.rho_c_gain #- (self.object.rho_derivative ** 2)
            self.tau_dot_c = self.object.tau_rate + self.tau_c_gain #- (self.object.tau_derivative ** 2)

            # Get, or compute and set focus, command camera pan and tilt
            # rates, and begin capturing images, if needed
            if self.use_camera:
                # Note that focus cannot be negative, since distance_to_tripod3d
                # is non-negative
                self.camera.update_pan_tilt_rates(self.rho_dot_c, self.tau_dot_c)

                if self.object is None:
                    logging.error(f"REALLY not sure why it is None here, but not earlier")
                    self.camera.stop_move()
                    return
                if not self.do_capture and self.object is not None:
                    logging.info(
                        f"Starting image capture of object: {self.object.object_id}"
                    )
                    self.do_capture = True
                    self.capture_time = time()

                if self.do_capture and time() - self.capture_time > self.capture_interval:
                    capture_thread = threading.Thread(target=self._capture_image)
                    capture_thread.daemon = True
                    capture_thread.start()

            elapsed_time = time() - start_time
            # logging.info(
            #     f"\t⏱️\t {round(elapsed_time,3)}s RATES - 🎥 Pan: {self.rho_dot_c}\tTilt: {self.tau_dot_c}\t🛩️  Pan: {self.object.rho_rate}\tTilt: {self.object.tau_rate} ANGLES: 🎥  Pan: {self.rho_c}\tTilt: {self.tau_c}\t🛩️  Pan: {self.object.rho}\tTilt: {self.object.tau} "
            # )

            # Compute position of aircraft relative to tripod in ENz, then XYZ,
            # then uvw coordinates
            if self.object is None:
                logging.error(f"Object is None, not tracking")
                self.camera.stop_move()
                return
            
            r_ENz_a_t = np.array(
                [
                    math.sin(math.radians(self.object.rho)) * math.cos(math.radians(self.object.tau)),
                    math.cos(math.radians(self.object.rho)) * math.cos(math.radians(self.object.tau)),
                    math.sin(math.radians(self.object.tau)),
                ]
            )
            r_XYZ_a_t = np.matmul(self.camera.get_xyz_to_enz_transformation_matrix().transpose(), r_ENz_a_t)
            r_uvw_a_t = np.matmul(self.camera.get_xyz_to_uvw_transformation_matrix(), r_XYZ_a_t)

            # Compute pan an tilt
            self.camera_pan = math.degrees(math.atan2(r_uvw_a_t[0], r_uvw_a_t[1]))  # [deg]
            self.camera_tilt = math.degrees(
                math.atan2(r_uvw_a_t[2], axis_ptz_utilities.norm(r_uvw_a_t[0:2]))
            )  # [deg]


            # Log camera pointing using MQTT
            if self.log_to_mqtt:
                logger_msg = self.generate_payload_json(
                    push_timestamp=int(datetime.now(timezone.utc).timestamp()),
                    device_type=os.environ.get("DEVICE_TYPE", "Collector"),
                    id_=self.hostname,
                    deployment_id=os.environ.get(
                        "DEPLOYMENT_ID", f"Unknown-Location-{self.hostname}"
                    ),
                    current_location=os.environ.get(
                        "CURRENT_LOCATION",
                        f"{self.camera.tripod_latitude}, {self.camera.tripod_longitude}",
                    ),
                    status="Debug",
                    message_type="Event",
                    model_version="null",
                    firmware_version="v0.0.0",
                    data_payload_type="Logger",
                    data_payload=json.dumps(
                        {
                            "camera-pointing": {
                                "timestamp_c": self.timestamp_c,
                                "rho_o": self.object.rho,
                                "tau_o": self.object.tau,
                                "rho_camera_o": self.camera_pan,
                                "tau_camera_o": self.camera_tilt,
                                "corrected_rho_delta": self.object.rho - self.camera_pan,
                                "corrected_tau_delta": self.object.tau - self.camera_tilt,
                                "rho_dot_o": self.object.rho_rate,
                                "tau_dot_o": self.object.tau_rate,
                                "rho_c": self.camera.rho,
                                "tau_c": self.camera.tau,
                                "rho_dot_c": self.rho_dot_c,
                                "tau_dot_c": self.tau_dot_c,
                                "rho_c_gain": self.rho_c_gain,
                                "tau_c_gain": self.tau_c_gain,
                                "rst_vel_o_0": self.object.rst_velocity_msg_relative_to_tripod[0],
                                "rst_vel_o_1": self.object.rst_velocity_msg_relative_to_tripod[1],
                                "rst_vel_o_2": self.object.rst_velocity_msg_relative_to_tripod[2],
                                "rst_point_o_0": self.object.rst_point_msg_relative_to_tripod[0],
                                "rst_point_o_1": self.object.rst_point_msg_relative_to_tripod[1],
                                "rst_point_o_2": self.object.rst_point_msg_relative_to_tripod[2],
                                "distance": self.object.distance_to_tripod3d,
                                "focus": _focus,
                                "zoom": _zoom,
                                "object_location_update_period": self.object.location_update_period,
                                "rho_derivative": self.object.rho_derivative,
                                "tau_derivative": self.object.tau_derivative,
                                "pan_rate_index": self.camera.pan_rate_index,
                                "tilt_rate_index": self.camera.tilt_rate_index,
                                "delta_rho_dot_c": self.delta_rho_dot_c,
                                "delta_tau_dot_c": self.delta_tau_dot_c,
                                "delta_rho": self.delta_rho,
                                "delta_tau": self.delta_tau,
                                "tracking_loop_time": elapsed_time,
                                "time_since_last_update": time_since_last_update,
                                "object_id": self.object.object_id,
                            }
                        }
                    ),
                )
                self.publish_to_topic(self.logger_topic, logger_msg)
        except Exception as e:
            logging.error(f"Error in tracking object: {e}")
            logging.error(traceback.format_exc())

    def _slew_camera(self, rho_target: float, tau_target: float) -> None:
        if self.status == Status.SLEWING:
            logging.error("Camera is already slewing")
            return
        if self.use_camera and ( (self.object.tau < self.min_camera_tilt) or (self.object.tau > self.max_camera_tilt) ):
            self.object = None
            self.do_capture = False
            self.status = Status.SLEEPING
            logging.info(
                "Inhibiting slew - object is outside the camera's physical limits"
            )
            return
        self.status = Status.SLEWING
        self.camera.slew_camera(rho_target, tau_target)

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

        self.camera.update_tripod_orientation(
            data["tripod_yaw"], data["tripod_pitch"], data["tripod_roll"]
        )
        self._publish_config()

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
            #logging.info(f"Required keys missing from object message data: {data}")
            if self.status == Status.TRACKING:
                logging.info(f"Stopping continuous pan and tilt - Object is no longer being tracked")
                self.camera.stop_move()
                self.do_capture = False
                self.status = Status.SLEEPING
                self.object = None
            elif self.status == Status.SLEWING:
                logging.info(f"Stopping continuous pan and tilt - Object is no longer being tracked")
                self.camera.stop_move()
                self.do_capture = False
                self.status = Status.SLEEPING
                self.object = None
            elif self.status == Status.SLEEPING and self.object != None:
                self.object = None
                logging.info(f"We are sleeping, but we have an object, so we will stop tracking it")
            return
        logging.info(
            f"\t🗒️\tProcessing object msg data: {data['object_id']} \t {data['latitude']} \t {data['longitude']} \t {data['altitude']}"
        )

        self.object_update_time = time()
        if self.object == None or self.object.object_id != data["object_id"]:
            self.object = Object(
                object_id=data["object_id"],
                camera=self.camera,
                include_age=self.include_age,
                lead_time=self.lead_time,
            )

            self.object.update_from_msg(data)
            self.object.recompute_location()

            logging.info(f"Tracking object: {self.object.object_id}")

            if self.use_camera:
                self._slew_camera(self.object.rho, self.object.tau)
                return
        else:
            self.object.update_from_msg(data)
            #self.object.recompute_location()

        if self.use_camera and ( (self.object.tau < self.min_camera_tilt) or (self.object.tau > self.max_camera_tilt) ):
            logging.info(f"Stopping image capture of object: {self.object.object_id}")
            self.object = None
            self.do_capture = False
            self.status = Status.SLEEPING
            logging.info(
                "Stopping continuous pan and tilt - Object is outside the camera's physical limits"
            )
            self.camera.stop_move()

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


        if set(["azimuth", "elevation", "zoom"]) <= set(data.keys()):
            azimuth = data["azimuth"]
            elevation = data["elevation"]
            zoom = data["zoom"]
            logging.info(f"Setting camera to azimuth: {azimuth}, elevation: {elevation}, zoom: {zoom}")
            self.camera.move_to_azimuth_elevation(azimuth, elevation, zoom)
        
        if set(["pan_rate", "tilt_rate"]) <= set(data.keys()):
            pan_rate = data["pan_rate"]
            tilt_rate = data["tilt_rate"]
            logging.info(f"Setting camera pan rate to: {pan_rate}, tilt rate to: {tilt_rate}")
            self.camera.update_pan_tilt_rates(pan_rate, tilt_rate)

        if set(["pan", "tilt"]) <= set(data.keys()):
            pan = data["pan"]
            tilt = data["tilt"]
            logging.info(f"Setting camera to pan: {pan}, tilt: {tilt}")
            self.camera.slew_camera(pan, tilt)

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
            push_timestamp=int(datetime.now(timezone.utc).timestamp()),
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
                self.object.object_id,
                int(self.object.azm) % 360,
                int(self.object.elv),
                int(self.object.distance_to_tripod3d),
                timestr,
            )
            logging.info(
                f"\t📸\tCapturing image of object: {self.object.object_id}, at: {self.capture_time}, in: {image_filepath}"
            )

            # Populate and publish image metadata, getting current pan
            # and tilt, and accounting for object message age relative
            # to the image capture
            try:
                ptz_time = time()
                rho_c, tau_c, _zoom, _focus = self.camera_control.get_ptz()
            except Exception as e:
                logging.error(f"Error: {e}")
                return

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
                f"Publishing filename: {image_filepath}, for object: {self.object.object_id}, at: {self.capture_time}"
            )
            self._send_data(
                {
                    "type": "ImageFileName",
                    "payload": str(image_filepath),
                }
            )

            ptz_age = time() - ptz_time
            adjusted_rho_c = rho_c + self.rho_dot_c * self.capture_lead_time
            adjusted_tau_c = tau_c + self.tau_dot_c * self.capture_lead_time
            # Note: this time does not include any leading
            object_msg_age = datetime_c.timestamp() - self.object.msg_timestamp  # [s] 
            if object_msg_age < 0:
                logging.error(f"Object message age is negative: {object_msg_age} - {datetime_c.timestamp()} - {self.object.msg_timestamp}")
                object_msg_age = 0
            if object_msg_age > 60:
                logging.error(f"Object message age is too old: {object_msg_age} - {datetime_c.timestamp()} - {self.object.msg_timestamp}")

            image_metadata = {
                "timestamp": timestr,
                "imagefile": str(image_filepath),
                "camera": {
                    "rho_c": adjusted_rho_c,
                    "tau_c": adjusted_tau_c,
                    "tripod_longitude": self.camera.tripod_longitude,
                    "tripod_latitude": self.camera.tripod_latitude,
                    "zoom": self.camera.zoom,
                },
                "object": {
                    "object_msg_age": object_msg_age,
                    "r_ENz_o_0_t": self.object.enz_point_msg_relative_to_tripod.tolist(),
                    "v_ENz_o_0_t": self.object.enz_velocity_msg_relative_to_tripod.tolist(),
                    "delta_rho": axis_ptz_utilities.compute_angle_delta(
                        rho_c, self.object.rho_now
                    ),
                    "delta_tau": axis_ptz_utilities.compute_angle_delta(
                        tau_c, self.object.tau_now
                    ),
                },
            }
            logging.debug(
                f"Publishing metadata: {image_metadata}, for object: {self.object.object_id}, at: {self.capture_time}"
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

    def _exit_handler(self) -> None:
        """Exit the controller gracefully by stopping continuous pan
        and tilt.

        Returns
        -------
        None
        """

        logging.info("Stopping continuous pan and tilt")

        self.camera_control.stop_move()

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
            schedule.every(self.heartbeat_interval).seconds.do(
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
        update_focus_time = time()

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

                # Focus Update
                if (
                    self.use_camera and
                    not self.camera.auto_focus and
                    self.object != None and
                    time() - update_focus_time > self.focus_interval
                ):
                    update_focus_time = time()
                    self.camera.update_focus(self.object.distance_to_tripod3d)
                
                # Track object
                if (
                    self.use_camera
                    and time() - update_tracking_time > self.tracking_interval
                ):
                    time_since_last_update = time() - update_tracking_time
                    update_tracking_time = time()
                    self._track_object(time_since_last_update)

                # Command zero camera pan and tilt rates, and stop
                # capturing images if a object message has not been
                # received in twice the capture interval
                if (
                    self.status == Status.TRACKING
                    and time() - self.object_update_time > 2.0 * self.capture_interval
                ):
                    logging.info(
                        f"Stopping tracking image capture of object, no longer receiving updates"
                    )
                    self.do_capture = False
                    self.camera.stop_move()
                    self.status = Status.SLEEPING

            except KeyboardInterrupt:
                # If keyboard interrupt, fail gracefully
                logging.warning("Received keyboard interrupt")

                logging.info("Stopping continuous pan and tilt")

                self.camera.stop_move()

                logging.warning("Exiting")
                sys.exit()

            except Exception as e:
                # Optionally continue on exception
                if self.continue_on_exception:
                    logging.error(f"Unhandled exception: {e}")
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
        tripod_longitude=float(os.environ.get("TRIPOD_LONGITUDE", 0.0)),
        tripod_latitude=float(os.environ.get("TRIPOD_LATITUDE", 0.0)),
        tripod_altitude=float(os.environ.get("TRIPOD_ALTITUDE", 0.0)),
        heartbeat_interval=int(os.environ.get("HEARTBEAT_INTERVAL", 10)),
        loop_interval=float(os.environ.get("LOOP_INTERVAL", 0.1)),
        capture_interval=int(os.environ.get("CAPTURE_INTERVAL", 2)),
        capture_dir=os.environ.get("CAPTURE_DIR", "."),
        tracking_interval=float(os.environ.get("TRACKING_INTERVAL", 1.0)),
        tripod_yaw=float(os.environ.get("TRIPOD_YAW", 0.0)),
        tripod_pitch=float(os.environ.get("TRIPOD_PITCH", 0.0)),
        tripod_roll=float(os.environ.get("TRIPOD_ROLL", 0.0)),
        pan_gain=float(os.environ.get("PAN_GAIN", 0.2)),
        pan_derivative_gain_max=float(os.environ.get("PAN_DERIVATIVE_GAIN_MAX", 10.0)),
        pan_rate_max=float(os.environ.get("PAN_RATE_MAX", 150.0)),
        tilt_gain=float(os.environ.get("TILT_GAIN", 0.2)),
        tilt_derivative_gain_max=float(os.environ.get("TILT_DERIVATIVE_GAIN_MAX", 10.0)),
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
        lead_time=float(os.environ.get("LEAD_TIME", 0.0)),
        capture_lead_time=float(os.environ.get("CAPTURE_LEAD_TIME", 0.0)),
        log_to_mqtt=ast.literal_eval(os.environ.get("LOG_TO_MQTT", "False")),
        log_level=os.environ.get("LOG_LEVEL", "False"),
        continue_on_exception=ast.literal_eval(
            os.environ.get("CONTINUE_ON_EXCEPTION", "False")
        ),
        is_dome=ast.literal_eval(os.environ.get("IS_DOME", "True")),
        min_camera_tilt=float(os.environ.get("MIN_CAMERA_ANGLE", 0.0)),
        max_camera_tilt=float(os.environ.get("MAX_CAMERA_ANGLE", 90.0)),
    )


if __name__ == "__main__":
    # Instantiate controller and execute
    controller = make_controller()
    controller.main()
