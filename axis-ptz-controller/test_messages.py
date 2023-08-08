from argparse import ArgumentParser
from datetime import datetime
import json
import logging
import os
import time
from typing import Any, Dict, Union

import coloredlogs
import paho.mqtt.client as mqtt
import pandas as pd

from base_mqtt_pub_sub import BaseMQTTPubSub
from test_integration import (
    get_config_msg,
    get_orientation_msg,
    make_controller,
    make_object_msg,
    plot_time_series,
    read_track_data,
)

STYLES = {
    "critical": {"bold": True, "color": "red"},
    "debug": {"color": "green"},
    "error": {"color": "red"},
    "info": {"color": "white"},
    "notice": {"color": "magenta"},
    "spam": {"color": "green", "faint": True},
    "success": {"bold": True, "color": "green"},
    "verbose": {"color": "blue"},
    "warning": {"color": "yellow"},
}
coloredlogs.install(
    level=logging.INFO,
    fmt="%(asctime)s.%(msecs)03d \033[0;90m%(levelname)-8s "
    ""
    "\033[0;36m%(filename)-18s%(lineno)3d\033[00m "
    "%(message)s",
    level_styles=STYLES,
)

LOOP_INTERVAL = 0.1


class MessageHandler(BaseMQTTPubSub):
    """Subscribe to all required topics, and open files for logging."""

    def __init__(
        self,
        config_json_topic: str,
        orientation_json_topic: str,
        object_json_topic: str,
        logger_json_topic: str,
        **kwargs: Any,
    ):
        """Initialize a MessageHandler by subscribing to all required
        topics, connecting to the MQTT broker, and opening logging
        files.

        Parameters
        ----------
        config_json_topic: str
            MQTT topic for publishing or subscribing to configuration
            messages
        orientation_json_topic: str
            MQTT topic for publishing or subscribing to orientation
            messages
        object_json_topic: str
            MQTT topic for publishing or subscribing to object
            messages
        logger_json_topic: str
            MQTT topic for publishing or subscribing to logger
            messages

        Returns
        -------
        MessageHandler
        """
        # Parent class handles kwargs, including MQTT IP
        super().__init__(**kwargs)
        self.config_json_topic = config_json_topic
        self.orientation_json_topic = orientation_json_topic
        self.object_json_topic = object_json_topic
        self.logger_json_topic = logger_json_topic

        # Connect MQTT client
        logging.info("Connecting MQTT client")
        self.connect_client()
        time.sleep(5)
        self.publish_registration("Message Handler Module Registration")

        # Open files for logging
        self.camera_pointing_filename = "camera-pointing.csv"
        self.camera_pointing_file = open(self.camera_pointing_filename, "w")
        self.camera_pointing_keys = [
            "timestamp_c",
            "rho_o",
            "tau_o",
            "rho_dot_o",
            "tau_dot_o",
            "rho_c",
            "tau_c",
            "rho_dot_c",
            "tau_dot_c",
        ]
        self.camera_pointing_file.write(",".join(self.camera_pointing_keys) + "\n")

    def decode_payload(
        self, msg: Union[mqtt.MQTTMessage, str], data_payload_type: str
    ) -> Dict[Any, Any]:
        """
        Decode the payload carried by a message.

        Parameters
        ----------
        payload: mqtt.MQTTMessage
            The MQTT message

        Returns
        -------
        data : Dict[Any, Any]
            The data payload of the message payload
        """
        if type(msg) == mqtt.MQTTMessage:
            payload = msg.payload.decode()
        else:
            payload = msg
        data_payload = json.loads(payload)[data_payload_type]
        return json.loads(data_payload)

    def _logger_callback(
        self, _client: mqtt.Client, _userdata: Dict[Any, Any], msg: mqtt.MQTTMessage
    ) -> None:
        """
        Process logging message based on keys.

        Parameters
        ----------
        _client: mqtt.Client
            MQTT client
        _userdata: dict
            Any required user data
        msg: Any
            A JSON string with {timestamp: ____, data: ____,}

        Returns
        -------
        None
        """
        data = self.decode_payload(msg, "Logger")
        if "camera-pointing" in data:
            logging.info(data["camera-pointing"])
            self.camera_pointing_file.write(
                ",".join(
                    [str(data["camera-pointing"][k]) for k in self.camera_pointing_keys]
                )
                + "\n"
            )
            self.camera_pointing_file.flush()
        elif "info" in data:
            logging.info(data["info"]["message"])


def make_handler() -> MessageHandler:
    """Construct a MessageHandler.

    Note that an MQTT broker must be started manually.

    Parameters
    ----------
    None

    Returns
    -------
    handler: MessageHandler
        The message handler
    """
    handler = MessageHandler(
        mqtt_ip=os.getenv("MQTT_IP", ""),
        config_json_topic=os.getenv("CONFIG_JSON_TOPIC", ""),
        orientation_json_topic=os.getenv("ORIENTATION_JSON_TOPIC", ""),
        object_json_topic=os.getenv("OBJECT_JSON_TOPIC", ""),
        logger_json_topic=os.getenv("LOGGER_JSON_TOPIC", ""),
    )
    return handler


def main() -> None:
    """Read a track file and publish the corresponding messages using
    MQTT. Subscribe to the logger topic and process log messages to
    collect camera pointing time series for plotting.

    Parameters
    ----------
    None

    Returns
    -------
    None
    """

    # Provide for some command line arguments
    parser = ArgumentParser(
        description="Read a track file and process the corresponding messages"
    )
    parser.add_argument(
        "-t",
        "--track-id",
        default="A1E946",
        help="The track identifier to process: A1E946 (the default) or A19A08",
    )
    args = parser.parse_args()

    # Read the track data
    logging.info(f"Reading track for id: {args.track_id}")
    track = read_track_data(args.track_id)

    # Make the handler, and subscribe to the logger topic
    logging.info("Making the handler, and subscribing to topics")
    handler = make_handler()
    handler.add_subscribe_topic(handler.logger_json_topic, handler._logger_callback)
    data = {
        "info": {
            "message": "Subscribed to the logger",
        }
    }
    logger_msg = handler.generate_payload_json(
        push_timestamp=int(datetime.utcnow().timestamp()),
        device_type="TBC",
        id_="TBC",
        deployment_id="TBC",
        current_location="TBC",
        status="Debug",
        message_type="Event",
        model_version="null",
        firmware_version="v0.0.0",
        data_payload_type="Logger",
        data_payload=json.dumps(data),
    )
    handler.publish_to_topic(handler.logger_json_topic, logger_msg)

    # Publish the configuration and orientation message, and the first
    # object message
    controller = make_controller(True)
    config_msg = get_config_msg(controller)
    orientation_msg = get_orientation_msg(controller)
    index = 0
    object_msg = make_object_msg(controller, track, index)
    logging.info(f"Publishing config msg: {config_msg}")
    handler.publish_to_topic(handler.config_json_topic, config_msg)
    time.sleep(LOOP_INTERVAL)

    logging.info(f"Publishing orientation msg: {orientation_msg}")
    handler.publish_to_topic(handler.orientation_json_topic, orientation_msg)
    time.sleep(LOOP_INTERVAL)

    logging.info(f"Publishing object msg: {object_msg}")
    handler.publish_to_topic(handler.object_json_topic, object_msg)
    time.sleep(LOOP_INTERVAL)

    # Loop in camera time
    dt_c = LOOP_INTERVAL
    timestamp_c = track["timestamp"][index]
    while index < track.shape[0] - 1:
        time.sleep(LOOP_INTERVAL)
        timestamp_c += dt_c

        # Process each object message when received
        if timestamp_c >= track["timestamp"][index + 1]:
            index = track["timestamp"][timestamp_c >= track["timestamp"]].index[-1]
            object_msg = make_object_msg(controller, track, index)
            logging.info(f"Publishing object msg: {object_msg}")
            handler.publish_to_topic(handler.object_json_topic, object_msg)

    # Read camera pointing file as a dataframe, and plot
    time.sleep(5)
    handler.camera_pointing_file.close()
    ts = pd.read_csv(handler.camera_pointing_filename)
    plot_time_series(ts)


if __name__ == "__main__":
    main()
