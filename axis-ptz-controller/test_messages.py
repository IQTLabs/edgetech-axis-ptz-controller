from argparse import ArgumentParser
from datetime import datetime
import json
import logging
import os
import time
from typing import Any, Dict, Union

import paho.mqtt.client as mqtt
import pandas as pd

from base_mqtt_pub_sub import BaseMQTTPubSub
from test_integration import (
    read_track_data,
    get_config_msg,
    get_orientation_msg,
    make_object_msg,
    plot_time_series,
)

UPDATE_INTERVAL = 0.1

logger = logging.getLogger("ptz-messages")
logger.setLevel(logging.INFO)


class MessageHandler(BaseMQTTPubSub):
    """Subscribe to all required topics, and open files for logging."""

    def __init__(
        self,
        config_topic: str,
        orientation_topic: str,
        object_topic: str,
        logger_topic: str,
        **kwargs: Any,
    ):
        """Initialize a MessageHandler by subscribing to all required
        topics, connecting to the MQTT broker, and opening logging
        files.

        Parameters
        ----------
        config_topic: str
            MQTT topic for publishing or subscribing to configuration
            messages
        orientation_topic: str
            MQTT topic for publishing or subscribing to orientation
            messages
        object_topic: str
            MQTT topic for publishing or subscribing to object
            messages
        logger_topic: str
            MQTT topic for publishing or subscribing to logger
            messages

        Returns
        -------
        MessageHandler
        """
        # Parent class handles kwargs, including MQTT IP
        super().__init__(**kwargs)
        self.config_topic = config_topic
        self.orientation_topic = orientation_topic
        self.object_topic = object_topic
        self.logger_topic = logger_topic

        # Connect MQTT client
        logger.info("Connecting MQTT client")
        self.connect_client()
        time.sleep(5)
        self.publish_registration("Message Handler Module Registration")

        # Open files for logging
        self.camera_pointing_filename = "camera-pointing.csv"
        self.camera_pointing_file = open(self.camera_pointing_filename, "w")
        self.camera_pointing_keys = [
            "time_c",
            "rho_a",
            "tau_a",
            "rho_dot_a",
            "tau_dot_a",
            "rho_c",
            "tau_c",
            "rho_dot_c",
            "tau_dot_c",
        ]
        self.camera_pointing_file.write(",".join(self.camera_pointing_keys) + "\n")

    def decode_payload(self, payload: mqtt.MQTTMessage) -> Dict[Any, Any]:
        """
        Decode the payload carried by a message.

        Parameters
        ----------
        payload: Any
            A JSON string with {timestamp: ____, data: ____,}

        Returns
        -------
        data : dict
            The data component of the payload
        """
        data = json.loads(str(payload.decode("utf-8")))["data"]
        return data

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
        data = self.decode_payload(msg.payload)
        if "camera-pointing" in data:
            p = data["camera-pointing"]
            self.camera_pointing_file.write(
                ",".join([str(p[k]) for k in self.camera_pointing_keys]) + "\n"
            )
        elif "info" in data:
            logger.info(data["info"]["message"])


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
        config_topic=os.getenv("CONFIG_TOPIC", ""),
        orientation_topic=os.getenv("ORIENTATION_TOPIC", ""),
        object_topic=os.getenv("OBJECT_TOPIC", ""),
        logger_topic=os.getenv("LOGGER_TOPIC", ""),
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
    logger.info(f"Reading track for id: {args.track_id}")
    track = read_track_data(args.track_id)

    # Make the handler, and subscribe to the logger topic
    logger.info("Making the handler, and subscribing to topics")
    handler = make_handler()
    handler.add_subscribe_topic(handler.logger_topic, handler._logger_callback)
    logger_msg = {
        "timestamp": str(int(datetime.utcnow().timestamp())),
        "data": {
            "info": {
                "message": "Subscribed to the logger",
            }
        },
    }
    handler.publish_to_topic(handler.logger_topic, json.dumps(logger_msg))

    # Publish the configuration and orientation message, and the first
    # object message
    config_msg = get_config_msg()
    orientation_msg = get_orientation_msg()
    index = 0
    object_msg = make_object_msg(track, index)
    logger.info(f"Publishing config msg: {config_msg}")
    handler.publish_to_topic(handler.config_topic, json.dumps(config_msg))
    time.sleep(UPDATE_INTERVAL)
    logger.info(f"Publishing orientation msg: {orientation_msg}")
    handler.publish_to_topic(handler.orientation_topic, json.dumps(orientation_msg))
    time.sleep(UPDATE_INTERVAL)
    logger.info(f"Publishing object msg: {object_msg}")
    handler.publish_to_topic(handler.object_topic, json.dumps(object_msg))
    time.sleep(UPDATE_INTERVAL)

    # Loop in camera time
    dt_c = UPDATE_INTERVAL
    time_c = track["latLonTime"][index]
    while index < track.shape[0] - 1:
        time.sleep(UPDATE_INTERVAL)
        time_c += dt_c

        # Process each object message when received
        if time_c >= track["latLonTime"][index + 1]:
            index = track["latLonTime"][time_c >= track["latLonTime"]].index[-1]
            object_msg = make_object_msg(track, index)
            logger.info(f"Publishing object msg: {object_msg}")
            handler.publish_to_topic(handler.object_topic, json.dumps(object_msg))

    # Read camera pointing file as a dataframe, and plot
    handler.camera_pointing_file.close()
    ts = pd.read_csv(handler.camera_pointing_filename)
    plot_time_series(ts)


if __name__ == "__main__":
    main()
