from argparse import ArgumentParser
from datetime import datetime
import json
import logging
import os
import time
from typing import Any, Dict

from matplotlib import pyplot as plt
import pandas as pd

from axis_ptz_controller import AxisPtzController

HEARTBEAT_INTERVAL = 10
LOOP_INTERVAL = 0.01
CAPTURE_INTERVAL = 2
LEAD_TIME = 0.0
PAN_GAIN = 0.1
PAN_RATE_MIN = 1.0
PAN_RATE_MAX = 100.0
TILT_GAIN = 0.4
TILT_RATE_MIN = 1.0
TILT_RATE_MAX = 100.0
JPEG_RESOLUTION = "1920x1080"
JPEG_COMPRESSION = 5


def make_controller(use_mqtt: bool) -> AxisPtzController:
    """Construct a controller.

    Note that if use_mqtt = True then an MQTT broker must be started
    manually.

    Parameters
    ----------
    use_mqtt: bool
        Flag to use MQTT, or not

    Returns
    -------
    None
    """
    controller = AxisPtzController(
        hostname=os.environ.get("HOSTNAME", ""),
        camera_ip=os.environ.get("CAMERA_IP", ""),
        camera_user=os.environ.get("CAMERA_USER", ""),
        camera_password=os.environ.get("CAMERA_PASSWORD", ""),
        mqtt_ip=os.environ.get("MQTT_IP", ""),
        config_topic=os.environ.get("CONFIG_TOPIC", ""),
        orientation_topic=os.environ.get("ORIENTATION_TOPIC", ""),
        object_topic=os.environ.get("OBJECT_TOPIC", ""),
        image_filename_topic=os.environ.get("IMAGE_FILENAME_TOPIC", ""),
        image_capture_topic=os.environ.get("IMAGE_CAPTURE_TOPIC", ""),
        logger_topic=os.environ.get("LOGGER_TOPIC", ""),
        heartbeat_interval=HEARTBEAT_INTERVAL,
        loop_interval=LOOP_INTERVAL,
        capture_interval=CAPTURE_INTERVAL,
        lead_time=LEAD_TIME,
        pan_gain=PAN_GAIN,
        pan_rate_min=PAN_RATE_MIN,
        pan_rate_max=PAN_RATE_MAX,
        tilt_gain=TILT_GAIN,
        tilt_rate_min=TILT_RATE_MIN,
        tilt_rate_max=TILT_RATE_MAX,
        jpeg_resolution=JPEG_RESOLUTION,
        jpeg_compression=JPEG_COMPRESSION,
        use_mqtt=use_mqtt,
        use_camera=False,
        include_age=False,
    )
    return controller


def get_config_msg(controller: AxisPtzController) -> str:
    """Populate a config message, reading actual, by private longitude
    and latitude from the environment.

    Parameters
    ----------
    None

    Returns
    -------
    msg : str
        The configuration message
    """
    data: Dict[str, Any] = {}
    data["axis-ptz-controller"] = {}
    data["axis-ptz-controller"]["tripod_longitude"] = float(
        os.environ.get("TRIPOD_LONGITUDE", "-77.0")
    )  # [deg]
    data["axis-ptz-controller"]["tripod_latitude"] = float(
        os.environ.get("TRIPOD_LATITUDE", "38.0")
    )  # [deg]
    data["axis-ptz-controller"]["tripod_altitude"] = 86.46  # [m]
    msg = controller.generate_payload_json(
        push_timestamp=int(datetime.utcnow().timestamp()),
        device_type="TBC",
        id_="TBC",
        deployment_id="TBC",
        current_location="TBC",
        status="Debug",
        message_type="Event",
        model_version="null",
        firmware_version="v0.0.0",
        data_payload_type="Configuration",
        data_payload=json.dumps(data),
    )
    return msg


def get_orientation_msg(controller: AxisPtzController) -> str:
    """Populate an orientation message with all 0 deg angles.

    Parameters
    ----------
    None

    Returns
    -------
    msg : str
        The orientation message
    """
    with open("data/orientation_msg_data_0s.json", "r") as f:
        data = json.load(f)
    msg = controller.generate_payload_json(
        push_timestamp=int(datetime.utcnow().timestamp()),
        device_type="TBC",
        id_="TBC",
        deployment_id="TBC",
        current_location="TBC",
        status="Debug",
        message_type="Event",
        model_version="null",
        firmware_version="v0.0.0",
        data_payload_type="Orientation",
        data_payload=json.dumps(data),
    )
    return msg


def make_object_msg(
    controller: AxisPtzController, track: pd.DataFrame, index: int
) -> str:
    """Populate a object message with track data at the specified
    index.

    Parameters
    ----------
    track : pd.DataFrame()
        The track data
    index : int
        The index of the track data to use

    Returns
    -------
    msg : str
        The object message
    """
    data = track.iloc[index, :].to_dict()
    msg = controller.generate_payload_json(
        push_timestamp=int(datetime.utcnow().timestamp()),
        device_type="TBC",
        id_="TBC",
        deployment_id="TBC",
        current_location="TBC",
        status="Debug",
        message_type="Event",
        model_version="null",
        firmware_version="v0.0.0",
        data_payload_type="Selected Object",
        data_payload=json.dumps(data),
    )
    return msg


def read_track_data(track_id: str) -> pd.DataFrame:
    """Read a track file and convert to standard units of measure.

    Parameters
    ----------
    track_id : str
        The track identifier

    Returns
    -------
    track : pd.DataFrame()
        The track data read from the file
    """
    track = pd.read_csv(f"data/{track_id}-processed-track.csv")
    track["altitude"] *= 0.3048  # [ft] * [m/ft] = [m]
    track["horizontal_velocity"] *= (
        6076.12 / 3600 * 0.3048
    )  # [nm/h] * [ft/nm] / [s/h] * [m/ft] = [m/s]
    track["vertical_velocity"] *= 0.3048 / 60  # [ft/s] * [m/ft] / [s/m] = [m/s]
    return track


def plot_time_series(ts: pd.DataFrame) -> None:
    """Plot time series produced by processing messages.

    Parameters
    ----------
    ts : pd.DataFrame()
        Dataframe containing time series to plot

    Returns
    -------
    None
    """

    # Plot pan angle
    fig, axs = plt.subplots(2, 2, figsize=[12.8, 9.6])
    axs[0, 0].plot(ts["timestamp_c"], ts["rho_c"] - ts["rho_o"], label="error")
    axs[0, 0].plot(ts["timestamp_c"], ts["rho_c"], label="camera")
    axs[0, 0].plot(ts["timestamp_c"], ts["rho_o"], label="object")
    axs[0, 0].legend()
    axs[0, 0].set_title("Camera and Object Pan Angle and Difference")
    axs[0, 0].set_xlabel("Timestamp [s]")
    axs[0, 0].set_ylabel("Pan Angle [deg]")

    # Plot tilt angle
    axs[1, 0].plot(ts["timestamp_c"], ts["tau_c"] - ts["tau_o"], label="error")
    axs[1, 0].plot(ts["timestamp_c"], ts["tau_c"], label="camera")
    axs[1, 0].plot(ts["timestamp_c"], ts["tau_o"], label="object")
    axs[1, 0].legend()
    axs[1, 0].set_title("Camera and Object Tilt Angle and Difference")
    axs[1, 0].set_xlabel("Timestamp [s]")
    axs[1, 0].set_ylabel("Tilt Angle [deg]")

    # Plot pan angular rate angle
    axs[0, 1].plot(ts["timestamp_c"], ts["rho_dot_c"] - ts["rho_dot_o"], label="error")
    axs[0, 1].plot(ts["timestamp_c"], ts["rho_dot_c"], label="camera")
    axs[0, 1].plot(ts["timestamp_c"], ts["rho_dot_o"], label="object")
    axs[0, 1].legend()
    axs[0, 1].set_title("Camera and Object Pan Angular Rate and Difference")
    axs[0, 0].set_xlabel("Timestamp [s]")
    axs[0, 1].set_ylabel("Pan Anglular Rate [deg/s]")

    # Plot tilt angular rate angle
    axs[1, 1].plot(ts["timestamp_c"], ts["tau_dot_c"] - ts["tau_dot_o"], label="error")
    axs[1, 1].plot(ts["timestamp_c"], ts["tau_dot_c"], label="camera")
    axs[1, 1].plot(ts["timestamp_c"], ts["tau_dot_o"], label="object")
    axs[1, 1].legend()
    axs[1, 1].set_title("Camera and Object Tilt Angular Rate and Difference")
    axs[0, 0].set_xlabel("Timestamp [s]")
    axs[1, 1].set_ylabel("Tilt Anglular Rate [deg/s]")

    plt.show()


def main() -> None:
    """Read a track file and process the corresponding messages using
    MQTT, or not.

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
    parser.add_argument(
        "-m",
        "--use-mqtt",
        action="store_true",
        help="Use MQTT to process messages",
    )
    args = parser.parse_args()

    # Read the track data
    logging.info(f"Reading track for id: {args.track_id}")
    track = read_track_data(args.track_id)

    # Make the controller, subscribe to all topics, and publish, or
    # process, one message to each topic
    logging.info("Making the controller, and subscribing to topics")
    controller = make_controller(args.use_mqtt)
    controller.add_subscribe_topic(controller.config_topic, controller._config_callback)
    controller.add_subscribe_topic(
        controller.orientation_topic, controller._orientation_callback
    )
    controller.add_subscribe_topic(controller.object_topic, controller._object_callback)
    config_msg = get_config_msg(controller)
    orientation_msg = get_orientation_msg(controller)
    index = 0
    object_msg = make_object_msg(controller, track, index)
    if controller.use_mqtt:
        logging.info(f"Publishing config msg: {config_msg}")
        controller.publish_to_topic(controller.config_topic, config_msg)
        time.sleep(LOOP_INTERVAL)

        logging.info(f"Publishing orientation msg: {orientation_msg}")
        controller.publish_to_topic(controller.orientation_topic, orientation_msg)
        time.sleep(LOOP_INTERVAL)

        logging.info(f"Publishing object msg: {object_msg}")
        controller.publish_to_topic(controller.object_topic, object_msg)
        time.sleep(LOOP_INTERVAL)

    else:
        _client = None
        _userdata = None
        controller._config_callback(_client, _userdata, config_msg)
        controller._orientation_callback(_client, _userdata, orientation_msg)
        controller._object_callback(_client, _userdata, object_msg)

    # Initialize history for plotting
    history = {}
    data = controller.decode_payload(object_msg, "Selected Object")
    history["timestamp_c"] = [data["timestamp"]]
    history["rho_o"] = [controller.rho_o]
    history["tau_o"] = [controller.tau_o]
    history["rho_dot_o"] = [controller.rho_dot_o]
    history["tau_dot_o"] = [controller.tau_dot_o]
    history["rho_c"] = [controller.rho_c]
    history["tau_c"] = [controller.tau_c]
    history["rho_dot_c"] = [controller.rho_dot_c]
    history["tau_dot_c"] = [controller.tau_dot_c]

    # Loop in camera time
    dt_c = controller.loop_interval
    timestamp_c = history["timestamp_c"][0]
    while index < track.shape[0] - 1:
        timestamp_c += dt_c

        # Process each object message when received
        if timestamp_c >= track["timestamp"][index + 1]:
            index = track["timestamp"][timestamp_c >= track["timestamp"]].index[-1]
            object_msg = make_object_msg(controller, track, index)
            if controller.use_mqtt:
                logging.info(f"Publishing object msg: {object_msg}")
                controller.publish_to_topic(controller.object_topic, object_msg)
                time.sleep(LOOP_INTERVAL)

            else:
                controller._object_callback(_client, _userdata, object_msg)

        # Always update pointing
        controller._update_pointing()

        # Append to history for plotting
        history["timestamp_c"].append(timestamp_c)
        history["rho_o"].append(controller.rho_o)
        history["tau_o"].append(controller.tau_o)
        history["rho_dot_o"].append(controller.rho_dot_o)
        history["tau_dot_o"].append(controller.tau_dot_o)
        history["rho_c"].append(controller.rho_c)
        history["tau_c"].append(controller.tau_c)
        history["rho_dot_c"].append(controller.rho_dot_c)
        history["tau_dot_c"].append(controller.tau_dot_c)

    # Convert history dictionary to a dataframe, and plot
    ts = pd.DataFrame.from_dict(history)
    plot_time_series(ts)


if __name__ == "__main__":
    main()
