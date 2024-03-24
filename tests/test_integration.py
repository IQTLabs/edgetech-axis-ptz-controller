import json

from matplotlib import pyplot as plt
import pandas as pd


def test_integration() -> None:
    """
    Plots output for manual inspection. Designed to be run after
    docker-compose.yaml.  Recommended to run from test.sh.
    """
    # Load output
    with open("./test-data/test-output.json") as output_file:
        output = json.load(output_file)

    # Construct and plot time series
    ts = pd.DataFrame.from_dict(
        [
            json.loads(d["message_content"]["Logger"])["camera-pointing"]
            for d in output[:-1]
        ]
    )
    plot_time_series(ts)


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
