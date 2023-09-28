import json
import math
import os
from typing import Any, Dict

import numpy as np
import numpy.typing as npt
import pytest
import quaternion
import axis_ptz_controller
import axis_ptz_utilities


PRECISION = 1.0e-12
RELATIVE_DIFFERENCE = 2.0  # %
ANGULAR_DIFFERENCE = 1.0  # [deg]

# Aligns ENz with XYZ
LAMBDA_T = 270.0  # [deg]
VARPHI_T = 90.0  # [deg]
H_T = 0.0  # [m]

# Places object south of tripod
LAMBDA_O = 270.0  # [deg]
VARPHI_O = 89.99  # [deg]
H_O = 1000.0  # [m]
AIR_SPEED = 100.0  # [m/s]

HEARTBEAT_INTERVAL = 10
UPDATE_INTERVAL = 0.10
CAPTURE_INTERVAL = 2
LEAD_TIME = 0.0
PAN_GAIN = 0.2
PAN_RATE_MIN = 1.0
PAN_RATE_MAX = 100.0
TILT_GAIN = 0.2
TILT_RATE_MIN = 1.0
TILT_RATE_MAX = 100.0
JPEG_RESOLUTION = "1920x1080"
JPEG_COMPRESSION = 5


def qnorm(q: quaternion.quaternion) -> float:
    """Compute the quaternion norm."""
    return math.sqrt((q * q.conjugate()).w)


def R_pole() -> float:
    """Compute the semi-minor axis of the geoid."""
    f = 1.0 / axis_ptz_utilities.F_INV
    N_pole = axis_ptz_utilities.R_OPLUS / math.sqrt(1.0 - f * (2.0 - f))
    return (1.0 - f) ** 2 * N_pole


@pytest.fixture
def controller() -> axis_ptz_controller.AxisPtzController:
    """Construct a controller."""
    controller = axis_ptz_controller.AxisPtzController(
        camera_ip=os.getenv("CAMERA_IP", ""),
        camera_user=os.getenv("CAMERA_USER", ""),
        camera_password=os.getenv("CAMERA_PASSWORD", ""),
        mqtt_ip=os.getenv("MQTT_IP", ""),
        config_topic=os.getenv("CONFIG_TOPIC", ""),
        orientation_topic=os.getenv("ORIENTATION_TOPIC", ""),
        object_topic=os.getenv("OBJECT_TOPIC", ""),
        capture_topic=os.getenv("CAPTURE_TOPIC", ""),
        logger_topic=os.getenv("LOGGER_TOPIC", ""),
        image_filename_topic=os.getenv("IMAGE_FILENAME_TOPIC", ""),
        hostname=os.getenv("HOSTNAME", ""),
        heartbeat_interval=HEARTBEAT_INTERVAL,
        update_interval=UPDATE_INTERVAL,
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
        use_mqtt=False,
        use_camera=False,
        include_age=False,
    )
    return controller


@pytest.fixture
def config_msg() -> Dict[Any, Any]:
    """Populate a config message."""
    with open("data/config_msg.json", "r") as f:
        msg = json.load(f)
    return msg


@pytest.fixture
def orientation_msg_0s() -> Dict[Any, Any]:
    """Populate an orientation message with all 0 deg angles."""
    with open("data/orientation_msg_0s.json", "r") as f:
        msg = json.load(f)
    return msg


@pytest.fixture
def orientation_msg_90s() -> Dict[Any, Any]:
    """Populate an orientation message with all 90 deg angles."""
    with open("data/orientation_msg_90s.json", "r") as f:
        msg = json.load(f)
    return msg


@pytest.fixture
def object_msg() -> Dict[Any, Any]:
    """Populate a object message with velocity along the line of
    sight, using the calculation noted below.

    # ENz at the tripod aligns with XYZ, and the tripod remains stationary
    r_XYZ_t = axis_ptz_utilities.compute_r_XYZ(LAMBDA_T, VARPHI_T, H_T)
    r_XYZ_o = axis_ptz_utilities.compute_r_XYZ(LAMBDA_O, VARPHI_O, H_O)
    r_XYZ_o_t = r_XYZ_o - r_XYZ_t
    v_ENz_T_o = AIR_SPEED * r_XYZ_o_t / axis_ptz_utilities.norm(r_XYZ_o_t)

    # ENz directly below the object miss-aligns with XYZ slightly
    E_XYZ_to_ENz, _, _, _ = axis_ptz_utilities.compute_E_XYZ_to_ENz(LAMBDA_O, VARPHI_O)
    v_ENz_A_o = np.matmul(E_XYZ_to_ENz, v_ENz_T_o)
    """
    with open("data/object_msg.json", "r") as f:
        msg = json.load(f)
    return msg


class TestAxisPtzController:
    """Test message callbacks and update method."""

    def test_config_callback(
        self,
        controller: axis_ptz_controller.AxisPtzController,
        config_msg: Dict[Any, Any],
    ) -> None:
        # Align ENz with XYZ
        _client = None
        _userdata = None
        controller._config_callback(_client, _userdata, config_msg)

        # Assign expected values
        r_XYZ_t_exp = np.array([0.0, 0.0, R_pole()])
        E_XYZ_to_ENz_exp = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        e_E_XYZ_exp = np.array([1.0, 0.0, 0.0])
        e_N_XYZ_exp = np.array([0.0, 1.0, 0.0])
        e_z_XYZ_exp = np.array([0.0, 0.0, 1.0])

        # Y component of r_XYZ_t precision lower due to precision of
        # cos(90) and magnitude of R_oplus
        assert controller.lambda_t == LAMBDA_T
        assert controller.varphi_t == VARPHI_T
        assert controller.h_t == H_T
        assert np.linalg.norm(controller.r_XYZ_t - r_XYZ_t_exp) < 400 * PRECISION
        assert np.linalg.norm(controller.E_XYZ_to_ENz - E_XYZ_to_ENz_exp) < PRECISION
        assert np.linalg.norm(controller.e_E_XYZ - e_E_XYZ_exp) < PRECISION
        assert np.linalg.norm(controller.e_N_XYZ - e_N_XYZ_exp) < PRECISION
        assert np.linalg.norm(controller.e_z_XYZ - e_z_XYZ_exp) < PRECISION

    def test_orientation_callback(
        self,
        controller: axis_ptz_controller.AxisPtzController,
        config_msg: Dict[Any, Any],
        orientation_msg_90s: Dict[Any, Any],
    ) -> None:
        # Align ENz with XYZ
        _client = None
        _userdata = None
        controller._config_callback(_client, _userdata, config_msg)

        # Use 90 degree rotations
        controller._orientation_callback(_client, _userdata, orientation_msg_90s)

        # Assign expected values by performing some mental rotation
        # gymnastics
        alpha = 90.0
        q_alpha_exp = axis_ptz_utilities.as_rotation_quaternion(
            alpha, np.array([0.0, 0.0, -1.0])
        )  # About -w or -Z
        beta = 90.0
        q_beta_exp = axis_ptz_utilities.as_rotation_quaternion(
            beta, np.array([0.0, -1.0, 0.0])
        )  # About u_alpha or -Y
        gamma = 90.0
        q_gamma_exp = axis_ptz_utilities.as_rotation_quaternion(
            gamma, np.array([0.0, 0.0, 1.0])
        )  # About v_beta_alpha or Z
        E_XYZ_to_uvw_exp = np.array(
            [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]]  # [X, Z, -Y]
        )

        assert qnorm(controller.q_alpha - q_alpha_exp) < PRECISION
        assert qnorm(controller.q_beta - q_beta_exp) < PRECISION
        assert qnorm(controller.q_gamma - q_gamma_exp) < PRECISION
        assert np.linalg.norm(controller.E_XYZ_to_uvw - E_XYZ_to_uvw_exp) < PRECISION

    def test_compute_angle_delta(
        self, controller: axis_ptz_controller.AxisPtzController
    ) -> None:
        assert round(controller._compute_angle_delta(1.0, 359.0)) == -2.0
        assert round(controller._compute_angle_delta(359.0, 1.0)) == +2.0

    def test_compute_pan_rate_index(
        self, controller: axis_ptz_controller.AxisPtzController
    ) -> None:
        assert controller._compute_pan_rate_index(-PAN_RATE_MAX * 2.0) == -100
        assert controller._compute_pan_rate_index(0.0) == 0
        assert controller._compute_pan_rate_index(PAN_RATE_MAX * 2.0) == +100

    def test_compute_tilt_rate_index(
        self, controller: axis_ptz_controller.AxisPtzController
    ) -> None:
        assert controller._compute_tilt_rate_index(-TILT_RATE_MAX * 2.0) == -100
        assert controller._compute_tilt_rate_index(0.0) == 0
        assert controller._compute_tilt_rate_index(TILT_RATE_MAX * 2.0) == +100

    def test_object_callback(
        self,
        controller: axis_ptz_controller.AxisPtzController,
        config_msg: Dict[Any, Any],
        orientation_msg_0s: Dict[Any, Any],
        object_msg: Dict[Any, Any],
    ) -> None:
        # Align ENz with XYZ
        _client = None
        _userdata = None
        controller._config_callback(_client, _userdata, config_msg)

        # Use 0 degree rotations so uvw aligns with XYZ
        controller._orientation_callback(_client, _userdata, orientation_msg_0s)

        # Align velocity along the line of sight
        controller._object_callback(_client, _userdata, object_msg)

        # Compute expected values
        r_uvw_t = axis_ptz_utilities.compute_r_XYZ(LAMBDA_T, VARPHI_T, H_T)
        r_uvw_o = axis_ptz_utilities.compute_r_XYZ(LAMBDA_O, VARPHI_O, H_O)
        r_uvw_o_t = r_uvw_o - r_uvw_t
        # Expect +/-180.0
        rho_o_exp = math.fabs(math.degrees(math.atan2(r_uvw_o_t[0], r_uvw_o_t[1])))
        tau_o_exp = math.degrees(
            math.atan2(r_uvw_o_t[2], math.sqrt(r_uvw_o_t[0] ** 2 + r_uvw_o_t[1] ** 2))
        )
        delta_rho_dot_c_exp = PAN_GAIN * rho_o_exp  # Since rho_c = 0.0
        delta_tau_dot_c_exp = TILT_GAIN * tau_o_exp  # Since tau_c = 0.0
        r_rst_o_0_t = np.array([0.0, axis_ptz_utilities.norm(r_uvw_o_t), 0.0])
        v_rst_o_0_t = np.array([0.0, AIR_SPEED, 0.0])

        assert math.fabs(controller.rho_o) == rho_o_exp
        assert controller.tau_o == tau_o_exp
        assert controller.delta_rho_dot_c == delta_rho_dot_c_exp
        assert controller.delta_tau_dot_c == delta_tau_dot_c_exp
        assert np.linalg.norm(controller.r_rst_o_0_t - r_rst_o_0_t) < PRECISION
        # Magnitude of velocity difference less than 0.02% of velocity magnitude
        assert (
            100
            * np.linalg.norm(controller.v_rst_o_0_t - v_rst_o_0_t)
            / np.linalg.norm(v_rst_o_0_t)
            < RELATIVE_DIFFERENCE / 100
        )


class TestAxisPtzUtilities:
    """Test construction of directions, a corresponding direction
    cosine matrix, and quaternions."""

    # Rotate east through Y, -X, and -Y
    @pytest.mark.parametrize(
        "o_lambda, e_E_XYZ_exp",
        [
            (0.0, np.array([0.0, 1.0, 0.0])),
            (90.0, np.array([-1.0, 0.0, 0.0])),
            (180.0, np.array([0.0, -1.0, 0.0])),
        ],
    )
    def test_compute_e_E_XYZ(
        self, o_lambda: float, e_E_XYZ_exp: npt.NDArray[np.float64]
    ) -> None:
        e_E_XYZ_act = axis_ptz_utilities.compute_e_E_XYZ(o_lambda)
        assert np.linalg.norm(e_E_XYZ_act - e_E_XYZ_exp) < PRECISION

    # Rotate north through Z, Z, and between -X and Z
    @pytest.mark.parametrize(
        "o_lambda, o_varphi, e_N_XYZ_exp",
        [
            (0.0, 0.0, np.array([0.0, 0.0, 1.0])),
            (90.0, 0.0, np.array([0.0, 0.0, 1.0])),
            (
                0.0,
                45.0,
                np.array([-1.0 / math.sqrt(2.0), 0.0, 1.0 / math.sqrt(2.0)]),
            ),
        ],
    )
    def test_compute_e_N_XYZ(
        self, o_lambda: float, o_varphi: float, e_N_XYZ_exp: npt.NDArray[np.float64]
    ) -> None:
        e_N_XYZ_act = axis_ptz_utilities.compute_e_N_XYZ(o_lambda, o_varphi)
        assert np.linalg.norm(e_N_XYZ_act - e_N_XYZ_exp) < PRECISION

    # Rotate zenith through X, Y, and Z
    @pytest.mark.parametrize(
        "o_lambda, o_varphi, e_z_XYZ_exp",
        [
            (0.0, 0.0, np.array([1.0, 0.0, 0.0])),
            (90.0, 0.0, np.array([0.0, 1.0, 0.0])),
            (0.0, 90.0, np.array([0.0, 0.0, 1.0])),
        ],
    )
    def test_compute_e_z_XYZ(
        self, o_lambda: float, o_varphi: float, e_z_XYZ_exp: npt.NDArray[np.float64]
    ) -> None:
        e_z_XYZ_act = axis_ptz_utilities.compute_e_z_XYZ(o_lambda, o_varphi)
        assert np.linalg.norm(e_z_XYZ_act - e_z_XYZ_exp) < PRECISION

    # Rotate zenith to between X, Y, and Z
    @pytest.mark.parametrize(
        "o_lambda, o_varphi, E_exp",
        [
            (
                45.0,
                45.0,
                np.array(
                    [
                        [-1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2), 0.0],
                        [-0.5, -0.5, 1.0 / math.sqrt(2)],
                        [0.5, 0.5, 1.0 / math.sqrt(2)],
                    ]
                ),
            ),
        ],
    )
    def test_compute_E_XYZ_to_ENz(
        self, o_lambda: float, o_varphi: float, E_exp: npt.NDArray[np.float64]
    ) -> None:
        E_act, _, _, _ = axis_ptz_utilities.compute_E_XYZ_to_ENz(o_lambda, o_varphi)
        assert np.linalg.norm(E_act - E_exp) < PRECISION

    # Compute two positions at the equator, and one at the pole
    @pytest.mark.parametrize(
        "o_lambda, o_varphi, o_h, r_XYZ_exp",
        [
            (0.0, 0.0, 0.0, np.array([axis_ptz_utilities.R_OPLUS, 0.0, 0.0])),
            (90.0, 0.0, 0.0, np.array([0.0, axis_ptz_utilities.R_OPLUS, 0.0])),
            (0.0, 90.0, 0.0, np.array([0.0, 0.0, R_pole()])),
        ],
    )
    def test_compute_r_XYZ(
        self,
        o_lambda: float,
        o_varphi: float,
        o_h: float,
        r_XYZ_exp: npt.NDArray[np.float64],
    ) -> None:
        r_XYZ_act = axis_ptz_utilities.compute_r_XYZ(o_lambda, o_varphi, o_h)
        # Decrease precision to accommodate R_OPLUS [ft]
        assert np.linalg.norm(r_XYZ_act - r_XYZ_exp) < 10000 * PRECISION

    # Construct quaternions from a numpy.ndarray
    @pytest.mark.parametrize(
        "s, v, q_exp",
        [
            (0.0, np.array([1.0, 2.0, 3.0]), quaternion.quaternion(0.0, 1.0, 2.0, 3.0)),
        ],
    )
    def test_as_quaternion(
        self, s: float, v: npt.NDArray[np.float64], q_exp: quaternion.quaternion
    ) -> None:
        q_act = axis_ptz_utilities.as_quaternion(s, v)
        assert np.equal(q_act, q_exp).any()

    # Construct rotation quaternions from numpy.ndarrays
    @pytest.mark.parametrize(
        "s, v, r_exp",
        [
            (0.0, np.array([1.0, 2.0, 3.0]), quaternion.quaternion(1.0, 0.0, 0.0, 0.0)),
            (
                180.0,
                np.array([1.0, 2.0, 3.0]),
                quaternion.quaternion(0.0, 1.0, 2.0, 3.0),
            ),
        ],
    )
    def test_as_rotation_quaternion(
        self, s: float, v: npt.NDArray[np.float64], r_exp: quaternion.quaternion
    ) -> None:
        r_act = axis_ptz_utilities.as_rotation_quaternion(s, v)
        assert qnorm(r_act - r_exp) < PRECISION

    # Get the vector part of a vector quaternion
    @pytest.mark.parametrize(
        "q, v_exp",
        [
            (quaternion.quaternion(0.0, 1.0, 2.0, 3.0), np.array([1.0, 2.0, 3.0])),
        ],
    )
    def test_as_vector(
        self, q: quaternion.quaternion, v_exp: npt.NDArray[np.float64]
    ) -> None:
        v_act = axis_ptz_utilities.as_vector(q)
        assert np.equal(v_act, v_exp).any()

    # Compute the cross product of two vectors
    def test_cross(self) -> None:
        u = np.array([2.0, 3.0, 4.0])
        v = np.array([3.0, 4.0, 5.0])
        w_exp = np.array([-1, 2, -1])
        w_act = axis_ptz_utilities.cross(u, v)
        assert np.equal(w_act, w_exp).any()

        # Test using external package
        w_npq = np.cross(u, v)
        assert np.equal(w_npq, w_exp).any()

    # Compute the Euclidean norm of a vector
    def test_norm(self) -> None:
        v = np.array([3.0, 4.0, 5.0])
        n_exp = math.sqrt(50)
        n_act = axis_ptz_utilities.norm(v)
        assert n_exp == n_act

    # Compute camera rotations aligning ENz with XYZ and using only 90
    # deg rotations
    def test_compute_camera_rotations(self) -> None:
        # Align ENz with XYZ
        o_lambda = 270.0
        o_varphi = 90.0
        e_E_XYZ = axis_ptz_utilities.compute_e_E_XYZ(o_lambda)  # [1, 0, 0] or X
        e_N_XYZ = axis_ptz_utilities.compute_e_N_XYZ(
            o_lambda, o_varphi
        )  # [0, 1, 0] or Y
        e_z_XYZ = axis_ptz_utilities.compute_e_z_XYZ(
            o_lambda, o_varphi
        )  # [0, 0, 1] or Z

        # Only use 90 degree rotations
        alpha = 90.0
        beta = 90.0
        gamma = 90.0
        rho = 90.0
        tau = 90.0

        # Perform some mental rotation gymnastics
        q_alpha_exp = axis_ptz_utilities.as_rotation_quaternion(
            alpha, np.array([0.0, 0.0, -1.0])
        )  # About -w or -Z
        q_beta_exp = axis_ptz_utilities.as_rotation_quaternion(
            beta, np.array([0.0, -1.0, 0.0])
        )  # About u_alpha or -Y
        q_gamma_exp = axis_ptz_utilities.as_rotation_quaternion(
            gamma, np.array([0.0, 0.0, 1.0])
        )  # About v_beta_alpha or Z
        E_XYZ_to_uvw_exp = np.array(
            [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]]  # [X, Z, -Y]
        )
        q_rho_exp = axis_ptz_utilities.as_rotation_quaternion(
            rho, np.array([0.0, 1.0, 0.0])
        )  # About -t_gamma_beta_alpha or Y
        q_tau_exp = axis_ptz_utilities.as_rotation_quaternion(
            tau, np.array([0.0, 0.0, -1.0])
        )  # About r_rho_gamma_beta_alpha or -Z
        E_XYZ_to_rst_exp = np.array(
            [[0.0, 0.0, -1.0], [0.0, -1.0, 0.0], [-1.0, 0.0, 0.0]]  # [-Z, -N, -E]
        )

        # Compute and compare the camera rotations
        (
            q_alpha_act,
            q_beta_act,
            q_gamma_act,
            E_XYZ_to_uvw_act,
            q_rho_act,
            q_tau_act,
            E_XYZ_to_rst_act,
        ) = axis_ptz_utilities.compute_camera_rotations(
            e_E_XYZ, e_N_XYZ, e_z_XYZ, alpha, beta, gamma, rho, tau
        )
        assert qnorm(q_alpha_act - q_alpha_exp) < PRECISION
        assert qnorm(q_beta_act - q_beta_exp) < PRECISION
        assert qnorm(q_gamma_act - q_gamma_exp) < PRECISION
        assert np.linalg.norm(E_XYZ_to_uvw_act - E_XYZ_to_uvw_exp) < PRECISION
        assert qnorm(q_rho_act - q_rho_exp) < PRECISION
        assert qnorm(q_tau_act - q_tau_exp) < PRECISION
        assert np.linalg.norm(E_XYZ_to_rst_act - E_XYZ_to_rst_exp) < PRECISION

    # Compute the great-circle distance between two points on a sphere
    # separated by a quarter circumference
    @pytest.mark.parametrize(
        "lambda_1, varphi_1, lambda_2, varphi_2, d_exp",
        [
            (0.0, 0.0, 90.0, 0.0, math.pi * axis_ptz_utilities.R_OPLUS / 2.0),
            (0.0, 0.0, 0.0, 90.0, math.pi * axis_ptz_utilities.R_OPLUS / 2.0),
        ],
    )
    def test_great_circle_distance(
        self,
        lambda_1: float,
        varphi_1: float,
        lambda_2: float,
        varphi_2: float,
        d_exp: float,
    ) -> None:
        d_act = axis_ptz_utilities.compute_great_circle_distance(
            lambda_1, varphi_1, lambda_2, varphi_2
        )
        assert math.fabs((d_act - d_exp) / d_exp) < PRECISION
