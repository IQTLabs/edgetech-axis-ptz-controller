import base64
import contextlib
from datetime import datetime, timezone
import logging
import math
import os
from pathlib import Path
from typing import cast, Generator, Tuple, Union

import numpy as np
import numpy.typing as npt
import quaternion

# WGS84 parameters
R_OPLUS = 6378137 #6371008.7714 #6378137  # [m]
F_INV = 298.257223563


def compute_e_E_XYZ(d_lambda: float) -> npt.NDArray[np.float64]:
    """Compute components of the east unit vector at a given geodetic
    longitude.

    Parameters
    ----------
    d_lambda : float
        Geodetic longitude [deg]

    Returns
    -------
    e_E_XYZ : npt.NDArray[np.float64]
        Components of the east unit vector in an Earth fixed
        geocentric equatorial coordinate system
    """
    r_lambda = math.radians(d_lambda)
    e_E_XYZ = np.array([-math.sin(r_lambda), math.cos(r_lambda), 0.0])
    return e_E_XYZ


def compute_e_N_XYZ(d_lambda: float, d_varphi: float) -> npt.NDArray[np.float64]:
    """Compute components of the north unit vector at a given geodetic
    longitude and latitude.

    Parameters
    ----------
    d_lambda : float
        Geodetic longitude [deg]
    d_varphi : float
        Geodetic latitude [deg]

    Returns
    -------
    e_N_XYZ : npt.NDArray[np.float64]
        Components of the north unit vector in an Earth fixed
        geocentric equatorial coordinate system
    """
    r_lambda = math.radians(d_lambda)
    r_varphi = math.radians(d_varphi)
    e_N_XYZ = np.array(
        [
            -math.sin(r_varphi) * math.cos(r_lambda),
            -math.sin(r_varphi) * math.sin(r_lambda),
            math.cos(r_varphi),
        ]
    )
    return e_N_XYZ


def compute_e_z_XYZ(d_lambda: float, d_varphi: float) -> npt.NDArray[np.float64]:
    """Compute components of the zenith unit vector at a given
    geodetic longitude and latitude.

    Parameters
    ----------
    d_lambda : float
        Geodetic longitude [deg]
    d_varphi : float
        Geodetic latitude [deg]

    Returns
    -------
    e_z_XYZ : npt.NDArray[np.float64]
        Components of the zenith unit vector in an Earth fixed
        geocentric equatorial coordinate system
    """
    r_lambda = math.radians(d_lambda)
    r_varphi = math.radians(d_varphi)
    e_z_XYZ = np.array(
        [
            math.cos(r_varphi) * math.cos(r_lambda),
            math.cos(r_varphi) * math.sin(r_lambda),
            math.sin(r_varphi),
        ]
    )
    return e_z_XYZ


def compute_E_XYZ_to_ENz(
    d_lambda: float, d_varphi: float
) -> Tuple[
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
]:
    """Compute orthogonal transformation matrix from geocentric to
    topocentric coordinates.

    Parameters
    ----------
    d_lambda : float
        Geodetic longitude [deg]
    d_varphi : float
        Geodetic latitude [deg]

    Returns
    -------
    E_XYZ_to_ENz : npt.NDArray[np.float64]
        Orthogonal transformation matrix from geocentric to
        topocentric coordinates
    """
    e_E_XYZ = compute_e_E_XYZ(d_lambda)
    e_N_XYZ = compute_e_N_XYZ(d_lambda, d_varphi)
    e_z_XYZ = compute_e_z_XYZ(d_lambda, d_varphi)
    E_XYZ_to_ENz = np.row_stack((e_E_XYZ, e_N_XYZ, e_z_XYZ))
    return E_XYZ_to_ENz, e_E_XYZ, e_N_XYZ, e_z_XYZ


def compute_r_XYZ(
    d_lambda: Union[float, npt.NDArray[np.float64]],
    d_varphi: Union[float, npt.NDArray[np.float64]],
    o_h: Union[float, npt.NDArray[np.float64]],
) -> npt.NDArray[np.float64]:
    """Compute the position given geodetic longitude and latitude, and
    altitude.

    Parameters
    ----------
    d_lambda : float | npt.NDArray[np.float64]
        Geodetic longitude [deg]
    d_varphi : float | npt.NDArray[np.float64]
        Geodetic latitude [deg]
    o_h : float | npt.NDArray[np.float64]
        Altitude [m]

    Returns
    -------
    r_XYZ : npt.NDArray[np.float64]
        Position in an Earth fixed geocentric equatorial
        coordinate system [m]
    """
    f = 1.0 / F_INV
    if type(d_lambda) == float:
        r_lambda = math.radians(d_lambda)
        r_varphi = math.radians(d_varphi)
        N = R_OPLUS / math.sqrt(1.0 - f * (2.0 - f) * math.sin(r_varphi) ** 2)
        r_XYZ = np.array(
            [
                (N + o_h) * math.cos(r_varphi) * math.cos(r_lambda),
                (N + o_h) * math.cos(r_varphi) * math.sin(r_lambda),
                ((1.0 - f) ** 2 * N + o_h) * math.sin(r_varphi),
            ]
        )
    elif type(d_lambda) == np.ndarray:
        r_lambda_np = np.radians(d_lambda)
        r_varphi_np = np.radians(d_varphi)
        N = R_OPLUS / np.sqrt(1.0 - f * (2.0 - f) * np.sin(r_varphi_np) ** 2)
        r_XYZ = np.row_stack(
            (
                (N + o_h) * np.cos(r_varphi_np) * np.cos(r_lambda_np),
                (N + o_h) * np.cos(r_varphi_np) * np.sin(r_lambda_np),
                ((1.0 - f) ** 2 * N + o_h) * np.sin(r_varphi_np),
            ),
        )
    else:
        logging.error(f"compute_r_XYZ - Invalid data type for d_lambda: {type(d_lambda)}")
        raise ValueError("Invalid data type for d_lambda")
    return r_XYZ


def as_quaternion(s: float, v: npt.NDArray[np.float64]) -> quaternion.quaternion:
    """Construct a quaternion given a scalar and vector.

    Parameters
    ----------
    s : float
        A scalar value
    v : npt.NDArray[np.float64]
        A vector of floats

    Returns
    -------
    quaternion.quaternion
        A quaternion with the specified scalar and vector parts
    """
    return quaternion.quaternion(s, v[0], v[1], v[2])


def as_rotation_quaternion(
    d_omega: float, u: npt.NDArray[np.float64]
) -> quaternion.quaternion:
    """Construct a rotation quaternion given an angle and direction of
    rotation.

    Parameters
    ----------
    d_omega : float
        An angle [deg]
    u : npt.NDArray[np.float64]
        A vector of floats

    Returns
    -------
    quaternion.quaternion
        A rotation quaternion with the specified angle and direction
    """
    r_omega = math.radians(d_omega)
    v = math.sin(r_omega / 2.0) * u
    return quaternion.quaternion(math.cos(r_omega / 2.0), v[0], v[1], v[2])


def as_vector(q: quaternion.quaternion) -> npt.NDArray[np.float64]:
    """Return the vector part of a quaternion.

    Parameters
    ----------
    q : quaternion.quaternion
        A quaternion, assumed to be a vector quaternion with scalar
        part zero

    Returns
    -------
    npt.NDArray[np.float64]
       A vector of floats
    """
    return np.array([q.x, q.y, q.z])


def cross(
    u: npt.NDArray[np.float64], v: npt.NDArray[np.float64]
) -> npt.NDArray[np.float64]:
    """Compute the cross product of two vectors.

    Parameters
    ----------
    u: npt.NDArray[np.float64]
       A vector of floats
    v: npt.NDArray[np.float64]
       A vector of floats

    Returns
    -------
    v: npt.NDArray[np.float64]
       The cross product vector of floats
    """
    w = np.array([0.0, 0.0, 0.0])
    w[0] = u[1] * v[2] - u[2] * v[1]
    w[1] = u[2] * v[0] - u[0] * v[2]
    w[2] = u[0] * v[1] - u[1] * v[0]
    return w


def norm(v: npt.NDArray[np.float64]) -> float:
    """Compute the Euclidean norm of a vector.

    Parameters
    ----------
    v: npt.NDArray[np.float64]
       A vector of floats

    Returns
    -------
    float
       the Euclidean norm of the vector
    """
    s = 0.0
    for i in range(len(v)):
        s += v[i] ** 2
    return math.sqrt(s)

def compute_angle_delta( theta_c: float, theta_o: float) -> float:
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
    if (d > 1.0) or (d < -1.0):
        logging.info(
            f"D fail - theta_c: {theta_c}, theta_o: {theta_o}, d: {d}, c: {c}"
        )
        return 0
    return math.degrees(math.acos(d)) * c / math.fabs(c)

def compute_camera_rotations(
    e_E_XYZ: npt.NDArray[np.float64],
    e_N_XYZ: npt.NDArray[np.float64],
    e_z_XYZ: npt.NDArray[np.float64],
    alpha: float,
    beta: float,
    gamma: float,
    rho: float,
    tau: float,
) -> Tuple[
    quaternion.quaternion,
    quaternion.quaternion,
    quaternion.quaternion,
    npt.NDArray[np.float64],
    quaternion.quaternion,
    quaternion.quaternion,
    npt.NDArray[np.float64],
]:
    """Compute the rotations from the geocentric (XYZ) coordinate
    system to the camera housing fixed (uvw) and camera fixed (rst)
    coordinate systems.

    Parameters
    ----------
    e_E_XYZ : npt.NDArray[np.float64]
        East unit vector
    e_N_XYZ : npt.NDArray[np.float64]
        North unit vector
    e_z_XYZ : npt.NDArray[np.float64]
        Zenith unit vector
    alpha : float
        Yaw angle about -w axis [deg]
    beta : float
        Pitch angle about u axis [deg]
    gamma : float
        Roll angle about v axis [deg]
    rho : float
        Pan angle about -t axis [deg]
    tau : float
        Tilt angle about w axis [deg]

    Returns
    -------
    q_alpha : quaternion.quaternion
        Yaw rotation quaternion
    q_beta : quaternion.quaternion
        Pitch rotation quaternion
    q_gamma : quaternion.quaternion
        Roll rotation quaternion
    E_XYZ_to_uvw : npt.NDArray[np.float64]
        Orthogonal transformation matrix from XYZ to uvw
    q_rho : quaternion.quaternion
        Pan rotation quaternion
    q_tau : quaternion.quaternion
        Tilt rotation quaternion
    E_XYZ_to_rst : npt.NDArray[np.float64]
        Orthogonal transformation matrix from XYZ to rst
    """
    # Assign unit vectors of the camera housing fixed (uvw) coordinate
    # system prior to rotation
    e_u_XYZ = e_E_XYZ
    e_v_XYZ = e_N_XYZ
    e_w_XYZ = e_z_XYZ

    # Construct the yaw rotation quaternion
    q_alpha = as_rotation_quaternion(alpha, -e_w_XYZ)

    # Construct the pitch rotation quaternion
    e_u_XYZ_alpha = as_vector(
        q_alpha * as_quaternion(0.0, e_u_XYZ) * q_alpha.conjugate()
    )
    q_beta = as_rotation_quaternion(beta, e_u_XYZ_alpha)

    # Construct the roll rotation quaternion
    q_beta_alpha = q_beta * q_alpha
    e_v_XYZ_beta_alpha = as_vector(
        q_beta_alpha * as_quaternion(0.0, e_v_XYZ) * q_beta_alpha.conjugate()
    )
    q_gamma = as_rotation_quaternion(gamma, e_v_XYZ_beta_alpha)

    # Compute the orthogonal transformation matrix from the geocentric
    # (XYZ) to the camera housing fixed (uvw) coordinate system
    q_gamma_beta_alpha = q_gamma * q_beta_alpha
    e_u_XYZ_gamma_beta_alpha = as_vector(
        q_gamma_beta_alpha
        * as_quaternion(0.0, e_u_XYZ)
        * q_gamma_beta_alpha.conjugate()
    )
    e_v_XYZ_gamma_beta_alpha = as_vector(
        q_gamma_beta_alpha
        * as_quaternion(0.0, e_v_XYZ)
        * q_gamma_beta_alpha.conjugate()
    )
    e_w_XYZ_gamma_beta_alpha = as_vector(
        q_gamma_beta_alpha
        * as_quaternion(0.0, e_w_XYZ)
        * q_gamma_beta_alpha.conjugate()
    )
    E_XYZ_to_uvw = np.row_stack(
        (
            e_u_XYZ_gamma_beta_alpha,
            e_v_XYZ_gamma_beta_alpha,
            e_w_XYZ_gamma_beta_alpha,
        )
    )

    # Assign unit vectors of the camera fixed (rst) coordinate system
    # prior to rotation
    e_r_XYZ = e_u_XYZ
    e_s_XYZ = e_v_XYZ
    e_t_XYZ = e_w_XYZ

    # Construct the pan rotation quaternion
    e_t_XYZ_gamma_beta_alpha = as_vector(
        q_gamma_beta_alpha
        * as_quaternion(0.0, e_t_XYZ)
        * q_gamma_beta_alpha.conjugate()
    )
    q_rho = as_rotation_quaternion(rho, -e_t_XYZ_gamma_beta_alpha)

    # Construct the tilt rotation quaternion
    q_rho_gamma_beta_alpha = q_rho * q_gamma_beta_alpha
    e_r_XYZ_rho_gamma_beta_alpha = as_vector(
        q_rho_gamma_beta_alpha
        * as_quaternion(0.0, e_r_XYZ)
        * q_rho_gamma_beta_alpha.conjugate()
    )
    q_tau = as_rotation_quaternion(tau, e_r_XYZ_rho_gamma_beta_alpha)

    # Compute the orthogonal transformation matrix from the geocentric
    # (XYZ) to the camera fixed (rst) coordinate system
    q_tau_rho_gamma_beta_alpha = q_tau * q_rho_gamma_beta_alpha
    e_r_XYZ_tau_rho_gamma_beta_alpha = as_vector(
        q_tau_rho_gamma_beta_alpha
        * as_quaternion(0.0, e_r_XYZ)
        * q_tau_rho_gamma_beta_alpha.conjugate()
    )
    e_s_XYZ_tau_rho_gamma_beta_alpha = as_vector(
        q_tau_rho_gamma_beta_alpha
        * as_quaternion(0.0, e_s_XYZ)
        * q_tau_rho_gamma_beta_alpha.conjugate()
    )
    e_t_XYZ_tau_rho_gamma_beta_alpha = as_vector(
        q_tau_rho_gamma_beta_alpha
        * as_quaternion(0.0, e_t_XYZ)
        * q_tau_rho_gamma_beta_alpha.conjugate()
    )
    E_XYZ_to_rst = np.row_stack(
        (
            e_r_XYZ_tau_rho_gamma_beta_alpha,
            e_s_XYZ_tau_rho_gamma_beta_alpha,
            e_t_XYZ_tau_rho_gamma_beta_alpha,
        )
    )

    return q_alpha, q_beta, q_gamma, E_XYZ_to_uvw, q_rho, q_tau, E_XYZ_to_rst


def compute_great_circle_distance(
    lambda_1: float, varphi_1: float, lambda_2: float, varphi_2: float
) -> float:
    """Use the haversine formula to compute the great-circle distance
    between two points on a sphere given their longitudes and
    latitudes.

    See:
        https://en.wikipedia.org/wiki/Haversine_formula

    Parameters
    ----------
    lambda_1 : float
        Longitude [deg]
    varphi_1 : float
        Latitude [deg]
    lambda_2 : float
        Longitude [deg]
    varphi_2 : float
        Latitude [deg]

    Returns
    -------
    float
        Great-circle distance [m]

    """
    return (
        2.0
        * R_OPLUS
        * math.asin(
            math.sqrt(
                math.sin(math.radians((varphi_2 - varphi_1) / 2.0)) ** 2
                + math.cos(math.radians(varphi_1))
                * math.cos(math.radians(varphi_2))
                * math.sin(math.radians((lambda_2 - lambda_1) / 2.0)) ** 2
            )
        )
    )


def convert_time(time_a: Union[str, float]) -> datetime:
    """Convert object time to datetime object.

    Parameters
    ----------
    time_a : Union[str, float]
        Object time reported by ADS-B

    Returns
    -------
    datetime_a : datetime
        Object datetime object
    """
    # Parse object time as string with decimal seconds
    try:
        datetime_a = datetime.strptime(str(time_a), "%Y-%m-%d %H:%M:%S.%f")
    except Exception as e:
        logging.debug(
            f"Could not parse object time as string with decimal seconds: {e}"
        )

        # Parse object time as string
        try:
            datetime_a = datetime.strptime(str(time_a), "%Y-%m-%d %H:%M:%S")
        except Exception as e:
            logging.debug(f"Could not parse object time as string: {e}")

            # Construct datetime from object time
            try:
                datetime_a = datetime.fromtimestamp(cast(float, time_a))
            except Exception as e:
                logging.debug(f"Could not construct datetime from object time: {e}")

    return datetime_a


@contextlib.contextmanager
def pushd(new_dir: str) -> Generator[None, None, None]:
    """Change to a new working directory, and back.

    Parameters
    ----------
    new_dir : str
        Path of the target directory

    Returns
    -------
    None

    """
    try:
        old_dir = os.getcwd()
        os.chdir(new_dir)
        yield
    finally:
        os.chdir(old_dir)


# TODO: Remove if passing data through filesystem is acceptable
def encode_image(image_filepath: Path) -> str:
    """Base64 encode an image from a file.

    Parameters
    ----------
    image_filepath: Path
        Path of the image file

    Returns
    -------
    encoded_image: str
        The Base64 encoded image as an ASCII string
    """
    with open(image_filepath, "rb") as image_file:
        encoded_image = base64.b64encode(image_file.read()).decode(
            encoding="utf-8", errors="strict"
        )
    return encoded_image


# TODO: Remove if passing data through filesystem is acceptable
def decode_image(encoded_image: str) -> bytes:
    """Base64 decode an image from an ASCII string.

    Parameters
    ----------
    encoded_image: str
        A Base64 encoded image as an ASCII string

    Returns
    -------
    decoded_image: bytes
        The Base64 decoded image
    """
    decoded_image = base64.b64decode(encoded_image)
    return decoded_image
