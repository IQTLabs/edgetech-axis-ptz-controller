import logging
from sensecam_control import vapix_control

logging.getLogger("vapix_control").setLevel(logging.CRITICAL)
logging.getLogger("sensecam_control").setLevel(logging.CRITICAL)
from typing import Tuple, Union


class CameraControl(vapix_control.CameraControl):
    """
    Extends module for control cameras AXIS using Vapix found at
    https://github.com/smartsenselab/sensecam-control.  Allows focus
    to be gettable and settable.
    """

    def absolute_move(
        self,
        pan: Union[float, None] = None,
        tilt: Union[float, None] = None,
        zoom: Union[int, None] = None,
        speed: Union[int, None] = None,
        focus: Union[int, None] = None,
    ) -> str:
        """
        Operation to move pan, tilt or zoom to an absolute destination. Sets focus.

        Args:
            pan: pans the device relative to the (0,0) position.
            tilt: tilts the device relative to the (0,0) position.
            zoom: zooms the device n steps.
            speed: speed move camera.
            focus: focus value.

        Returns:
            Returns the response from the device to the command sent.

        """
        command = {"pan": pan, "tilt": tilt, "zoom": zoom, "speed": speed}
        # Defensively only set focus if needed
        if focus is not None:
            command["focus"] = focus
        # logging.info(f"command: {command}")
        logging.disable(logging.INFO)
        try:
            self._camera_command(command)
        except Exception as e:
            logging.error(f"Error: {e}")
        logging.disable(logging.NOTSET)

    def get_ptz(self) -> Tuple[float, float, float, float]:
        """
        Operation to request PTZ status.

        Returns:
            Returns a tuple with the position and focus of the camera (P, T, Z, F)

        """
        try:
            logging.disable(logging.INFO)
            resp = self._camera_command({"query": "position"})
            logging.disable(logging.NOTSET)
            pan = float(resp.text.split()[0].split("=")[1])
            tilt = float(resp.text.split()[1].split("=")[1])
            zoom = float(resp.text.split()[2].split("=")[1])
            focus = float(resp.text.split()[3].split("=")[1])
            ptz_list = (pan, tilt, zoom, focus)
            logging.debug(f"PTZ: {ptz_list}")
            return ptz_list

        except Exception as e:
            logging.error(f"Error: {e}")
            return (0, 0, 0, 0)

    def set_focus(self, focus: Union[int, None] = None) -> str:
        """
        Sets the focus of the device that is connected to the specified camera.
        Args:
            focus: focus value.

        Returns:
            Returns the response from the device to the command sent.

        """
        try:
            return self._camera_command({"focus": focus})
        except Exception as e:
            logging.error(f"Error: {e}")
            return str(e)

    def is_connected(self) -> bool:
        """
        Operation to test camera connection.

        Returns:
            Bool indicating connection, or not

        """
        try:
            resp = self._camera_command({"query": "position"})
            if resp.status_code == 200:
                return True
        except Exception as e:
            logging.error(f"Error: {e}")
        return False
