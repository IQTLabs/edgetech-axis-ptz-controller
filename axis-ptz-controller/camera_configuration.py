# mypy: ignore-errors
import datetime
import logging
import os

import requests
from requests.auth import HTTPDigestAuth
from sensecam_control import vapix_config


class CameraConfiguration(vapix_config.CameraConfiguration):
    """
    Extends module for AXIS camera configuration using Vapix found at
    https://github.com/smartsenselab/sensecam-control.
    """

    def get_jpeg_request(
        self,
        resolution: str = None,
        camera: str = None,
        square_pixel: int = None,
        compression: int = None,
        clock: int = None,
        date: int = None,
        text: int = None,
        text_string: str = None,
        text_color: str = None,
        text_background_color: str = None,
        rotation: int = None,
        text_position: str = None,
        overlay_image: int = None,
        overlay_position: str = None,
    ) -> str:  # 5.2.4.1
        """
        The requests specified in the JPEG/MJPG section are supported by those video products
        that use JPEG and MJPG encoding.

        Args:
            resolution: Resolution of the returned image. Check the product’s Release notes.
            camera: Selects the source camera or the quad stream.
            square_pixel: Enable/disable square pixel correction. Applies only to video encoders.
            compression: Adjusts the compression level of the image.
            clock: Shows/hides the time stamp. (0 = hide, 1 = show)
            date: Shows/hides the date. (0 = hide, 1 = show)
            text: Shows/hides the text. (0 = hide, 1 = show)
            text_string: The text shown in the image, the string must be URL encoded.
            text_color: The color of the text shown in the image. (black, white)
            text_background_color: The color of the text background shown in the image.
            (black, white, transparent, semitransparent)
            rotation: Rotate the image clockwise.
            text_position: The position of the string shown in the image. (top, bottom)
            overlay_image: Enable/disable overlay image.(0 = disable, 1 = enable)
            overlay_position:The x and y coordinates defining the position of the overlay image.
            (<int>x<int>)

        Returns:
            Success ('image save' and save the image in the file folder) or Failure (Error and
            description).

        """
        payload = {
            "resolution": resolution,
            "camera": camera,
            "square_pixel": square_pixel,
            "compression": compression,
            "clock": clock,
            "date": date,
            "text": text,
            "text_string": text_string,
            "text_color": text_color,
            "text_background_color": text_background_color,
            "rotation": rotation,
            "text_position": text_position,
            "overlay_image": overlay_image,
            "overlay_position": overlay_position,
        }
        url = "http://" + self.cam_ip + "/axis-cgi/jpg/image.cgi"
        logging.debug(f"url: {url}")
        logging.debug(f"payload: {payload}")
        resp = requests.get(
            url, auth=HTTPDigestAuth(self.cam_user, self.cam_password), params=payload
        )

        if resp.status_code == 200:
            now = datetime.datetime.now()
            with open(str(now.strftime("%d-%m-%Y_%Hh%Mm%Ss")) + ".jpg", "wb") as var:
                var.write(resp.content)
            return str("Image saved")

        text = str(resp)
        text += str(resp.text)
        return text
