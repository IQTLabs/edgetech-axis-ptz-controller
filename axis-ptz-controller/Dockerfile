FROM iqtlabs/edgetech-core:latest
RUN pip3 install poetry==1.8.2

COPY ./pyproject.toml .
COPY ./poetry.lock .
RUN poetry config virtualenvs.create false
RUN poetry install --without dev

COPY ./axis_ptz_controller.py .
COPY ./axis_ptz_utilities.py .
COPY ./camera_configuration.py .
COPY ./camera_control.py .
COPY ./camera.py .
COPY ./object.py .
ENTRYPOINT [ "python3", "axis_ptz_controller.py" ]
