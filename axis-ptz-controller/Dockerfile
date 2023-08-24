FROM iqtlabs/edgetech-core:latest
RUN pip3 install poetry==1.5.1

COPY ./pyproject.toml .
COPY ./poetry.lock .
RUN poetry config virtualenvs.create false
RUN poetry install --without dev

COPY ./axis_ptz_controller.py .
COPY ./axis_ptz_utilities.py .
COPY ./camera_configuration.py .
COPY ./camera_control.py .
RUN mkdir -p /sensor-data/imagery
ENTRYPOINT [ "python3", "axis_ptz_controller.py" ]
