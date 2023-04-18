FROM iqtlabs/edgetech-core:latest
RUN pip3 install poetry==1.4.2

COPY ./pyproject.toml .
COPY ./poetry.lock .
RUN poetry config virtualenvs.create false
RUN poetry install --without dev

COPY ./axis_ptz_controller.py .
COPY ./axis_ptz_utilities.py .
RUN mkdir -p /data/tosort
ENTRYPOINT [ "python3", "axis_ptz_controller.py" ]
