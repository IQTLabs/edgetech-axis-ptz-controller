FROM iqtlabs/edgetech-core:latest
RUN apt-get update && apt-get install -y \
  build-essential \
  curl \
  && rm -rf /var/lib/apt/lists/*
RUN curl -sSL https://install.python-poetry.org | python3 -
RUN cp ~/.local/bin/poetry /usr/local/bin
COPY ./pyproject.toml .
COPY ./poetry.lock .
RUN poetry config virtualenvs.create false
RUN poetry install --without dev
COPY ./base_mqtt_pub_sub.py .
COPY ./axis-ptz-controller.env .
COPY ./axis_ptz_controller.py .
COPY ./axis_ptz_utilities.py .
RUN mkdir -p /data/tosort
ENTRYPOINT [ "python3", "axis_ptz_controller.py" ]
