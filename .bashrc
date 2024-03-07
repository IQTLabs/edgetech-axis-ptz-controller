export PROJECT_NAME=skyscan
export HOSTNAME=dev_laptop

export MQTT_IP=100.98.5.102

export CONFIG_TOPIC=/${PROJECT_NAME}/${HOSTNAME}/Config/edgetech-c2/JSON
export ORIENTATION_TOPIC=/${PROJECT_NAME}/${HOSTNAME}/Orientation/edgetech-auto-orienter/JSON
export OBJECT_TOPIC=/${PROJECT_NAME}/${HOSTNAME}/Object/shipscan-c2/JSON
export IMAGE_FILENAME_TOPIC=/${PROJECT_NAME}/${HOSTNAME}/Image_Filename/edgetech-axis-ptz-controller/JSON
export IMAGE_CAPTURE_TOPIC=/${PROJECT_NAME}/${HOSTNAME}/Image_Capture/edgetech-axis-ptz-controller/JSON
export LOGGER_TOPIC=/${PROJECT_NAME}/${HOSTNAME}/Logger/edgetech-axis-ptz-controller/JSON

export CAMERA_IP=100.104.43.27
export CAMERA_USER=root
export CAMERA_PASSWORD=iqtpass1

export TRIPOD_LATITUDE=38.92218155926105
export TRIPOD_LONGITUDE=-77.22231710406469
export TRIPOD_ALTITUDE=150.0
export HEARTBEAT_INTERVAL=10
export LOOP_INTERVAL=0.1
export CAPTURE_INTERVAL=7
export CAPTURE_DIR=../../edgetech-yolo-detect/yolo-detect/data/to_sort
export LEAD_TIME=1.0
export PAN_GAIN=6.0
export PAN_RATE_MIN=0.1
export PAN_RATE_MAX=150.0
export TILT_GAIN=6.0
export TILT_RATE_MIN=0.1
export TILT_RATE_MAX=150.0
export FOCUS_SLOPE=0.0006
export FOCUS_INTERCEPT=54
export JPEG_RESOLUTION="1920x1080"
export JPEG_COMPRESSION=5
export USE_MQTT=True
export USE_CAMERA=True
export AUTO_FOCUS=True
export INCLUDE_AGE=True
export LOG_TO_MQTT=False
