import requests
from typing import Any, Dict
import schedule
import time
import threading
import numpy as np
import csv

# Replace these with your camera's IP address, username, and password
camera_ip = "xx.xx.xx.xx"
username = "username"
password = "password"

csv_filename = "pan-rate-analysis.csv"

experiment_time = 120 # how many seconds at each pan rate command
# Starts fastest and moves to slowest
pan_rate_range = range(100, -1, -1)



# Save data to CSV
with open(csv_filename, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Pan Rate (%)", "Measured Pan Rate (degrees/second)"])


class cameraTestSuite():
    def __init__(
        self: Any,
        camera_ip: str,
        username: str,
        password: str,
    ):
        self.url = f"http://{camera_ip}/axis-cgi/com/ptz.cgi?"
        self.auth = requests.auth.HTTPDigestAuth(username, password)
        self.pan = 0
        self.tilt = 0
        self.zoom = 0
        self.data = []

    def reset(self):
        self.pan = 0
        self.tilt = 0
        self.data = []

    # Axis PTZ control URL and parameters
    def moveCamera(self,pan, tilt):
        response = requests.get(f'{self.url}pan={pan}&tilt={tilt}', auth=self.auth)
        # Check response
        if response.status_code != 204:
            print(f"Failed to move camera: {response.status_code}, Response: {response.text}")

    def zoomCamera(self,zoom):
        response = requests.get(f'{self.url}zoom={zoom}', auth=self.auth)
        # Check response
        if response.status_code != 204:
            print(f"Failed to move camera: {response.status_code}, Response: {response.text}")
        
    
    def moveCameraRate(self, panRate=0, tiltRate=0):
        response = requests.get(f'{self.url}continuouspantiltmove={panRate},{tiltRate}', auth=self.auth)
        # Check response
        if response.status_code != 204:
            print(f"Failed to move camera: {response.status_code}, Response: {response.text}")

    def getCurrentPosition(self):
        params = {
            "query": "position"  # This query parameter should be adjusted based on camera API documentation
        }
        # Send the request to get the camera's current position
        response_start = time.time()
        response = requests.get(self.url, params=params, auth=self.auth)
        response_time = time.time() - response_start
        if response.status_code != 200:
            print(f"Failed to get camera position: {response.status_code}, Response: {response.text}")
        
        response_str = response.content.decode('UTF-8').split('\r\n')
        response_dict = {}
        for line in response_str:
            if line:
                key, value = line.split('=')
                response_dict[key] = value
        self.pan = float(response_dict['pan'])
        self.tilt = float(response_dict['tilt'])
        self.zoom = float(response_dict['zoom'])
        response_dict['timestamp'] = time.time()
        response_dict['response_time'] = response_time
        self.data.append(response_dict)
        # Check response
        return response_dict
    
    def run_schedule(self):
        while True:
            schedule.run_pending()
            time.sleep(0.01)

    def start_pan(self, panRate=0, tiltRate=0):
        
        self.moveCameraRate(panRate, tiltRate)
        self.moveCamera(pan=0, tilt=0)
        #self.zoomCamera(1)
        time.sleep(10)
        self.moveCamera(pan=0, tilt=0)
        time.sleep(10)
        self.reset()
        schedule.every(0.05).seconds.do(self.getCurrentPosition)
        #print("Moving camera at panRate: ", panRate, "tiltRate: ", tiltRate)
        self.moveCameraRate(tiltRate=0, panRate=panRate)
        
    def start_tilt(self, panRate=0, tiltRate=0):
        
        self.moveCameraRate(panRate, tiltRate)
        self.moveCamera(pan=0, tilt=-90)
        time.sleep(10)
        self.moveCamera(pan=0, tilt=-90)
        time.sleep(10)
        self.reset()
        schedule.every(0.05).seconds.do(self.getCurrentPosition)
        #print("Moving camera at panRate: ", panRate, "tiltRate: ", tiltRate)
        self.moveCameraRate(tiltRate=tiltRate, panRate=0)
    
    def stop(self):
        self.moveCameraRate(0, 0)
        schedule.clear()

    def main(self):
        self.moveCamera(pan=0, tilt=0)
        self.zoomCamera(1)
        
        # Start the scheduler thread
        scheduler_thread = threading.Thread(target=self.run_schedule)
        scheduler_thread.daemon = True
        scheduler_thread.start()
        
camera = cameraTestSuite(camera_ip=camera_ip, username=username, password=password)
camera.main()

zoom=0
tilt_rate = 0
camera.zoomCamera(zoom)
results = []
for pan_rate in pan_rate_range:
    camera.start_pan(panRate=pan_rate)
    timestart = time.time()
    while time.time()-timestart < experiment_time:
        time.sleep(0.1)
    camera.stop()
    time.sleep(8)
    

    timestamps = [d['timestamp'] - camera.data[0]['timestamp'] for d in camera.data]
    # unwrap the pan values so they're not limited to 0-360 degrees
    pan_values = np.unwrap([float(d['pan']) for d in camera.data], period=360)

    # calculate pan rate from the first and last samples (yaw_final - yaw_initial)/(time_final-time_initial)
    measured_pan_rate = (pan_values[-1]-pan_values[0])/(timestamps[-1]-timestamps[0])
    print(f"Measured pan rate for {pan_rate}%: {measured_pan_rate:.3f} degrees/second")
    measurement = {
        'pan_rate': pan_rate,
        'measured_pan_rate': measured_pan_rate
    }
    results.append(measurement)
    pan_rates = [result['pan_rate'] for result in results]
    measured_pan_rates = [result['measured_pan_rate'] for result in results]

    with open(csv_filename, mode="a+", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([pan_rates[-1], measured_pan_rates[-1]])

print(f"Data saved to '{csv_filename}'.")