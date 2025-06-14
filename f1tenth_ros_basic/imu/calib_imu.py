import json

calib = {
    'accel_offset': {'x': -20, 'y': 165, 'z': -24},
    'accel_radius': 1000,
    'mag_offset': {'x': -75, 'y': -354, 'z': 635},
    'mag_radius': 728,
    'gyro_offset': {'x': 0, 'y': -1, 'z': 1}
}

with open('bno055_calibration.json', 'w') as f:
    json.dump(calib, f, indent=4)
