# This Python file is able to convert Events, Frames and IMU data from the aedat format to
# a more manageable format (.csv file for Events and IMU, .png files for Frames)

import dv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def explore_aedat_file(file_path):
    with dv.AedatFile(file_path) as f:
        # Check if the file has 'names' attribute
        if hasattr(f, 'names'):
            data_streams = f.names
            print("Available data streams in AEDAT file:")
            for stream in data_streams:
                print(stream)


# Functions to load DVS data saved in aedat4
# EVENTS __________________________________________________________________________________________


def load_aedat(file_path):
    with dv.AedatFile(file_path) as f:
        events = np.hstack([packet for packet in f['events'].numpy()])  # Collect all events
    return events

# FRAMES __________________________________________________________________________________________


def load_frames(file_path):
    with dv.AedatFile(file_path) as f:
        frames = []
        for frame in f['frames']:
            frame_data = frame.image  # Assuming 'image' is the attribute holding the image data
            frames.append(frame_data)
    return frames

# IMU ____________________________________________________________________________________________


def inspect_imu_data(file_path):
    with dv.AedatFile(file_path) as f:
        # Check if the file contains IMU data
        if 'imu' in f.names:
            for imu_packet in f['imu']:  # Iterate over IMU packets
                print("Available attributes in IMU packet:")
                print(dir(imu_packet))  # List all attributes of the IMUSample object
                break
        else:
            print("No IMU data found in the AEDAT file.")


def extract_imu_data(file_path):
    with dv.AedatFile(file_path) as f:
        if 'imu' in f.names:
            imu_data = []
            for imu_packet in f['imu']:  # Iterate over IMU packets
                try:
                    imu_data.append({
                        'timestamp': imu_packet.timestamp,  # Direct attribute access
                        'accel': imu_packet.accelerometer,  # Accessing nested attributes
                        'gyro': imu_packet.gyroscope
                    })
                except AttributeError as e:
                    print(f"Error extracting IMU data: {e}")
                    continue
            return imu_data
        else:
            print("No IMU data found in the AEDAT file.")
            return None


# EXECUTION __________________________________________________________________________________________

# ______________ Specify the file path of the aedat file: ______________
file_path = 'C:/Users/Utente/Desktop/DV_projects/dvSave-2025_01_28_11_23_55.aedat4'


# ______________ To take a look of the file's contents (Events, Frames, IMU, Triggers): ______________
# explore_aedat_file(file_path)


# ______________ To save events in a .csv file: ______________
# events = load_aedat(file_path)  # A LOT OF EVENTS!
# for event in events:
#     print(event)

# df_events = pd.DataFrame(events)
# df_events.to_csv('Events.csv', index=False)  # in the current directory


# ______________ To save frames as a series of .png files: ______________
# frames = load_frames(file_path)
# i = 1
# for frame in frames:
#     plt.imsave(f'./Frames/Frame_{i}.png', frame)  # specify the desired path
#     i = i+1


# ______________ To inspect IMU data and save it in a .csv file: ______________
# inspect_imu_data(file_path)

imu_data_raw = extract_imu_data(file_path)
imu_data = []
values = []
for imu_packet in imu_data_raw:
    values = list(imu_packet.values())  # creates a list of the values contained in the IMU packet
    # converts the list elements into strings
    string_accel = f"{values[1]}"
    string_gyro = f"{values[2]}"
    # splits the strings to access the singular x, y, z values (with brackets and blank spaces)
    list_accel = string_accel.split(",")
    list_gyro = string_gyro.split(",")
    # separates the singular x, y, z values and gets rid of brackets and blanks
    accel_x = list_accel[0][1:]
    accel_y = list_accel[1][1:]
    accel_z = list_accel[2][1:-1]
    gyro_x = list_gyro[0][1:]
    gyro_y = list_gyro[1][1:]
    gyro_z = list_gyro[2][1:-1]
    # composes the final IMU dataset
    imu_data.append({
        'timestamp': values[0],
        'accel_X': accel_x,
        'accel_Y': accel_y,
        'accel_Z': accel_z,
        'gyro_X': gyro_x,
        'gyro_Y': gyro_y,
        'gyro_Z': gyro_z
    })

df_IMU = pd.DataFrame(imu_data)
df_IMU.to_csv('IMU.csv', index=False)  # in the current directory
