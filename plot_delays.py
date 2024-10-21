import csv
import matplotlib.pyplot as plt

# Initialize lists to store data
timestamps = []
lidar_delays = []
imu_delays = []
camera_delays = []

# Read the CSV file
with open('../../delays.csv', 'r') as file:
    reader = csv.reader(file)
    next(reader)  # Skip the header row
    for row in reader:
        timestamps.append(float(row[0]))
        lidar_delays.append(float(row[1]) if row[1] else None)
        imu_delays.append(float(row[2]) if row[2] else None)
        camera_delays.append(float(row[3]) if row[3] else None)

# Convert timestamps to relative time (seconds since start)
start_time = timestamps[0]
relative_times = [t - start_time for t in timestamps]

# Plot the delays
plt.figure(figsize=(10, 6))

# Plot LiDAR delays
plt.plot(relative_times, lidar_delays, label='LiDAR Delay', marker='o')

# Plot IMU delays
plt.plot(relative_times, imu_delays, label='IMU Delay', marker='x')

# Plot Camera delays
plt.plot(relative_times, camera_delays, label='Camera Delay', marker='s')

plt.xlabel('Time (s)')
plt.ylabel('Delay (s)')
plt.title('Message Delays Over Time')
plt.legend()
plt.grid(True)

# Save the plot as an image
plt.savefig('message_delays.png')
