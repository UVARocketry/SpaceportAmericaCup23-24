import re
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#
# NOT WORKING
#

def extract_coordinates(input_string):
    pattern = r'(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)'
    matches = re.findall(pattern, input_string)
    longitudes = [float(match[0]) for match in matches]
    latitudes = [float(match[1]) for match in matches]
    heights = [float(match[2]) for match in matches]
    return longitudes, latitudes, heights

def calculate_distance(lon1, lat1, lon2, lat2):
    R = 6371000  # Earth radius in meters
    lon1, lat1, lon2, lat2 = map(np.radians, [lon1, lat1, lon2, lat2])
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    return R * c

def simple_find_peaks(x, height):
    peaks = []
    for i in range(1, len(x)-1):
        if x[i-1] < x[i] > x[i+1] and x[i] > height:
            peaks.append(i)
    return peaks

def analyze_trajectory(longitudes, latitudes, heights):
    # Calculate distances
    distances = [0]
    for i in range(1, len(longitudes)):
        dist = calculate_distance(longitudes[i-1], latitudes[i-1], longitudes[i], latitudes[i])
        distances.append(distances[-1] + dist)
    
    # Find apogee
    apogee_index = np.argmax(heights)
    apogee_height = heights[apogee_index]
    apogee_distance = distances[apogee_index]
    
    # Estimate launch angle
    launch_index = 10  # Use 10th point to avoid potential ground noise
    dh = heights[launch_index] - heights[0]
    dd = distances[launch_index]
    launch_angle = np.degrees(np.arctan2(dh, dd))
    
    # Analyze trajectory deviations
    smooth_heights = np.convolve(heights, np.ones(5)/5, mode='valid')
    height_perturbations = simple_find_peaks(np.abs(np.array(heights[2:-2]) - smooth_heights), 1)
    
    # Analyze direction changes
    directions = np.arctan2(np.diff(latitudes), np.diff(longitudes))
    direction_changes = np.where(np.abs(np.diff(directions)) > np.radians(10))[0]
    
    return {
        'apogee_height': apogee_height,
        'apogee_distance': apogee_distance,
        'launch_angle': launch_angle,
        'height_perturbations': height_perturbations,
        'direction_changes': direction_changes
    }

# Read and extract coordinates
with open('Featherweight Tracker Data 041324_fw2kml.kml', 'r') as file:
    content = file.read()
start = content.find('<coordinates>') + len('<coordinates>')
end = content.find('</coordinates>')
coordinates_string = content[start:end].strip()
longitudes, latitudes, heights = extract_coordinates(coordinates_string)

# Analyze trajectory
analysis = analyze_trajectory(longitudes, latitudes, heights)

# Print analysis results
print(f"Total number of coordinates: {len(longitudes)}")
print(f"Maximum height (apogee): {analysis['apogee_height']:.2f} meters")
print(f"Distance to apogee: {analysis['apogee_distance']:.2f} meters")
print(f"Estimated launch angle: {analysis['launch_angle']:.2f} degrees")
print(f"Number of significant height perturbations: {len(analysis['height_perturbations'])}")
print(f"Number of significant direction changes: {len(analysis['direction_changes'])}")

# Create 3D plot
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')
scatter = ax.scatter(longitudes, latitudes, heights, c=heights, cmap='viridis')
ax.set_xlabel('Longitude')
ax.set_ylabel('Latitude')
ax.set_zlabel('Height (m)')
ax.set_title('Rocket Launch Trajectory')
plt.colorbar(scatter, label='Height (m)')

# Mark important points
ax.scatter([longitudes[0]], [latitudes[0]], [heights[0]], color='green', s=100, label='Launch')
ax.scatter([longitudes[analysis['apogee_height']]], [latitudes[analysis['apogee_height']]], 
           [analysis['apogee_height']], color='red', s=100, label='Apogee')

plt.legend()
plt.show()

# Plot height vs. distance
distances = [calculate_distance(longitudes[0], latitudes[0], lon, lat) 
             for lon, lat in zip(longitudes, latitudes)]
plt.figure(figsize=(10, 6))
plt.plot(distances, heights)
plt.xlabel('Distance (m)')
plt.ylabel('Height (m)')
plt.title('Height vs. Distance')
plt.grid(True)
plt.show()