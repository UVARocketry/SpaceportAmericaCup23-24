import re
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os


##from Coordinate_Plot.py
def extract_coordinates(input_string):
    pattern = r'(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)'
    matches = re.findall(pattern, input_string)
    longitudes = [float(match[0]) for match in matches]
    latitudes = [float(match[1]) for match in matches]
    heights = [float(match[2]) for match in matches]
    return longitudes, latitudes, heights

##from LatLongConversiontoXY.py
def convert_to_xy(latitudes, longitudes):
    delta_lat = np.array(latitudes) - latitudes[0]
    delta_lon = np.array(longitudes) - longitudes[0]
    
    x = delta_lat * 111111.111111111111
    y = 111111.111111111111 * np.cos(np.array(latitudes) * np.pi / 180) * delta_lon
    
    return x, y


##from Coordinate_Plot.py
def plot_3d(x, y, heights):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    scatter = ax.scatter(x, y, heights, c=heights, cmap='viridis')
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_zlabel('height (meters)')
    ax.set_title('3D plot of coordinates')
    plt.colorbar(scatter, label='Height')
    plt.show()


def plot_2d(x, y):
    plt.figure(figsize=(10, 8))
    plt.scatter(x, y, c=range(len(x)), cmap='viridis')
    plt.colorbar(label='point index')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('2D Plot of Coordinates')
    plt.axis('equal')
    plt.show()

def main():
    filename = 'coordinate_data_analysis/Featherweight Tracker Data 041324_fw2kml.kml'

    #print("Current Working Directory:", os.getcwd())
    #print("Files in the current directory:", os.listdir())
    
    with open(filename, 'r') as file:
        content = file.read()
    
    start = content.find('<coordinates>') + len('<coordinates>')
    end = content.find('</coordinates>')
    coordinates_string = content[start:end].strip()
    
    longitudes, latitudes, heights = extract_coordinates(coordinates_string)
    x, y = convert_to_xy(latitudes, longitudes)
    
    plot_3d(x, y, heights)
    plot_2d(x, y)

    start_to_end_distance_xy = np.sqrt((x[-1] - x[0])**2 + (y[-1] - y[0])**2)

    # key data
    print(f"total numbof coordinates: {len(x)}")
    print(f"X-coordinate range: {x.min():.2f} to {x.max():.2f} meters")
    print(f"Y-coordinate range: {y.min():.2f} to {y.max():.2f} meters")
    print(f"height range: {min(heights):.2f} to {max(heights):.2f} meters")
    
    print(f"flat pure x-y coordinate space horizontal distance from start to end: {start_to_end_distance_xy:.2f} meters")

    #####
    ### I tried accounting for earth curvature but it dont work and its also def the exact same number
    #####
    start_to_end_distance_haversine =  haversine_distance(latitudes[0], longitudes[0], latitudes[-1], longitudes[-1])
    print(f"ground distance from start to end w/ haversine formula for cuvrature: {start_to_end_distance_haversine:.2f} meters")

    print(f"net altitude change: {heights[-1] - heights[0]:.2f} meters")
    print(f"Maximum altitude reached: {max(heights):.2f} meters")
    # add more ! or idk

# if we wanna be extra sweaty
# https://www.geeksforgeeks.org/haversine-formula-to-find-distance-between-two-points-on-a-sphere/
def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius in meters
    
    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])
    
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon)**2
    c = np.arctan2(np.sqrt(a), np.sqrt(a))
    
    return R * c



if __name__ == "__main__":
    main()



