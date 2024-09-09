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

def plot_3d(x, y, heights):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    scatter = ax.scatter(x, y, heights, c=heights, cmap='viridis')
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_zlabel('Height (meters)')
    ax.set_title('3D Plot of Coordinates')
    plt.colorbar(scatter, label='Height')
    plt.show()

def plot_2d(x, y):
    plt.figure(figsize=(10, 8))
    plt.scatter(x, y, c=range(len(x)), cmap='viridis')
    plt.colorbar(label='Point Index')
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
    
    ###key data
    print(f"Total number of coordinates: {len(x)}")
    print(f"X-coordinate range: {x.min():.2f} to {x.max():.2f} meters")
    print(f"Y-coordinate range: {y.min():.2f} to {y.max():.2f} meters")
    print(f"Height range: {min(heights):.2f} to {max(heights):.2f} meters")
    print(f"Total distance traveled: {np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2)):.2f} meters")
    ## add more

if __name__ == "__main__":
    main()