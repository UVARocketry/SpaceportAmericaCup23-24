import re
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#   requires matplotlib & numpy
#
#   install: 
#            run:     pip install -r requirements.txt
#
#   (written by claude) (prompted by paris)
#   3D plot of coordinates
#   reads coordinates from file
#   filecontent: longitude,latitude,height<space>longitude,latitude,height<space>...
#

filename = 'Featherweight Tracker Data 041324_fw2kml.kml'

def extract_coordinates(input_string):
    # Regular expression to match the coordinate pattern
    pattern = r'(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)'
    
    # Find all matches in the input string
    matches = re.findall(pattern, input_string)
    
    # Separate the coordinates into lists
    latitudes = [float(match[0]) for match in matches]
    longitudes = [float(match[1]) for match in matches]
    heights = [float(match[2]) for match in matches]
    
    return latitudes, longitudes, heights

# Read the content from the file
with open(filename, 'r') as file:
    content = file.read()

# Find the coordinates section
start = content.find('<coordinates>') + len('<coordinates>')
end = content.find('</coordinates>')
coordinates_string = content[start:end].strip()

# Extract the coordinates
latitudes, longitudes, heights = extract_coordinates(coordinates_string)

# Create a 3D scatter plot
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot the points
scatter = ax.scatter(latitudes, longitudes, heights, c=heights, cmap='viridis')

# Set labels and title
ax.set_xlabel('latitude')
ax.set_ylabel('longitude')
ax.set_zlabel('Height')
ax.set_title('3D Plot of Coordinates')

# Add a color bar
plt.colorbar(scatter, label='Height')

# Show the plot
plt.show()

# Print total number of coordinates
print(f"Total number of coordinates plotted: {len(latitudes)}")