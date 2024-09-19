import re
import csv

def extract_coordinates(input_string):
    pattern = r'(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)'
    matches = re.findall(pattern, input_string)
    return matches

# Read the KML file
input_file = 'Featherweight Tracker Data 041324_fw2kml.kml'
output_file = 'rocket_coordinates.csv'

with open(input_file, 'r') as file:
    content = file.read()

# Extract coordinates section
start = content.find('<coordinates>') + len('<coordinates>')
end = content.find('</coordinates>')
coordinates_string = content[start:end].strip()

# Extract coordinates
coordinates = extract_coordinates(coordinates_string)

# Write to CSV
with open(output_file, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    
    # Write header
    writer.writerow(['Longitude', 'Latitude', 'Height'])
    
    # Write coordinates
    for coord in coordinates:
        writer.writerow(coord)

print(f"Coordinates have been extracted and written to {output_file}")
print(f"Total number of coordinates: {len(coordinates)}")
