# Author: Daniel Tohti
# Date: 9/5/24
# Description: Analyzes GPS Coordinate data from a featherweight in a .csv file format. 
# Assumes that the data comes in the default exported format from Featherweights.

# Importing data analysis packages
import pandas as pd
import numpy as np

# Importing the data itself (Change file name here)
filename = "Featherweight Tracker Data 041324.csv"
coords = pd.read_csv(filename)

# Saving only the latitude, longitude, and altitude data
coords = coords[["LAT", "LON", "Altitude AGL"]]

# Zeroing out lat/long to find change in lat/long. Subtracting each column by their respective first value.
coords["Delta LAT"] = coords["LAT"] - coords.iloc[0, 0]
coords["Delta LON"] = coords["LON"] - coords.iloc[0, 1]

# Converting deltas into meters

# LAT -> x: North Pole to equator is about 10,000 km split into 90 deg. So, 10,000km/90 deg = 111,111.111111m
coords["x (m)"] = coords["Delta LAT"]*111111.111111111111
# LON -> y: 111,111*cos(LAT) because the further you are from the equator, the lon lines become closer
coords["y (m)"] = 111111.111111111111*np.cos(coords["LAT"]*np.pi/180)*coords["Delta LON"]

# Removing unnecessary columns and rearranging
coords = coords.drop(columns=["Delta LAT", "Delta LON"])
coords = coords[["LAT", "LON", "x (m)", "y (m)", "Altitude AGL"]]

# Writing to a .csv file in the following format: [Analyzed Featherweight Tracker Data MMDDYY.csv]
written_filename = "Analyzed " + filename
coords.to_csv(written_filename, index=False)