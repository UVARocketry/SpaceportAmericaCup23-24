import matplotlib.pyplot as plt
import numpy as np
import csv

file_name = 'm1939orsim120223.csv'

#Fin Parameters
shear_modulus_psi = 40000
semi_span_in = 6.25              
root_chord_in = 15.0
tip_chord_in = 7.0
thickness_in = 0.35
fin_area_in2 = 0.5 * (root_chord_in + tip_chord_in) * semi_span_in
aspect_ratio = semi_span_in**2 / fin_area_in2
chord_ratio = tip_chord_in / root_chord_in
alt_ft = np.linspace(1, 10000, 100)

temp_F = 59 - 0.00356 * alt_ft
pres_lbspin2 = (2116 / 144) * ((temp_F + 459.7) / 518.6)**5.256
sound_speed_ftps = (1.4 * 1716.59 * (temp_F + 460))**(1/2)
fin_flutter_velocity_ftpsec = sound_speed_ftps * (shear_modulus_psi / ((1.337 * aspect_ratio**3 * pres_lbspin2 * (chord_ratio + 1)) / (2 * (aspect_ratio + 2) * (thickness_in / root_chord_in)**3)))**(1/2)
print(fin_flutter_velocity_ftpsec)

#Data from open rocket sim
def read_csv_file(file_name):
    open_rocket_alt = []
    open_rocket_velocity = []

    with open(file_name, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        for row in csv_reader:
            if len(row) >= 2: 
                open_rocket_alt.append(float(row[0]))
                open_rocket_velocity.append(float(row[1]))

    return open_rocket_alt, open_rocket_velocity

altitudes, velocities = read_csv_file(file_name)
print("Open Rocket Altitudes:", altitudes)
print("Open Rocket Velocities:", velocities)

plt.plot(alt_ft, fin_flutter_velocity_ftpsec, label = "Fin Flutter Velocity (ft/s)")
plt.plot(altitudes, velocities, label = "Open Rocket Velocity (ft/s)")
plt.title("Velocity vs. Altitude")
plt.legend()
plt.show()