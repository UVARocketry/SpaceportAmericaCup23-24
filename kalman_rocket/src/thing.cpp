#include "rocket_sim.hpp"
#include <fstream>
#include <iostream>
// #include <ifstream>
using namespace std;

int yeet() {
    RocketSim<double> rocket = RocketSim<double>();
    rocket.import_motor("../build/AeroTech_M1939W.eng", 5);
    rocket.import_aero_cd("../build/Sabre1_CD.CSV", false);

    auto outputFile = ofstream("output.csv");
    if (!outputFile.is_open()) {
        cout << "Error opening output file." << endl;
        return 1;
    }
    auto motorDataFile = ofstream("motor.txt");

    for (size_t i = 0; i < rocket.motor_times_s.size(); i++) {
        motorDataFile << rocket.motor_times_s[i] << " ";
        motorDataFile << rocket.motor_thrust_N[i] << endl;
    }

    for (size_t i = 0; i < rocket.aero_cd_mach.size(); i++) {
        if (rocket.aero_cd_mach[i] <= 2.0) {
            motorDataFile << rocket.aero_cd_mach[i] << " ";
            motorDataFile << rocket.aero_cd_power_off[i] << endl;
        }
    }
    motorDataFile.close();

    outputFile << "t,x,y,z,xd,yd,zd,xdd,ydd,zdd,zdd_grav,atmos_pres\n";
    while (rocket.time_s <= 0.1 || rocket.flight_path_angle_deg() > 0.0) {
        if (rocket.time_s < 2.0) {
            outputFile << rocket.time_s << ",";
            outputFile << rocket.position_m.x() << ",";
            outputFile << rocket.position_m.y() << ",";
            outputFile << rocket.position_m.z() << ",";
            outputFile << rocket.velocity_mps.x() << ",";
            outputFile << rocket.velocity_mps.y() << ",";
            outputFile << rocket.velocity_mps.z() << ",";
            outputFile << rocket.acceleration_mps2.x() << ",";
            outputFile << rocket.acceleration_mps2.y() << ",";
            outputFile << rocket.acceleration_mps2.z() << ",";
            outputFile << rocket.grav_accel_mps2.z() << ",";
            outputFile << rocket.atmos_pres_KPa;
            outputFile << endl;
        }

        rocket.step();
    }
    outputFile.close();

    cout << "Apogee: " << rocket.get_altitude_ft() << " ft" << endl;
    // cout << "Done." << endl;
    return 0;
}
