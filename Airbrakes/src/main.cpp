#include "rocket_sim.hpp"
#include <iostream>
using namespace std;

int main() {
    RocketSim rocket = RocketSim();
    cout << "t,x,y,z,xd,yd,zd,xdd,ydd,zdd,zdd_grav\n";
    
    while (rocket.time_s <= 0.1 || rocket.flight_path_angle_deg() > 0.0) {
        cout << rocket.time_s << ",";
        cout << rocket.position_m.x() << ",";
        cout << rocket.position_m.y() << ",";
        cout << rocket.position_m.z() << ",";
        cout << rocket.velocity_mps.x() << ",";
        cout << rocket.velocity_mps.y() << ",";
        cout << rocket.velocity_mps.z() << ",";
        cout << rocket.acceleration_mps2.x() << ",";
        cout << rocket.acceleration_mps2.y() << ",";
        cout << rocket.acceleration_mps2.z() << ",";
        cout << rocket.grav_accel_mps2.z();
        cout << endl;

        /*
        cout << rocket.time_s << " s" << endl;
        cout << rocket.position_m << " m" << endl;
        cout << rocket.velocity_mps << " m/s" << endl;
        cout <<  "Total Accel: \n" << rocket.acceleration_mps2 << " m/s^2" << endl;
        cout <<  "Aero Accel: \n" << rocket.aero_accel_mps2 << " m/s^2" << endl;
        cout <<  "Grav Accel: \n" << rocket.grav_accel_mps2 << " m/s^2" << endl;
        cout <<  "Thrust Accel: \n" << rocket.thrust_accel_mps2 << " m/s^2" << endl;
        cout <<  "Rail Accel: \n" << rocket.rail_accel_mps2 << " m/s^2" << endl;
        cout <<  "Flight Angle: \n" << rocket.flight_path_angle_deg() << " deg" << endl << endl;
        */

        rocket.step();
    }

    cout << "Apogee: " << rocket.get_altitude_ft() << " ft" << endl;
    //cout << "Done." << endl;
    return 0;   
}