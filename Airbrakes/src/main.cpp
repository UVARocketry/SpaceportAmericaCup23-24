#include "rocket_sim.hpp"
#include <iostream>
using namespace std;

int main() {
    RocketSim rocket = RocketSim();
    rocket.import_motor("AeroTech_M1939W.eng", 5);
    
    for (int i = 0; i < rocket.motor_times_s.size(); i++)
    {
        cout << rocket.motor_times_s[i] << " ";
        cout << rocket.motor_thrust_N[i] << endl;
    }

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

        rocket.step();
    }

    cout << "Apogee: " << rocket.get_altitude_ft() << " ft" << endl;
    //cout << "Done." << endl;
    return 0;   
}