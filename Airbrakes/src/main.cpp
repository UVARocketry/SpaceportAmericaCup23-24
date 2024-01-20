#include "rocket_sim.hpp"
#include <iostream>
using namespace std;

int main() {
    RocketSim rocket = RocketSim();
    rocket.import_motor("AeroTech_M1939W.eng", 5);
    rocket.import_aero_cd("Sabre1_CD.CSV", false);

    for (int i = 0; i < rocket.motor_times_s.size(); i++)
    {
        cout << rocket.motor_times_s[i] << " ";
        cout << rocket.motor_thrust_N[i] << endl;
    }

    for (int i = 0; i < rocket.aero_cd_mach.size(); i++)
    {
        if (rocket.aero_cd_mach[i] <= 2.0)
        {
        cout << rocket.aero_cd_mach[i] << " ";
        cout << rocket.aero_cd_power_off[i] << endl;
        }
    }

    cout << "t,x,y,z,xd,yd,zd,xdd,ydd,zdd,zdd_grav,atmos_pres\n";  
    while (rocket.time_s <= 0.1 || rocket.flight_path_angle_deg() > 0.0) {
        if (rocket.time_s < 2.0) {
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
        cout << rocket.grav_accel_mps2.z() << ",";
        cout << rocket.atmos_pres_KPa;
        cout << endl;}

        rocket.step();
    }
    

    cout << "Apogee: " << rocket.get_altitude_ft() << " ft" << endl;
    //cout << "Done." << endl;
    return 0;   
}