#ifndef ROCKET_SIM_HPP
#define ROCKET_SIM_HPP

#include <eigen/Eigen/Dense>
using namespace std;
using Eigen::Vector3d;

constexpr auto M_PI = 3.14159265358979323846;

class RocketSim {
    public:
        bool print_steps;
        bool print_events;
        bool stop_at_apogee;

        // STATE VARIABLES & DERIVS
        // state variables
        double time_s;
        double delta_time_s;
        Vector3d position_m;
        Vector3d velocity_mps;
        double mass_kg;
        Vector3d acceleration_mps2;

        // other variables
        Vector3d aero_accel_mps2;
        Vector3d grav_accel_mps2;
        Vector3d thrust_accel_mps2;
        Vector3d rail_accel_mps2;

        // ROCKET PROPERTIES
        // mass properties
        double mass_empty_motor_kg;  // mass of rocket without propellant
        double mass_full_motor_kg;  // mass of rocket with propellant

        // motor properties
        double motor_burnout_time_s;
        bool const_thrust_tf; // true if using constant thrust model
        //      constant thrust
        double const_thrust_N;
        //      variable thrust
        vector<double> motor_times_s;
        vector<double> motor_thrust_N;

        // aero
        double aero_ref_area_m3;
        bool const_aero_cd_tf; // true if using constant CD model
        //      constant aero cd
        double const_aero_cd;
        //      variable aero cd
        bool using_power_onoff_aero_cd_tf; // true if using CD power on/off model
        vector<double> aero_cd_mach;
        vector<double> aero_cd_power_off;
        vector<double> aero_cd_power_on;

        // gravity properties
        bool const_grav_tf; // true if using constant gravity
        //      constant gravity
        double const_grav_accel_mps2 = -9.81;
        //      variable gravity
        const double mass_earth_kg = 5.97216787e+24;
        const double radius_earth_m = 6378100.0;
        const double grav_const = 6.6743e-11; //m^3 / (kg * s^2) (does not change)
        double ref_altitude_m = 0.0; // can be set by user

        //      rail properties
        bool using_rail_tf; // true if using a rail
        double rail_length_m;
        double rail_azimuth_deg;
        double rail_elevation_deg; 
        
        //      atmosphere properties
        double atmos_temp_C;
        double atmos_pres_KPa;
        double atmos_dens_kgpm3;
        double atmos_snd_spd_mps;


        // CONSTRUCTORS
        // default constructor
        RocketSim(){};
        //RocketSim(string){}; // initialize using rocket_properties.txt file

        int set_position(double, double, double);
        int set_velocity(double, double, double);
        int set_rail(double, double, double);

        void set_diameter_in(double);
        
        int import_motor(string){};
        int import_aero_cd(string){};

        int step(){};
        int run_sim(){};

    private:
        void calc_state_derivs(){};
        void calc_grav_accel(){};
        void calc_aero_accel(){};
        void calc_atmos_props(){};
        void calc_thrust_accel(){};
        void calc_rail_accel(){};
        void calc_state_integ(){};
};

#endif