#pragma once
#include <string>
#include <Eigen/Dense>
using namespace std;
using Eigen::Vector3d;
using Eigen::Matrix3d;

constexpr auto PI = 3.14159265358979323846;

class RocketSim {
    public:
        bool sim_done = false;
        bool print_events = false;
        bool stop_at_apogee = true;

        // file logging
        bool log_to_file = false;
        string log_file_name = "flight.txt";


        // STATE VARIABLES & DERIVS
        // state variables
        double time_s{};
        double delta_time_s{};
        Vector3d position_m = Vector3d(0.0, 0.0, 0.0);
        Vector3d velocity_mps = Vector3d(0.0, 0.0, 0.0);
        double mass_kg{};
        double mass_flow_rate_kgps{};
        Vector3d acceleration_mps2 = Vector3d(0.0, 0.0, 0.0);

        // other variables
        Vector3d aero_accel_mps2 = Vector3d(0.0, 0.0, 0.0);
        Vector3d grav_accel_mps2 = Vector3d(0.0, 0.0, 0.0);
        Vector3d thrust_accel_mps2 = Vector3d(0.0, 0.0, 0.0);
        Vector3d rail_accel_mps2 = Vector3d(0.0, 0.0, 0.0);

        // ROCKET PROPERTIES
        // mass properties
        double mass_empty_motor_kg{};  // mass of rocket without propellant
        double mass_full_motor_kg{};  // mass of rocket with propellant

        // motor properties
        bool using_motor_tf = false;
        double motor_burnout_time_s;
        bool const_thrust_tf = true; // true if using constant thrust model
        //      constant thrust
        double const_thrust_N;
        //      variable thrust
        vector<double> motor_times_s;
        vector<double> motor_thrust_N;

        // aero
        double aero_ref_area_m3{};
        bool const_aero_cd_tf = true; // true if using constant CD model
        //      constant aero cd
        double const_aero_cd = 0.5;
        //      variable aero cd
        bool using_power_onoff_aero_cd_tf; // true if using CD power on/off model
        vector<double> aero_cd_mach;
        vector<double> aero_cd_power_off;
        vector<double> aero_cd_power_on;

        // gravity properties
        bool const_grav_tf = false; // true if using constant gravity
        //      constant gravity
        double const_grav_accel_mps2 = -9.81;
        //      variable gravity
        const double mass_earth_kg = 5.97216787e+24;
        const double radius_earth_m = 6378100.0;
        const double grav_const = 6.6743e-11; //m^3 / (kg * s^2) (does not change)
        double ref_altitude_m = 0.0; // can be set by user, also used by atmos

        //      atmosphere properties
        double ref_temp_C = 15.04;
        double atmos_temp_C{};
        double atmos_pres_KPa{};
        double atmos_dens_kgpm3{};
        double atmos_snd_spd_mps{};

        // rail properties
        bool using_rail_tf = false; // true if using a rail
        //      if using rail
        bool off_rail = false; // if using a rail, this is set to true when the rocket is past the rail
        double rail_length_m{};
        double rail_azimuth_deg{};
        double rail_elevation_deg{};
        Vector3d rail_dir = Vector3d(0.0, 0.0, 0.0);
        //      if not using rail, the user must set the initial direction
        double initial_azimuth_deg{};
        double initial_elevation_deg{};
        Vector3d initial_dir = Vector3d(0.0, 0.0, 0.0);
        
        


        // CONSTRUCTORS
        // default constructor
        RocketSim();
        //RocketSim(string){}; // initialize using rocket_properties.txt file

        //int set_position(double, double, double);
        //int set_velocity(double, double, double);
        void set_rail(double, double, double);

        void set_diameter_in(double);
        
        //int import_motor(string);
        //int import_aero_cd(string);

        void step();
        void run_sim();

        double get_altitude_ft();
        double flight_path_angle_deg();

    private:
        void initialize();
        void log_data();
        void calc_state_derivs();
        void calc_grav_accel();
        void calc_aero_accel();
        void calc_thrust_accel();
        void calc_rail_accel();
        void calc_mass_deriv();
        void calc_state_integ();

        void calc_atmos_props();
};