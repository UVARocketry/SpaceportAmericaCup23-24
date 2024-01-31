#pragma once
#include <Eigen/Dense>
#include <string>
using namespace std;
// using Eigen::Matrix3d;
// using Eigen::Vector3d;

constexpr auto PI = 3.14159265358979323846;

class RocketSim {
public:
	typedef double Number;
	typedef Eigen::Vector<double, 3> Vector3d;
	typedef Eigen::Matrix<double, 3, 3> Matrix3d;

	bool sim_done = false;
	bool print_events = false;
	bool stop_at_apogee = true;

	// file logging
	bool log_to_file = false;
	string log_file_name = "flight.txt";

	// STATE VARIABLES & DERIVS
	// state variables
	Number time_s{};
	Number delta_time_s{};
	Vector3d position_m = Vector3d(0.0, 0.0, 0.0);
	Vector3d velocity_mps = Vector3d(0.0, 0.0, 0.0);
	Number mass_kg{};
	Number mass_flow_rate_kgps{};
	Vector3d acceleration_mps2 = Vector3d(0.0, 0.0, 0.0);

	// other variables
	Vector3d aero_accel_mps2 = Vector3d(0.0, 0.0, 0.0);
	Vector3d grav_accel_mps2 = Vector3d(0.0, 0.0, 0.0);
	Vector3d thrust_accel_mps2 = Vector3d(0.0, 0.0, 0.0);
	Vector3d rail_accel_mps2 = Vector3d(0.0, 0.0, 0.0);

	// ROCKET PROPERTIES
	// mass properties
	Number mass_empty_motor_kg{}; // mass of rocket without propellant
	Number mass_full_motor_kg{};  // mass of rocket with propellant

	// motor properties
	bool using_motor_tf = false;
	Number motor_burnout_time_s;
	bool const_thrust_tf = true; // true if using constant thrust model
	//      constant thrust
	Number const_thrust_N;
	//      variable thrust
	vector<Number> motor_times_s = { 0.0 };
	vector<Number> motor_thrust_N = { 0.0 };
	Number total_motor_impulse_Ns{};
	Number avg_motor_thrust_N{};
	Number avg_mass_flow_rate_kgps{};
	Number thrust_mag_N{};

	// aero
	Number aero_ref_area_m3{};
	bool const_aero_cd_tf = true; // true if using constant CD model
	//      constant aero cd
	Number const_aero_cd = 0.5;
	//      variable aero cd
	bool using_power_onoff_aero_cd_tf; // true if using CD power on/off model
	vector<Number> aero_cd_mach = { 0.0 };
	vector<Number> aero_cd_power_off = { 0.0 };
	vector<Number> aero_cd_power_on = { 0.0 };

	// gravity properties
	bool const_grav_tf = false; // true if using constant gravity
	//      constant gravity
	Number const_grav_accel_mps2 = -9.81;
	//      variable gravity
	const Number mass_earth_kg = 5.97216787e+24;
	const Number radius_earth_m = 6378100.0;
	const Number grav_const = 6.6743e-11; // m^3 / (kg * s^2) (does not change)
	Number ref_altitude_m = 0.0; // can be set by user, also used by atmos

	//      atmosphere properties
	Number ref_temp_C = 15.04;
	Number atmos_temp_C{};
	Number atmos_pres_KPa{};
	Number atmos_dens_kgpm3{};
	Number atmos_snd_spd_mps{};

	// rail properties
	bool using_rail_tf = false; // true if using a rail
	//      if using rail
	bool off_rail = false; // if using a rail, this is set to true when the
	                       // rocket is past the rail
	Number rail_length_m{};
	Number rail_azimuth_deg{};
	Number rail_elevation_deg{};
	Vector3d rail_dir = Vector3d(0.0, 0.0, 0.0);
	//      if not using rail, the user must set the initial direction
	Number initial_azimuth_deg{};
	Number initial_elevation_deg{};
	Vector3d initial_dir = Vector3d(0.0, 0.0, 0.0);

	// CONSTRUCTORS
	// default constructor
	RocketSim();
	// RocketSim(string){}; // initialize using rocket_properties.txt file

	// int set_position(Number, Number, Number);
	// int set_velocity(Number, Number, Number);
	void set_rail(Number, Number, Number);

	void set_diameter_in(Number);

	void import_motor(string, int);
	void set_variable_motor(string, int, Number, Number);
	void import_aero_cd(string, bool);

	void step();
	void run_sim();

	Number get_altitude_ft();
	Number flight_path_angle_deg();

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
