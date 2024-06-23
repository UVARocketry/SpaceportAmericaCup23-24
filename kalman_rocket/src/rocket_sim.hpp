#pragma once
#include <Eigen/Dense>
#include <string>
#include <fstream>
#include <iostream>
using namespace std;
// using Eigen::Matrix3d;
// using Eigen::Vector3d;

constexpr auto PI = 3.14159265358979323846;

template <class Number>
class RocketSim {
public:
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
    // RocketSim(string){}; // initialize using rocket_properties.txt file

    // int set_position(Number, Number, Number);
    // int set_velocity(Number, Number, Number);

    RocketSim() {

        // default
        print_events = true;
        time_s = 0.0;
        delta_time_s = 0.01;
        position_m = Vector3d(0.0, 0.0, 0.0);
        velocity_mps = Vector3d(0.0, 0.0, 0.0);

        using_motor_tf = true;
        mass_empty_motor_kg = (43.8 + 7.35) / 2.205;
        mass_full_motor_kg = 64.5 / 2.205;
        const_thrust_tf = true;
        motor_burnout_time_s = 5.3476;
        const_thrust_N = 1939.0;

        const_aero_cd = 0.557;
        set_diameter_in(6.00); // OR says 6.17

        const_grav_tf = false;
        ref_altitude_m = 1400.556; // MSL Altitude at SA Launch Site, 4595.0 ft
        // ref_temp_C = 35.0; // 95 deg F

        set_rail(17.0 / 3.28084, 0.0, 84.0);

        // initailize function must be called prior to running sim
        initialize();
    }

    void initialize() {
        // mass
        if (using_motor_tf)
            mass_kg = mass_full_motor_kg;
        else
            mass_kg = mass_empty_motor_kg;

        // motor
        /*
            TODO:
                import variable motor thrust tables, if necessary
                check for zero thrust
        */

        // aero
        /*
            TODO:
            import variable aero cd tables, if necessary
            check for no table entries for variable aero cd
        */

        // rail
        /*
            TODO:
                if not using rail, make sure taht initial AZ and EL are set
        */

        // state derivatives
        calc_state_derivs();
    }

    void step() {

        // integrate state derivatives
        // TODO: write integ function
        calc_state_integ();

        // calculate new state derivatives
        // TODO: add error checking
        calc_state_derivs();
    }

    void run_sim() {
        if (stop_at_apogee) {
            while (time_s <= 0.1 || flight_path_angle_deg() > 0.0) {
                step();

                // TODO: check for events and print

                // TODO: record data somewhere
            }

            sim_done = true;
        }
    }

    void calc_state_derivs() {
        // calculate aero acceleration
        calc_aero_accel();

        // calculate grav acceleration
        calc_grav_accel();

        // calculate thrust acceleration
        calc_thrust_accel();

        // calculate rail acceleration
        calc_rail_accel();

        // calculate total acceleration
        acceleration_mps2 = aero_accel_mps2 + grav_accel_mps2 +
                            thrust_accel_mps2 + rail_accel_mps2;

        // calculate mass flow rate
        calc_mass_deriv();
    }

    void calc_aero_accel() {
        // this assumes aero force is in opposite direction of velocity

        // Aerodynamic Force
        //      F_drag = CD * Q * S
        //      CD = coefficient of drag
        //      Q = dynamic pressure
        //      S = aero reference area

        // calculate Q
        calc_atmos_props(
        ); // need to update the local atmospheric properties first

        // calculate CD
        Number cd;
        if (const_aero_cd_tf) {
            // constant CD
            cd = const_aero_cd;
        } else {
            Number mach = velocity_mps.norm() / atmos_snd_spd_mps;

            // binary search for mach interval
            int left = 0;
            int right = aero_cd_mach.size() - 1;
            int mid = right / 2;
            while ((right - left) > 1) {
                if (mach >= aero_cd_mach[mid])
                    // set left to mid
                    left = mid;
                else
                    right = mid;

                mid = (left + right) / 2;
            }

            // TODO: implement power on/off model
            Number cd_rate =
                (aero_cd_power_off[right] - aero_cd_power_off[left]) /
                (aero_cd_mach[right] - aero_cd_mach[left]);
            cd =
                aero_cd_power_off[left] + cd_rate * (mach - aero_cd_mach[left]);
        }

        if (velocity_mps.norm() > 0.01) {
            Number vel_mag_mps = velocity_mps.norm();
            Number dyn_pres = 0.5 * atmos_dens_kgpm3 * pow(vel_mag_mps, 2.0);

            // calculate force magnitude
            Number F_aero_mag = cd * dyn_pres * aero_ref_area_m3;

            // calculate direction of aero accel (opposite in direction of
            // velocity)
            Vector3d aero_accel_dir = -1.0 * (velocity_mps / vel_mag_mps);

            // calculate & update aero accel
            aero_accel_mps2 = (F_aero_mag / mass_kg) * aero_accel_dir;
        } else {
            aero_accel_mps2 = Vector3d(0.0, 0.0, 0.0);
        }
    }

    void calc_atmos_props() {
        // uses the atmosphere model in the link below
        // https://www.grc.nasa.gov/www/k-12/airplane/atmosmet.html
        // https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/speed-of-sound-interactive/

        // calculate height
        Number height = position_m.z() + ref_altitude_m;

        // all magic numbers are from the link above
        if (height <= 11000.0) {
            atmos_temp_C = ref_temp_C - 0.00649 * height;
            atmos_pres_KPa =
                101.29 * pow((atmos_temp_C + 273.15) / (288.08), 5.256);
        } else if ((height > 11000.0) || (height <= 25000.0)) {
            atmos_temp_C = -56.46;
            atmos_pres_KPa = 22.65 * exp(1.73 - 0.000157 * height);
        } else {
            atmos_temp_C = -131.21 + 0.00299 * height;
            atmos_pres_KPa =
                2.488 * pow(((atmos_temp_C + 273.1) / 288.08), -11.388);
        }

        atmos_dens_kgpm3 = atmos_pres_KPa / (0.2869 * (atmos_temp_C + 273.15));

        atmos_snd_spd_mps = sqrt(1.4 * 286 * (atmos_temp_C + 273.15));
    }

    void calc_grav_accel() {
        if (const_grav_tf) {
            // constant gravity
            grav_accel_mps2 = Vector3d(0.0, 0.0, const_grav_accel_mps2);
        } else {
            // Newton's Law of Gravitation!!
            Number grav_accel_mag_N =
                (grav_const * mass_earth_kg) /
                pow((radius_earth_m + ref_altitude_m + position_m.z()), 2.0);
            grav_accel_mps2 = Vector3d(0.0, 0.0, -grav_accel_mag_N);
        }
    }

    void calc_thrust_accel() {
        // calculate thrust magnitude
        if (const_thrust_tf && time_s < motor_burnout_time_s) {
            // constant thrust
            thrust_mag_N = const_thrust_N;
        } else if (time_s < motor_burnout_time_s) {
            // use binary search to find the thrust times interval the current
            // time is in
            int left, right, mid;
            left = 0;
            right = motor_times_s.size() - 1;
            mid = right / 2;
            while ((right - left) > 1) {
                if (time_s >= motor_times_s[mid])
                    // left now becomes mid
                    left = mid;
                else
                    // right now becomes mid
                    right = mid;

                // mid is recalculated
                mid = (right + left) / 2;
            }

            // now we have the interval, so we can do linear interpolation
            Number slope_Nps = (motor_thrust_N[right] - motor_thrust_N[left]) /
                               (motor_times_s[right] - motor_times_s[left]);
            thrust_mag_N = motor_thrust_N[left] +
                           slope_Nps * (time_s - motor_times_s[left]);
        } else {
            // burn time exceeded, so zero thrust
            thrust_mag_N = 0.0;
        }

        // for 3DOF motion, we assume that the thrust is in the same direction
        // of velocity vector thus, if the velocity is approximately zero, we
        // can't assume that holds
        if (velocity_mps.norm() > 0.01) {
            // calculate thrust direction
            Vector3d thrust_dir = velocity_mps / velocity_mps.norm();

            // calc & update thrust
            thrust_accel_mps2 = (thrust_mag_N / mass_kg) * thrust_dir;
        } else {
            // if using rail, the thrust is in line with the rail direction
            if (using_rail_tf) {
                thrust_accel_mps2 = (thrust_mag_N / mass_kg) * rail_dir;
            } else {
                // if not using rail, thrust is in line with the initial
                // direction
                thrust_accel_mps2 = (thrust_mag_N / mass_kg) * initial_dir;
            }
        }
    }

    void calc_rail_accel() {
        // a rail lets the rocket move freely (without opposing force)
        // in its pointing direction until the rocket's position vector
        // magnitude is greater than the rail length

        // Note: we could probably improve this a bit by adding a static/kinetic
        // friction component to the pointing direction

        if (using_rail_tf && !off_rail) {

            if (position_m.norm() > rail_length_m) {
                // set the off_rail flag to true
                off_rail = true;

                // and set rail accel to 0
                rail_accel_mps2 = Vector3d(0.0, 0.0, 0.0);
            } else {
                // we want to find the acceleration without a rail
                Vector3d accel_without_rail_mps2 =
                    grav_accel_mps2 + aero_accel_mps2 + thrust_accel_mps2;

                // now we find the magnitude of the acceleration in the
                // direction of the rail using the dot product
                Number accel_along_rail_mps2 =
                    accel_without_rail_mps2.dot(rail_dir);

                // at this point, the magnitude of the acceleration in the rail
                // direction could be positive or negative (thrust not yet high
                // enough for rocket to move)
                if (accel_along_rail_mps2 <= 0.0) {
                    // the acceleration along the rail is negative
                    // so the rail needs to push up on the rocket to keep it
                    // stationary
                    rail_accel_mps2 = -accel_without_rail_mps2;
                } else {
                    // the acceleration along the rail is positive
                    // so we limit the direction of motion of the rocket to the

                    // what this statement is doing is calculating the vector
                    // acceleration along the rail, subtracting that from the
                    // acceleration without the rail, which gives us the net
                    // acceleration thats NOT along the rail. We then negate
                    // this because we want the rail to oppose this motion
                    rail_accel_mps2 =
                        -(accel_without_rail_mps2 -
                          (accel_along_rail_mps2 * rail_dir));
                }
            }
        } else {
            rail_accel_mps2 = Vector3d(0.0, 0.0, 0.0);
        }
    }

    void calc_mass_deriv() {
        if (const_thrust_tf && time_s < motor_burnout_time_s) {
            // constant thrust model for mass flow rate
            // mass flow rate is just propellant weight divided by burntime
            mass_flow_rate_kgps = (mass_full_motor_kg - mass_empty_motor_kg) /
                                  (motor_burnout_time_s);
        } else if (time_s < motor_burnout_time_s) {
            // mass flow rate is proportional to the current thrust relative to
            // the avg thrust
            mass_flow_rate_kgps =
                (thrust_mag_N / avg_motor_thrust_N) * avg_mass_flow_rate_kgps;
        } else {
            // motor done burning
            mass_flow_rate_kgps = 0.0;
        }
    }

    void set_diameter_in(Number diameter_in) {
        aero_ref_area_m3 = PI * pow((diameter_in / (2 * 39.37)), 2.0);
    }

    void set_rail(Number rail_len_m, double rail_az_deg, double rail_el_deg) {
        using_rail_tf = true;

        rail_length_m = rail_len_m;
        rail_azimuth_deg = rail_az_deg;
        rail_elevation_deg = rail_el_deg;

        // convert deg to rad
        // note that the pitch rotation matrix below has a sign convention that
        // is ccw == positive, but the user inputs cw == positive, so we need to
        // flip its sign
        Number rail_az_rad = (PI / 180) * rail_azimuth_deg;
        Number rail_el_rad = -(PI / 180) * rail_elevation_deg;

        // we basically do a pitch, then a yaw of a unit x 3d vector
        // https://msl.cs.uiuc.edu/planning/node102.html

        rail_dir = Vector3d(1.0, 0.0, 0.0);

        Matrix3d pitch_rot_mat;
        pitch_rot_mat << cos(rail_el_rad), 0.0, sin(rail_el_rad), 0.0, 1.0, 0.0,
            -sin(rail_el_rad), 0.0, cos(rail_el_rad);
        Matrix3d yaw_rot_mat;
        yaw_rot_mat << cos(rail_az_rad), -sin(rail_az_rad), 0.0,
            sin(rail_az_rad), cos(rail_az_rad), 0.0, 0.0, 0.0, 1.0;

        rail_dir = yaw_rot_mat * (pitch_rot_mat * Vector3d(1.0, 0.0, 0.0));
    }

    void calc_state_integ() {
        // Euler integration
        // time
        time_s = time_s + delta_time_s;

        // position
        position_m = position_m + velocity_mps * delta_time_s;

        // velocity
        velocity_mps = velocity_mps + acceleration_mps2 * delta_time_s;

        // mass
        mass_kg = mass_kg + mass_flow_rate_kgps * delta_time_s;
    }

    Number flight_path_angle_deg() {
        Number xy_mag =
            sqrt(pow(velocity_mps.x(), 2.0) + pow(velocity_mps.y(), 2.0));
        Number z_mag = velocity_mps.z();
        return atan(z_mag / xy_mag) * 180.0 / PI;
    }

    Number get_altitude_ft() {
        return position_m.z() * 3.28084;
    }

    void import_motor(string filename, int data_start_line) {
        using_motor_tf = true;
        const_thrust_tf = false;

        // https://www.geeksforgeeks.org/csv-file-management-using-c/

        // file pointer
        fstream fin;

        // open file
        fin.open(filename, ios::in);

        if (!fin.is_open()) {
            std::cout << "Error opening file " << filename << endl;
            return;
        }

        // note that for .eng motor files, the first couple lines can be ignored
        // so we skip over those here
        string line;
        for (int i = 0; i < data_start_line; i++) {
            getline(fin, line);
        }

        // read data from the file
        string time, thrust;
        while (fin >> time) {
            // time already read in
            // so we read in thrust
            fin >> thrust;

            // convert time and thrust strings to Numbers and push
            // to times and thrust vectors
            motor_times_s.push_back(stod(time));
            motor_thrust_N.push_back(stod(thrust));
        }

        // calculate total motor impulse using trapezoidal integration
        // of motor thrust over time
        // https://en.wikipedia.org/wiki/Trapezoidal_rule
        total_motor_impulse_Ns = 0.0;
        for (size_t i = 1; i < motor_times_s.size(); i++) {
            total_motor_impulse_Ns +=
                (motor_times_s[i] - motor_times_s[i - 1]) *
                (motor_thrust_N[i] + motor_thrust_N[i - 1]) / 2;
        }

        // set motor burnout time to last entry in motor times vector
        motor_burnout_time_s = motor_times_s[motor_times_s.size() - 1];

        // calculate average thrust
        avg_motor_thrust_N = total_motor_impulse_Ns / motor_burnout_time_s;

        // calculate average mass flow rate
        avg_mass_flow_rate_kgps =
            (mass_full_motor_kg - mass_empty_motor_kg) / motor_burnout_time_s;
    }

    void set_variable_motor(
        string filename, int data_start_line, Number rocket_mass_empty_motor_kg,
        Number rocket_mass_full_motor_kg
    ) {
        mass_empty_motor_kg = rocket_mass_empty_motor_kg;

        mass_full_motor_kg = rocket_mass_full_motor_kg;

        import_motor(filename, data_start_line);
    }

    void import_aero_cd(string filename, bool power_onoff_model_tf) {
        const_aero_cd_tf = false;
        using_power_onoff_aero_cd_tf = power_onoff_model_tf;

        fstream fin;
        fin.open(filename, ios::in);

        if (!fin.is_open()) {
            cout << "Error opening file " << filename << endl;
            return;
        }

        // the first line is just a header
        vector<string> row;
        string line, word;
        Number mach, aoa, cd = 0.0;
        getline(fin, line);

        while (fin >> line && (aoa < 0.01)) {
            stringstream sstream(line);

            // first column is the mach number
            getline(sstream, word, ',');
            mach = stod(word);

            // second column is angle of attack (ignore, should be zero)
            getline(sstream, word, ',');
            aoa = stod(word);

            // third column is CD
            getline(sstream, word, ',');
            cd = stod(word);

            // ignore the rest

            // only add mach and cd to tables if AOA is zero
            if (aoa < 0.01) {
                aero_cd_mach.push_back(mach);
                aero_cd_power_off.push_back(cd);
            }
        }
    }

    void log_data() {
        if (log_to_file) {
        }
    }
};
