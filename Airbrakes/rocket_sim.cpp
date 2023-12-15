#include "rocket_sim.hpp"
#define _USE_MATH_DEFINES
#include <cmath>

RocketSim::RocketSim()
{
    // default
    print_events = true;
    stop_at_apogee = true;
    time_s = 0.0;
    delta_time_s = 0.1;
    position_m = Vector3d(0.0, 0.0, 0.0);
    velocity_mps = Vector3d(0.0, 0.0, 0.0);

    mass_empty_motor_kg = 40.0/2.2;
    mass_full_motor_kg = 60.0/2.2;
    motor_burnout_time_s = 5.5;
    const_thrust_tf = true;
    const_thrust_N = 1939.0;

    const_aero_cd_tf = true;
    const_aero_cd = 0.5;
    set_diameter_in(6.0);

    const_grav_tf = true;
}

int RocketSim::step()
{
    // calculate state derivatives
    // TODO: add error checking
    calc_state_derivs();

    // integrate state derivatives
    calc_state_integ();
}

void RocketSim::calc_state_derivs()
{
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

void RocketSim::calc_aero_accel()
{
    // this assumes aero force is in opposite direction of velocity

    // Aerodynamic Force
    //      F_drag = CD * Q * S
    //      CD = coefficient of drag
    //      Q = dynamic pressure
    //      S = aero reference area

    // calculate CD
    double cd;
    if (const_aero_cd_tf)
    {
        // constant CD
        cd = const_aero_cd;
    }
    else
    {
        // TODO: implement variable drag coeff calc
        cd = 0.5;
    }

    // calculate Q
    calc_atmos_props();
    double vel_mag_mps = velocity_mps.norm();
    double dyn_pres = 0.5 * atmos_dens_kgpm3 * pow(vel_mag_mps, 2.0);

    // calculate force magnitude
    double F_aero_mag = cd * dyn_pres * aero_ref_area_m3;

    // calculate direction of aero accel (opposite in direction of velocity)
    Vector3d aero_accel_dir = -1.0 * (velocity_mps / vel_mag_mps);

    // calculate & update aero accel
    aero_accel_mps2 = (F_aero_mag / mass_kg) * aero_accel_dir;
}

void RocketSim::calc_atmos_props()
{
    // uses the atmosphere model in the link below
    // https://www.grc.nasa.gov/www/k-12/airplane/atmosmet.html
    // https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/speed-of-sound-interactive/

    // calculate height
    double height = position_m.z() + ref_altitude_m;

    // don't burn me at the stake
    // all magic numbers are from the link above
    if (height <= 11000.0)
    {
        atmos_temp_C = 15.04 - 0.00649 * height;
        atmos_pres_KPa = 101.29 * pow(((atmos_temp_C + 273.1)/288.08), 5.256);
    }
    else if ((height > 11000.0) || (height <= 25000.0))
    {
        atmos_temp_C = -56.46;
        atmos_pres_KPa = 22.65 * exp(1.73 - 0.000157 * height);
    }
    else
    {
        atmos_temp_C = -131.21 + 0.00299 * height;
        atmos_pres_KPa = 2.488 * pow(((atmos_temp_C + 273.1)/288.08), -11.388);
    }

    atmos_dens_kgpm3 = atmos_pres_KPa / (0.2869 * (atmos_temp_C + 273.1));

    atmos_snd_spd_mps = sqrt(1.4 * 286 * (atmos_temp_C + 273.1));
}


void RocketSim::calc_grav_accel()
{
    if (const_grav_tf)
    {
        // constant gravity
        grav_accel_mps2 = Vector3d(0.0, 0.0, const_grav_accel_mps2);
    }
    else
    {
        // Newton's Law of Gravitation!!
        double grav_accel_mag_N = (grav_const * mass_earth_kg) / pow((radius_earth_m + ref_altitude_m + position_m.z()), 2.0);
        grav_accel_mps2 = Vector3d(0.0, 0.0, -grav_accel_mag_N);
    }
}

void RocketSim::calc_thrust_accel()
{
    // this assumes thrust is in the direction of motion
    // TODO: fix determination of thrust direction when velocity is zero (or very low)

    // calculate thrust magnitude
    double thrust_mag_N;
    if (const_thrust_tf && time_s < motor_burnout_time_s)
    {
        // constant thrust
        thrust_mag_N = const_thrust_N;
    }
    else if (time_s < motor_burnout_time_s)
    {
        // TODO: implement variable thrust
        thrust_mag_N = 0.0;
    }
    else
    {
        // burn time exceeded, so zero thrust
        thrust_mag_N = 0.0;
    }


    // calculate thrust direction
    Vector3d thrust_dir = velocity_mps / velocity_mps.norm();

    // calc & update thrust
    thrust_accel_mps2 = (thrust_mag_N / mass_kg) * thrust_dir;
}

void RocketSim::calc_rail_accel()
{
    // a rail lets the rocket move freely (without opposing force)
    // in its pointing direction until the rocket's position vector
    // magnitude is greater than the rail length

    // Note: we could probably improve this a bit by adding a static/kinetic
    // friction component to the pointing direction

    if (using_rail_tf && !off_rail) {
        
        if (position_m.norm() > rail_length_m)
        {
            // set the off_rail flag to true
            off_rail = true;

            // and set rail accel to 0
            rail_accel_mps2 = Vector3d(0.0, 0.0, 0.0);
        }
        else
        {
            // we want to find the acceleration without a rail
            Vector3d accel_without_rail_mps2 = grav_accel_mps2 + aero_accel_mps2 + thrust_accel_mps2;

            // now we find the magnitude of the acceleration in the direction of the rail
            // using the dot product
            double accel_along_rail_mps2 = accel_without_rail_mps2.dot(rail_dir);

            // at this point, the magnitude of the acceleration in the rail direction
            // could be positive or negative (thrust not yet high enough for rocket to move)
            if (accel_along_rail_mps2 <= 0.0)
            {
                // the acceleration along the rail is negative
                // so the rail needs to push up on the rocket to keep it stationary
                rail_accel_mps2 = -accel_without_rail_mps2;
            }
            else
            {
                // the acceleration along the rail is positive
                // so we limit the direction of motion of the rocket to the 
                
                // what this statement is doing is calculating the vector acceleration along the rail,
                // subtracting that from the acceleration without the rail, which gives us the net acceleration
                // thats NOT along the rail. We then negate this because we want the rail to oppose this motion 
                rail_accel_mps2 = -(accel_without_rail_mps2 - (accel_along_rail_mps2 * rail_dir)); 
            }
        }
    }
    else {
        rail_accel_mps2 = Vector3d(0.0, 0.0, 0.0);
    }
}

void RocketSim::calc_mass_deriv()
{
    if (const_thrust_tf && time_s < motor_burnout_time_s)
    {
       // constant thrust model for mass flow rate
       // mass flow rate is just propellant weight divided by burntime
       mass_flow_rate_kgps = (mass_full_motor_kg - mass_empty_motor_kg) / (motor_burnout_time_s); 
    }
    else if (time_s < motor_burnout_time_s)
    {
        // TODO: implement variable mass flow rate model
        mass_flow_rate_kgps = 0.0;
        
    }
    else
    {
        // motor done burning
        mass_flow_rate_kgps = 0.0;
    }

}



void RocketSim::set_diameter_in(double diameter_in)
{
    aero_ref_area_m3 = M_PI * pow((diameter_in / (2*39.37)), 2.0);
}

void RocketSim::set_rail(double rail_len_m, double rail_az_deg, double rail_el_deg)
{
    rail_length_m = rail_len_m;
    rail_azimuth_deg = rail_az_deg;
    rail_elevation_deg = rail_el_deg;

    double rail_az_rad = (M_PI / 180) * rail_azimuth_deg;
    double rail_el_rad = (M_PI / 180) * rail_elevation_deg;


    // we basically do a pitch, then a yaw of a unit x 3d vector 
    // https://msl.cs.uiuc.edu/planning/node102.html

    rail_dir = Vector3d(1.0, 0.0, 0.0);

    Matrix3d pitch_rot_mat; pitch_rot_mat << cos(rail_el_rad), 0.0, sin(rail_el_rad),
                                             0.0,           1.0,             0.0,
                                             -sin(rail_el_rad), 0.0, cos(rail_el_rad);
    Matrix3d yaw_rot_mat; yaw_rot_mat << cos(rail_az_rad), -sin(rail_az_rad), 0.0,
                                        sin(rail_az_rad), cos(rail_az_rad), 0.0,
                                        0.0, 0.0, 1.0;

    rail_dir = yaw_rot_mat * (pitch_rot_mat * Vector3d(1.0, 0.0, 0.0));

}
