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

}

void RocketSim::calc_aero_accel()
{
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


void RocketSim::set_diameter_in(double diameter_in)
{
    aero_ref_area_m3 = M_PI * pow((diameter_in / (2*39.37)), 2.0);
}
