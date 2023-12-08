
import numpy as np
import math
from typing import Union

class FinFlutterAnalysis:
    def __init__(self):
        # default parameters (must be changed)
        self.shear_modulus_psi = -1.0
        self.semi_span_in = -1.0
        self.root_chord_in = -1.0
        self.tip_chord_in = -1.0
        self.thickness_in = -1.0

        # default parameters (can be changed)
        self.base_temp_F = 59.0
        self.base_alt_ft = 0.0


        # default parameters (should only be changed if necessary)
        self.temp_lapse_rate_Fpft = 0.00356
        self.specific_heat_ratio = 1.4
        self.gas_constant_air = 1716.59


    # Mutators
    def setShearModulus(self, shear_modulus_psi: float) -> None:
        if (isinstance(shear_modulus_psi, float) and shear_modulus_psi > 0.0):
            self.shear_modulus_psi = shear_modulus_psi
        else:
            raise Exception(f"Invalid Shear Modulus Value of '{shear_modulus_psi}'")

    def setSemiSpan(self, semi_span_in: float) -> None:
        if (isinstance(semi_span_in, float) and semi_span_in > 0.0):
            self.semi_span_in = semi_span_in
        else:
            raise Exception(f"Invalid Fin Semi Span Value of '{semi_span_in}'")
    
    def setRootChord(self, root_chord_in: float) -> None:
        if (isinstance(root_chord_in, float) and root_chord_in > 0.0):
            self.root_chord_in = root_chord_in
        else:
            raise Exception(f"Invalid Fin Root Chord Value of '{root_chord_in}'")

    def setTipChord(self, tip_chord_in: float) -> None:
        if (isinstance(tip_chord_in, float) and tip_chord_in > 0.0):
            self.tip_chord_in = tip_chord_in
        else:
            raise Exception(f"Invalid Fin Tip Chord Value of '{tip_chord_in}'")

    def setThickness(self, thickness_in: float) -> None:
        if (isinstance(thickness_in, float) and thickness_in > 0.0):
            self.thickness_in = thickness_in
        else:
            raise Exception(f"Invalid Fin Thickness Value of '{thickness_in}'")

    def setFinParameters(self, shear_modulus_psi: float, semi_span_in: float, root_chord_in: float,
                        tip_chord_in: float, thickness_in: float) -> None:

        # set fin parameters
        self.setShearModulus(shear_modulus_psi)
        self.setSemiSpan(semi_span_in)
        self.setRootChord(root_chord_in)
        self.setTipChord(tip_chord_in)
        self.setThickness(thickness_in)
        
        # check for weird dimensions
        if (root_chord_in <= tip_chord_in):
            print("FinFlutterAnalysis Warning: Tip Chord longer than Root Chord")
        if (semi_span_in >= root_chord_in):
            print("FinFlutterAnalysis Warning: Semi Span longer than Root Chord")

    def setBaseTemp_F(self, base_temp_F):
        self.base_temp_F = base_temp_F
        

    # Auxilliary Functions

    def fin_area_in2(self) -> float:
        if (self.root_chord_in < 0.0 or self.tip_chord_in < 0.0 or self.semi_span_in < 0.0):
            raise Exception(f"Cannot calculate fin area because of root chord, tip chord, and/or semi span is less than 0")
        else:
            return 0.5 * (self.root_chord_in + self.tip_chord_in) * self.semi_span_in
        
    def aspect_ratio(self) -> float:
        if (self.semi_span_in < 0.0):
            raise Exception(f"Cannot calculate aspect ratio because semi span is less than 0")
        else:
            return self.semi_span_in**2 / self.fin_area_in2()

    def chord_ratio(self) -> float:
        if (self.root_chord_in < 0.0 or self.tip_chord_in < 0.0):
            raise Exception(f"Cannot calculate chord ratio because root chord and/or tip chord is less than 0")
        else:
            return self.tip_chord_in / self.root_chord_in

    def temp_F(self, altitude_ft: Union[float, np.array]) -> Union[float, np.array]:
        '''
            https://www.grc.nasa.gov/www/k-12/VirtualAero/BottleRocket/airplane/atmos.html
        '''

        return self.base_temp_F - self.temp_lapse_rate_Fpft * (altitude_ft - self.base_alt_ft)
    
    def temp_R(self, altitude_ft: Union[float, np.array]) -> Union[float, np.array]:
        return self.temp_F(altitude_ft) + 459.7
    
    def pres_lbpft2(self, altitude_ft: Union[float, np.array]) -> Union[float, np.array]:
        '''
            https://www.grc.nasa.gov/www/k-12/VirtualAero/BottleRocket/airplane/atmos.html
        '''

        return 2116.0 * (self.temp_R(altitude_ft) / (self.base_temp_F + 459.7))**5.256
    
    def pres_lbpin2(self, altitude_ft: Union[float, np.array]) -> Union[float, np.array]:
        return (1.0 / 144) * self.pres_lbpft2(altitude_ft)
    
    def sound_speed_ftps(self, altitude_ft: Union[float, np.array]) -> Union[float, np.array]:
        return math.sqrt(self.specific_heat_ratio * self.gas_constant_air * self.temp_R(altitude_ft))
    
    def flutter_velocity_ftps(self, altitude_ft: Union[float, np.array]) -> Union[float, np.array]:
        '''
            See Equations 16-18 in NACA TN 4197
            https://ntrs.nasa.gov/api/citations/19930085030/downloads/19930085030.pdf
            
        '''
        epsilon = 0.25 # magic number from NACA TN 4197

        # computing various parts of the denominator
        d1 = (24.0 * epsilon) / math.pi
        d2 = self.specific_heat_ratio * self.pres_lbpin2(altitude_ft)
        aspect_ratio = self.aspect_ratio()
        d3 = math.pow(aspect_ratio, 3.0) / (math.pow((self.thickness_in / self.root_chord_in), 3) * (aspect_ratio + 2))
        d4 = (self.chord_ratio() + 1) / 2

        return self.sound_speed_ftps(altitude_ft) * math.sqrt(self.shear_modulus_psi / (d1 * d2 * d3 * d4))

    def calcDerivs(self, altitude_ft: float) -> None:
        '''
            This method calculates and prints the derivative of the flutter velocity
            with respect to all the fin parameters.

            Example:
                dV/d'root_chord' = 50.2 [ft/s / in]
                means that for a single inch change in root chord length,
                the flutter velocity increases by 50.2 ft/s
        '''

        print("")
        print("Flutter Velocity Derivatives:")

        # Current Flutter Velocity
        ref_flutter_velocity = self.flutter_velocity_ftps(altitude_ft)

        # Shear Modulus
        shear_modulus_psi = self.shear_modulus_psi
        self.shear_modulus_psi += 1000
        delta_shear_mod_flutter_vel = self.flutter_velocity_ftps(altitude_ft) - ref_flutter_velocity
        self.shear_modulus_psi = shear_modulus_psi
        print(f"dV/d'shear modulus' = {delta_shear_mod_flutter_vel} [ft/s / kpsi]")

        # Semi Span
        semi_span_in = self.semi_span_in
        self.semi_span_in += 1
        delta_semi_span_flutter_vel = self.flutter_velocity_ftps(altitude_ft) - ref_flutter_velocity
        self.semi_span_in = semi_span_in
        print(f"dV/d'semi span' = {delta_semi_span_flutter_vel} [ft/s / in]")

        # Root Chord
        root_chord_in = self.root_chord_in
        self.root_chord_in += 1
        delta_root_chord_flutter_vel = self.flutter_velocity_ftps(altitude_ft) - ref_flutter_velocity
        self.root_chord_in = root_chord_in
        print(f"dV/d'root chord' = {delta_root_chord_flutter_vel} [ft/s / in]")

        # Tip Chord
        tip_chord_in = self.tip_chord_in
        self.tip_chord_in += 1
        delta_tip_chord_flutter_vel = self.flutter_velocity_ftps(altitude_ft) - ref_flutter_velocity
        self.tip_chord_in = tip_chord_in
        print(f"dV/d'tip cord' = {delta_tip_chord_flutter_vel} [ft/s / in]")

        # Thickness
        thickness_in = self.thickness_in
        self.thickness_in += 0.1
        delta_thickness_flutter_vel = self.flutter_velocity_ftps(altitude_ft) - ref_flutter_velocity
        self.thickness_in = thickness_in
        print(f"dV/d'thickness' = {delta_thickness_flutter_vel} [ft/s / (in/10)]")

        print("")



sa_ground_level_ft = 4595.0
altitude_max_vel_agl_ft = 2500.0
flutteranalysis1 = FinFlutterAnalysis()
flutteranalysis1.setBaseTemp_F(100.0)
flutteranalysis1.base_alt_ft = sa_ground_level_ft
flutteranalysis1.setFinParameters(shear_modulus_psi=1.0E6, semi_span_in=4.0, root_chord_in=15.00, tip_chord_in=8.00, thickness_in=3.0/16)
print(f"Sound Speed: {flutteranalysis1.sound_speed_ftps(altitude_ft=altitude_max_vel_agl_ft)}")
print(f"Fin Flutter Velocity: {flutteranalysis1.flutter_velocity_ftps(altitude_ft=(altitude_max_vel_agl_ft))} ft/s")
flutteranalysis1.calcDerivs(altitude_ft=(altitude_max_vel_agl_ft))
