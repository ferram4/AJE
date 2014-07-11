using System;
using System.Collections.Generic;
using System.Linq;

namespace AJE
{
    public class AJENewSolver
    {
        private double g_0 = 9.81;
        //Gas properties
        //These are "constants" for gas relations; they do vary with gas composition and temperature, and so must have separate values for different parts of the engine

        //These are set up in three sections: compressor (_c), which is everything up to the burner, turbine (_t) everything from the turbine to the afterburner, and afterburner (_ab), which is everything after that.  If the afterburner is not on, _ab = _t

        //Ratio of specific heat at constant pressure (Cp) to specific heat at constant volume (Cv); _c > _t >= _ab
        //Used for compressibility calculations
        private double gamma_c, gamma_t, gamma_ab;

        //Specific heat at constant pressure; _c < _t <= _ab
        //Used for handling fuel usage
        private double Cp_c, Cp_t, Cp_ab;

        //Gas constant, specific to composition; _c > _t >= _ab
        //Used for handling pressure calculations
        private double R_c, R_t, R_ab;


        //Pressure and Temperature ratios, including efficiencies
        //These are used to handle the energy added to the flow to create thrust; subscripts indicate sections; Tau indicates temp ratio, Pi indicates pressure ratio.  Eta is for efficiencies

        //Ram effects; solely a result of slowing the flow down
        private double pi_r, tau_r;

        //Diffuser effects; tau_d = 1, pi_d slightly less than 1
        private double pi_d, tau_d;

        //Compressor effects; pi_c is by the power from the turbine or is limited by design, tau_c is directly related to pi_c, eta_c is an efficiency used to relate pi_c to tau_c
        private double pi_c, tau_c, eta_c;

        //Other Compressor; this is the maximum pressure ratio the compressor is capable of
        private double pi_c_max;

        //Burner effects; pi_b is slightly less than 1, tau_b > 1
        private double pi_b, tau_b;

        //Other burner; tau_lambda is the ratio of the total temperature at the burner to the outside non-stagnation temperature; Tt4 / To * Cp_t / Cp_c
        private double tau_lambda;

        //Turbine effects; pi_t and tau_t are related, tau_t directly related to tau_c (energy balancing) and eta_t is an efficiency to relate pi_t and tau_t
        private double pi_t, tau_t, eta_t;

        //Afterburner effects; pi_ab is slightly less than 1, tau_ab > 1
        private double pi_ab, tau_ab;

        //Other AB; ratio of total temperature downstream of the ab to that of the outside non-stagnation temperature; Tt7 / To * Cp_ab / Cp_c
        private double tau_lambda_ab;

        //Nozzle effects; pi_n is slightly less than 1, tau_n = 1
        private double pi_n, tau_n;

        //Fan effects; pi_fc is limited by design or is proportional to pi_c; tau_fc is related to pi_fc by eta_fc
        private double pi_fc, tau_fc, eta_fc;

        //Other fan; maximum pressure ratio of fan
        private double pi_fc_max;

        //Important flow variables
        //These are necessary for calculating the thrust and efficiency of the engine

        //The total temperature at the end of the burner and afterburner, respectively; used in calculating tau_lambda and tau_lambda_ab
        private double Tt4, Tt7;

        //The pressure, temperature, mach number, sound speed and velocity at the exit of the engine nozzle (station 9); used to calculate thrust
        private double P9, T9, M9, a9, U9;

        //The pressure, temperature, mach number, sound speed and velocity of the freestream; used to calculate initial properties for engine flow and for thrust
        private double P0, T0, M0, a0, U0;

        //The area sucked into the core engine at the freestream, the area of the compressor inlet, area of the nozzle throat and the exit area of the engine
        private double A0, A2, A8, A9;

        //The ratio of air going through the fan rather than through the core
        private double bypassRatio;

        //The pressure, temperature, mach number, sound speed and velocity at the exit of the fan nozzle (station 9f); used to calculate thrust
        private double P9f, T9f, M9f, a9f, U9f;

        //Fraction fuel flow for burner and afterburner respectively; used for both exit mass flow and for fuel consumption
        private double f, f_ab;

        //Heat of burning from fuel per unit mass
        private double h_f;

        //Mass flow of air through engine
        private double mdot;

        //Engine Geometry
        //Needed to calculate mass flow and relations between components

        //Needed for mass flow calculations
        //Ratio of area at burner exit to area at nozzle throat; star indicates that flow is sonic (M=1) at these conditions
        private double astar_4_rat_astar_8;

        //Ratio of area needed for sonic conditions at compressor inlet to actual area at compressor inlet
        private double astar_2_rat_a_2;

        //Ratio of compressor inlet area to burner exit area
        private double a_2_rat_astar_4;


        //Needed for exit area pressure
        //Ratio of nozzle exit area to nozzle throat area
        private double a_9_rat_astar_8_min, a_9_rat_astar_8_max, a_9_rat_astar_8_actual;

        //Must be first initialization function called
        public void InitializeEngineGeometry(double compressorInletArea, double burnerExitArea, double nozzleThroatArea, double nozzleMinExitArea, double nozzleMaxExitArea, double bypass)
        {
            astar_4_rat_astar_8 = burnerExitArea / nozzleThroatArea;
            a_2_rat_astar_4 = compressorInletArea / burnerExitArea;
            a_9_rat_astar_8_min = nozzleMinExitArea / nozzleThroatArea;
            a_9_rat_astar_8_max = nozzleMaxExitArea / nozzleThroatArea;
            A8 = nozzleThroatArea;
            A2 = compressorInletArea;
            bypassRatio = bypass;
        }

        //Must be called after InitializeEngineGeometry
        public void InitializeComponentEfficiencies(double diffuserPresRatio, double compressorEfficiency, double maxCompressorPressureRatio, double fanEfficiency, double maxFanPressureRatio, double burnerPresRatio, double turbineEfficiency, double nozzlePresRatio)
        {
            pi_d = diffuserPresRatio;
            eta_c = compressorEfficiency;
            pi_c_max = maxCompressorPressureRatio;
            eta_fc = fanEfficiency;
            pi_fc_max = maxFanPressureRatio;
            pi_b = burnerPresRatio;
            eta_t = turbineEfficiency;
            pi_n = nozzlePresRatio;
            tau_d = 1;
            tau_n = 1;
            pi_ab = 1;
        }

        //Must be called after InitializeComponentEfficiencies
        public void InitializeFuelProperties(double maxBurnerTemp, double maxABTemp, double heatPerUnitMassFuel)
        {
            Tt4 = maxBurnerTemp;
            Tt7 = maxABTemp;
            h_f = heatPerUnitMassFuel;
        }

        public void CalculatePerformance(double pressure, double temperature, double velocity, double soundspeed, double nozzleExitFrac, double air_density,
            out double thrust, out double I_sp)
        {
            P0 = pressure;
            T0 = temperature;
            U0 = velocity;
            a0 = soundspeed;
            M0 = U0 / a0;

            a_9_rat_astar_8_actual = a_9_rat_astar_8_min + (a_9_rat_astar_8_max - a_9_rat_astar_8_min) * nozzleExitFrac;

            gamma_c = CalculateGamma(T0, 0);
            gamma_t = CalculateGamma(Tt4, f);
            gamma_ab = CalculateGamma(Tt7, f_ab);

            Cp_c = CalculateCp(T0, 0);
            Cp_t = CalculateCp(Tt4, f);
            Cp_ab = CalculateCp(Tt7, f_ab);

            R_c = CalculateR(T0, 0);
            R_t = CalculateR(Tt4, f);
            R_ab = CalculateR(Tt7, f_ab);

            CalculateRamProperties();
            CalculateTurbinePropertiesFromNozzleMassBalancing();
            CalculateBurnerProperties();
            CalculateCompressorAndFanPropertiesFromTurbineEnergyBalance();
            CalculateAfterburnerProperties();

            CalculateSuctionAreaForMassFlow();
            CalculateMassFlow(air_density);

            CalculateExitTemperature();
            CalculateExitPressure();

            a9 = Math.Sqrt(gamma_ab * R_ab * T9);
            U9 = M9 * a9;

            a9f = Math.Sqrt(gamma_c * R_c * T9f);
            U9f = M9f * a9f;

            //Effects of velocity difference at inlet and nozzle of core
            thrust = ((1 + f + f_ab) * U9 - U0);

            //Effects of velocity difference across fan
            thrust += (U9f - U0) * bypassRatio;
            thrust *= mdot;

            //Effects of pressure difference at inlet and nozzle of core
            thrust += (P9 - P0) * A9;

            //Effects of pressure difference across fan
            thrust += (P9f - P0) * A9 * bypassRatio;

            I_sp = mdot * (f + f_ab) * g_0;
            I_sp = thrust / I_sp;
        }

        public void CalculateRamProperties()
        {
            //Uses isentropic compressible flow to calculate temp rise due to ram effects
            tau_r = gamma_c - 1;
            tau_r *= 0.5;
            tau_r *= M0 * M0;
            tau_r++;

            double tmp = gamma_c - 1;
            tmp = gamma_c / tmp;

            //pressure ratio is simply temperature ratio raised to the gamma / (gamma - 1) for isentropic flow
            pi_r = Math.Pow(tau_r, tmp);
        }

        public void CalculateCompressorAndFanPropertiesFromTurbineEnergyBalance()
        {
            tau_c = tau_t - 1;
            tau_c *= -(1 + f) * tau_lambda;
            tau_c /= tau_r;
            tau_c++;
            tau_c /= (bypassRatio + 1);

            tau_fc = tau_c * bypassRatio;

            pi_c = pi_fc = gamma_c - 1;
            pi_c *= eta_c;
            pi_c = gamma_c / pi_c;
            pi_c = Math.Pow(tau_c, pi_c);

            pi_fc *= eta_fc;
            pi_fc = gamma_c / pi_fc;
            pi_fc = Math.Pow(tau_fc, pi_fc);
        }

        public void CalculateBurnerProperties()
        {
            tau_lambda = Tt4 * Cp_t / (T0 * Cp_c);

            //Fuel usage is equal to the temperature added by the burner minus the temp added from ram and compression
            f = tau_lambda - tau_r * tau_c;

            //Divided by the ratio of heat per unit mass and heat capacity of air - the temp ratio of the burner to the ambient
            f /= h_f / (Cp_c * T0) - tau_lambda;
        }

        public void CalculateTurbinePropertiesFromNozzleMassBalancing()
        {
            //Mass flow balance on engine nozzle throat and turbine inlet, both choked
            tau_t = 2 * (gamma_t - 1);
            tau_t /= (gamma_t + 1);
            tau_t = Math.Pow(astar_4_rat_astar_8, tau_t);

            pi_t = gamma_t - 1;
            pi_t *= eta_t;
            pi_t = gamma_t / pi_t;
            pi_t = Math.Pow(tau_t, pi_t);
        }

        public void CalculateAfterburnerProperties()
        {
            tau_lambda_ab = Tt7 * Cp_ab / (T0 * Cp_c);

            f_ab = tau_lambda_ab - tau_lambda * tau_t;

            //Divided by the ratio of heat per unit mass and heat capacity of air - the temp ratio of the burner to the ambient
            f_ab /= h_f / (Cp_t * T0) - tau_lambda_ab;

            f_ab *= (1 + f);
        }

        public void CalculateSuctionAreaForMassFlow()
        {
            CalculateCompressorLineFromCompressorTurbineMassBalance();

            A0 = A2 * astar_2_rat_a_2 / CalculateAreaRatioFromMachNumber(M0, gamma_c);
        }

        public void CalculateMassFlow(double air_density)
        {
            mdot = A0 * U0 * air_density;
        }

        public void CalculateCompressorLineFromCompressorTurbineMassBalance()
        {
            astar_2_rat_a_2 = 1 - tau_t;
            astar_2_rat_a_2 /= (1 + f);
            astar_2_rat_a_2 = Math.Sqrt(astar_2_rat_a_2);

            astar_2_rat_a_2 /= Math.Sqrt(tau_c - 1);
            astar_2_rat_a_2 *= pi_c;
            astar_2_rat_a_2 *= a_2_rat_astar_4;
        }

        public void CalculateExitMachNumberAndExitArea()
        {
            M9 = CalculateMachNumberFromAreaRatioSupersonic(a_9_rat_astar_8_actual, gamma_ab);
            A9 = A8 * a_9_rat_astar_8_actual;
            M9f = 0.5;
        }

        public void CalculateExitTemperature()
        {
            //total temperature at exit of the core
            double Tt9 = T0 * tau_lambda_ab * Cp_c / Cp_ab;

            T9 = M9 * M9;
            T9 *= (gamma_ab - 1) * 0.5;
            T9++;
            T9 = Tt9 / T9;

            //total temp at exit of the fan
            Tt9 = T0 * tau_fc;
            T9f = M9f * M9f;
            T9f *= (gamma_c - 1) * 0.5;
            T9f++;
            T9f = Tt9 / T9f;
        }

        public void CalculateExitPressure()
        {
            //total pressure at exit of the core
            double Pt9 = T0 * pi_r * pi_c * pi_b * pi_t * pi_ab * pi_n;

            P9 = M9 * M9;
            P9 *= (gamma_ab - 1) * 0.5;
            P9++;
            P9 = Math.Pow(P9, gamma_ab / (gamma_ab - 1));
            P9 = Pt9 / P9;

            //total pressure at exit of fan
            Pt9 = T0 * pi_r * pi_fc;
            P9f = M9f * M9f;
            P9f *= (gamma_c - 1) * 0.5;
            P9f++;
            P9f = Math.Pow(P9f, gamma_c / (gamma_c - 1));
            P9f = Pt9 / P9f;
        }

        public double CalculateGamma(double temperature, double fuel_fraction)
        {
            return 1.4;
        }

        public double CalculateCp(double temperature, double fuel_fraction)
        {
            return 1004;
        }

        public double CalculateR(double temperature, double fuel_fraction)
        {
            return 287;
        }

        //Calculates the area ratio necessary for sonic flow directly from mach number
        public double CalculateAreaRatioFromMachNumber(double mach, double gamma)
        {
            double gammaPower = gamma - 1;
            gammaPower *= 2;
            gammaPower = (gamma + 1) / gammaPower;

            return CalculateAreaRatioFromMachNumber(mach, gamma, gammaPower);
        }

        public double CalculateAreaRatioFromMachNumber(double mach, double gamma, double gammaPower)
        {
            double result = gamma - 1;
            result *= 0.5;

            result *= mach * mach;
            result++;

            result *= 2;
            result /= (gamma + 1);

            result = Math.Pow(result, gammaPower);
            result /= mach;
            return result;
        }

        public double CalculateMachNumberFromAreaRatioSupersonic(double areaRatio, double gamma)
        {
            return CalculateMachNumberFromAreaRatio(areaRatio, gamma, 2);
        }


        public double CalculateMachNumberFromAreaRatio(double areaRatio, double gamma, double guess)
        {
            double numeratorConstant = 1 - 3 * gamma;
            numeratorConstant /= (2 - 2 * gamma);
            numeratorConstant = Math.Pow(2, numeratorConstant);

            double gammaPower = gamma - 1;
            gammaPower *= 2;
            gammaPower = (gamma + 1) / gammaPower;

            double mach_last = 0;
            double fn = 1;
            double fn_prime = 1;
            while(Math.Abs(guess - mach_last) <= 0.01)
            {
                mach_last = guess;
                fn = CalculateAreaRatioFromMachNumber(guess, gamma, gammaPower);

                fn_prime = gamma - 1;
                fn_prime *= guess * guess;
                fn_prime += 2;
                fn_prime *= guess * guess;

                fn_prime = gammaPower / fn_prime;
                fn_prime *= (guess * guess - 1);

                double tmp = guess * guess * 0.5 * (gamma - 1);
                tmp++;
                tmp /= (gamma + 1);

                fn_prime *= Math.Pow(tmp, gammaPower);

                guess = guess - fn / fn_prime;
            }

            return guess;
        }

        public double CalculateMachNumberFromAreaRatioSubsonic(double areaRatio, double gamma)
        {
            return CalculateMachNumberFromAreaRatio(areaRatio, gamma, 0.5);
        }
    }
}
