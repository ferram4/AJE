using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AJE
{
    public class AJETransientGasGeneratorModel
    {
        //freestream flight conditions; pressure, temperature, and mach number
        private double P0, T0, M0;

        //conditions at inlet; pressure, temperature;
        private double P2, T2;

        //Conditions at burner inlet / compressor exit
        private double P3, T3, mdot3;

        //conditions at burner exit / turbine entrance; pressure, temperature, mass flow rate
        private double P4, T4, mdot4;

        //conditions at ab inlet / turbine exit
        private double P5, T5, mdot5;

        //conditions at ab rear / nozzle entrance;
        private double P6, T6, mdot6;

        //burner mixing volume properties; volume, gas mass
        private double V4, m4;

        //gas properties, pre-burner, post-burner, post afterburner
        private double gamma_c, gamma_t, gamma_ab;
        private double R_c, R_t, R_ab;
        private double Cp_c, Cp_t, Cp_ab;
        private double Cv_c, Cv_t, Cv_ab;

        //Throttles for burner and afterburner
        private double mainThrottle, abThrottle;

        //Fuel mass flow rate for burner and afterburner
        private double mdot_f, mdot_f_ab;

        //Fuel control integrals
        private double f_error_integral, f_ab_error_integral;

        //Temperature required to maintain HP idle
        private double T4_idle;

        //Fuel heat of burning and peak temperatures
        private double h_f, Tt4, Tt6;

        //Reference area of the engine, combustor, and nozzle
        private double Aref, Acomb, Anozzle;

        //thrust and Isp of the engine
        private double thrust, Isp;

        private List<AJEGasGeneratorSpool> spools = new List<AJEGasGeneratorSpool>();

        public bool started = false;

        //---------------------------------------------------------
        //Initialization Functions

        public void InitializeOverallEngineData(double referenceArea, double combustorArea, double nozzleArea, double combustorVolume, double heatOfFuel, double max_Tt4, double max_Tt6)
        {
            Aref = referenceArea;
            Acomb = combustorArea;
            Anozzle = nozzleArea;
            V4 = combustorVolume;
            h_f = heatOfFuel;
            Tt4 = max_Tt4;
            Tt6 = max_Tt6;
        }

        //Must add spools from outermost / lowest pressure to innermost / highest pressure in order for calculations to be correct.  This means that for a two-spool turbofan the fan's spool must be added before the high-pressure spool
        public void AddNewSpool(double compressorDesignPressureRatio, double compressorInletDesignTemperature, double compressorEfficiency, double compressorVolume,
                                         double turbineInletDesignTemperature, double turbineEfficiency, double turbineVolume,
                                         double designRPM, double idleRPM,
                                         double frictionalEfficiency, double machineEfficiency,
                                         double referencePressure, double referenceTemperature,
                                         double rotationalInertia, double massFlowDesign)
        {
            AJEGasGeneratorSpool newSpool = new AJEGasGeneratorSpool(compressorDesignPressureRatio, compressorInletDesignTemperature, compressorEfficiency, compressorVolume,
                                                                       turbineInletDesignTemperature, turbineEfficiency, turbineVolume,
                                                                       designRPM, idleRPM, frictionalEfficiency, machineEfficiency,
                                                                       referencePressure, referenceTemperature,
                                                                       rotationalInertia, massFlowDesign);

            spools.Add(newSpool);
        }

        public void StartupFunction(double pressure, double temperature, double velocity)
        {
            P0 = pressure;
            T0 = temperature;
            P2 = P0;
            P3 = P2 * 1.1;
            P4 = P3 * 0.99;
            P5 = P4 / 1.08;
            P6 = P5;
            T2 = T0;
            T3 = T2 * Math.Pow(1.1, 0.4 / 1.4);
            T4 = T3;
            T5 = T4 * Math.Pow(1/1.08, 0.4 / 1.4);
            T6 = T5;

            mdot3 = mdot4 = mdot5 = mdot6 = 0.1;

            CalculateGasPropertiesAtAllStations();
            M0 = velocity / Math.Sqrt(gamma_c * R_c * T0);

            double density = P4 / (T0 * R_c);
            m4 = density * V4;

            foreach (AJEGasGeneratorSpool spool in spools)
                spool.SetStartup(T2, P2, T3, P3, T4, P4, T5, P5, mdot3, density, true);


            started = true;
        }

        public void CalculatePerformance(double pressure, double temperature, double velocity, double timeStep, double commandedThrottle)
        {
            P0 = pressure;
            T0 = temperature;
            M0 = velocity / Math.Sqrt(gamma_c * R_c * T0);

            CalculateFuelFlow(commandedThrottle, timeStep);

            CalculateGasPropertiesAtAllStations();  //Update all gas properties

            CalculateStation2PropertiesFrom0();     //Inlet and ram compression

            int i = 0;
            P3 = P2;
            T3 = T2;
            while (i < spools.Count)
            {
                AJEGasGeneratorSpool currentSpool = spools[i];
                if (i + 1 < spools.Count)
                    currentSpool.CalculateCompressorEffects(P3, T3, spools[i + 1].CompressorInletMassFlow(), gamma_c, Cp_c, Cv_c, R_c, timeStep);
                else
                    currentSpool.CalculateCompressorEffects(P3, T3, mdot3, gamma_c, Cp_c, Cv_c, R_c, timeStep);

                P3 = currentSpool.CompressorOutletPressure();
                T3 = currentSpool.CompressorOutletTemperature();

                i++;
            }

            CalculateStation4PropertiesFrom3();     //Combustor and associated transients
            CalculateStation4VolumeTransients(timeStep);

            P5 = P4;
            T5 = T4;
            mdot4 = spools[0].TurbineInletMassFlow();
            i = spools.Count - 1;
            while (i >= 0)
            {
                AJEGasGeneratorSpool currentSpool = spools[i];
                if (i - 1 >= 0)
                    currentSpool.CalculateTurbineEffects(P5, T5, spools[i - 1].TurbineInletMassFlow(), gamma_t, Cp_t, Cv_t, R_t, timeStep);
                else
                    currentSpool.CalculateTurbineEffects(P5, T5, mdot5, gamma_t, Cp_t, Cv_t, R_t, timeStep);

                P5 = currentSpool.TurbineOutletPressure();
                T5 = currentSpool.TurbineOutletTemperature();

                i--;
            }

            CalculateStation6FromStation5();        //Afterburner

            CalculateStation7FromStation6();        //Nozzle


            Isp = thrust / (mdot_f + mdot_f_ab * 9.81);

            string debugstring = "";
            debugstring += "EngineProperties\n\r";
            debugstring += "P0: " + P0 + " T0: " + T0 + "\n\r";
            debugstring += "P2: " + P2 + " T2: " + T2 + "\n\r";
            debugstring += "P3: " + P3 + " T3: " + T3 + " modt3: " + mdot3 + "\n\r";
            debugstring += "P4: " + P4 + " T4: " + T4 + " modt4: " + mdot4 + "\n\r";
            debugstring += "P5: " + P5 + " T5: " + T5 + " modt5: " + mdot5 + "\n\r";
            debugstring += "P6: " + P6 + " T6: " + T6 + " modt6: " + mdot6 + "\n\r";
            debugstring += "Cp_c: " + Cp_c + " Cp_t: " + Cp_t + " Cp_ab: " + Cp_ab + "\n\r";
            debugstring += "Cv_c: " + Cv_c + " Cv_t: " + Cv_t + " Cv_ab: " + Cv_ab + "\n\r";
            debugstring += "R_c: " + R_c + " R_t: " + R_t + " R_ab: " + R_ab + "\n\r";
            debugstring += "gamma_c: " + gamma_c + " gamma_t: " + gamma_t + " gamma_ab: " + gamma_ab + "\n\r";


            Debug.Log(debugstring);
        }

        public double GetThrust() { return thrust; }
        public double GetIsp() { return Isp; }

        private void CalculateFuelFlow(double commandedThrottle, double timeStep)
        {
            //if (hasAB)
            //{
                //Set AB at 2/3 throttle
            mainThrottle = Math.Min(commandedThrottle * 1.5, 1.0);
            abThrottle = Math.Max(commandedThrottle - 0.67, 0);


            //Afterburner is simple, it doesn't need to power engine
            double T6_desired = T5 + (Tt6 - T5) * abThrottle;
            T6_desired = Math.Max(Tt6, T5);             //Can't use fuel to cool it down

            double abTempError = T6_desired - T6;

            mdot_f_ab += mdot6 * (0.5 * abTempError + 0.05 * abTempError) * timeStep * Cp_ab / h_f;         //PI controller for AB flow

            f_ab_error_integral += abTempError;

            mdot_f_ab = Math.Max(mdot_f_ab, 0);     //Can't be negative


            /*//Idling temperature adjustment
            double ratioIdle = spools.Last().FractionIdle();
            if(ratioIdle < 1)
            {
            }*/


            //QuickBurnerModel is simple, it doesn't need to power engine
            double T4_desired = T3 + (Tt4 - T3) * mainThrottle;
            T4_desired = Math.Max(Tt4, T3);             //Can't use fuel to cool it down

            double bTempError = T4_desired - T3;

            mdot_f += mdot4 * (0.5 * bTempError + 0.05 * bTempError) * timeStep * Cp_t / h_f;         //PI controller for AB flow

            f_error_integral += bTempError;

            mdot_f = Math.Max(mdot_f, 0);     //Can't be negative

            //}
            /*else
            {
                mainThrottle = inputThrottle;
                gamma_ab = gamma_t;
                Cp_ab = Cp_t;
                R_ab = R_t;
            }*/
        }

        private void CalculateGasPropertiesAtAllStations()
        {
            double f, f_ab;
            f = mdot_f / (mdot6 + mdot_f + mdot_f_ab);
            f_ab = mdot_f_ab / (mdot6 + mdot_f + mdot_f_ab);

            gamma_c = CalculateGamma(T0, 0);
            gamma_t = CalculateGamma(T4, f);
            gamma_ab = CalculateGamma(T6, f + f_ab);

            Cp_c = CalculateCp(T0, 0);
            Cp_t = CalculateCp(T4, 0);
            Cp_ab = CalculateCp(T6, f + f_ab);

            Cv_c = Cp_c / gamma_c;
            Cv_t = Cp_t / gamma_t;
            Cv_ab = Cp_ab / gamma_ab;

            R_c = Cv_c * (gamma_c - 1);
            R_t = Cv_t * (gamma_t - 1);
            R_ab = Cv_ab * (gamma_ab - 1);
        }

        private double CalculateGamma(double temperature, double fuel_fraction)
        {
            double gamma = 1.4 - 0.1 * Math.Max((temperature - 300) * 0.0005, 0) * (1 + fuel_fraction);
            gamma = Math.Min(1.4, gamma);
            gamma = Math.Max(1.1, gamma);
            return gamma;
        }

        private double CalculateCp(double temperature, double fuel_fraction)
        {
            double Cp = 1004.5 + 250 * Math.Max((temperature - 300) * 0.0005, 0) * (1 + 10 * fuel_fraction);
            Cp = Math.Min(1404.5, Cp);
            Cp = Math.Max(1004.5, Cp);
            return Cp;
        }

        //Calculate stagnation conditions at compressor face from freestream through the inlet
        private void CalculateStation2PropertiesFrom0()
        {
            double factor = M0 * M0;
            factor *= (gamma_c - 1) * 0.5;
            factor += 1;

            T2 = T0 * factor;

            double eta_d = 1;       //Pressure loss in the inlet, using US milspec pressure recovery

            if (M0 > 1)
                eta_d -= 0.075 * Math.Pow(M0 - 1, 1.35);

            factor = Math.Pow(factor, gamma_c / (gamma_c - 1));

            P2 = P0 * factor;
        }

        //Calculate the properties of the burner
        private void CalculateStation4PropertiesFrom3()
        {
            double pressureLossFactor = 28;
            double overallPressureLoss = 0.06;
            //overallPressureLoss += T4 / T3;
            //overallPressureLoss--;

            mdot3 = overallPressureLoss / pressureLossFactor;
            mdot3 *= 2 / (R_t * T3);
            mdot3 = Math.Sqrt(mdot3);
            mdot3 *= Acomb * P3;

            mdot3 = Math.Sqrt(mdot3);
        }

        private void CalculateStation4VolumeTransients(double timeStep)
        {
            //Change in mass in compressor
            double d_dt_m4 = (mdot3 + mdot_f - mdot4);

            //Change in temperature at end of compressor
            double d_dt_T4_ = mdot3 * (Cp_t * T3 - Cv_t * T4);
            d_dt_T4_ += mdot_f * (h_f - Cv_t * T4) - (mdot4 * R_t * T4);
            d_dt_T4_ /= (m4 * Cv_t);

            P4 = m4 * R_t * T4 / V4;

            m4 += d_dt_m4 * timeStep;
            m4 = Math.Max(m4, 1);
            T4 += d_dt_T4_ * timeStep;
            T4 = Math.Max(T4, 100);
        }

        private void CalculateStation6FromStation5()
        {
            mdot5 = mdot6 - mdot_f_ab;
            T6 = T5 * Cp_t * mdot6 + h_f * mdot_f_ab;
            T6 /= Cp_ab * (mdot_f_ab + mdot6);
            P6 = P5;
        }

        private void CalculateStation7FromStation6()
        {
            double P_cr = 2 / (gamma_ab + 1);
            P_cr = Math.Pow(P_cr, gamma_ab / (gamma_ab - 1));
            P_cr *= P6;

            if(P0 > P_cr)   //Subsonic flow
            {
                double tmp = P0 / P6;
                tmp = Math.Pow(tmp, (gamma_ab - 1) / gamma_ab);
                tmp = 1 - tmp;
                tmp *= 2 * gamma_ab;
                tmp /= (gamma_ab - 1);

                tmp = Math.Sqrt(tmp);

                mdot6 = P0 / P6;
                mdot6 = Math.Pow(mdot6, 1 / gamma_ab);
                mdot6 *= Anozzle * P6 / (Math.Sqrt(R_ab * T6));
                mdot6 *= tmp;

                double Ve = R_ab * T6;
                Ve = Math.Sqrt(Ve);
                Ve *= tmp;

                thrust = Ve * mdot6;

                Debug.Log("Subsonic Nozzle");
            }
            else
            //Choked nozzle
            {
                double tmp = P0 / P6;
                tmp = Math.Pow(tmp, (gamma_ab - 1) / gamma_ab);
                tmp = 1 - tmp;
                tmp *= 2 * gamma_ab;
                tmp /= (gamma_ab - 1);

                tmp = Math.Sqrt(tmp);

                double Ve = R_ab * T6;
                Ve = Math.Sqrt(Ve);
                Ve *= tmp;

                mdot6 = 2 / (gamma_ab + 1);
                mdot6 = Math.Pow(mdot6, (gamma_ab + 1) / (gamma_ab - 1));
                mdot6 *= gamma_ab;

                mdot6 /= (R_ab * T6);
                mdot6 = Math.Sqrt(mdot6);
                mdot6 *= P6 * Anozzle;

                thrust = Ve * mdot6;
                thrust += Anozzle * (P_cr - P0);

                Debug.Log("Choked Nozzle");
            }
        }
    }

    //A class that handles the compressor-turbine-shaft mess on a turbojet
    public class AJEGasGeneratorSpool
    {
        //Compressor Properties
        //----------------------------------------------------------------------------

        //Compressor inlet properties; all total
        double P_comp_in, T_comp_in;

        //Compressor exit properties; all total
        double P_comp_ex, T_comp_ex, mdot_comp_ex;

        //Compressor exit properties due to transient effects
        double T_comp_ex_transient, mdot_comp_ex_transient;

        //Compressor design properties
        double P_comp_ratio_design, T_comp_in_design, mdot_comp_design;

        //Compressor efficiency
        double eta_comp;

        //Compressor gas mass and volume
        double m_comp, V_comp;

        //Compressor power
        double Pw_comp;

        //Turbine Properties
        //----------------------------------------------------------------------------

        //Turbine inlet properties; all total
        double P_turb_in, T_turb_in;

        //Turbine exit properties; all total
        double P_turb_ex, T_turb_ex, mdot_turb_ex;

        //Compressor exit properties due to transient effects
        double T_turb_ex_transient, mdot_turb_ex_transient;

        //Turbine design properties
        double P_turb_ratio_choke, T_turb_in_design;

        //Turbine efficiency
        double eta_turb;

        //Turbine gas mass and volume
        double m_turb, V_turb;

        //Turbine power
        double Pw_turb;

        //Rotor and Shaft Properties
        //----------------------------------------------------------------------------

        //Current, idle and design angular rotation rates
        double omega, omega_idle, omega_design;

        //Rotational moment of inertia
        double J;

        //Frictional and extracted power efficiencies
        double eta_f, eta_m;

        //Reference properties for compressor and turbine maps
        double P_ref, T_ref;

        double starterTorque = 1000;

        bool runningStarter = false;
        //Constructor and related functions
        //----------------------------------------------------------------------------

        public AJEGasGeneratorSpool(double compressorDesignPressureRatio, double compressorInletDesignTemperature, double compressorEfficiency, double compressorVolume,
                                         double turbineInletDesignTemperature, double turbineEfficiency, double turbineVolume,
                                         double designRPM, double idleRPM,
                                         double frictionalEfficiency, double machineEfficiency,
                                         double referencePressure, double referenceTemperature,
                                         double rotationalInertia, double massFlowDesign)
        {
            P_comp_ratio_design = compressorDesignPressureRatio;
            T_comp_in_design = compressorInletDesignTemperature;
            eta_comp = compressorEfficiency;
            V_comp = compressorVolume;

            T_turb_in_design = turbineInletDesignTemperature;
            eta_turb = turbineEfficiency;
            V_turb = turbineVolume;

            omega_design = designRPM * Math.PI / 30;
            omega_idle = idleRPM * Math.PI / 30;

            eta_f = frictionalEfficiency;
            eta_m = machineEfficiency;

            P_ref = referencePressure;
            T_ref = referenceTemperature;

            J = rotationalInertia;
            mdot_comp_design = massFlowDesign;

            CalculateTurbineChokingPressureRatio();
        }

        private void CalculateTurbineChokingPressureRatio()
        {
            P_turb_ratio_choke = Math.Pow(P_comp_ratio_design, 0.4 / 1.4);
            P_turb_ratio_choke--;
            P_turb_ratio_choke /= (T_turb_in_design);
            P_turb_ratio_choke *= T_ref;
            //P_turb_ratio_choke *= P_ref;

            P_turb_ratio_choke = 1 - P_turb_ratio_choke;

            P_turb_ratio_choke = Math.Pow(P_turb_ratio_choke, 3.5);
        }

        //Public Calculation Functions
        //----------------------------------------------------------------------------

        public void CalculateCompressorEffects(double P_in, double T_in, double mdot_out,
            double gamma, double Cp, double Cv, double R, 
            double timeStep)
        {
            P_comp_in = P_in;
            T_comp_in = T_in;
            mdot_comp_ex_transient = mdot_out;

            CalculateCompressorProperties(gamma, Cp);
            CalculateCompressorVolumeTransients(R, Cp, Cv, timeStep);
        }

        public void CalculateTurbineEffects(double P_in, double T_in, double mdot_out,
            double gamma, double Cp, double Cv, double R,
            double timeStep)
        {
            P_turb_in = P_in;
            T_turb_in = T_in;
            mdot_turb_ex_transient = mdot_out;

            CalculateTurbineProperties(gamma, Cp);
            CalculateTurbineVolumeTransients(R, Cp, Cv, timeStep);

            CalculateSpoolTransientsFromPowerImbalance(timeStep);
        }

        public double CompressorOutletPressure() { return P_comp_ex; }
        public double CompressorOutletTemperature() { return T_comp_ex_transient; }
        public double CompressorInletMassFlow() { return mdot_comp_ex; }

        public double TurbineOutletPressure() { return P_turb_ex; }
        public double TurbineOutletTemperature() { return T_turb_ex_transient; }
        public double TurbineInletMassFlow() { return mdot_turb_ex; }

        public double FractionIdle() { return omega / omega_idle; }
        public double FractionPeak() { return omega / omega_design; }

        public void SetStartup(double comp_temperature_in, double comp_pressure_in, double comp_temperature_out, double comp_pressure_out,
                               double turb_temperature_in, double turb_pressure_in, double turb_temperature_out, double turb_pressure_out,
                               double massFlow, double density, bool runStarter)
        {
            omega = omega_idle * 0.05;
            P_comp_in = comp_pressure_in;
            P_comp_ex = comp_pressure_out;
            T_comp_in = comp_temperature_in;
            T_comp_ex = T_comp_ex_transient = comp_temperature_out;

            P_turb_in = turb_pressure_in;
            P_turb_ex = turb_pressure_out;
            T_turb_in = turb_temperature_in;
            T_turb_ex = T_turb_ex_transient = turb_temperature_out;

            mdot_comp_ex = massFlow;
            mdot_comp_ex_transient = massFlow;
            mdot_turb_ex = massFlow;
            mdot_turb_ex_transient = massFlow;

            m_comp = density * V_comp;
            m_turb = density * V_turb;

            runningStarter = runStarter;
        }

        //Compressor Calculation Functions
        //----------------------------------------------------------------------------

        //Calculate conditions due to work done by the compressor on the flow
        private void CalculateCompressorProperties(double gamma, double Cp)
        {
            double presRat = P_comp_ex / P_comp_in;
            double omega_corrected = omega * Math.Sqrt(T_comp_in_design / T_comp_in);
            double mdot_comp_ex_corr = EvaluateCompressorMap(omega_corrected, presRat);

            mdot_comp_ex = mdot_comp_ex_corr * P_comp_in;
            mdot_comp_ex /= (P_ref * Math.Sqrt(T_comp_in / T_ref));

            T_comp_ex = T_comp_in * (1 + (Math.Pow(presRat, (gamma - 1) / gamma) - 1) / eta_comp);

            Pw_comp = mdot_comp_ex * Cp * (T_comp_in - T_comp_ex);
        }

        //Quick and dirty compressor map based on design mass flow and design pressure ratio
        private double EvaluateCompressorMap(double omega, double presRat)
        {
            double pi_c, mdot_offset, mdot_corr;

            pi_c = mdot_offset = Math.Sqrt(omega / omega_design);

            pi_c *= P_comp_ratio_design;

            mdot_offset *= mdot_comp_design;

            //Debug.Log("Pi_c: " + pi_c + " mdot_offset: " + mdot_offset + " PresRat: " + presRat);

            mdot_corr = 1.05 * pi_c - presRat;

            //If the above is less than one, then there is no solution; this indicates compressor stall, so no mass flow this frame
            if (mdot_corr < 0)
                return 0;

            mdot_corr = Math.Sqrt(mdot_corr);
            mdot_corr *= -1.2880071555262936522535972282875;
            mdot_corr += mdot_offset + 0.2880071555262936522535972282875;

            Debug.Log(mdot_corr);

            return mdot_corr;
        }

        //Calculate the effects of transients in the compressor
        private void CalculateCompressorVolumeTransients(double R, double Cp, double Cv, double timeStep)
        {
            //Change in mass in compressor
            double d_dt_m_comp = (mdot_comp_ex - mdot_comp_ex_transient);

            //Change in temperature at end of compressor
            double d_dt_T_comp_ex_trans = mdot_comp_ex * (Cp * T_comp_ex - Cv * T_comp_ex_transient);
            d_dt_T_comp_ex_trans -= (mdot_comp_ex_transient * R * T_comp_ex_transient);
            d_dt_T_comp_ex_trans /= (m_comp * Cv);

            P_comp_ex = m_comp * R * T_comp_ex_transient / V_comp;

            m_comp += d_dt_m_comp * timeStep;
            m_comp = Math.Max(m_comp, 1);
            T_comp_ex_transient += d_dt_T_comp_ex_trans * timeStep;
            T_comp_ex_transient = Math.Max(T_comp_ex_transient, 100);
        }

        //Turbine Calculation Functions
        //----------------------------------------------------------------------------
        
        private void CalculateTurbineProperties(double gamma, double Cp)
        {
            double presRat = P_turb_ex / P_turb_in;
            double omega_corrected = omega * Math.Sqrt(T_turb_in_design / T_turb_in);

            double mdot_turb_ex_corr = EvaluateTurbineMap(omega_corrected, presRat);

            mdot_turb_ex = mdot_turb_ex_corr * P_turb_in;
            mdot_turb_ex /= (P_ref * Math.Sqrt(T_turb_in / T_ref));

            T_turb_ex = T_turb_in * (1 + eta_turb * (Math.Pow(presRat, (gamma - 1) / gamma) -1 ));

            Pw_turb = mdot_turb_ex * Cp * (T_turb_in - T_turb_ex);
        }

        //Quick turbine map based on known properties
        private double EvaluateTurbineMap(double omega, double presRat)
        {
            double mdot_turb_ex_corr;

            //pressure ratio is sufficient to choke flow; mdot is already known
            if (presRat < P_turb_ratio_choke)
                mdot_turb_ex_corr = mdot_comp_design;
            //turbine inlet unchoked; mdot calculated via approximate map
            else
            {
                mdot_turb_ex_corr = presRat - P_turb_ratio_choke;
                mdot_turb_ex_corr *= mdot_turb_ex_corr;

                double tmp = 1 - P_turb_ratio_choke;
                tmp *= tmp;
                tmp = -mdot_comp_design / tmp;

                mdot_turb_ex_corr *= tmp;
                mdot_turb_ex_corr += mdot_comp_design;
            }

            Debug.Log("mdot_turb_corr: " + mdot_turb_ex_corr + " presRat: " + presRat + " P_turb_ratio_choke: " + P_turb_ratio_choke);

            return mdot_turb_ex_corr;
        }

        private void CalculateTurbineVolumeTransients(double R, double Cp, double Cv, double timeStep)
        {
            //Change in mass in compressor
            double d_dt_m_turb = (mdot_turb_ex - mdot_turb_ex_transient);

            //Change in temperature at end of compressor
            double d_dt_T_turb_ex_trans = mdot_turb_ex * (Cp * T_turb_ex - Cv * T_turb_ex_transient);
            d_dt_T_turb_ex_trans -= (mdot_turb_ex_transient * R * T_turb_ex_transient);
            d_dt_T_turb_ex_trans /= (m_turb * Cv);

            P_turb_ex = m_turb * R * T_turb_ex_transient / V_turb;

            m_turb += d_dt_m_turb * timeStep;
            m_turb = Math.Max(m_turb, 1);
            T_turb_ex_transient += d_dt_T_turb_ex_trans * timeStep;
            T_turb_ex_transient = Math.Max(T_turb_ex_transient, 100);
        }

        private void CalculateSpoolTransientsFromPowerImbalance(double timeStep)
        {
            double angAcc = (Pw_turb * eta_m + Pw_comp);
            angAcc /= omega;

            if (runningStarter)
                angAcc += starterTorque;

            angAcc *= eta_f;                //frictional losses
            angAcc /= J;

            omega += angAcc * timeStep;

            Debug.Log("Omega: " + omega);

            if (omega > omega_idle)
                runningStarter = false;
        }
    }
}
