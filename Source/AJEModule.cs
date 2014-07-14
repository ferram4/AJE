using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;
using KSP;

namespace AJE
{


    public class AJEModule : PartModule
    {
        public AJETransientGasGeneratorModel aje;

        [KSPField(isPersistant = false, guiActive = false)]
        public float IspMultiplier = 1f;
        [KSPField(isPersistant = false, guiActive = false)]
        public int defaultentype = 0;

        [KSPField(isPersistant = false, guiActive = false)]
        public bool useOverheat = true;
        //       [KSPField(isPersistant = false, guiActive = true)]
        public float parttemp;
        //       [KSPField(isPersistant = false, guiActive = true)]
        public float maxtemp;


        [KSPField(isPersistant = false, guiActive = false)]
        public bool useMultiMode = false;
        [KSPField(isPersistant = false, guiActive = false)]
        public bool isReactionEngine = false;
        [KSPField(isPersistant = false, guiActive = false)]
        public float idle = 0.03f;
        [KSPField(isPersistant = false, guiActive = false)]
        public int abflag = -1;
        [KSPField(isPersistant = false, guiActive = false)]
        public float acore = -1;
        [KSPField(isPersistant = false, guiActive = false)]
        public float byprat = -1;
        [KSPField(isPersistant = false, guiActive = false)]
        public float fhv = -1;
        [KSPField(isPersistant = false, guiActive = false)]
        public float prat13 = -1;
        [KSPField(isPersistant = false, guiActive = false)]
        public float prat3 = -1;
        [KSPField(isPersistant = false, guiActive = false)]
        public float prat2 = -1;
        [KSPField(isPersistant = false, guiActive = false)]
        public float prat4 = -1;
        [KSPField(isPersistant = false, guiActive = false)]
        public float tinlt = -1;
        [KSPField(isPersistant = false, guiActive = false)]
        public float tfan = -1;
        [KSPField(isPersistant = false, guiActive = false)]
        public float tt7 = -1;
        [KSPField(isPersistant = false, guiActive = false)]
        public float tt4 = -1;
        [KSPField(isPersistant = false, guiActive = false)]
        public float tcomp = -1;
        [KSPField(isPersistant = false, guiActive = false)]
        public float eta2 = -1;
        [KSPField(isPersistant = false, guiActive = false)]
        public float eta13 = -1;
        [KSPField(isPersistant = false, guiActive = false)]
        public float eta3 = -1;
        [KSPField(isPersistant = false, guiActive = false)]
        public float eta4 = -1;
        [KSPField(isPersistant = false, guiActive = false)]
        public float eta5 = -1;
        [KSPField(isPersistant = false, guiActive = false)]
        public float eta7 = -1;
        [KSPField(isPersistant = false, guiActive = false)]
        public float maxThrust = 99999;

        [KSPField(isPersistant = false, guiActive = false)]
        public float ABIspMult = 1.0f;


        public EngineWrapper engine;
        public bool useAB = false;
        public float ABthreshold = 0.667f;
        public float ABmax, ABmin;

        [KSPField(isPersistant = false, guiActive = true)]
        public String Mode;
        [KSPField(isPersistant = false, guiActive = true)]
        public String Inlet;

        [KSPField(isPersistant = false, guiActiveEditor = true)]
        public float Need_Area;
        //      [KSPField(isPersistant = false, guiActive = true)]
        public float fireflag;

        public float OverallThrottle = 0;


        // NK
        [KSPField(isPersistant = false, guiActive = false)]
        public FloatCurve prat3Curve = new FloatCurve();

        [KSPField(isPersistant = false, guiActive = false)]
        bool usePrat3Curve = false;


        public override void OnLoad(ConfigNode node)
        {
            base.OnLoad(node);
            if (node.HasNode("prat3Curve"))
            {
                //prat3Curve.Load(node.GetNode("prat3Curve"));
                print("AJE for part " + part.name + " found prat3Curve, usePrat3Curve = " + usePrat3Curve);
                float min, max;
                prat3Curve.FindMinMaxValue(out min, out max);
                print("curve: " + prat3Curve.minTime + ": " + min + "; " + prat3Curve.maxTime + ": " + max);
            }
        }


        public override void OnStart(StartState state)
        {

            if (state == StartState.Editor)
            {
                Need_Area = acore * (1 + byprat);
                return;
            }
            if (vessel == null)
                return;
            engine = new EngineWrapper(part);
            engine.useVelocityCurve = false;
            //engine.IspMultiplier = IspMultiplier;
            //engine.idle = idle;
            //engine.ThrustUpperLimit = maxThrust;
            engine.useEngineResponseTime = false;
            part.maxTemp = 3600f;
            engine.heatProduction = 360f;
            aje = new AJETransientGasGeneratorModel();

            acore *= 0.092903f;

            useAB = abflag > 0;
            if (eta5 < 0)
                eta5 = 0.95f;
            if (eta3 < 0)
                eta3 = 0.95f;
            if (eta13 < 0)
                eta3 = 0.95f;

            aje.InitializeOverallEngineData(acore, acore * 0.1, acore * 0.9, acore * 0.1 * 0.3, 4.3e7, tt4, tt7);
            aje.AddNewSpool(prat3, 288, eta3, acore * 1.5, tt4, eta5, acore * 0.3, 7685, 5000, 0.99, 0.95, 101300, 288, 3000, 77);




            /*aje.InitializeEngineGeometry(acore, acore * 0.2, acore * 0.9, acore * 0.9, acore * 1.2, byprat, 8000, 0.5, 3000);
            aje.InitializeComponentEfficiencies(1, eta3, prat3, eta13, prat13, prat4, eta5, 1);
            aje.InitializeFuelProperties(tt4, tt7, 4.3e7, useAB);*/

            /*if (part.partInfo.partPrefab.Modules.Contains("AJEModule"))
            {
                AJEModule a = (AJEModule)part.partInfo.partPrefab.Modules["AJEModule"];
                usePrat3Curve = a.usePrat3Curve;
                prat3Curve = a.prat3Curve;
            }
            if (usePrat3Curve)
            {
                print("AJE OnStart for part " + part.name + " found prat3Curve");
                float min, max;
                prat3Curve.FindMinMaxValue(out min, out max);
                print("curve: " + prat3Curve.minTime + ": " + min + "; " + prat3Curve.maxTime + ": " + max);
                aje.fsmach = 0.0;
                aje.prat[3] = aje.p3p2d = prat3Curve.Evaluate(0.0f);
            }*/
        }


        public void FixedUpdate()
        {
            if (HighLogic.LoadedSceneIsEditor)
                return;
            if (engine.type == EngineWrapper.EngineType.NONE || !engine.EngineIgnited)
            {
                aje.started = false;
                return;
            }
            if (vessel.mainBody.atmosphereContainsOxygen == false || part.vessel.altitude > vessel.mainBody.maxAtmosphereAltitude)
            {
                engine.SetThrust(0);
                return;
            }

            #region Inlet

            float Arearatio, OverallTPR = 0, EngineArea = 0, InletArea = 0;
            AJEModule e;
            AJEInlet i;
            foreach (Part p in vessel.parts)
            {

                if (p.Modules.Contains("AJEModule"))
                {
                    e = (AJEModule)p.Modules["AJEModule"];
                    EngineArea += (float)(e.acore * (1 + e.byprat));
                }
                if (p.Modules.Contains("AJEInlet"))
                {
                    i = (AJEInlet)p.Modules["AJEInlet"];
                    if (((ModuleResourceIntake)p.Modules["ModuleResourceIntake"]).intakeEnabled)
                    {
                        InletArea += i.Area;
                        OverallTPR += i.Area * i.cosine * i.cosine * i.GetTPR(0);
                    }
                }
            }
            if (InletArea > 0)
                OverallTPR /= InletArea;
            Arearatio = Mathf.Min(InletArea / EngineArea, 1f);
            //aje.eta[2] = Mathf.Clamp(Mathf.Sqrt(Arearatio) * OverallTPR, 0.001f, 1f);

            Inlet = "Area:" + ((int)(Arearatio * 100f)).ToString() + "%  TPR:" + ((int)(OverallTPR * 100f)).ToString() + "%";

            #endregion

            double pressure = FlightGlobals.getStaticPressure(vessel.altitude, vessel.mainBody) * 101300;
            double temperature = FlightGlobals.getExternalTemperature((float)vessel.altitude, vessel.mainBody) + 273.15;
            //double density = FlightGlobals.getAtmDensity(pressure);
            double velocity = part.vessel.srfSpeed;

            if (!aje.started)
                aje.StartupFunction(pressure, temperature, velocity);

            /*if (usePrat3Curve)
            {
                aje.prat[3] = aje.p3p2d = (double)(prat3Curve.Evaluate((float)aje.fsmach));
            }*/
            int stepsPerTimestep = 4;
            float timeStep = TimeWarp.fixedDeltaTime / stepsPerTimestep;
            double thrust, I_sp;
            thrust = I_sp = 0;
            OverallThrottle = vessel.ctrlState.mainThrottle;
            if (!useAB)
            {
                for (int j = 0; j < stepsPerTimestep; j++)
                    aje.CalculatePerformance(pressure, temperature, velocity, timeStep, OverallThrottle);
                thrust = aje.GetThrust();
                I_sp = aje.GetIsp();
                //aje.CalculatePerformance(pressure, temperature, part.vessel.srfSpeed, OverallThrottle, density, OverallThrottle, TimeWarp.fixedDeltaTime, out thrust, out I_sp);
                engine.SetThrust((float)thrust);
                engine.SetIsp((float)I_sp);
                Mode = "Cruise " + System.Convert.ToString((int)(OverallThrottle * 100f)) + "%";
            }
            else
            {
                for (int j = 0; j < stepsPerTimestep; j++)
                    aje.CalculatePerformance(pressure, temperature, velocity, timeStep, OverallThrottle);
                thrust = aje.GetThrust();
                I_sp = aje.GetIsp();
                //aje.CalculatePerformance(pressure, temperature, part.vessel.srfSpeed, OverallThrottle, density, OverallThrottle, TimeWarp.fixedDeltaTime, out thrust, out I_sp);
                engine.SetThrust((float)thrust);
                engine.SetIsp((float)I_sp);

                if (OverallThrottle <= ABthreshold)
                    Mode = "Cruise " + System.Convert.ToString((int)(OverallThrottle / ABthreshold * 100f)) + "%";
                else
                    Mode = "Afterburner " + System.Convert.ToString((int)((OverallThrottle - ABthreshold) / (1 - ABthreshold) * 100f)) + "%";

            }

            engine.currentThrottle = 1;
            Mode += " (" + (thrust).ToString("N2") + "kN gr)";




            /*if (aje.fireflag > 0.9f && useOverheat)
            {
                part.temperature = (aje.fireflag * 2f - 1.2f) * part.maxTemp;

            }*/

            //fireflag = aje.fireflag;
            parttemp = part.temperature;
            maxtemp = part.maxTemp;


            //           mach = (float)aje.fsmach;


        }


    }





}