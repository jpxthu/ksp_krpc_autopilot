using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using KrpcAutoPilot;
using KrpcAutoPilot.Utils;
using System;
using System.Threading;

namespace CsSamples
{
    public class VesselControl
    {
        public static void Launch(
            string vessel_name,
            CommonData common_data,
            Connection connection,
            Service space_center,
            Vessel vessel,
            double apoapsis)
        {
            KrpcAutoPilot.Control control = new KrpcAutoPilot.Control(vessel_name, common_data, connection, space_center, vessel);
            control.UpdateData();

            control.Engage();

            vessel.Control.RCS = false;

            while (true)
            {
                if (!control.UpdateData())
                    break;
                if (control.LaunchIntoApoapsis(apoapsis, 0d))
                    break;
                if (!control.Execute())
                    break;
                Thread.Sleep(100);
            }

            control.LaunchIntoPeriapsisInit();

            while (true)
            {
                if (!control.UpdateData())
                    break;
                if (control.LaunchIntoPeriapsis(apoapsis))
                    break;
                if (!control.Execute())
                    break;
                Thread.Sleep(100);
            }

            control.DisEngage();
            control.Dispose();
        }

        private static void SwitchEngineMode(Vessel vessel)
        {
            foreach (var e in vessel.Parts.Engines)
            {
                if (e.Active && e.HasModes)
                {
                    e.ToggleMode();
                    break;
                }
            }
        }

        public static void Recycle(
            string vessel_name,
            CommonData common_data,
            Connection connection,
            Service space_center,
            Vessel vessel,
            KrpcAutoPilot.Control.RcsLayout rcs_layout,
            Vector3d tar_pos, double tar_altitude, double heading,
            ref KrpcAutoPilot.Control.LandingAdjustBurnStatus landing_adjust_burn_status,
            ref bool landing_adjust_could_burn)
        {
            KrpcAutoPilot.Control control = new KrpcAutoPilot.Control(vessel_name, common_data, connection, space_center, vessel);
            control.UpdateData();
            control.Command.SetHeadingAngle(Math.PI);

            control.Engage();

            vessel.Control.RCS = true;

            //vessel.Control.Brakes = true;
            SwitchEngineMode(vessel);
            control.Trajectory.ReCacheAvailableThrust();
            SwitchEngineMode(vessel);

            Console.WriteLine("Landing init");
            control.LandingInit(tar_altitude);

            Console.WriteLine("Adjust landing position");
            while (true)
            {
                if (!control.UpdateData())
                    break;
                landing_adjust_burn_status = control.AdjustLandingPosition(tar_pos, tar_altitude, landing_adjust_could_burn);
                if (landing_adjust_burn_status == KrpcAutoPilot.Control.LandingAdjustBurnStatus.FINISHED)
                    break;
                if (!control.Execute())
                    break;
                Thread.Sleep(100);
            }

            SwitchEngineMode(vessel);

            while (true)
            {
                if (!control.UpdateData())
                    break;
                if (control.Landing(tar_pos, tar_altitude, rcs_layout, 5d, heading))
                    break;
                if (!control.Execute())
                    break;
                Thread.Sleep(100);
            }

            control.DisEngage();
            control.Dispose();
            //vessel.Control.Brakes = false;
        }

        public static void Hover(
            string vessel_name,
            CommonData common_data,
            Connection connection,
            Service space_center,
            Vessel vessel,
            double tar_altitude,
            Vector3d tar_pos = null,
            KrpcAutoPilot.Control.RcsLayout rcs_layout = KrpcAutoPilot.Control.RcsLayout.TOP)
        {
            KrpcAutoPilot.Control control = new KrpcAutoPilot.Control(vessel_name, common_data, connection, space_center, vessel);
            control.UpdateData();

            control.Engage();

            vessel.Control.RCS = true;

            while (true)
            {
                if (!control.UpdateData())
                    break;
                if (control.Hover(tar_altitude, tar_pos, rcs_layout))
                    break;
                if (!control.Execute())
                    break;
                Thread.Sleep(100);
            }

            control.DisEngage();
            control.Dispose();
        }
    }
}
