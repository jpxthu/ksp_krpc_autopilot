using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using KrpcAutoPilot.Utils;
using System;
using System.Threading;

namespace CsSamples
{
    public class VesselControl
    {
        public static void Launch(
            Connection connection,
            Service space_center,
            Vessel vessel,
            KrpcAutoPilot.Data.CommonData data,
            double apoapsis)
        {
            KrpcAutoPilot.Control control = new KrpcAutoPilot.Control(connection, space_center, data, vessel);
            control.UpdateData();

            control.Engage();

            vessel.Control.RCS = false;

            while (true)
            {
                control.UpdateData();
                if (control.LaunchIntoApoapsis(apoapsis, 0d))
                    break;
                control.Execute();
                Thread.Sleep(100);
            }

            control.DisEngage();
        }

        public static void Recycle(
            Connection connection,
            Service space_center,
            Vessel vessel,
            KrpcAutoPilot.Control.RcsLayout rcs_layout,
            KrpcAutoPilot.Data.CommonData data,
            Vector3d tar_pos, double tar_altitude, double heading,
            ref KrpcAutoPilot.Control.LandingAdjustBurnStatus landing_adjust_burn_status,
            ref bool landing_adjust_could_burn)
        {
            KrpcAutoPilot.Control control = new KrpcAutoPilot.Control(connection, space_center, data, vessel);
            control.UpdateData();
            control.Command.SetHeading(Math.PI);

            control.Engage();

            vessel.Control.RCS = true;

            Console.WriteLine("Landing init");
            control.LandingInit(tar_altitude);

            Console.WriteLine("Adjust landing position");
            while (true)
            {
                control.UpdateData();
                landing_adjust_burn_status = control.AdjustLandingPosition(tar_pos, tar_altitude, heading, landing_adjust_could_burn);
                if (landing_adjust_burn_status == KrpcAutoPilot.Control.LandingAdjustBurnStatus.FINISHED)
                    break;
                control.Execute();
                Thread.Sleep(100);
            }

            foreach (var e in vessel.Parts.Engines)
            {
                if (e.Active)
                {
                    e.ToggleMode();
                    break;
                }
            }
            control.Trajectory.ReCacheAvailableThrust();

            while (true)
            {
                control.UpdateData();
                if (control.Landing(tar_pos, tar_altitude, rcs_layout, 5d, heading))
                    break;
                control.Execute();
                Thread.Sleep(100);
            }

            control.DisEngage();

            control.Trajectory.CalculateStop();
        }

        public static void Hover(
            Connection connection,
            Service space_center,
            Vessel vessel,
            KrpcAutoPilot.Data.CommonData data,
            double tar_altitude,
            Vector3d tar_pos = null,
            KrpcAutoPilot.Control.RcsLayout rcs_layout = KrpcAutoPilot.Control.RcsLayout.TOP)
        {
            KrpcAutoPilot.Control control = new KrpcAutoPilot.Control(connection, space_center, data, vessel);
            control.UpdateData();

            control.Engage();

            vessel.Control.RCS = true;

            while (true)
            {
                control.UpdateData();
                if (control.Hover(tar_altitude, tar_pos, rcs_layout))
                    break;
                control.Execute();
                Thread.Sleep(100);
            }

            control.DisEngage();
        }
    }
}
