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
            Vector3d tar_pos, double tar_altitude)
        {
            KrpcAutoPilot.Control control = new KrpcAutoPilot.Control(connection, space_center, data, vessel);
            control.UpdateData();
            control.Command.SetHeading(Math.PI);

            control.Engage();

            vessel.Control.RCS = true;

            Console.WriteLine("Landing init");
            control.LandingInit(tar_altitude);
            Thread.Sleep(5000);

            Console.WriteLine("Adjust landing position");
            while (true)
            {
                control.UpdateData();
                if (control.AdjustLandingPosition(tar_pos, tar_altitude))
                    break;
                control.Execute();
                Thread.Sleep(100);
            }

            while (true)
            {
                control.UpdateData();
                if (control.Landing(tar_pos, tar_altitude, rcs_layout, 5d))
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
