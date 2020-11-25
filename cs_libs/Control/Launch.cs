using KRPC.Client.Services.SpaceCenter;

namespace KrpcAutoPilot
{
    public partial class Control
    {
        private bool launch_burn_stage = false;
        private double launch_apoapsis;
        private Node launch_periapsis_node;

        public void LaunchIntoPeriapsisInit()
        {
            launch_burn_stage = false;
            launch_apoapsis = State.Orbit.Apoapsis;
        }

        public bool LaunchIntoApoapsis(double tar_apoapsis, double max_q = 15000)
        {
            double tar_pitch = (1.0 - (State.Orbit.Apoapsis - State.Body.Radius) / tar_apoapsis) * 90;

            tar_apoapsis += State.Body.Radius;

            double q = Utils.Common.DynamicPressure(
                State.Environment.StaticPressure,
                State.Environment.Temperature,
                State.Vessel.VelocityMag);
            double throttle = System.Math.Min(1, System.Math.Max(0, (1.1 - q / max_q) * 5));

            Command.SetTargetPitchAndHeading(tar_pitch, 90);
            Command.SetThrottle(throttle);

            return State.Orbit.Apoapsis >= tar_apoapsis;
        }

        public bool LaunchIntoPeriapsis(double tar_periapsis)
        {
            tar_periapsis += State.Body.Radius;

            if (launch_burn_stage) {
                Command.SetThrottle(1);
                if (State.Orbit.Apoapsis + State.Orbit.Periapsis >= tar_periapsis + launch_apoapsis) {
                    launch_periapsis_node.Remove();
                    return true;
                } else {
                    return false;
                }
            }

            double max_acc = State.Vessel.AvailableThrust / State.Vessel.Mass;

            double vel_apoapsis = State.Vessel.VelocityMag * (State.Vessel.Altitude + State.Body.Radius) / State.Orbit.Apoapsis;
            double vel_apoapsis_tar = System.Math.Sqrt(State.Body.GravitationalParameter / State.Orbit.Apoapsis);
            double dvel_tar = vel_apoapsis_tar - vel_apoapsis;
            double burn_time = dvel_tar / max_acc;
            double drift_time = State.Orbit.TimeToApoapsis - burn_time / 2 - 2;

            Command.SetTargetPitchAndHeading((1.0 - (State.Orbit.Apoapsis - State.Body.Radius) / (tar_periapsis - State.Body.Radius)) * 90, 90);
            Command.SetThrottle(System.Math.Max(0.0, System.Math.Min(1.0, (tar_periapsis - State.Orbit.Apoapsis) / 20)));

            if (drift_time < 0) {
                Node node = ActiveVessel.Control.AddNode(Data.UT + State.Orbit.TimeToApoapsis, System.Convert.ToSingle(dvel_tar));
                //Command.SetTargetDirection(node.Direction(ActiveVessel.ReferenceFrame));
                ActiveVessel.AutoPilot.ReferenceFrame = node.ReferenceFrame;
                Command.SetTargetDirection(0.0f, 1.0f, 0.0f);
                launch_burn_stage = true;
                launch_periapsis_node = node;
            }

            return false;
        }
    }
}
