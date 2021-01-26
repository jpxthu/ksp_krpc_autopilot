using KrpcAutoPilot.Utils;
using System;

namespace KrpcAutoPilot
{
    public partial class Control
    {
        private bool HoverDirection()
        {
            Command.SetTargetDirection(State.Vessel.SurfUp);
            return false;
        }

        private bool HoverThrust(double tar_altitude)
        {
            double vessel_up_ratio = State.Vessel.VelocityMag > 100d ?
                -State.Vessel.Velocity.Norm() * State.Vessel.SurfUp :
                State.Vessel.Direction * State.Vessel.SurfUp;
            if (vessel_up_ratio < 0.5d)
            {
                Command.SetThrottle(0d);
                return false;
            }

            double max_thrust_up = vessel_up_ratio * State.Vessel.AvailableThrust;
            double max_acc_up = Math.Max(0.01d, max_thrust_up / State.Vessel.Mass * LANDING_MAX_THROTTLE - State.Vessel.GravityMag);
            double max_acc_down = State.Vessel.GravityMag * LANDING_MAX_THROTTLE;
            LinearPlanner.Hover(State.Vessel.Altitude - tar_altitude,
                0.3d, max_acc_up, max_acc_down,
                out double tar_vel, out double tar_acc);
            tar_acc += (tar_vel - State.Vessel.VelocityUp) * 2d;
            double tar_throttle = Math.Clamp((tar_acc + State.Vessel.GravityMag) * State.Vessel.Mass / max_thrust_up, 0d, 1d);
            Command.SetThrottle(tar_throttle);

            return false;
        }

        private bool HoverRcs(Vector3d tar_pos = null, RcsLayout rcs_layout = RcsLayout.TOP)
        {
            if (!Trajectory.ResultAvailable)
                return false;

            double vessel_up_ratio = -State.Vessel.Direction * State.Vessel.Velocity.Norm();
            if (vessel_up_ratio < 0.9d && State.Vessel.VelocityMag > 50d)
            {
                Command.SetRcsRight(0d);
                Command.SetRcsUp(0d);
                return false;
            }

            double rcs_force_limit = RcsMaxHorizonForce(rcs_layout);
            double max_rcs_acc = rcs_force_limit / State.Vessel.Mass * 0.9d;

            Vector3d tar_vel_h;
            Vector3d tar_acc_h;
            if (tar_pos == null)
            {
                tar_vel_h = Vector3d.Zero;
                tar_acc_h = Vector3d.Zero;
            }
            else
            {
                Vector3d err_v = tar_pos - State.Vessel.Position;// Trajectory.ImpactPositionWithAction;
                Vector3d err_h = VectorHorizonPart(err_v);
                double err = err_h.Length();
                LinearPlanner.Hover(
                    err, 1d, max_rcs_acc, max_rcs_acc,
                    out double tar_vel, out double tar_acc);

                tar_vel_h = -err_h.Norm() * tar_vel;
                tar_acc_h = -err_h.Norm() * tar_acc;
            }

            tar_acc_h += (tar_vel_h - State.Vessel.VelocityHorizon) * 1d;
            RcsSetByForce(
                Math.Clamp(tar_acc_h * State.Vessel.Right * State.Vessel.Mass, -rcs_force_limit, rcs_force_limit),
                Math.Clamp(tar_acc_h * State.Vessel.Up * State.Vessel.Mass, -rcs_force_limit, rcs_force_limit));

            return false;
        }

        public bool Hover(double tar_altitude, Vector3d tar_pos = null, RcsLayout rcs_layout = RcsLayout.TOP)
        {
            ActiveVessel.AutoPilot.TargetHeading = 90f;
            HoverDirection();
            HoverThrust(tar_altitude);
            HoverRcs(tar_pos, rcs_layout);
            return false;
        }
    }
}
