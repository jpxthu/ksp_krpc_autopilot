using KrpcAutoPilot.Utils;
using System;

namespace KrpcAutoPilot
{
    public partial class Control
    {
        private bool LandingRcs(Vector3d tar_pos, double tar_altitude, RcsLayout rcs_layout)
        {
            if (!Trajectory.ResultAvailable)
                return false;

            double vessel_up_ratio = -State.Vessel.Direction * State.Vessel.Velocity.Norm();
            if ((vessel_up_ratio < 0.9d && State.Vessel.VelocityMag > 50d) ||
                State.Vessel.Direction * State.Vessel.SurfUp < 0.5d)
            {
                Command.SetRcsRight(0d);
                Command.SetRcsUp(0d);
                return false;
            }

            double rcs_force_limit = RcsMaxHorizonForce(rcs_layout);

            Vector3d v_err = tar_pos - Trajectory.ImpactPositionWithAction;
            Vector3d tar_acc = v_err / 10d;
            landing_rcs_tar_acc_i += tar_acc * 0.03d;
            double int_upper_bound = Math.Min(
                Math.Abs(tar_altitude - State.Vessel.Altitude) / 1000d,
                Math.Abs(State.Vessel.VelocityUp) / 100d);
            int_upper_bound = Math.Min(int_upper_bound, 5d);
            if (landing_rcs_tar_acc_i.Length() > int_upper_bound)
                landing_rcs_tar_acc_i *= int_upper_bound / landing_rcs_tar_acc_i.Length();
            tar_acc += landing_rcs_tar_acc_i;

            RcsSetByForce(
                Math.Clamp(tar_acc * State.Vessel.Right * State.Vessel.Mass, -rcs_force_limit, rcs_force_limit),
                Math.Clamp(tar_acc * State.Vessel.Up * State.Vessel.Mass, -rcs_force_limit, rcs_force_limit));

            return false;
        }

        private bool LandingRcsLast(Vector3d tar_pos, RcsLayout rcs_layout)
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

            Vector3d err_v = tar_pos - State.Vessel.Position;// Trajectory.ImpactPositionWithAction;
            Vector3d err_h = VectorHorizonPart(err_v);
            double err = err_h.Length();
            LinearPlanner.Hover(
                err, 1d, max_rcs_acc, max_rcs_acc,
                out double tar_vel, out double tar_acc);

            Vector3d tar_vel_h = -err_h.Norm() * tar_vel;
            Vector3d tar_acc_h = -err_h.Norm() * tar_acc;
            //Vector3d tar_vel_h = Vector3d.Zero;
            //Vector3d tar_acc_h = Vector3d.Zero;
            //Vector3d engine_acc_h = VectorHorizonPart(State.Vessel.Thrust * State.Vessel.Direction);

            tar_acc_h += (tar_vel_h - State.Vessel.VelocityHorizon) * 1d;// - engine_acc_h * 0.5d;
            RcsSetByForce(
                Math.Clamp(tar_acc_h * State.Vessel.Right * State.Vessel.Mass, -rcs_force_limit, rcs_force_limit),
                Math.Clamp(tar_acc_h * State.Vessel.Up * State.Vessel.Mass, -rcs_force_limit, rcs_force_limit));

            return false;
        }
    }
}
