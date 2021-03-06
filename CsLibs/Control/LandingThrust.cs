﻿using KrpcAutoPilot.Utils;
using System;

namespace KrpcAutoPilot
{
    public partial class Control
    {
        public Trajectory.SimulationResult LandingSimThrust(Trajectory.SimulationData data)
        {
            Vector3d up = data.pos.Norm();
            Vector3d vel_dir = data.vel.Norm();
            double vessel_up_ratio = data.vel.Length() > 100d ? -up * vel_dir : 1d;
            if (vessel_up_ratio < 0.5d)
                return new Trajectory.SimulationResult();

            double max_thrust_up = /*vessel_up_ratio */ data.available_thrust;
            double max_acc_up = Math.Max(0.01d, max_thrust_up / data.mass * LANDING_MAX_THROTTLE - data.g);
            LinearPlanner.OneWay(data.altitude - data.tar_altitude,
                0.5d, max_acc_up, landing_min_velocity_,
                out double tar_vel, out double tar_acc);
            tar_acc += (tar_vel - data.vel * up) * 2d;
            double tar_throttle = Math.Clamp((tar_acc + data.g) * data.mass / max_thrust_up, 0d, 1d);

            return new Trajectory.SimulationResult(
                -tar_throttle * data.available_thrust * vel_dir,
                tar_throttle * data.available_thrust,
                tar_throttle);
        }

        private bool LandingThrust(double tar_altitude)
        {
            if (landing_adjust_throttle_ > 0d)
            {
                Command.SetThrottle(landing_adjust_throttle_);
                return false;
            }

            double vessel_up_ratio = State.Vessel.VelocityMag > 100d ?
                -State.Vessel.Velocity.Norm() * State.Vessel.SurfUp :
                State.Vessel.Direction * State.Vessel.SurfUp;
            if (vessel_up_ratio < 0.5d)
            {
                Command.SetThrottle(0d);
                return false;
            }

            double max_thrust_up = /*vessel_up_ratio */ State.Vessel.AvailableThrust;
            double max_acc_up = Math.Max(0.01d, max_thrust_up / State.Vessel.Mass * LANDING_MAX_THROTTLE - State.Vessel.GravityMag);
            LinearPlanner.OneWay(State.Vessel.Altitude - tar_altitude,
                0.5d, max_acc_up, landing_min_velocity_,
                out double tar_vel, out double tar_acc);
            tar_acc += (tar_vel - State.Vessel.VelocityUp) * 2d;
            double tar_throttle = Math.Clamp((tar_acc + State.Vessel.GravityMag) * State.Vessel.Mass / max_thrust_up, 0d, 1d);
            Command.SetThrottle(tar_throttle);

            //sw.WriteLine("{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\t{7}",
            //    Data.UT,
            //    State.Vessel.Velocity,
            //    State.Vessel.Position,
            //    tar_throttle,
            //    State.Vessel.Mass,
            //    State.Vessel.AvailableThrust,
            //    State.Vessel.Gravity,
            //    new Vector3d(ActiveVessel.Flight().AerodynamicForce));

            return false;
        }

    }
}
