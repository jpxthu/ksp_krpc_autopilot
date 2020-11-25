using KRPC.Client.Services.Drawing;
using KrpcAutoPilot.Utils;
using System;
using System.IO;

namespace KrpcAutoPilot
{
    public partial class Control
    {
        const double LANDING_MAX_THROTTLE = 0.95d;
        const double MIN_LANDING_VELOCITY = 8d;

        private bool rcs_altitude;
        private double landing1_lift_angle = 0d;
        private StreamWriter sw;

        public bool LandingInit(double tar_height)
        {
            sw = new StreamWriter("r.tsv");
            foreach (var rcs in ActiveVessel.Parts.RCS)
            {
                rcs.RollEnabled = true;
                rcs.PitchEnabled = true;
                rcs.YawEnabled = true;
            }
            rcs_altitude = true;
            landing1_lift_angle = 0d;
            Trajectory.CalculateStart(tar_height: tar_height);
            return true;
        }

        /// <summary>
        /// 高空调整姿态和点火，调整下落轨迹
        /// </summary>
        /// <returns></returns>
        public bool AdjustLandingPosition(Vector3d tar_pos)
        {
            if (!Trajectory.ResultAvailable)
                return false;
            var tar_v = tar_pos - Trajectory.ImpactPositionWithAction;
            var distance = tar_v.Length();
            tar_v = (tar_v - tar_v * State.Vessel.BodyUp * State.Vessel.BodyUp).Norm();
            var dir_error = State.Vessel.Direction * tar_v;
            var tar_t = Math.Clamp(distance / 15000d, 0.5d, 1d) *
                Math.Max(0d, (dir_error - 0.95d) * 20d);
            Conn.Drawing().Clear();
            Conn.Drawing().AddDirection(
                SpaceCenter.TransformDirection(tar_v.ToTuple(), ActiveVessel.Orbit.Body.ReferenceFrame, ActiveVessel.ReferenceFrame),
                ActiveVessel.ReferenceFrame);
            if (distance < 1000d && tar_v * State.Vessel.Direction < 0.985d)
            {
                Command.SetThrottle(0d);
                return true;
            }
            else
            {
                Console.WriteLine(distance);
                Command.SetTargetDirection(tar_v);
                Command.SetThrottle(tar_t);
                return false;
            }
        }

        public bool AdjustLandingPosition2(Vector3d tar_pos)
        {
            if (!Trajectory.ResultAvailable)
                return false;
            Vector3d tar_pos_dir = (tar_pos - State.Vessel.Position).Norm();
            Vector3d tar_dir = Vector3d.Cross(Vector3d.Cross(tar_pos_dir, State.Vessel.BodyUp).Norm(), tar_pos_dir).Norm();
            Vector3d error = tar_pos - Trajectory.ImpactPositionWithAction;
            double distance = error.Length();
            var dir_error = State.Vessel.Direction * tar_dir;
            var tar_t = Math.Clamp(distance / 15000d, 0.4d, 1d) *
                Math.Max(0d, (dir_error - 0.95d) * 20d);
            Conn.Drawing().Clear();
            Conn.Drawing().AddDirection(
                SpaceCenter.TransformDirection(tar_dir.ToTuple(), ActiveVessel.Orbit.Body.ReferenceFrame, ActiveVessel.ReferenceFrame),
                ActiveVessel.ReferenceFrame);
            if (tar_t > 0d && error * tar_dir <= 0d)
            {
                Command.SetThrottle(0d);
                return true;
            }
            else
            {
                Console.WriteLine(distance);
                Command.SetTargetDirection(tar_dir);
                Command.SetThrottle(tar_t);
                return false;
            }
        }

        public bool Landing1Direction(Vector3d tar_pos)
        {
            if (!Trajectory.ResultAvailable)
            {
                Command.SetTargetDirection(-State.Vessel.Velocity.Norm());
                return false;
            }
            else if (Trajectory.NextBurnTime > 5d)
            {
                if (Trajectory.LiftEstimationForceMin > 1d)
                {
                    landing1_lift_angle = 10d / 180d * Math.PI;
                }
                else
                {
                    Command.SetTargetDirection(Trajectory.EnterAtmosphereDirection);
                    Console.WriteLine("{0:0.0}", (tar_pos - Trajectory.ImpactPositionWithAction).Length());
                    return false;
                }
            }
            else if (Trajectory.NextBurnTime < 1d)
            {
                double f = Trajectory.LiftEstimationThrustAve * Math.Sin(Trajectory.LiftEstimationAngle);
                double diff = f - Trajectory.LiftEstimationForceAve;
                if (landing1_lift_angle < 0d)
                {
                    if (diff < State.Vessel.Mass)
                    {
                        landing1_lift_angle = 0d;
                    }
                    else
                    {
                        landing1_lift_angle = -Trajectory.LiftEstimationAngle;
                    }
                }
                else if (landing1_lift_angle > 0d)
                {
                    if (diff > -State.Vessel.Mass)
                    {
                        landing1_lift_angle = 0d;
                    }
                    else
                    {
                        landing1_lift_angle = Trajectory.LiftEstimationAngle;
                    }
                }
                else
                {
                    if (diff > State.Vessel.Mass * 1.5d)
                    {
                        landing1_lift_angle = -Trajectory.LiftEstimationAngle;
                    }
                    else if (diff < -State.Vessel.Mass * 1.5d)
                    {
                        landing1_lift_angle = Trajectory.LiftEstimationAngle;
                    }
                    else
                    {
                        landing1_lift_angle = 0d;
                    }
                }
            }
            else
            {
                landing1_lift_angle = 0d;
            }

            var tar_pos_v = tar_pos - Trajectory.ImpactPositionWithAction;
            Vector3d turn_v = -landing1_lift_angle
                * Math.Clamp(tar_pos_v.LengthSquared() / 10d / Math.Max(1000d, State.Vessel.Altitude), 0d, 1d)
                * Vector3d.Cross(Trajectory.ImpactPositionWithAction.Norm(), tar_pos.Norm()).Norm();
            var r = turn_v.RotationMatrix();
            var tar_dir = r * (-State.Vessel.Velocity).Norm();

            Command.SetTargetDirection(tar_dir);

            Conn.Drawing().AddDirection(
                SpaceCenter.TransformDirection(tar_pos_v.ToTuple(), OrbitBody.ReferenceFrame, ActiveVessel.ReferenceFrame),
                ActiveVessel.ReferenceFrame);
            Conn.Drawing().AddDirection(
                new Tuple<double, double, double>(0, 2, 0),
                ActiveVessel.ReferenceFrame);
            Conn.Drawing().AddDirection(
                SpaceCenter.TransformDirection(tar_dir.ToTuple(), OrbitBody.ReferenceFrame, ActiveVessel.ReferenceFrame),
                ActiveVessel.ReferenceFrame,
                30);
            Conn.Drawing().AddLine(
                State.Vessel.Position.ToTuple(),
                Trajectory.ImpactPositionWithAction.ToTuple(),
                OrbitBody.ReferenceFrame);
            //Console.WriteLine("{0:0.0}\t{1:0}\t{2:0}\t{3:0.00}\t{4:0}\t{5:0}",
            //    Trajectory.NextBurnTime,
            //    Trajectory.LiftEstimationThrustAve,
            //    Trajectory.LiftEstimationForceAve,
            //    landing1_lift_angle / Math.PI * 180d,
            //    tar_pos_v.Length(),
            //    State.Vessel.Altitude);

            return false;
        }

        public bool Landing1DirectionLast()
        {
            if (-State.Vessel.VelocityUp < 30d)
                Command.SetTargetDirection(State.Vessel.BodyUp);
            else
                Command.SetTargetDirection(-State.Vessel.Velocity);

            return false;
        }

        public Trajectory.SimulationResult LandingSimThrust(Trajectory.SimulationData data)
        {
            Vector3d up = data.pos.Norm();
            Vector3d vel_dir = data.vel.Norm();
            double vessel_up_ratio = -up * vel_dir;
            if (vessel_up_ratio < 0.5d)
                return new Trajectory.SimulationResult();

            double max_thrust_up = vessel_up_ratio * data.available_thrust;
            double max_acc_up = Math.Max(0.01d, max_thrust_up / data.mass * LANDING_MAX_THROTTLE - data.g);
            double max_acc_down = data.g * LANDING_MAX_THROTTLE;
            double tmp = data.altitude - data.tar_height;
            double tar_vel, tar_acc;
            if (tmp > 2d * max_acc_up)
            {
                tar_vel = -Math.Sqrt(2d * max_acc_up * tmp);
                tar_acc = max_acc_up;
            }
            else if (tmp < -2d * max_acc_down)
            {
                tar_vel = Math.Sqrt(2d * max_acc_down * -tmp);
                tar_acc = -max_acc_down;
            }
            else
            {
                tar_vel = -tmp;
                tar_acc = tmp;
            }
            tar_acc += (tar_vel - data.vel * up) * 2d;
            double tar_throttle = Math.Clamp((tar_acc + data.g) * data.mass / max_thrust_up, 0d, 1d);

            return new Trajectory.SimulationResult(
                -tar_throttle * data.available_thrust * vel_dir,
                tar_throttle * data.available_thrust,
                tar_throttle);
        }

        public bool Landing1Thrust(double tar_height)
        {
            double vessel_up_ratio = State.Vessel.VelocityMag > 100 ?
                -State.Vessel.Velocity.Norm() * State.Vessel.BodyUp :
                State.Vessel.Direction * State.Vessel.BodyUp;
            if (vessel_up_ratio < 0.5d)
            {
                Command.SetThrottle(0d);
                return false;
            }

            double max_thrust_up = vessel_up_ratio * State.Vessel.AvailableThrust;
            double max_acc_up = Math.Max(0.01d, max_thrust_up / State.Vessel.Mass * LANDING_MAX_THROTTLE - State.Vessel.Gravity);
            double max_acc_down = State.Vessel.Gravity * LANDING_MAX_THROTTLE;
            double tmp = State.Vessel.Altitude - tar_height;
            double tar_vel, tar_acc;
            LongitudinalPlanner.Hover(
                tmp, 1d, max_acc_up, max_acc_down,
                out tar_vel, out tar_acc);
            /*if (tmp > 2d * max_acc_up)
            {
                tar_vel = -Math.Sqrt(2d * max_acc_up * tmp);
                tar_acc = max_acc_up;
            }
            else if (tmp < -2d * max_acc_down)
            {
                tar_vel = Math.Sqrt(2d * max_acc_down * -tmp);
                tar_acc = -max_acc_down;
            }
            else
            {
                tar_vel = -tmp;
                tar_acc = tmp;
            }
            if (Math.Abs(tar_vel) < 1d)
            {
                tar_vel = -1d;
            }*/
            Console.Write("{0:0.00}\t{1:0.00}\t",
                tar_vel,
                tar_acc);
            if (tar_vel > -MIN_LANDING_VELOCITY)
            {
                tar_vel = -MIN_LANDING_VELOCITY;
                tar_acc = 0d;
            }
            Console.Write("{0:0.00}\t{1:0.00}\t",
                tar_vel,
                tar_acc);

            tar_acc += (tar_vel - State.Vessel.VelocityUp) * 2d;
            double tar_throttle = (tar_acc + State.Vessel.Gravity) * State.Vessel.Mass / max_thrust_up;
            Command.SetThrottle(Math.Clamp(tar_throttle, 0d, 1d));

            Console.WriteLine("{0:0.00}\t{1:0.00}\t{2:0.00}",
                State.Vessel.VelocityUp,
                tar_acc,
                tar_throttle);

            sw.WriteLine("{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\t{7}",
                Data.UT,
                State.Vessel.Velocity,
                State.Vessel.Position,
                tar_throttle,
                State.Vessel.Mass,
                State.Vessel.AvailableThrust,
                State.Vessel.Gravity,
                new Vector3d(ActiveVessel.Flight().AerodynamicForce));

            return false;
        }

        private bool Landing1Rcs(Vector3d tar_pos, double tar_height)
        {
            if (!Trajectory.ResultAvailable)
                return false;

            double vessel_up_ratio = -State.Vessel.Direction * State.Vessel.Velocity.Norm();
            if ((vessel_up_ratio < 0.9d && State.Vessel.VelocityMag > 50d) ||
                State.Vessel.Direction * State.Vessel.BodyUp < 0.5d)
            {
                Command.SetRcsRight(0d);
                Command.SetRcsUp(0d);
                return false;
            }

            double rcs_thrust_limit = 0d;
            foreach (var engine in ActiveVessel.Parts.Engines)
            {
                if (engine.Active)
                {
                    rcs_thrust_limit += Convert.ToDouble(engine.Thrust) *
                        Math.Sin(Convert.ToDouble(engine.GimbalRange) / 180d * Math.PI);
                }
            }
            double max_rcs_thrust = ActiveVessel.Parts.RCS[0].MaxThrust * 12d;
            rcs_thrust_limit = max_rcs_thrust;
            //double max_rcs_thrust = ActiveVessel.Parts.RCS[0].MaxThrust * 6d;
            //rcs_thrust_limit = Math.Min(rcs_thrust_limit * 0.8d, max_rcs_thrust);

            Vector3d v_err = tar_pos - Trajectory.ImpactPositionWithAction;

            Vector3d tar_acc = v_err / Math.Clamp(State.Vessel.Altitude - tar_height, 1000d, 10000d) * 500d;
            Command.SetRcsRight(Math.Clamp(tar_acc * State.Vessel.Right * State.Vessel.Mass, -rcs_thrust_limit, rcs_thrust_limit) / max_rcs_thrust * 0.9d);
            Command.SetRcsUp(Math.Clamp(tar_acc * State.Vessel.Up * State.Vessel.Mass, -rcs_thrust_limit, rcs_thrust_limit) / max_rcs_thrust * 0.9d);

            return false;
        }

        private bool Landing1RcsLast(Vector3d tar_pos, double tar_height)
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

            double rcs_thrust_limit = 0d;
            foreach (var engine in ActiveVessel.Parts.Engines)
            {
                if (engine.Active)
                {
                    rcs_thrust_limit += Convert.ToDouble(engine.Thrust) *
                        Math.Sin(Convert.ToDouble(engine.GimbalRange) / 180d * Math.PI);
                }
            }
            double max_rcs_thrust = ActiveVessel.Parts.RCS[0].MaxThrust * 12d;
            rcs_thrust_limit = max_rcs_thrust;
            //double max_rcs_thrust = ActiveVessel.Parts.RCS[0].MaxThrust * 6d;
            //rcs_thrust_limit = Math.Min(rcs_thrust_limit * 0.8d, max_rcs_thrust);
            double max_rcs_acc = rcs_thrust_limit / State.Vessel.Mass;

            Vector3d err_v = tar_pos - Trajectory.ImpactPositionWithAction;
            Vector3d err_h = err_v - err_v * State.Vessel.BodyUp * State.Vessel.BodyUp;
            double err = err_h.Length();

            Vector3d tar_vel = err_v / Math.Clamp(State.Vessel.Altitude - tar_height, 1000d, 10000d) * 50d;
            Vector3d tar_vel_h = tar_vel - tar_vel * State.Vessel.Forward * State.Vessel.Forward;
            Vector3d vel_h = State.Vessel.Velocity - State.Vessel.Velocity * State.Vessel.Forward * State.Vessel.Forward;

            Vector3d tar_acc = Math.Min(Math.Sqrt(2d * max_rcs_acc * err), err) * err_h.Norm();
            tar_acc += (tar_vel_h - vel_h) * 2d;
            Command.SetRcsRight(Math.Clamp(tar_acc * State.Vessel.Right * State.Vessel.Mass, -rcs_thrust_limit, rcs_thrust_limit) / max_rcs_thrust * 0.9d);
            Command.SetRcsUp(Math.Clamp(tar_acc * State.Vessel.Up * State.Vessel.Mass, -rcs_thrust_limit, rcs_thrust_limit) / max_rcs_thrust * 0.9d);

            return false;
        }

        public bool Landing1(Vector3d tar_pos, double tar_height)
        {
            if (rcs_altitude &&
                (Trajectory.NextBurnTime < 0.5d /*|| State.Vessel.Altitude < 80000d*/))
            {
                rcs_altitude = false;
                foreach (var rcs in ActiveVessel.Parts.RCS)
                {
                    rcs.RollEnabled = false;
                    rcs.PitchEnabled = false;
                    rcs.YawEnabled = false;
                }
            }

            Conn.Drawing().Clear();
            if (State.Vessel.Altitude - tar_height < 10000d && -State.Vessel.VelocityUp < 10d)
            {
                Landing1DirectionLast();
                Landing1RcsLast(tar_pos, tar_height);
            }
            //else if (State.Vessel.Altitude - tar_height < 10000d && -State.Vessel.VelocityUp < 500d)
            //{
            //    Landing1Direction(tar_pos);
            //    Landing1Rcs(tar_pos, tar_height);
            //}
            else
            {
                //Command.SetTargetDirection(-State.Vessel.Velocity);
                Landing1Direction(tar_pos);
                Landing1Rcs(tar_pos, tar_height);
            }
            Landing1Thrust(tar_height);

            return true;
        }
    }
}
