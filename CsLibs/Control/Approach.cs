using KRPC.Client;
using KRPC.Client.Services.Drawing;
using KRPC.Client.Services.SpaceCenter;
using KrpcAutoPilot.Utils;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;

namespace KrpcAutoPilot
{
    public partial class Control
    {
        private ApproachStage approach_stage_;
        private ApproachTargetStage approach_target_stage_;
        private string approach_dock_tag_, approach_control_node_tag_;
        private double approach_distance_max_;
        private Vessel approach_vessel_;
        private Stream<Tuple<double, double, double>> approach_tar_pos_stream_;
        private Stream<Tuple<double, double, double>> approach_tar_vel_stream_;
        private Stream<Tuple<double, double, double>> approach_tar_dir_stream_;

        public Status ApproachInit(
            string ship_name,
            string dock_tag = null,
            string control_node_tag = null,
            double distance = 50d)
        {
            approach_dock_tag_ = dock_tag;
            approach_control_node_tag_ = control_node_tag;
            approach_distance_max_ = distance;

            IEnumerable<Vessel> vessels = SpaceCenter.Vessels.Where(v => v.Name == ship_name);
            if (vessels.Count() == 0)
                return Status.FAIL;
            approach_vessel_ = vessels.First();
            SpaceCenter.TargetVessel = approach_vessel_;
            approach_tar_pos_stream_ = Conn.AddStream(() => approach_vessel_.Position(OrbitBodyFrame));
            approach_tar_vel_stream_ = Conn.AddStream(() => approach_vessel_.Velocity(OrbitBodyFrame));
            approach_tar_dir_stream_ = null;

            Vector3d tar_pos = new Vector3d(approach_tar_pos_stream_.Get());
            Vector3d tar_vel = new Vector3d(approach_tar_vel_stream_.Get());
            double tar_distance = (tar_pos - State.Vessel.Position).Length();
            double tar_rel_vel = (tar_vel - State.Vessel.Velocity).Length();
            if (tar_distance < 1000d && tar_rel_vel < Math.Max(10d, Math.Sqrt(tar_distance)))
                approach_stage_ = ApproachStage.DOCKING;
            else
                approach_stage_ = ApproachStage.ENGINE;
            if (string.IsNullOrEmpty(dock_tag))
                approach_target_stage_ = ApproachTargetStage.PORT;
            else
                approach_target_stage_ = ApproachTargetStage.SHIP;

            return Status.SUCCESS;
        }

        public void ApproachDeinit()
        {
            try
            {
                approach_tar_pos_stream_.Remove();
                approach_tar_vel_stream_.Remove();
                approach_tar_dir_stream_.Remove();
            }
            catch (Exception)
            { }
        }

        public Status ApproachFindDockingPort(double distance)
        {
            if (approach_target_stage_ != ApproachTargetStage.SHIP)
                return Status.FINISHED;

            if (distance > TARGET_HAS_DETAILS_DISTANCE_THRESHOLD)
                return Status.UNFINISHED;

            var ports = approach_vessel_.Parts.DockingPorts.Where(p => p.Part.Tag == approach_dock_tag_);
            if (ports.Count() == 0)
                return Status.UNFINISHED;
            DockingPort port = ports.First();

            approach_tar_pos_stream_.Remove();
            approach_tar_vel_stream_.Remove();
            SpaceCenter.TargetDockingPort = port;
            approach_tar_pos_stream_ = Conn.AddStream(() => port.Position(OrbitBodyFrame));
            approach_tar_vel_stream_ = Conn.AddStream(() => port.Part.Velocity(OrbitBodyFrame));
            approach_tar_dir_stream_ = Conn.AddStream(() => port.Direction(OrbitBodyFrame));
            approach_target_stage_ = ApproachTargetStage.PORT;
            return Status.SUCCESS;
        }

        private double ApproachEstimatedTurnTime(Vector3d tar_dir)
        {
            double max_acc = 0.5d * (
                Math.Abs(State.Vessel.AvailableTorque.Item1.X) / State.Vessel.MomentOfInertia.X +
                Math.Abs(State.Vessel.AvailableTorque.Item1.Z) / State.Vessel.MomentOfInertia.Z);
            return Math.Sqrt(Math.PI / max_acc) * (1d - State.Vessel.Direction * tar_dir) + 10d;
        }

        private Status ApproachGetDstInfo(
            out Vector3d dst_pos, out Vector3d dst_vel,
            out Vector3d dst_acc, out Vector3d dst_dir)
        {
            try
            {
                dst_pos = new Vector3d(approach_tar_pos_stream_.Get());
                dst_vel = new Vector3d(approach_tar_vel_stream_.Get());
                if (approach_tar_dir_stream_ is null)
                    dst_dir = dst_pos.Norm();
                else
                    dst_dir = new Vector3d(approach_tar_dir_stream_.Get());
                dst_acc = -CommonData.Body.GravitationalParameter / Math.Pow(dst_pos.Length(), 3) * dst_pos;
            }
            catch (Exception)
            {
                dst_pos = dst_vel = dst_acc = dst_dir = Vector3d.Zero;
                return Status.FAIL;
            }
            return Status.SUCCESS;
        }

        private double ApproachTarFacingDistance(double lateral_distance)
        {
            if (approach_stage_ == ApproachStage.DOCKING)
            {
                double tar = Math.Max(0, lateral_distance - 0.1d) * 10d;
                return Math.Min(approach_distance_max_, tar);
            }
            else
            {
                return approach_distance_max_;
            }
        }

        private void ApproachGetTarInfo(
            Vector3d dst_pos, Vector3d dst_dir,
            out Vector3d tar_vec, out Vector3d tar_vel_on_dir, out Vector3d tar_vec_lateral, out Vector3d tar_vec_dir,
            out double tar_distance_lateral, out double tar_distance_facing, out double tar_distance)
        {
            tar_vec = dst_pos - State.Vessel.Position;
            tar_vel_on_dir = tar_vec * dst_dir * dst_dir;
            tar_vec_lateral = tar_vec - tar_vel_on_dir;
            tar_distance_lateral = tar_vec_lateral.Length();
            double distance = ApproachTarFacingDistance(tar_distance_lateral);
            tar_vec += distance * dst_dir;
            tar_vec_dir = tar_vec.Norm();
            tar_distance = tar_vec.Length();
            tar_distance_facing = -tar_vec * dst_dir;
        }

        public void ApproachCalculateThrust(
            Vector3d dst_vel, Vector3d dst_acc, Vector3d tar_vec_dir, double tar_distance, Vector3d rel_vel_vec,
            out Vector3d tar_acc_vec, out Vector3d tar_dir, out double route_vel, out double tar_vel_err_mag,
            out double tar_thrust_acc, out Vector3d tar_thrust_acc_vec, out double throttle)
        {
            double turn_time = ApproachEstimatedTurnTime(-tar_vec_dir);
            double rel_vel = rel_vel_vec * tar_vec_dir;
            double thrust_acc_max = State.Vessel.AvailableThrust / State.Vessel.Mass;
            double thrust_acc = thrust_acc_max * 0.9d;
            LinearPlanner.OneWay(
                tar_distance - rel_vel * turn_time,
                1d, thrust_acc, 0d,
                out double route_vel_tmp, out double route_acc_tmp);
            route_vel = -route_vel_tmp;
            double route_acc = -route_acc_tmp;

            Vector3d tar_vel_vec = dst_vel + route_vel * tar_vec_dir;
            Vector3d tar_vel_err = tar_vel_vec - State.Vessel.Velocity;
            tar_vel_err_mag = tar_vel_err.Length();
            Vector3d tar_vel_err_on_dir = tar_vel_err * tar_vec_dir * tar_vec_dir;
            Vector3d tar_vel_err_lateral = tar_vel_err - tar_vel_err_on_dir;
            double lateral_pi = approach_stage_ == ApproachStage.MIX ? 2.0d :
                Math.Max(0.1d, tar_vel_err_lateral.Length() / tar_vel_err_on_dir.Length() / 10d);

            tar_acc_vec = dst_acc - State.Vessel.Gravity + route_acc * tar_vec_dir
                + tar_vel_err_lateral * lateral_pi + tar_vel_err_on_dir * 2d;
            tar_dir = tar_acc_vec.Norm();

            tar_thrust_acc = Math.Max(0d, tar_acc_vec * State.Vessel.Direction);
            tar_thrust_acc_vec = tar_thrust_acc * State.Vessel.Direction;
            double min_thrust = approach_stage_ == ApproachStage.MIX ? 0.0d : 0.05d;
            throttle = Math.Max(tar_thrust_acc / thrust_acc_max, min_thrust);
        }

        public void ApproachCalculatePcs(
            Vector3d dst_dir, Vector3d tar_vec_lateral,
            double tar_distance_lateral, double tar_distance_facing,
            Vector3d rel_vel_vec,
            out Vector3d tar_acc_vec)
        {
            RcsForceOnSpecificDirection(dst_dir, out double f_facing, out double fr_facing);
            double a_facing = f_facing / State.Vessel.Mass, ar_facing = fr_facing / State.Vessel.Mass;
            double k = tar_distance_facing < 0d ?
                Math.Clamp(-tar_distance_facing, 0.01d, 1d) :
                Math.Clamp((tar_distance_facing - 1d) / 100d, 0.01d, 0.1d);
            LinearPlanner.Hover(
                tar_distance_facing, k,
                a_facing * 0.8d, ar_facing * 0.8d,
                out double route_vel_facing, out double route_acc_facing);
            Vector3d rel_vel_facing_vec = rel_vel_vec * dst_dir * dst_dir;
            Vector3d tar_rel_vel_facing_vec = route_vel_facing * dst_dir;
            Vector3d tar_rel_acc_facing_vec = route_acc_facing * dst_dir
                + (tar_rel_vel_facing_vec - rel_vel_facing_vec) * 2d;

            Vector3d lat_dir = tar_vec_lateral.Norm();
            RcsForceOnSpecificDirection(lat_dir, out double f_lat, out double fr_lat); ;
            double a_lat = f_lat / State.Vessel.Mass, ar_lat = fr_lat / State.Vessel.Mass;
            LinearPlanner.Hover(
                tar_distance_lateral, 1d,
                a_lat * 0.8d, ar_lat * 0.8d,
                out double route_vel_lat, out double route_acc_lat);
            route_vel_lat = -route_vel_lat;
            route_acc_lat = -route_acc_lat;
            Vector3d rel_vel_lat_vec = rel_vel_vec - rel_vel_facing_vec;
            Vector3d tar_rel_vel_lat_vec = route_vel_lat * lat_dir;
            Vector3d tar_rel_acc_lat_vec = route_acc_lat * lat_dir +
                (tar_rel_vel_lat_vec - rel_vel_lat_vec) * 5d;

            tar_acc_vec = tar_rel_acc_facing_vec + tar_rel_acc_lat_vec;
        }

        public Status Approach(out double distance)
        {
            if (ApproachGetDstInfo(
                    out Vector3d dst_pos, out Vector3d dst_vel,
                    out Vector3d dst_acc, out Vector3d dst_dir) == Status.FAIL)
            {
                distance = -1d;
                return Status.FAIL;
            }

            ApproachGetTarInfo(
                dst_pos, dst_dir,
                out Vector3d tar_vec, out Vector3d tar_vel_on_dir, out Vector3d tar_vec_lateral, out Vector3d tar_vec_dir,
                out double tar_distance_lateral, out double tar_distance_facing, out double tar_distance);
            distance = tar_distance;

            if (ApproachFindDockingPort(tar_distance) == Status.FAIL)
                return Status.FAIL;

            Vector3d rel_vel_vec = State.Vessel.Velocity - dst_vel;
            double rel_vel = rel_vel_vec.Length();

            switch (approach_stage_)
            {
                case ApproachStage.ENGINE:
                    {
                        ApproachCalculateThrust(
                            dst_vel, dst_acc, tar_vec_dir, tar_distance, rel_vel_vec,
                            out Vector3d tar_acc_vec, out Vector3d tar_dir, out double route_vel, out double tar_vel_err_mag,
                            out double tar_thrust_acc, out Vector3d tar_thrust_acc_vec, out double throttle);
                        if (route_vel < 50d && tar_vel_err_mag < 50d)
                        {
                            Command.SetTargetDirection(-rel_vel_vec.Norm());
                            approach_stage_ = ApproachStage.MIX;
                        }
                        else
                        {
                            Command.SetTargetDirection(tar_dir);
                        }
                        Command.SetThrottle(throttle);
                    }
                    break;
                case ApproachStage.MIX:
                    {
                        ApproachCalculateThrust(
                            dst_vel, dst_acc, tar_vec_dir, tar_distance, rel_vel_vec,
                            out Vector3d tar_acc_vec, out Vector3d tar_dir, out double route_vel, out double tar_vel_err_mag,
                            out double tar_thrust_acc, out Vector3d tar_thrust_acc_vec, out double throttle);
                        if (tar_acc_vec.Length() * State.Vessel.Mass <= 0.2d * Math.Min(Math.Min(State.Vessel.AvailableRCSForce.Item1.X, State.Vessel.AvailableRCSForce.Item1.Y), State.Vessel.AvailableRCSForce.Item1.Z) &&
                            rel_vel < 1d)
                        {
                            throttle = 0d;
                            approach_stage_ = string.IsNullOrEmpty(approach_dock_tag_) ?
                                ApproachStage.PCS : ApproachStage.DOCKING;
                            if (!string.IsNullOrEmpty(approach_control_node_tag_))
                            {
                                try
                                {
                                    ActiveVessel.Parts.Controlling =
                                        ActiveVessel.Parts.WithTag(approach_control_node_tag_).First();
                                    Thread.Sleep(500);
                                    State.Reset();
                                }
                                catch (Exception)
                                { }
                            }
                        }
                        Command.SetThrottle(throttle);
                        RcsSetByForce((tar_acc_vec - tar_thrust_acc_vec) * State.Vessel.Mass);
                    }
                    break;
                case ApproachStage.PCS:
                    {
                        ApproachCalculatePcs(
                            dst_dir, tar_vec_lateral,
                            tar_distance_lateral, tar_distance_facing,
                            rel_vel_vec, out Vector3d tar_acc_vec);
                        Command.SetThrottle(0d);
                        RcsSetByForce(tar_acc_vec * State.Vessel.Mass);
                    }
                    break;
                case ApproachStage.DOCKING:
                    {
                        ApproachCalculatePcs(
                            dst_dir, tar_vec_lateral,
                            tar_distance_lateral, tar_distance_facing,
                            rel_vel_vec, out Vector3d tar_acc_vec);
                        Command.SetTargetDirection(-dst_dir);
                        Command.SetThrottle(0d);
                        RcsSetByForce(tar_acc_vec * State.Vessel.Mass);
                    }
                    break;
            }

            //Conn.Drawing().Clear();
            //Conn.Drawing().AddDirection(
            //    SpaceCenter.TransformDirection(tar_vec.ToTuple(), OrbitBodyFrame, ActiveVessel.ReferenceFrame),
            //    ActiveVessel.ReferenceFrame);
            //Conn.Drawing().AddDirection(
            //    SpaceCenter.TransformDirection(Command.DirectionVector.ToTuple(), OrbitBodyFrame, ActiveVessel.ReferenceFrame),
            //    ActiveVessel.ReferenceFrame);
            //Conn.Drawing().AddDirection(
            //    SpaceCenter.TransformDirection(rel_vel_vec.Norm().ToTuple(), OrbitBodyFrame, ActiveVessel.ReferenceFrame),
            //    ActiveVessel.ReferenceFrame,
            //    50f);
            //Console.WriteLine("{0}\t{1:0.00}\t{2:0.0}",
            //    approach_stage_,
            //    tar_distance_lateral,
            //    tar_distance);

            return Status.UNFINISHED;
        }
    }
}
