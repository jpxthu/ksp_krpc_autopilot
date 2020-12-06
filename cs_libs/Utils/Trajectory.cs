using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using System;
using System.IO;
using System.Threading;

namespace KrpcAutoPilot.Utils
{
    public class Trajectory
    {
        public struct SimulationData
        {
            public Vector3d pos;
            public Vector3d vel;
            public double altitude;
            public double pressure;
            public double g;
            public double mass;
            public double tar_altitude;
            public double fuel_rate_at_full_throttle;
            public double available_thrust;
            public double t;
        }

        public class SimulationResult
        {
            public SimulationResult()
            {
                Thrust = Vector3d.Zero;
                ThrustMag = 0d;
                Throttle = 0d;
            }
            public SimulationResult(Vector3d thrust, double thrust_mag, double throttle)
            {
                Thrust = thrust;
                ThrustMag = thrust_mag;
                Throttle = throttle;
            }

            public Vector3d Thrust { get; }
            public double ThrustMag { get; }
            public double Throttle { get; }
        }

        private Vector3d CalculateImpactPositionWithAction()
        {
            var time_start = DateTime.Now;

            double mass = State.Vessel.Mass;
            double dry_mass = State.Vessel.DryMass;
            Vector3d vel = State.Vessel.Velocity;
            Vector3d pos = State.Vessel.Position;
            double altitude = pos.Length() - State.Body.Radius;
            if (altitude <= TarAltitude)
                return pos;

            double atm_depth = State.Body.AtmosphereDepth;
            bool enter_atm = false;
            double next_burn_time = -1d;

            double t = 0d;
            double sim_lift_t = 0d;
            double sim_lift_sum = 0d;
            double sim_lift_min = double.MaxValue;
            double sim_lift_max = double.MinValue;
            double sim_thrust_sum = 0d;

            double drag_sum = 0d;
            double vel2_sum = 0d;
            while (t < 1000d)
            {
                double atm = cache_.AtmAt(altitude);
                double available_thrust = cache_.AvailableThrustAt(altitude);
                Vector3d g = -pos * (State.Body.GravitationalParameter / Math.Pow(State.Body.Radius + altitude, 3));
                double vel_mag = vel.Length();

                var sim_data = new SimulationData
                {
                    pos = pos,
                    vel = vel,
                    altitude = altitude,
                    pressure = atm,
                    g = g.Length(),
                    mass = mass,
                    tar_altitude = TarAltitude,
                    fuel_rate_at_full_throttle = State.Vessel.MaxFuelRate,
                    available_thrust = available_thrust,
                    t = t
                };
                SimulationResult sim_res = mass > dry_mass ? Planner(sim_data) : new SimulationResult();
                var thrust_acc = sim_res.Thrust / mass;

                double left_height = altitude - TarAltitude;
                double dt = Math.Min(1d, Math.Max(0.1d, left_height / Math.Max(1d, vel_mag) / 2d)) * t_ratio_;
                if (sim_res.Throttle > 1e-3d && next_burn_time < 0d)
                {
                    next_burn_time = t;
                    NextBurnTime = t;
                }

                // Lift estimation
                if (t < LiftEstimationTime)
                {
                    double sim_lift_mag = cache_.Lift(altitude, vel_mag);
                    sim_lift_t = t + dt;
                    sim_lift_sum += sim_lift_mag * dt;
                    sim_lift_min = Math.Min(sim_lift_min, sim_lift_mag);
                    sim_lift_max = Math.Max(sim_lift_max, sim_lift_mag);
                    sim_thrust_sum += sim_res.ThrustMag * dt;
                }

                double sim_drag_mag = cache_.Drift(altitude, vel_mag);
                drag_sum += sim_drag_mag * dt;
                vel2_sum += vel_mag * vel_mag * cache_.DensityAt(altitude) * dt;

                Vector3d sim_drag_acc = -sim_drag_mag / mass * vel.Norm();
                Vector3d acc = thrust_acc + g + sim_drag_acc;

                //if (ut_before_calc_ - last_print_ut_ >= 1d)
                //    sw_.WriteLine("{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\t{7}\t{8}",
                //        ut_before_calc_,
                //        vel,
                //        pos,
                //        sim_res.Throttle,
                //        mass,
                //        sim_data.available_thrust,
                //        g.Length(),
                //        sim_drag_mag,
                //        t);

                t += dt;
                vel += acc * dt;
                Vector3d pos_last = pos;
                double altitude_last = altitude;
                pos += vel * dt;
                mass -= State.Vessel.MaxFuelRate * sim_res.Throttle * dt;

                altitude = pos.Length() - State.Body.Radius;
                if (State.Body.HasAtmosphere)
                {
                    if (!enter_atm && altitude <= atm_depth && altitude_last >= atm_depth)
                    {
                        enter_atm = true;
                        EnterAtmosphereDirection = -vel.Norm();
                    }
                }
                else
                {
                    if (!enter_atm && next_burn_time < 0.5d)
                    {
                        enter_atm = true;
                        EnterAtmosphereDirection = -vel.Norm();
                    }
                }
                if (altitude <= TarAltitude)
                {
                    double ratio = MathLib.InverseLerp(altitude_last, altitude, TarAltitude);
                    pos = MathLib.Lerp(pos_last, pos, ratio);
                    break;
                }
            }
            last_print_ut_ = (ulong)ut_before_calc_;

            ImpactTime = t;
            LiftEstimationForceAve = sim_lift_t > 1e-3d ? sim_lift_sum / sim_lift_t : 0d;
            LiftEstimationForceMin = sim_lift_min;
            LiftEstimationThrustAve = sim_thrust_sum / sim_lift_t;

            drag_ratio_ = drag_sum == 0d ? 0d : drag_sum / vel2_sum;

            var time_used = (DateTime.Now - time_start).TotalMilliseconds;
            t_ave_ = Math.Max(time_used, 1d);
            t_ratio_ *= (t_ave_ / 100d - 1d) * 0.2d + 1d;
            t_ratio_ = Math.Clamp(t_ratio_, 0.01d, 100d);

            return pos;
        }

        private Vector3d CalculateImpactPosition()
        {
            double mass = State.Vessel.Mass;
            Vector3d vel = State.Vessel.Velocity;
            Vector3d pos = State.Vessel.Position;
            double altitude = pos.Length() - State.Body.Radius;
            if (altitude <= TarAltitude)
                return pos;

            double t = 0d;
            while (t < 1000d)
            {
                Vector3d g = -pos * (State.Body.GravitationalParameter / Math.Pow(State.Body.Radius + altitude, 3));
                double sim_drag_mag = vel.LengthSquared() * cache_.DensityAt(altitude) * drag_ratio_;
                Vector3d sim_drag_acc = -sim_drag_mag / mass * vel.Norm();
                Vector3d acc = g + sim_drag_acc;

                double left_height = altitude - TarAltitude;
                double dt = Math.Min(1d, Math.Max(0.1d, left_height / Math.Max(1d, vel.Length()) / 2d)) * t_ratio_;
                t += dt;
                vel += acc * dt;
                Vector3d pos_last = pos;
                double altitude_last = altitude;
                pos += vel * dt;

                altitude = pos.Length() - State.Body.Radius;
                if (altitude <= TarAltitude)
                {
                    double ratio = MathLib.InverseLerp(altitude_last, altitude, TarAltitude);
                    pos = MathLib.Lerp(pos_last, pos, ratio);
                    break;
                }
            }

            return pos;
        }

        private void CalculateFunction()
        {
            while (true)
            {
                Thread.Sleep(1);
                if (!calculate_)
                    break;
                if (!Data.Available ||
                    !State.Available)
                {
                    ResultAvailable = false;
                    continue;
                }
                if (ut_before_calc_ == Data.UT)
                    continue;
                ut_before_calc_ = Data.UT;
                Vector3d pos_without_action = null;
                Thread thread = new Thread(() => { pos_without_action = CalculateImpactPosition(); });
                thread.Start();
                Vector3d pos_with_action = CalculateImpactPositionWithAction();
                thread.Join();

                res_update_mut_.WaitOne();
                ImpactPositionWithAction = pos_with_action;
                ImpactPositionWithActionCalcUt = ut_before_calc_;
                impact_pos_diff_ = pos_with_action - pos_without_action;
                res_update_mut_.ReleaseMutex();
                ResultAvailable = true;
            }
        }

        private void SpeedUpFunction()
        {
            double ut_calc = 0d;
            while (true)
            {
                Thread.Sleep(1);
                if (!calculate_)
                    break;
                if (!ResultAvailable ||
                    !Data.Available ||
                    !State.Available)
                    continue;
                if (ut_calc == Data.UT)
                    continue;
                ut_calc = Data.UT;
                Vector3d ImpactPositionWithoutAction = CalculateImpactPosition();
                res_update_mut_.WaitOne();
                if (ut_calc - ImpactPositionWithActionCalcUt > (double)CalculateGap / 1000d * 2d)
                    ImpactPositionWithAction = ImpactPositionWithoutAction + impact_pos_diff_;
                res_update_mut_.ReleaseMutex();
            }
        }

        public void CalculateStart(double? tar_altitude = null, int? calculate_gap_in_ms = null)
        {
            if (tar_altitude.HasValue)
                TarAltitude = tar_altitude.Value;
            if (calculate_gap_in_ms.HasValue)
                CalculateGap = calculate_gap_in_ms.Value;
            calculate_ = true;
            ResultAvailable = false;
            calculate_thread_.Start();
            speed_up_thread_.Start();
        }

        public void CalculateStop()
        {
            calculate_ = false;
            ResultAvailable = false;
        }

        public void ReCache(double altitude_step, double velocity_step, double velocity_max)
        {
            cache_.Reset(altitude_step, velocity_step, velocity_max, LiftEstimationAngle);
        }

        public Trajectory(
            Data.CommonData data, Data.VesselData state,
            Connection conn, Service sc, CelestialBody body, Vessel vessel, double tar_altitude, int calculate_gap_in_ms,
            Func<SimulationData, SimulationResult> planner)
        {
            Data = data;
            State = state;
            TarAltitude = tar_altitude;
            CalculateGap = calculate_gap_in_ms;
            ResultAvailable = false;
            calculate_thread_ = new Thread(CalculateFunction);
            speed_up_thread_ = new Thread(SpeedUpFunction);
            Planner = planner;
            NextBurnTime = double.MaxValue;
            LiftEstimationAngle = 10d / 180d * Math.PI;
            LiftEstimationTime = 5d;

            //sw_ = new StreamWriter("t.tsv");
            cache_ = new Cache(
                conn, sc, body, vessel,
                200d, 20d, 5000d, LiftEstimationAngle);
        }

        public Trajectory(
            Data.CommonData data, Data.VesselData state,
            Connection conn, Service sc, CelestialBody body, Vessel vessel, double tar_altitude, int calculate_gap_in_ms)
            : this(data, state, conn, sc, body, vessel, tar_altitude, calculate_gap_in_ms,
                  (data) => { return new SimulationResult(); })
        { }

        ~Trajectory()
        {
            calculate_ = false;

            //sw_.Close();
        }

        private readonly Thread calculate_thread_;
        private readonly Thread speed_up_thread_;
        private bool calculate_ = false;

        private double ut_before_calc_ = 0d;
        private ulong last_print_ut_ = 0;
        private readonly Cache cache_;
        //private readonly StreamWriter sw_;
        private double drag_ratio_ = 0d;
        private double t_ave_ = -1d;
        private double t_ratio_ = 10d;
        private readonly Mutex res_update_mut_ = new Mutex();
        private Vector3d impact_pos_diff_ = Vector3d.Zero;

        private Data.CommonData Data { get; }
        private Data.VesselData State { get; }
        public double TarAltitude { get; set; }
        public int CalculateGap { get; set; }

        public Vector3d ImpactPositionWithAction { get; private set; }
        public Vector3d ImpactPositionWithoutAction { get; private set; }
        public Vector3d EnterAtmosphereDirection { get; private set; }
        public double ImpactPositionWithActionCalcUt { get; private set; }
        public bool ResultAvailable { get; private set; }
        public Func<SimulationData, SimulationResult> Planner { get; set; }
        public double NextBurnTime { get; private set; }
        public double ImpactTime { get; private set; }
        public double LiftEstimationAngle { get; set; }
        public double LiftEstimationTime { get; set; }
        public double LiftEstimationForceAve { get; private set; }
        public double LiftEstimationForceMin { get; private set; }
        public double LiftEstimationThrustAve { get; private set; }
    }
}
