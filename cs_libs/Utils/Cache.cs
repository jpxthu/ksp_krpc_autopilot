using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using System;
using System.Threading;

namespace KrpcAutoPilot.Utils
{
    public class Cache
    {
        private LinearTable atm_;
        private LinearTable available_thrust_;
        private LinearTable density_;
        private BiLinearTable sim_drift_;
        private BiLinearTable sim_lift_;
        private Mutex density_mut_ = new Mutex();

        private readonly CelestialBody body_;
        private readonly Connection conn_;
        private readonly Service sc_;
        private readonly Vessel vessel_;

        public Cache(
            Connection conn,
            Service sc,
            CelestialBody body,
            Vessel vessel,
            double altitude_step,
            double velocity_step, double velocity_max,
            double aoa)
        {
            conn_ = conn;
            sc_ = sc;
            body_ = body;
            vessel_ = vessel;
            Reset(altitude_step, velocity_step, velocity_max, aoa);
        }

        public void ResetAtm(double altitude_step, bool cache_all = true)
        {
            double altitude_max;
            if (body_.HasAtmosphere)
            {
                altitude_max = body_.AtmosphereDepth;
            }
            else
            {
                altitude_max = 1d;
                altitude_step = 1d;
            }

            uint altitude_n = (uint)Math.Ceiling(altitude_max / altitude_step);
            atm_.ResetCache(0d, altitude_step, altitude_n, cache_all);
        }

        public void ResetAvailableShrust(double altitude_step, bool cache_all = true)
        {
            double altitude_max;
            if (body_.HasAtmosphere)
            {
                altitude_max = body_.AtmosphereDepth;
            }
            else
            {
                altitude_max = 1d;
                altitude_step = 1d;
            }

            uint altitude_n = (uint)Math.Ceiling(altitude_max / altitude_step);
            available_thrust_.ResetCache(0d, altitude_step, altitude_n, cache_all);
        }

        public void Reset(
            double altitude_step,
            double velocity_step, double velocity_max,
            double aoa,
            bool init_simple = true)
        {
            double aoa_cos = Math.Cos(aoa);
            double aoa_sin = Math.Sin(aoa);

            double altitude_max;
            if (body_.HasAtmosphere)
            {
                altitude_max = body_.AtmosphereDepth;
            }
            else
            {
                altitude_max = 1d;
                altitude_step = 1d;
            }

            uint altitude_n = (uint)Math.Ceiling(altitude_max / altitude_step);
            uint velocity_n = (uint)Math.Ceiling(velocity_max / velocity_step);

            var flight = vessel_.Flight(body_.ReferenceFrame);
            double radius = body_.EquatorialRadius;

            var stream_pos = conn_.AddStream(() => vessel_.Position(body_.ReferenceFrame));
            var stream_backward = conn_.AddStream(() => sc_.TransformDirection(
                new Tuple<double, double, double>(0, -1, 0),
                vessel_.ReferenceFrame, body_.ReferenceFrame));
            var stream_right = conn_.AddStream(() => sc_.TransformDirection(
                new Tuple<double, double, double>(1, 0, 0),
                vessel_.ReferenceFrame, body_.ReferenceFrame));

            atm_ = new LinearTable(
                0d, altitude_step, altitude_n,
                (double altitude) =>
                {
                    double atm = body_.PressureAt(altitude) / Constants.Common.STANDARD_ATMOSPHERIC_PRESSURE;
                    return atm;
                },
                init_simple);

            available_thrust_ = new LinearTable(
                0d, altitude_step, altitude_n,
                (double altitude) =>
                {
                    double atm = atm_[altitude];
                    double thr = vessel_.AvailableThrustAt(atm);
                    return thr;
                },
                init_simple);

            density_ = new LinearTable(
                0d, altitude_step, altitude_n,
                (double altitude) =>
                {
                    double density = body_.DensityAt(altitude);
                    return density;
                },
                init_simple);

            sim_drift_ = new BiLinearTable(
                0d, altitude_step, altitude_n,
                0d, velocity_step, velocity_n,
                (double altitude, double velocity) =>
                {
                    Vector3d pos_now = new Vector3d(stream_pos.Get());
                    Vector3d backward = new Vector3d(stream_backward.Get());
                    Vector3d pos = pos_now.Norm() * (radius + altitude);
                    Vector3d vel_backward = velocity * backward;
                    var sim = flight.SimulateAerodynamicForceAt(body_, pos.ToTuple(), vel_backward.ToTuple());
                    return new Vector3d(sim).Length();
                });

            sim_lift_ = new BiLinearTable(
                0d, altitude_step, altitude_n,
                0d, velocity_step, velocity_n,
                (double altitude, double velocity) =>
                {
                    Vector3d pos_now = new Vector3d(stream_pos.Get());
                    Vector3d backward = new Vector3d(stream_backward.Get());
                    Vector3d right = new Vector3d(stream_right.Get());
                    Vector3d pos = pos_now.Norm() * (radius + altitude);
                    Vector3d vel_backward = velocity * backward;
                    Vector3d vel_tilt = velocity * (backward * aoa_cos + right * aoa_sin);
                    var sim = flight.SimulateAerodynamicForceAt(body_, pos.ToTuple(), vel_tilt.ToTuple());
                    return -(new Vector3d(sim) * right);
                });
        }

        public double AtmAt(double altitude)
        {
            return atm_[altitude];
        }
        public double AvailableThrustAt(double altitude)
        {
            return available_thrust_[altitude];
        }
        /// <summary>
        /// Gets the air density, in kg/m^3, for the specified altitude above sea level,
        /// in meters.
        /// </summary>
        /// <param name="altitude"></param>
        /// <returns></returns>
        public double DensityAt(double altitude)
        {
            density_mut_.WaitOne();
            double density = density_[altitude];
            density_mut_.ReleaseMutex();
            return density;
        }
        public double Drift(double altitude, double velocity)
        {
            return sim_drift_[altitude, velocity];
        }
        public double Lift(double altitude, double velocity)
        {
            return sim_lift_[altitude, velocity];
        }
    }
}
