using KrpcLibs.Math;

namespace KrpcLibs.Utils
{
    public class Trajectory
    {
        public static Vector3d ImpactPosition(Data.VesselData state, double tar_height, double k)
        {
            double radius = state.Body.Radius;

            Vector3d pos = new Vector3d(state.Vessel.Position);
            Vector3d vel = new Vector3d(state.Vessel.Velocity);

            while (true) {
                double distance_to_core = pos.Length();
                double altitude = distance_to_core - radius;
                if (altitude <= tar_height)
                    break;
                Vector3d g = -pos *
                    (state.Body.GravitationalParameter / System.Math.Pow(radius + altitude, 3));
                double air_density = Common.AirDensityKerbin(altitude);
                Vector3d drag = vel * (vel.Length() * air_density * k / 2);
                Vector3d acc = g - drag / state.Vessel.Mass;

                double left = altitude - tar_height;
                double dt = System.Math.Min(1.0, System.Math.Max(0.01,
                    left / System.Math.Max(1.0, vel.Length()) / 2));
                vel += acc * dt;
                pos += vel * dt;
            }

            return pos;
        }
    }
}
