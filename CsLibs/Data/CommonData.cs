using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using System;

namespace KrpcAutoPilot
{
    public class CommonData
    {
        public class Streams
        {
            public Streams(Connection conn, Service sc, CelestialBody body)
            {
                Init(conn, sc, body);
            }

            public void Init(Connection conn, Service sc, CelestialBody body)
            {
                UT = conn.AddStream(() => sc.UT);
                // Body
                AtmosphereDepth = conn.AddStream(() => body.AtmosphereDepth);
                GravitationalParameter = conn.AddStream(() => body.GravitationalParameter);
                HasAtmosphere = conn.AddStream(() => body.HasAtmosphere);
                Radius = conn.AddStream(() => body.EquatorialRadius);
            }

            public void Deinit()
            {
                UT.Remove();
                // Body
                AtmosphereDepth.Remove();
                GravitationalParameter.Remove();
                HasAtmosphere.Remove();
                Radius.Remove();
            }

            public Stream<double> UT { get; private set; }
            // Body
            public Stream<float> AtmosphereDepth { get; private set; }
            public Stream<float> GravitationalParameter { get; private set; }
            public Stream<bool> HasAtmosphere { get; private set; }
            public Stream<float> Radius { get; private set; }
        }

        public bool Update()
        {
            try
            {
                UT = streams_.UT.Get();

                // Body
                Body.AtmosphereDepth = streams_.AtmosphereDepth.Get();
                Body.GravitationalParameter = streams_.GravitationalParameter.Get();
                Body.HasAtmosphere = streams_.HasAtmosphere.Get();
                Body.Radius = streams_.Radius.Get();
            }
            catch (Exception e)
            {
                Console.WriteLine(
                    "Error when update states. Celestial Body or Space Center missed." +
                    "Error message: " + e.Message);
                Available = false;
                return false;
            }

            Available = true;
            return true;
        }

        public CommonData(Connection conn, Service sc, CelestialBody body)
        {
            streams_ = new Streams(conn, sc, body);
            UT = 0d;
            Body = new Data.Body();
            Available = false;
        }

        public void Dispose()
        {
            streams_.Deinit();
        }

        private readonly Streams streams_;
        public double UT { get; private set; }
        public Data.Body Body { get; private set; }
        public bool Available { get; private set; }
    }
}
