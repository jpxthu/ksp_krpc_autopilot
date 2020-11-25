using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;

namespace KrpcAutoPilot.Data
{
    public class CommonStreams
    {
        public CommonStreams(Connection conn, Service space_center)
        {
            UT = conn.AddStream(() => space_center.UT);
        }

        public Stream<double> UT { get; }
    }

    public class CommonData
    {
        public CommonData(Connection conn, Service space_center)
        {
            Streams = new CommonStreams(conn, space_center);
            UT = 0.0;
            Available = false;
        }

        public void Update()
        {
            UT = Streams.UT.Get();
            Available = true;
        }

        private CommonStreams Streams { get; }
        /// <summary>
        /// The current universal time in seconds.
        /// </summary>
        public double UT { get; private set; }
        public bool Available { get; private set; }
    }
}
