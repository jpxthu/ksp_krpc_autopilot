using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;

namespace KRPCLibs.Data
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
        }

        public void Update()
        {
            UT = Streams.UT.Get();
        }

        private CommonStreams Streams { get; }
        public double UT { get; private set; }
    }
}
