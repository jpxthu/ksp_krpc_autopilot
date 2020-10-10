using KrpcLibs.Math;

namespace KrpcLibs.Constants
{
    public class Position
    {
        /// <summary>
        /// 坎巴拉太空中心发射台
        /// </summary>
        public static readonly LatLng KERBAL_CENTER_LAUNCH_PAD_GEO = new LatLng(-0.0972079934541428, -74.5576762507677);

        /// <summary>
        /// 坎巴拉太空中心发射台北边几十米的地方
        /// </summary>
        public static readonly LatLng KERBAL_CENTER_LAUNCH_PAD_NORTH_GEO = new LatLng(-0.0853591972406485, -74.5641045902701);

        /// <summary>
        /// 坎巴拉太空中心发射台南边几十米的地方
        /// </summary>
        public static readonly LatLng KERBAL_CENTER_LAUNCH_PAD_SOUTH_GEO = new LatLng(-0.108690763956662, -74.5642229639099);

        /// <summary>
        /// 坎巴拉太空中心发射后回收主火箭的船，用 Hyper 放在 (0, -60) 的地方
        /// </summary>
        public static readonly LatLng KERBAL_SEA_LANDING_OFCOUSE_I_LOVE_U = new LatLng(0.00100000159454346, -60.0000009132963);
    }
}
