namespace KrpcLibs.Constants
{
    class CelestialBody
    {
        public enum Index
        {
            SUN, KERBIN, MUM, MINMUS, MOHO, EVE, DUNA, IKE, JOOL,
            LAYTHE, VALL, BOP, TYLO, GILLY, POL, DRES, EELOO
        };

        public static string Name(Index index)
        {
            return index switch
            {
                Index.SUN => "Sun",
                Index.KERBIN => "Kerbin",
                Index.MUM => "Mun",
                Index.MINMUS => "Minmus",
                Index.MOHO => "Moho",
                Index.EVE => "Eve",
                Index.DUNA => "Duna",
                Index.IKE => "Ike",
                Index.JOOL => "Jool",
                Index.LAYTHE => "Laythe",
                Index.VALL => "Vall",
                Index.BOP => "Bop",
                Index.TYLO => "Tylo",
                Index.GILLY => "Gilly",
                Index.POL => "Pol",
                Index.DRES => "Dres",
                Index.EELOO => "Eeloo",
                _ => string.Empty,
            };
        }
    }
}
