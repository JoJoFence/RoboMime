using System.Diagnostics;

namespace MediaPlayer.Media
{
    public static class MediaTimestamp
    {
        private static Stopwatch _stopwatch = Stopwatch.StartNew( );


        public static long Get( )
        {
            return _stopwatch.ElapsedMilliseconds * 10000L;
        }


        public static void Reset( )
        {
            _stopwatch.Restart( );
        }

    }
}
