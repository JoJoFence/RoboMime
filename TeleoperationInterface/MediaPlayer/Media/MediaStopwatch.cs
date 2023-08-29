using System.Diagnostics;

namespace MediaPlayer.Media
{
    public sealed class MediaStopwatch
    {
        private Stopwatch _stopwatch = Stopwatch.StartNew( );
        private long _elapsedMilliseconds = 0;


        public long ElapsedTimestamp
        {
            get
            {
                if( _stopwatch.IsRunning )
                {
                    _elapsedMilliseconds += _stopwatch.ElapsedMilliseconds;
                    _stopwatch.Restart( );
                }
                return _elapsedMilliseconds * 10000L;
            }
            set
            {
                _elapsedMilliseconds = value / 10000L;
                _stopwatch.Restart( );
            }
        }


        public void Pause( )
        {
            _stopwatch.Stop( );
        }


        public void Resume( )
        {
            _stopwatch.Start( );
        }


        public void Reset( )
        {
            _elapsedMilliseconds = 0;
            _stopwatch.Restart( );
        }

    }
}
