namespace AuthenticTeleoperation.Robots
{
    public sealed class AudioPlayback : RosProxyUdpConnection
    {
        public AudioPlayback( ) : base( 0x66AFDD87, Robovie.CTYPE_AUDIO_PLAYBACK )
        {
        }


        protected override void OnDataReceived( byte[] packet, int offset, int length )
        {
            // ignore everything
        }

    }
}
