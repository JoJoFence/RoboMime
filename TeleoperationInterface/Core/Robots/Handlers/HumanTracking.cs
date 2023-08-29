using AuthenticTeleoperation.Messages.Layer2;

namespace AuthenticTeleoperation.Robots
{
    public sealed class HumanTrackedL2 : RosProxyUdpConnection
    {
        public HumanTrackedL2( ) : base( 0x6E202A6F, Robovie.CTYPE_HUMAN_TRACKED_L2 )
        {
        }


        protected override void OnDataReceived( byte[] packet, int offset, int length )
        {
            HTEntityList data = new HTEntityList( );
            data.Deserialize( packet, offset, length );
            NotifyDataAvailable( data );
        }

    }
}
