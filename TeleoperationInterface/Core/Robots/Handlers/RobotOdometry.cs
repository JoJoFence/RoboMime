using AuthenticTeleoperation.Messages.Navigation;

namespace AuthenticTeleoperation.Robots
{
    public sealed class RobotOdometry : RosProxyUdpConnection
    {
        public RobotOdometry( ) : base( 0x762C4BC1, Robovie.CTYPE_ROBOT_ODOMETRY )
        {
        }


        protected override void OnDataReceived( byte[] packet, int offset, int length )
        {
            Odometry data = new Odometry( );
            data.Deserialize( packet, offset, length );
            NotifyDataAvailable( data );
        }

    }
}
