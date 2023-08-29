using AuthenticTeleoperation.Messages.Geometry;

namespace AuthenticTeleoperation.Robots
{
    public sealed class RobotPose6D : RosProxyUdpConnection
    {
        public RobotPose6D( ) : base( 0x762E2A6F, Robovie.CTYPE_ROBOT_POSE_6D )
        {
        }


        protected override void OnDataReceived( byte[] packet, int offset, int length )
        {
            PoseWithCovarianceStamped data = new PoseWithCovarianceStamped( );
            data.Deserialize( packet, offset, length );
            NotifyDataAvailable( data );
        }

    }
}
