namespace AuthenticTeleoperation.Robots
{
    public sealed class RobotCmdVel : RosProxyUdpConnection
    {
        public RobotCmdVel( ) : base( 0x07629960, Robovie.CTYPE_ROBOT_CMDVEL )
        {
        }


        protected override void OnDataReceived( byte[] packet, int offset, int length )
        {
            // ignore everything
        }

    }
}
