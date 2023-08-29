using AuthenticTeleoperation.Messages.Sensor;

namespace AuthenticTeleoperation.Robots
{
    public sealed class VelodynePoints : RosProxyTcpConnection
    {
        public VelodynePoints( ) : base( "VelodynePoints", Robovie.CTYPE_VELODYNE_POINTS )
        {
        }


        protected override void OnDataReceived( byte[] packet, int offset, int length )
        {
            PointCloud2 data = new PointCloud2( );
            data.Deserialize( packet, offset, length );
            // Console.WriteLine(data.ToString() );
            NotifyDataAvailable( data );
        }

    }
}
