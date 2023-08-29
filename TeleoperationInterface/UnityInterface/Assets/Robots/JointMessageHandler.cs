using SS.HRIKU.Network;
using SS.HRIKU.Network.Tcp;
using System;
using System.Text;

namespace AuthenticTeleoperation.Robots
{
    internal sealed class JointMessageHandler : MessageHandler
    {
        private const uint ID = 0xFFFFFFFF;
        private RobovieUpdater _robovie = null;


        public JointMessageHandler( RobovieUpdater robovie ) : base( "MotionVisualizerJointUpdate" )
        {
            _robovie = robovie;
        }


        protected override void OnConnected( NetConnection connection )
        {
            Log.Print( "New connection to {0}", connection.Name );
        }


        protected override void OnDataReceived( NetConnection connection, PacketEventArgs e )
        {
            StringBuilder sb = new StringBuilder( );
            sb.Append( "Received" );
            if( e.Data.Length >= sizeof( uint ) + sizeof( float ) * (int)RobovieJointType._LENGTH && ID == BitConverter.ToUInt32( e.Data, e.Offset ) )
            {
                double[] values = new double[(int)RobovieJointType._LENGTH];
                for( int i = 0; i < values.Length; ++i )
                {
                    float v = BitConverter.ToSingle( e.Data, e.Offset + sizeof( uint ) + sizeof( float ) * i );
                    if( !float.IsInfinity( v ) && !float.IsNaN( v ) )
                    {
                        sb.Append( string.Format( " {0:0.00}", v ) );
                        values[i] = v;
                    }
                    else
                    {
                        sb.Append( " NaN" );
                        values[i] = double.NaN;
                    }
                }
                _robovie?.UpdateJointAngles( values );
            }
            else
            {
                sb.Append( string.Format( " {0} bytes", e.Length ) );
            }
            Log.Print( sb.ToString( ) );
        }


        protected override void OnDisconnected( NetConnection connection )
        {
            Log.Print( "Disconnected from {0}", connection.Name );
        }

    }
}
