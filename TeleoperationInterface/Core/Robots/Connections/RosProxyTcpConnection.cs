using AuthenticTeleoperation.Messages;
using SS.HRIKU.Network;
using SS.HRIKU.Network.Tcp;
using System.Collections.Generic;

namespace AuthenticTeleoperation.Robots
{
    public abstract class RosProxyTcpConnection : RosProxyConnection
    {
        private NetConnection _connection = null;
        private bool _lastConnected = false;


        public RosProxyTcpConnection( string name, byte connectionType ) : base( connectionType )
        {
            _connection = new NetConnection( name );
            _connection.DataReceived += ConnectionDataReceived;
        }


        public override bool IsConnected
        {
            get { return _connection.IsConnected; }
        }


        public override void Disconnect( )
        {
            _connection.Disconnect( );
        }


        public override bool Send( Message message )
        {
            int dataLength = message.MessageSize;
            Packet packet = _connection.GetEmptyPacket( );
            packet.EnsureCapacity( Packet.MIN_PACKET_SIZE + dataLength );
            packet.DataLength = message.Serialize( packet.Data, packet.DataOffset, dataLength );
            bool success = _connection.Send( packet );
            packet.RemoveRef( );
            return success;
        }


        public override bool TryConnect( HashSet<byte> connectionTypes, string hostAddress, int hostPort )
        {
            bool notify = false;
            if( connectionTypes.Contains( Type ) && !IsConnected )
            {
                if( _connection.Connect( hostAddress, hostPort + Type ) )
                {
                    _lastConnected = true;
                    notify = true;
                }
                else if( _lastConnected )
                {
                    _connection.Disconnect( false );
                    _lastConnected = false;
                    notify = true;
                }
            }
            return notify;
        }


        private void ConnectionDataReceived( object sender, PacketEventArgs e )
        {
            OnDataReceived( e.Data, e.Offset, e.Length );
        }

    }
}
