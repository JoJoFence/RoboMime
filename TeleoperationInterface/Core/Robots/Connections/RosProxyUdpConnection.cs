using AuthenticTeleoperation.Messages;
using SS.HRIKU.Network;
using SS.HRIKU.Network.Udp;
using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace AuthenticTeleoperation.Robots
{
    public abstract class RosProxyUdpConnection : RosProxyConnection
    {
        private uint _commID = 0;
        private NetConnection _connection = null;
        private bool _lastConnected = false;
        private Stopwatch _stopwatch = new Stopwatch( );


        public RosProxyUdpConnection( uint communicationID, byte connectionType ) : base( connectionType )
        {
            _commID = communicationID;
            _connection = new NetConnection( );
            _connection.DataReceived += ConnectionDataReceived;
        }


        public override bool IsConnected
        {
            get { return _connection.IsConnected; }
        }


        protected virtual long HeartbeatInterval
        {
            get { return 3000; }
        }


        public override void Disconnect( )
        {
            _connection.Disconnect( );
        }


        public override bool Send( Message message )
        {
            Packet packet = _connection.GetEmptyPacket( );
            if( message != null )
            {
                int dataLength = message.MessageSize;
                packet.EnsureCapacity( Packet.MIN_PACKET_SIZE + sizeof( uint ) + dataLength );
                packet.AddUInt32( _commID );
                packet.DataLength += message.Serialize( packet.Data, packet.DataOffset + sizeof( uint ), dataLength );
            }
            else
            {
                packet.EnsureCapacity( Packet.MIN_PACKET_SIZE + sizeof( uint ) );
                packet.AddUInt32( _commID );
            }
            bool success = _connection.Send( packet );
            packet.RemoveRef( );
            return success;
        }


        public override bool TryConnect( HashSet<byte> connectionTypes, string hostAddress, int hostPort )
        {
            bool notify = false;
            if( connectionTypes.Contains( Type ) )
            {
                if( IsConnected )
                {
                    if( _stopwatch.ElapsedMilliseconds > HeartbeatInterval )
                    {
                        _stopwatch.Restart( );
                        Send( null );
                    }
                }
                else
                {
                    if( _connection.Connect( hostAddress, hostPort + Type ) )
                    {
                        _stopwatch.Reset( );
                        _stopwatch.Start( );
                        _lastConnected = true;
                        notify = true;
                    }
                    else if( _lastConnected )
                    {
                        _stopwatch.Stop( );
                        _connection.Disconnect( );
                        _lastConnected = false;
                        notify = true;
                    }
                }
            }
            return notify;
        }


        private void ConnectionDataReceived( object sender, PacketEventArgs e )
        {
            if( e.Length < sizeof( uint ) || BitConverter.ToUInt32( e.Data, e.Offset ) != _commID )
            {   // invalid data
                _connection.Disconnect( );
            }
            else if( e.Length > sizeof( uint ) )
            {
                OnDataReceived( e.Data, e.Offset + sizeof( uint ), e.Length - sizeof( uint ) );
            }
        }

    }
}
