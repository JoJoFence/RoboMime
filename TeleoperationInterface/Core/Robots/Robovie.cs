using AuthenticTeleoperation.Messages;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Threading;

namespace AuthenticTeleoperation.Robots
{
    public class Robovie : INotifyPropertyChanged
    {
        public const byte CTYPE_VIDEO_CAPTURE = 1;
        public const byte CTYPE_AUDIO_CAPTURE = 2;
        public const byte CTYPE_AUDIO_PLAYBACK = 3;
        public const byte CTYPE_ROBOT_CMDVEL = 4;
        public const byte CTYPE_ROBOT_ODOMETRY = 5;
        public const byte CTYPE_ROBOT_POSE_6D = 6;
        public const byte CTYPE_HUMAN_TRACKED_L2 = 7;
        public const byte CTYPE_VELODYNE_POINTS = 8;
        public delegate void MessageEventHandler( object sender, Message message );
        private Dictionary<byte, RosProxyConnection> _connections = new Dictionary<byte, RosProxyConnection>( );
        private HashSet<byte> _connectionTypes = new HashSet<byte>( );
        private string _hostAddress = "127.0.0.1";
        private int _hostPort = 9560;
        private Dictionary<byte, List<MessageEventHandler>> _messageHandlers = new Dictionary<byte, List<MessageEventHandler>>( );
        private bool _networkThreadRunning = false;
        private Thread _networkThread = null;


        public Robovie( )
        {
        }


        public event EventHandler ConnectionChanged;


        public event PropertyChangedEventHandler PropertyChanged;


        public string HostAddress
        {
            get { return _hostAddress; }
            set
            {
                _hostAddress = value;
                NotifyPropertyChanged( );
            }
        }


        public int HostPort
        {
            get { return _hostPort; }
            set
            {
                _hostPort = value;
                NotifyPropertyChanged( );
            }
        }


        public bool IsConnected
        {
            get
            {
                foreach( RosProxyConnection c in _connections.Values )
                {
                    if( c.IsConnected )
                    {
                        return true;
                    }
                }
                return false;
            }
        }


        public void Connect( params byte[] types )
        {
            if( _networkThread != null )
            {
                Disconnect( );
            }
            _connectionTypes.Clear( );
            if( types == null || types.Length <= 0 )
            {
                for( byte i = 0; i < byte.MaxValue; ++i )
                {
                    _connectionTypes.Add( i );
                }
            }
            else
            {
                foreach( byte type in types )
                {
                    _connectionTypes.Add( type );
                }
            }
            _networkThreadRunning = true;
            _networkThread = new Thread( NetworkThreadMain );
            _networkThread.IsBackground = true;
            _networkThread.Name = "RobovieNetworkThread";
            _networkThread.Start( );
        }


        public void Disconnect( )
        {
            _networkThreadRunning = false;
            if( _networkThread != null )
            {
                _networkThread.Join( );
                _networkThread = null;
            }
        }


        public void AddConnection( RosProxyConnection connection )
        {
            Debug.Assert( connection != null );
            Debug.Assert( !_connections.ContainsKey( connection.Type ) );
            if( _connections.ContainsKey( connection.Type ) )
            {
                throw new InvalidOperationException( "Media type " + connection.Type + " is already exists" );
            }
            else
            {
                connection.DataAvailable += Connection_DataAvailable;
                _connections.Add( connection.Type, connection );
                _messageHandlers.Add( connection.Type, new List<MessageEventHandler>( ) );
            }
        }


        public void AddMessageHandler( byte connectionType, MessageEventHandler handler )
        {
            if( _messageHandlers.ContainsKey( connectionType ) )
            {
                var handlers = _messageHandlers[connectionType];
                lock( handlers )
                {
                    handlers.Add( handler );
                }
            }
        }


        public void RemoveMessageHandler( byte connectionType, MessageEventHandler handler )
        {
            if( _messageHandlers.ContainsKey( connectionType ) )
            {
                var handlers = _messageHandlers[connectionType];
                lock( handlers )
                {
                    handlers.Remove( handler );
                }
            }
        }


        public void SendMessage( byte connectionType, Message message )
        {
            if( _connections.ContainsKey( connectionType ) )
            {
                _connections[connectionType].Send( message );
            }
        }


        protected void NotifyConnectionChanged( EventArgs e )
        {
            ConnectionChanged?.Invoke( this, e );
        }


        protected void NotifyPropertyChanged( [CallerMemberName] string name = "" )
        {
            PropertyChanged?.Invoke( this, new PropertyChangedEventArgs( name ) );
        }


        private void Connection_DataAvailable( object sender, Message e )
        {
            if( sender is RosProxyConnection connection )
            {
                Debug.Assert( _messageHandlers.ContainsKey( connection.Type ) );
                var handlers = _messageHandlers[connection.Type];
                lock( handlers )
                {
                    foreach( var handler in handlers )
                    {
                        handler( this, e );
                    }
                }
            }
        }


        private void NetworkThreadMain( )
        {
            while( _networkThreadRunning )
            {
                bool notify = false;
                var enumerator = _connections.Values.GetEnumerator( );
                while( _networkThreadRunning && enumerator.MoveNext( ) )
                {
                    RosProxyConnection connection = enumerator.Current;
                    notify = connection.TryConnect( _connectionTypes, _hostAddress, _hostPort ) || notify;
                }
                if( _networkThreadRunning && notify )
                {
                    NotifyPropertyChanged( "IsConnected" );
                    NotifyConnectionChanged( EventArgs.Empty );
                }
                Thread.Yield( );
                Thread.Sleep( 250 );
            }
            foreach( RosProxyConnection c in _connections.Values )
            {
                c.Disconnect( );
            }
            NotifyPropertyChanged( "IsConnected" );
            NotifyConnectionChanged( EventArgs.Empty );
        }

    }
}