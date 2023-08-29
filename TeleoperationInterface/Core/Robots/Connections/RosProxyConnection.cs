using AuthenticTeleoperation.Messages;
using System;
using System.Collections.Generic;

namespace AuthenticTeleoperation.Robots
{
    public abstract class RosProxyConnection
    {
        public RosProxyConnection( byte connectionType )
        {
            Type = connectionType;
        }


        public event EventHandler<Message> DataAvailable;


        public abstract bool IsConnected
        {
            get;
        }


        public byte Type
        {
            get;
            private set;
        }


        public abstract void Disconnect( );


        public abstract bool Send( Message message );


        public abstract bool TryConnect( HashSet<byte> connectionTypes, string hostAddress, int hostPort );


        protected void NotifyDataAvailable( Message message )
        {
            DataAvailable?.Invoke( this, message );
        }


        protected virtual void OnDataReceived( byte[] packet, int offset, int length )
        {
        }

    }
}
