namespace AuthenticTeleoperation.Messages
{
    public abstract class Message
    {
        internal const int MSG__TYPE_SIZE = sizeof( MsgType );


        public Message( )
        {
        }


        public abstract int MessageSize
        {
            get;
        }


        internal abstract int MessageMinimumSize
        {
            get;
        }


        internal abstract ushort MessageType
        {
            get;
        }


        /// <summary>
        /// Unpack the data from the buffer
        /// </summary>
        /// <param name="data">The buffer</param>
        /// <param name="offset">Offset on the buffer</param>
        /// <param name="length">Number of available bytes from the offset</param>
        /// <returns>Number of actual processed bytes</returns>
        public abstract int Deserialize( byte[] data, int offset, int length );


        /// <summary>
        /// Pack the data into the buffer
        /// </summary>
        /// <param name="data">The buffer</param>
        /// <param name="offset">Offset on the buffer</param>
        /// <param name="length">Number of available bytes from the offset</param>
        /// <returns>Number of actual stored bytes</returns>
        public abstract int Serialize( byte[] data, int offset, int length );

    }
}
