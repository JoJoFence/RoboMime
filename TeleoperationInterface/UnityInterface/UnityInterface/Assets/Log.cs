using UnityEngine;

namespace AuthenticTeleoperation
{
    public static class Log
    {
        public delegate void LogMessageHandler( string format, params object[] args );
        private static LogMessageHandler _error = DefaultError;
        private static LogMessageHandler _print = DefaultPrint;


        public static LogMessageHandler Error
        {
            get { return _error; }
            set { _error = value ?? DefaultError; }
        }


        public static LogMessageHandler Print
        {
            get { return _print; }
            set { _print = value ?? DefaultPrint; }
        }


        private static void DefaultError( string format, params object[] args )
        {
            Debug.LogError( string.Format( format, args ) );
        }


        private static void DefaultPrint( string format, params object[] args )
        {
            Debug.Log( string.Format( format, args ) );
        }

    }
}
