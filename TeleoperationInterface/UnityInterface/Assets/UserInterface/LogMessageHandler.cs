using AuthenticTeleoperation.Configurations;
using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using UnityEngine.UI;

namespace AuthenticTeleoperation.UserInterface
{
    internal sealed class LogMessageHandler : MonoBehaviour
    {
        private const int MAX_LOG_MESSAGES = 128;
        private delegate void SelectionHandler( string name );
        private int _uiLogMsgCount = 0;
        private int _uiLogMsgIndex = 0;
        private Mutex _uiLogMsgMutex = new Mutex( );
        private List<string> _uiLogMsgs = new List<string>( );
        public ScrollRect uiLogMessageListView = null;


        public void CloseApplication( )
        {
            Debug.Log( "Closing the application. . ." );
            Application.Quit( );
        }


        private void Awake( )
        {
            SS.HRIKU.Network.Log.Error = AppendErrorMessage;
            SS.HRIKU.Network.Log.Print = AppendInfoMessage;
            Log.Error = AppendErrorMessage;
            Log.Print = AppendInfoMessage;
            AppConfiguration.Instance.Read( );
            _uiLogMsgCount = 0;
            _uiLogMsgIndex = 0;
            _uiLogMsgs.Clear( );
        }


        private void OnApplicationQuit( )
        {
            for( int i = uiLogMessageListView.content.childCount - 1; i >= 0; --i )
            {
                Transform child = uiLogMessageListView.content.GetChild( i );
                Destroy( child.gameObject );
            }
            _uiLogMsgCount = 0;
            _uiLogMsgIndex = 0;
            _uiLogMsgs.Clear( );
            // AppConfiguration.Instance.Write( );
        }


        private void Update( )
        {
            if( _uiLogMsgIndex < _uiLogMsgs.Count )
            {
                _uiLogMsgMutex.WaitOne( );
                if( _uiLogMsgs.Count > MAX_LOG_MESSAGES )
                {   // remove old messages if necessary
                    int count = _uiLogMsgs.Count - MAX_LOG_MESSAGES;
                    int logEntryCount = _uiLogMsgIndex;
                    _uiLogMsgs.RemoveRange( 0, count );
                    _uiLogMsgIndex = Math.Max( 0, _uiLogMsgIndex - count );
                    logEntryCount = logEntryCount - _uiLogMsgIndex; // calculate how many UI elements we should remove
                    while( --logEntryCount >= 0 )
                    {
                        Transform child = uiLogMessageListView.content.GetChild( logEntryCount );
                        Destroy( child.gameObject );
                    }
                }
                for( ; _uiLogMsgIndex < _uiLogMsgs.Count; ++_uiLogMsgIndex )
                {   // add new log messages
                    AddLogListViewItem( uiLogMessageListView, "Log " + _uiLogMsgCount++, _uiLogMsgs[_uiLogMsgIndex] );
                }
                _uiLogMsgMutex.ReleaseMutex( );
                uiLogMessageListView.verticalNormalizedPosition = 1.0f;
            }
        }


        private void AppendErrorMessage( string format, params object[] args )
        {
            string msg = "[" + DateTime.Now.ToString( "HH:mm:ss" ) + "][ERROR] " + string.Format( format, args );
            _uiLogMsgMutex.WaitOne( );
            _uiLogMsgs.Add( msg );
            _uiLogMsgMutex.ReleaseMutex( );
        }


        private void AppendInfoMessage( string format, params object[] args )
        {
            string msg = "[" + DateTime.Now.ToString( "HH:mm:ss" ) + "][INFO ] " + string.Format( format, args );
            _uiLogMsgMutex.WaitOne( );
            _uiLogMsgs.Add( msg );
            _uiLogMsgMutex.ReleaseMutex( );
        }


        private static GameObject AddLogListViewItem( ScrollRect listview, string name, string message )
        {
            Transform template = Utilities.GetDescendantByName( listview.transform, "ListItemTemplate" );
            GameObject item = Instantiate( template.gameObject );
            item.name = name;
            Text text = item.GetComponent<Text>( );
            text.text = message;
            item.SetActive( true );
            item.transform.SetParent( listview.content, false );
            return item;
        }

    }
}
