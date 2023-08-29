using MediaPlayer.Media;
using MediaPlayer.SocialForceModule;
using System;
using System.ComponentModel;
using System.IO;
using System.Runtime.CompilerServices;
using System.Windows;
using System.Windows.Media.Media3D;
using System.Windows.Threading;

namespace MediaPlayer
{
    internal partial class MainWindow : Window, INotifyPropertyChanged
    {
        private const string CONVERT_TARGET_DIRECTORY = @"D:\Downloads\";
        private double _cntlUpDown = 0.0;
        private double _cntlLeftRight = 0.0;
        private double _cntlRotation = 0.0;
        private string[] _cvtFiles = new string[0];
        private int _cvtFilesIndex = 0;
        private string _mediaFilename = string.Empty;


        public MainWindow( )
        {
            MediaController = new MediaController( );
            MediaController.JoystickDataAvailable += JoystickDataAvailable;
            MediaController.SocialForceComputed += SocialForceComputed;
            MediaConverter = new MediaConverter( );
            InitializeComponent( );
        }


        public event PropertyChangedEventHandler PropertyChanged;


        public MediaController MediaController
        {
            get;
            private set;
        }


        public MediaConverter MediaConverter
        {
            get;
            private set;
        }


        public string MediaFileName
        {
            get { return _mediaFilename; }
            set
            {
                _mediaFilename = value ?? string.Empty;
                NotifyPropertyChanged( );
            }
        }


        private void MoveAreaSizeChanged( object sender, SizeChangedEventArgs e )
        {
            double size = e.NewSize.Width <= 0 ? e.NewSize.Height : Math.Min( e.NewSize.Width, e.NewSize.Height );
            if( e.NewSize.Width != e.NewSize.Height )
            {
                uiMoveCanvas.Width = size;
                uiMoveCanvas.Height = size;
            }
            uiMoveArea.Width = size;
            uiMoveArea.Height = size;
            uiMovePos.Width = size * 0.1;
            uiMovePos.Height = size * 0.1;
            UpdateJoystickVisualization( );
        }


        private void TurnAreaSizeChanged( object sender, SizeChangedEventArgs e )
        {
            double size = e.NewSize.Width <= 0 ? e.NewSize.Height : Math.Min( e.NewSize.Width, e.NewSize.Height );
            if( e.NewSize.Width * 2.0 != e.NewSize.Height )
            {
                uiTurnCanvas.Width = size * 2.0;
                uiTurnCanvas.Height = size;
            }
            uiTurnArea.Width = size * 2.0;
            uiTurnArea.Height = size * 0.25;
            uiTurnArea.Margin = new Thickness( 0.0, size * 0.5 - size * 0.125, 0.0, 0.0 );
            uiTurnPos.Width = size * 0.2;
            uiTurnPos.Height = size * 0.2;
            UpdateJoystickVisualization( );
        }


        private void UpdateJoystickVisualization( )
        {
            double x = uiMoveCanvas.ActualWidth * 0.5 + uiMoveCanvas.ActualWidth * 0.5 * _cntlLeftRight - uiMovePos.ActualWidth * 0.5;
            double y = uiMoveCanvas.ActualHeight * 0.5 + uiMoveCanvas.ActualHeight * 0.5 * _cntlUpDown - uiMovePos.ActualHeight * 0.5;
            uiMovePos.Margin = new Thickness( x, y, 0.0, 0.0 );
            x = uiTurnCanvas.ActualWidth * 0.5 + uiTurnCanvas.ActualWidth * 0.5 * _cntlRotation - uiTurnPos.ActualWidth * 0.5;
            y = uiTurnCanvas.ActualHeight * 0.5 - uiTurnPos.ActualHeight * 0.5;
            uiTurnPos.Margin = new Thickness( x, y, 0.0, 0.0 );
        }


        private void ClickOpenFile( object sender, RoutedEventArgs e )
        {

        }


        private void ConverterUpdateTimerTick( object sender, EventArgs e )
        {
            if( !MediaConverter.IsRunning )
            {
                if( _cvtFilesIndex < _cvtFiles.Length )
                {
                    string recFileName = _cvtFiles[_cvtFilesIndex++];
                    string baseFileName = recFileName.Substring( 0, recFileName.LastIndexOf( '.' ) ).Substring( recFileName.LastIndexOf( '\\' ) + 1 );
                    string newFileName = CONVERT_TARGET_DIRECTORY + baseFileName + ".mp4";
                    MediaConverter.Start( recFileName, newFileName );
                }
                else
                {
                    Close( );
                }
            }
        }


        private void NotifyPropertyChanged( [CallerMemberName] string name = "" )
        {
            PropertyChanged?.Invoke( this, new PropertyChangedEventArgs( name ) );
        }


        private void JoystickDataAvailable( object sender, Vector3D joystick )
        {
            _cntlUpDown = -joystick.Y;
            _cntlLeftRight = joystick.X;
            _cntlRotation = -joystick.Z;
            Dispatcher.BeginInvoke( new Action( ( ) =>
            {
                UpdateJoystickVisualization( );
            } ) );
        }


        private void SocialForceComputed( object sender, SocialForce sf )
        {
            Dispatcher.BeginInvoke( new Action( ( ) =>
            {
                foreach( Agent a in sf.Agents )
                {
                    Vector v = sf.GetSocialForce( a );
                    Console.WriteLine( "SocialForce: {0}, {1:F20}, {2:F20}", a.Name, v.X, v.Y );
                }
            } ) );
        }


        private void Window_Loaded( object sender, RoutedEventArgs e )
        {
            UpdateJoystickVisualization( );
            //MediaController.Start( MediaFileName );
            _cvtFiles = Directory.GetFiles( CONVERT_TARGET_DIRECTORY + "robot-raw\\", "*.rec" );
            DispatcherTimer cvtuptimer = new DispatcherTimer( );
            cvtuptimer.Interval = TimeSpan.FromMilliseconds( 1000.0 / 30.0 );
            cvtuptimer.Tick += ConverterUpdateTimerTick;
            cvtuptimer.Start( );
        }


        private void Window_Closing( object sender, CancelEventArgs e )
        {
            MediaController.Stop( );
            MediaConverter.Stop( );
        }

    }
}
