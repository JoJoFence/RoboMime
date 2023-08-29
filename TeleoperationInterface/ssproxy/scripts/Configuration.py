#!/usr/bin/python

class ConfigurationSection:
    def __init__( self ):
        self._config = { }
    #END __init__( )

    def __contains__( self, item ):
        return item in self._config
    #END __contains__( )

    def add( self, name, value ):
        self._config.update( {name: value} )
    #END add( )

    def get( self, name, defaultValue = None ):
        if name is not None and name in self._config:
            return self._config[name]
        return defaultValue
    #END get( )
#END ConfigurationSection


class Configuration:

    def __init__( self ):
        self._config = { }

        self._config.update( {'VideoCapture': ConfigurationSection( )} )
        self._config['VideoCapture'].add( 'enable', 1 )
        self._config['VideoCapture'].add( 'rosVideoName', "/usb_cam/image_raw/compressed" )

        self._config.update( {'AudioCapture': ConfigurationSection( )} )
        self._config['AudioCapture'].add( 'enable', 1 )
        self._config['AudioCapture'].add( 'rosAudioCaptureName', "/audio" )
        self._config['AudioCapture'].add( 'rosAudioInfoName', "/audio_info" )

        self._config.update( {'AudioPlayback': ConfigurationSection( )} )
        self._config['AudioPlayback'].add( 'enable', 1 )
        self._config['AudioPlayback'].add( 'device', "default" )
        self._config['AudioPlayback'].add( 'rate', 16000 )
        self._config['AudioPlayback'].add( 'channels', 1 )
        self._config['AudioPlayback'].add( 'periodsize', 2048 )
        self._config['AudioPlayback'].add( 'buffering', 0.5 )

        self._config.update( {'RobotCmdVel': ConfigurationSection( )} )
        self._config['RobotCmdVel'].add( 'enable', 1 )
        self._config['RobotCmdVel'].add( 'rosCmdVelName', "/cmd_vel" )

        self._config.update( {'RobotOdometry': ConfigurationSection( )} )
        self._config['RobotOdometry'].add( 'enable', 1 )
        self._config['RobotOdometry'].add( 'rosOdomName', "/odom" )

        self._config.update( {'RobotTracking': ConfigurationSection( )} )
        self._config['RobotTracking'].add( 'enable', 1 )
        self._config['RobotTracking'].add( 'rosRobotTrack', "/pose_6d" )

        self._config.update( {'HumanTracking': ConfigurationSection( )} )
        self._config['HumanTracking'].add( 'enable', 1 )
        self._config['HumanTracking'].add( 'rosHumanTrack', "/human_tracked_l2" )

        self._config.update( {'VelodynePoints': ConfigurationSection( )} )
        self._config['VelodynePoints'].add( 'enable', 1 )
        self._config['VelodynePoints'].add( 'rosVelodynePoints', "/velodyne_points" )
    #END __init__( )


    def __contains__( self, item ):
        return item in self._config
    #END __contains__( )

    def get( self, sectionName, entryName = None, defaultValue = None ):
        if sectionName is not None and sectionName in self._config:
            if entryName is not None:
                return self._config[sectionName].get( entryName, defaultValue )
            return self._config[sectionName]
        elif entryName is not None:
            return defaultValue
        return { }
    #END get( )

#END Configuration
