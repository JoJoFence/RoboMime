#!/usr/bin/python
import struct
from audio_common_msgs.msg import AudioData
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3

class Serialization:
    NONE = 0x0000

    STANDARD_BOOL                = 0x0001
    STANDARD_BYTE                = 0x0002
    STANDARD_BYTEMULTIARRAY      = 0x0003
    STANDARD_CHAR                = 0x0004
    STANDARD_COLORRGBA           = 0x0005
    STANDARD_DURATION            = 0x0006
    STANDARD_EMPTY               = 0x0007
    STANDARD_FLOAT32             = 0x0008
    STANDARD_FLOAT32MULTIARRAY   = 0x0009
    STANDARD_FLOAT64             = 0x000A
    STANDARD_FLOAT64MULTIARRAY   = 0x000B
    STANDARD_HEADER              = 0x000C
    STANDARD_INT16               = 0x000D
    STANDARD_INT16MULTIARRAY     = 0x000E
    STANDARD_INT32               = 0x000F
    STANDARD_INT32MULTIARRAY     = 0x0010
    STANDARD_INT64               = 0x0011
    STANDARD_INT64MULTIARRAY     = 0x0012
    STANDARD_INT8                = 0x0013
    STANDARD_INT8MULTIARRAY      = 0x0014
    STANDARD_MULTIARRAYDIMENSION = 0x0015
    STANDARD_MULTIARRAYLAYOUT    = 0x0016
    STANDARD_STRING              = 0x0017
    STANDARD_TIME                = 0x0018
    STANDARD_UINT16              = 0x0019
    STANDARD_UINT16MULTIARRAY    = 0x001A
    STANDARD_UINT32              = 0x001B
    STANDARD_UINT32MULTIARRAY    = 0x001C
    STANDARD_UINT64              = 0x001D
    STANDARD_UINT64MULTIARRAY    = 0x001E
    STANDARD_UINT8               = 0x001F
    STANDARD_UINT8MULTIARRAY     = 0x0020

    GEOMETRY_ACCEL                      = 0x0200
    GEOMETRY_ACCELSTAMPED               = 0x0201
    GEOMETRY_ACCELWITHCOVARIANCE        = 0x0202
    GEOMETRY_ACCELWITHCOVARIANCESTAMPED = 0x0203
    GEOMETRY_INERTIA                    = 0x0204
    GEOMETRY_INERTIASTAMPED             = 0x0205
    GEOMETRY_POINT                      = 0x0206
    GEOMETRY_POINT32                    = 0x0207
    GEOMETRY_POINTSTAMPED               = 0x0208
    GEOMETRY_POLYGON                    = 0x0209
    GEOMETRY_POLYGONSTAMPED             = 0x020A
    GEOMETRY_POSE                       = 0x020B
    GEOMETRY_POSE2D                     = 0x020C
    GEOMETRY_POSEARRAY                  = 0x020D
    GEOMETRY_POSESTAMPED                = 0x020E
    GEOMETRY_POSEWITHCOVARIANCE         = 0x020F
    GEOMETRY_POSEWITHCOVARIANCESTAMPED  = 0x0210
    GEOMETRY_QUATERNION                 = 0x0211
    GEOMETRY_QUATERNIONSTAMPED          = 0x0212
    GEOMETRY_TRANSFORM                  = 0x0213
    GEOMETRY_TRANSFORMSTAMPED           = 0x0214
    GEOMETRY_TWIST                      = 0x0215
    GEOMETRY_TWISTSTAMPED               = 0x0216
    GEOMETRY_TWISTWITHCOVARIANCE        = 0x0217
    GEOMETRY_TWISTWITHCOVARIANCESTAMPED = 0x0218
    GEOMETRY_VECTOR3                    = 0x0219
    GEOMETRY_VECTOR3STAMPED             = 0x021A
    GEOMETRY_WRENCH                     = 0x021B
    GEOMETRY_WRENCHSTAMPED              = 0x021C

    SENSOR_BATTERYSTATE       = 0x0300
    SENSOR_CAMERAINFO         = 0x0301
    SENSOR_CHANNELFLOAT32     = 0x0302
    SENSOR_COMPRESSEDIMAGE    = 0x0303
    SENSOR_FLUIDPRESSURE      = 0x0304
    SENSOR_ILLUMINANCE        = 0x0305
    SENSOR_IMAGE              = 0x0306
    SENSOR_IMU                = 0x0307
    SENSOR_JOINTSTATE         = 0x0308
    SENSOR_JOY                = 0x0309
    SENSOR_JOYFEEDBACK        = 0x030A
    SENSOR_JOYFEEDBACKARRAY   = 0x030B
    SENSOR_LASERECHO          = 0x030C
    SENSOR_LASERSCAN          = 0x030D
    SENSOR_MAGNETICFIELD      = 0x030E
    SENSOR_MULTIDOFJOINTSTATE = 0x030F
    SENSOR_MULTIECHOLASERSCAN = 0x0310
    SENSOR_NAVSATFIX          = 0x0311
    SENSOR_NAVSATSTATUS       = 0x0312
    SENSOR_POINTCLOUD         = 0x0313
    SENSOR_POINTCLOUD2        = 0x0314
    SENSOR_POINTFIELD         = 0x0315
    SENSOR_RANGE              = 0x0316
    SENSOR_REGIONOFINTEREST   = 0x0317
    SENSOR_RELATIVEHUMIDITY   = 0x0318
    SENSOR_TEMPERATURE        = 0x0319
    SENSOR_TIMEREFERENCE      = 0x031A

    AUDIO_DATA         = 0x0400
    AUDIO_DATA_STAMPED = 0x0401
    AUDIO_INFO         = 0x0402

    NAVIGATION_ODOMETRY = 0x0500

    Layer2_AMEntity         = 0xA000
    Layer2_AMEntityList     = 0xA001
    Layer2_HTEntity         = 0xA002
    Layer2_HTEntityList     = 0xA003
    Layer2_OKAOEntity       = 0xA004
    Layer2_OKAOEntityList   = 0xA005
    Layer2_SPEECHEntity     = 0xA006
    Layer2_SPEECHEntityList = 0xA007


    @staticmethod
    def packHeader( msg ):
        return struct.pack( "<HIIIi", Serialization.STANDARD_HEADER, msg.seq, msg.stamp.secs, msg.stamp.nsecs, len( msg.frame_id ) ) + msg.frame_id.encode( )
    #END packHeader( )


    @staticmethod
    def packPoint( msg ):
        return struct.pack( "<Hddd", Serialization.GEOMETRY_POINT, msg.x, msg.y, msg.z )
    #END packPoint( )

    @staticmethod
    def unpackPoint( msg ):
        msgType, x, y, z = struct.unpack( "<Hddd", msg )
        if msgType == Serialization.GEOMETRY_POINT:
            return Point( x, y, z )
        return Point( )
    #END unpackPoint( )

    @staticmethod
    def packQuaternion( msg ):
        return struct.pack( "<Hdddd", Serialization.GEOMETRY_QUATERNION, msg.x, msg.y, msg.z, msg.w )
    #END packQuaternion( )

    @staticmethod
    def unpackQuaternion( msg ):
        msgType, x, y, z, w = struct.unpack( "<Hdddd", msg )
        if msgType == Serialization.GEOMETRY_QUATERNION:
            return Quaternion( x, y, z, w )
        return Quaternion( )
    #END unpackQuaternion( )

    @staticmethod
    def packVector3( msg ):
        return struct.pack( "<Hddd", Serialization.GEOMETRY_VECTOR3, msg.x, msg.y, msg.z )
    #END packVector3( )

    @staticmethod
    def unpackVector3( msg ):
        msgType, x, y, z = struct.unpack( "<Hddd", msg )
        if msgType == Serialization.GEOMETRY_VECTOR3:
            return Vector3( x, y, z )
        return Vector3( )
    #END unpackVector3( )

    @staticmethod
    def packPoseWithCovariance( msg ):
        data = struct.pack( "<H", Serialization.GEOMETRY_POSEWITHCOVARIANCE )
        for v in msg.covariance:
            data = data + struct.pack( "<d", v )
        return data + Serialization.packPoint( msg.pose.position ) + Serialization.packQuaternion( msg.pose.orientation )
    #END packPoseWithCovariance( )

    @staticmethod
    def packPoseWithCovarianceStamped( msg ):
        return struct.pack( "<H", Serialization.GEOMETRY_POSEWITHCOVARIANCESTAMPED ) + Serialization.packHeader( msg.header ) + Serialization.packPoseWithCovariance( msg.pose )
    #END packPoseWithCovarianceStamped( )

    @staticmethod
    def packTwist( msg ):
        return struct.pack( "<H", Serialization.GEOMETRY_TWIST ) + Serialization.packVector3( msg.linear ) + Serialization.packVector3( msg.angular )
    #END packTwist( )

    @staticmethod
    def unpackTwist( msg ):
        msgType, lm, lx, ly, lz, am, ax, ay, az = struct.unpack( "<HHdddHddd", msg )
        if msgType == Serialization.GEOMETRY_TWIST and lm == Serialization.GEOMETRY_VECTOR3 and am == Serialization.GEOMETRY_VECTOR3:
            return Twist( Vector3( lx, ly, lz ), Vector3( ax, ay, az ) )
        return Twist( )
    #END unpackTwist( )

    @staticmethod
    def packTwistWithCovariance( msg ):
        data = struct.pack( "<H", Serialization.GEOMETRY_TWISTWITHCOVARIANCE )
        for v in msg.covariance:
            data = data + struct.pack( "<d", v )
        return data + Serialization.packVector3( msg.twist.linear ) + Serialization.packVector3( msg.twist.angular )
    #END packTwistWithCovariance( )


    @staticmethod
    def packCompressedImage( msg ):
        return struct.pack( "<Hii", Serialization.SENSOR_COMPRESSEDIMAGE, len( msg.format ), len( msg.data ) ) + Serialization.packHeader( msg.header ) + msg.format.encode( ) + msg.data
    #END packCompressedImage( )

    @staticmethod
    def packPointField( msg ):
        return struct.pack( "<HBIIi", Serialization.SENSOR_POINTFIELD, msg.datatype, msg.offset, msg.count, len( msg.name ) ) + msg.name.encode( )
    #END packPointField( )

    @staticmethod
    def packPointCloud2( msg ):
        data = struct.pack( "<HBBIIIIii", Serialization.SENSOR_POINTCLOUD2, 1 if msg.is_bigendian else 0, 1 if msg.is_dense else 0, msg.height, msg.width, msg.point_step, msg.row_step, len( msg.data ), len( msg.fields ) ) + Serialization.packHeader( msg.header ) + msg.data
        for field in msg.fields:
            data = data + Serialization.packPointField( field )
        return data
    #END packPointCloud2( )


    @staticmethod
    def packAudioData( msg ):
        return struct.pack( "<Hi", Serialization.AUDIO_DATA, len( msg.data ) ) + msg.data
    #END packAudioData( )

    @staticmethod
    def unpackAudioData( msg ):
        msgType, length = struct.unpack( "<Hi", msg[:6] )
        if msgType == Serialization.AUDIO_DATA:
            data = AudioData( )
            data.data = msg[6:6 + length]
            return data
        return AudioData( )
    #END unpackAudioData( )

    @staticmethod
    def packAudioDataStamped( msg ):
        return struct.pack( "<H", Serialization.AUDIO_DATA_STAMPED ) + Serialization.packHeader( msg.header ) + Serialization.packAudioData( msg.audio )
    #END packAudioDataStamped( )

    @staticmethod
    def packAudioInfo( msg ):
        return struct.pack( "<HBIIii", Serialization.AUDIO_INFO, msg.channels, msg.sample_rate, msg.bitrate, len( msg.sample_format ), len( msg.coding_format ) ) + msg.sample_format.encode( ) + msg.coding_format.encode( )
    #END packAudioInfo( )


    @staticmethod
    def packOdometry( msg ):
        return struct.pack( "<Hi", Serialization.NAVIGATION_ODOMETRY, len( msg.child_frame_id ) ) + msg.child_frame_id.encode( ) + Serialization.packHeader( msg.header ) + Serialization.packPoseWithCovariance( msg.pose ) + Serialization.packTwistWithCovariance( msg.pose )
    #END packOdometry( )


    @staticmethod
    def packHTEntity( msg ):
        return struct.pack( "<HiiifffffffBii", Serialization.Layer2_HTEntity, msg.id, msg.unique_id, msg.type, msg.x, msg.y, msg.z, msg.body_orientation, msg.motion_direction, msg.velocity, msg.head_orientation, 1 if msg.is_speaking else 0, len( msg.option_fields ), len( msg.k2body ) ) + msg.option_fields.encode( ) + msg.k2body.encode( )
    #END packHTEntity( )

    @staticmethod
    def packHTEntityList( msg ):
        data = struct.pack( "<Hi", Serialization.Layer2_HTEntityList, len( msg.list ) ) + Serialization.packHeader( msg.header )
        for entity in msg.list:
            data = data + Serialization.packHTEntity( entity )
        return data
    #END packHTEntityList( )

#END Serialization
