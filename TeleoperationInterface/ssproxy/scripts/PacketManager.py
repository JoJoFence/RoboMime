import struct
import time

class PacketManager:
    DATAGRAM_HEADER_SIZE = 10
    DATAGRAM_MAX_SIZE = 0x0000FF00
    DATAGRAM_MAX_DATA_SIZE = 0x0000FF00 - 10

    def __init__( self ):
        self._received = list( )
        self._recvMaxMergeTaskCount = 3
        self._recvMergeTask = list( )
        self._recvProvideIncomplete = False
        self._recvLatest = 0
    #END __init__( )

    def __del__( self ):
        del self._received[:]
        del self._recvMergeTask[:]
    #END __del__( )

    def hasReceivedPacket( self ):
        return len( self._received ) > 0
    #END hasReceivedPacket( )

    def getMaxReceiveTaskCount( self ):
        return self._recvMaxMergeTaskCount
    #END getMaxReceiveTaskCount( )

    def setMaxReceiveTaskCount( self, count ):
        self._recvMaxMergeTaskCount = count
    #END setMaxReceiveTaskCount( )

    def getToReceiveIncompletePacket( self ):
        return self._recvProvideIncomplete
    #END getToReceiveIncompletePacket( )

    def setToReceiveIncompletePacket( self, value ):
        self._recvProvideIncomplete = value
    #END setToReceiveIncompletePacket( )

    def getReceivedPacket( self ):
        if len( self._received ) > 0:
            task = self._received.pop( 0 )
            packet = bytearray( )
            nBytes = task["TotalSize"]
            for idx, data in enumerate( task["ReceivedData"] ):
                if data is not None:
                    packet = packet + data[PacketManager.DATAGRAM_HEADER_SIZE:]
                    nBytes = nBytes - (len( data ) - PacketManager.DATAGRAM_HEADER_SIZE)
                else:
                    packet = packet + bytearray( min( nBytes, PacketManager.DATAGRAM_MAX_DATA_SIZE ) )
                    nBytes = nBytes - min( nBytes, PacketManager.DATAGRAM_MAX_DATA_SIZE )
            #print( "[PacketManager] %d/%d piece(s) were merged into one %s packet of %d bytes" % (task["ReceivedCount"], len( task["ReceivedData"] ), "complete" if task["ReceivedCount"] >= len( task["ReceivedData"] ) else "incomplete", len( packet )) )
            return packet, task["ReceivedCount"] >= len( task["ReceivedData"] )
        return None, True
    #END getReceivedPacket( )

    def merge( self, data ):
        hasReceivedPacket = False
        if data is not None and len( data ) >= PacketManager.DATAGRAM_HEADER_SIZE:
            sequence, extraDataLength = struct.unpack( "<QH", data[:PacketManager.DATAGRAM_HEADER_SIZE] )
            index = (sequence >> 60) & 0x0F
            count = (sequence >> 56) & 0x0F
            timestamp = sequence & 0x00FFFFFFFFFFFFFF

            if timestamp >= self._recvLatest:
                i, task = self._findMergeTask( timestamp )
                if (task is not None) and (task["TotalSize"] != count * PacketManager.DATAGRAM_MAX_DATA_SIZE + extraDataLength or len( task["ReceivedData"] ) != count + 1):
                    # but the header does not match -> it means we did not get any data for at least one wavelength of the phase -> remove all
                    if self._recvProvideIncomplete:
                        for item in self._recvMergeTask:
                            self._received.append( item )
                        hasReceivedPacket = True
                    del self._recvMergeTask[:]
                    task = None
                    i = 0

                if task is None:
                    # either no previous working state found or it was in invalid state -> create new task
                    task = { "Timestamp": timestamp, "TotalSize": count * PacketManager.DATAGRAM_MAX_DATA_SIZE + extraDataLength, "ReceivedData": [None] * (count + 1), "ReceivedCount": 0 }
                    self._recvMergeTask.insert( i, task )

                if task["ReceivedData"][index] is None:
                    task["ReceivedCount"] = task["ReceivedCount"] + 1
                task["ReceivedData"][index] = data
                if task["ReceivedCount"] >= len( task["ReceivedData"] ):
                    # all pieces were received, remove any incomplete packets before this packet
                    if self._recvProvideIncomplete:
                        for item in self._recvMergeTask[:i]:
                            self._received.append( item )
                    self._recvLatest = timestamp
                    self._received.append( task )
                    self._recvMergeTask = self._recvMergeTask[i + 1:]
                    hasReceivedPacket = True
                elif i > self._recvMaxMergeTaskCount:
                    # this packet has not been completed, but there are too many unfinished tasks
                    if self._recvProvideIncomplete:
                        for item in self._recvMergeTask[:i - self._recvMaxMergeTaskCount]:
                            self._received.append( item )
                        hasReceivedPacket = True
                    self._recvMergeTask = self._recvMergeTask[i - self._recvMaxMergeTaskCount:]
        return hasReceivedPacket
    #END merge( )

    def split( self, packet ):
        if len( packet ) > PacketManager.DATAGRAM_MAX_DATA_SIZE * 0x0F:
            raise Exception( "Packet is too big to be handled by this module" )
        result = list( )
        count = int(len( packet ) / PacketManager.DATAGRAM_MAX_DATA_SIZE)
        extraDataLength = int(len( packet ) % PacketManager.DATAGRAM_MAX_DATA_SIZE)
        timestamp = int( time.time( ) * 1000.0 ) & 0x00FFFFFFFFFFFFFF
        offset = 0
        i = 0
        while i <= count and offset <= len( packet ):
            sequence = ((i & 0x0F) << 60) | ((count & 0x0F) << 56) | timestamp
            nBytes = min( PacketManager.DATAGRAM_MAX_DATA_SIZE, len( packet ) - offset )
            if nBytes > 0:
                result.append( struct.pack( "<QH", sequence, extraDataLength ) + packet[offset:offset + nBytes] )
            else:
                result.append( struct.pack( "<QH", sequence, extraDataLength ) )
            offset = offset + nBytes
            i = i + 1
        return result
    #END split( )

    def _findMergeTask( self, timestamp ):
        for idx, task in enumerate( self._recvMergeTask ):
            if task["Timestamp"] == timestamp: # we found the target
                return idx, task
            if task["Timestamp"] > timestamp: # task is not found, but this is where we need to insert the new task
                return idx, None
        return len( self._recvMergeTask ), None
    #END _findMergeTask( )
#END PacketManager
