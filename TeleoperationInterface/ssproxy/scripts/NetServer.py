import select
import socket
from NetClient import NetClient

class NetServer:
    def __init__( self, port = 9560 ):
        self._clients = list( )
        self._clientToMove = list( )
        self._isRunning = True
        self._sckServer = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
        self._sckServer.setsockopt( socket.SOL_SOCKET, socket.SO_REUSEADDR, 1 )
        self._sckServer.bind( ("0.0.0.0", port) )
        self._sckServer.listen( 8 )
        self._sckList = [self._sckServer]
    #END __init__( )

    def __del__( self ):
        for sock in self._sckList:
            sock.close( )
        self._srvSocket = None
        self._isRunning = False
        del self._sckList[:]
        del self._clients[:]
        del self._clientToMove[:]
    #END __del__( )

    def isRunning( self ):
        return self._isRunning
    #END isRunning( )

    def getNextClientToMove( self ):
        if len( self._clientToMove ) > 0:
            return self._clientToMove.pop( 0 )
        return None
    #END getNextClientToMove( )

    def update( self ):
        readable, writable, errors = select.select( self._sckList, self._sckList, self._sckList, 0.01 )
        self._updateReadable( readable )
        self._updateWritable( writable )
        self._updateErrors( errors )
    #END update( )

    def _acceptClient( self, srvSocket ):
        cliSocket, cliAddress = srvSocket.accept( )
        self._clients.append( { "socket": cliSocket, "address": cliAddress, "name": "", "message": None, "added": False } )
        self._sckList.append( cliSocket )
        print( "[NetServer] connection to %s" % str(cliAddress) )
    #END _acceptClient( )

    def _findClientBySocket( self, sck ):
        for i in range( 0, len( self._clients ) ):
            if self._clients[i]["socket"] == sck:
                return i
        return -1
    #END _findClientBySocket( )

    def _removeClient( self, index ):
        if 0 <= index and index < len( self._clients ):
            print( "[NetServer] disconnected from %s" % str(self._clients[index]["address"]) )
            self._sckList.remove( self._clients[index]["socket"] )
            self._clients[index]["socket"].close( ) # close socket
            self._clients.pop( index )
    #END _removeClient( )

    def _receiveDataFromClient( self, sck ):
        index = self._findClientBySocket( sck )
        if index >= 0:
            if self._clients[index]["message"] is None:
                try:
                    self._clients[index]["message"] = sck.recv( NetClient.PACKET_MINIMUM_SIZE )
                except:
                    print( "[NetServer] failed to receive data from %s" % str(self._clients[index]["address"]) )
                    self._removeClient( index )
            elif len( self._clients[index]["message"] ) < NetClient.PACKET_MINIMUM_SIZE:
                try:
                    self._clients[index]["message"] += sck.recv( NetClient.PACKET_MINIMUM_SIZE - len( self._clients[index]["message"] ) )
                except:
                    print( "[NetServer] failed to receive data from %s" % str(self._clients[index]["address"]) )
                    self._removeClient( index )
            else:
                type, length = NetClient.pckParseHeader( self._clients[index]["message"] )
                if len( self._clients[index]["message"] ) < length:
                    try:
                        self._clients[index]["message"] += sck.recv( length - len( self._clients[index]["message"] ) )
                    except:
                        print( "[NetServer] failed to receive data from %s" % str(self._clients[index]["address"]) )
                        self._removeClient( index )
                if len( self._clients[index]["message"] ) >= length:
                    if type == NetClient.PACKET_NORMAL:
                        pass # ignore any thing other than change connection type
                    elif type == NetClient.PACKET_CHANGECONNECTION:
                        self._clients[index]["name"] = self._clients[index]["message"][NetClient.PACKET_MINIMUM_SIZE:].decode( )
                        if self._clients[index]["name"][-1] == '\0':
                            self._clients[index]["name"] = self._clients[index]["name"][:-1]
                        self._clients[index]["message"] = None
                        print( "[NetServer] %s changing type to %s" % (str(self._clients[index]["address"]), str(self._clients[index]["name"])) )
                        self._sckList.remove( self._clients[index]["socket"] )
                        self._clientToMove.append( self._clients.pop( index ) )
                    elif type == NetClient.PACKET_DISCONNECTION:
                        self._removeClient( index )
    #END _receiveDataFromClient( )

    def _updateReadable( self, readable ):
        for sck in readable:
            if sck is self._sckServer:
                self._acceptClient( sck )
            else:
                self._receiveDataFromClient( sck )
    #END _updateReadable( )

    def _updateWritable( self, writable ):
        pass
    #END _updateWritable( )

    def _updateErrors( self, errors ):
        for sck in errors:
            if sck is self._sckServer:
                print( "[NetServer] error with the server socket" )
                self._isRunning = False
            else:
                print( "[NetServer] error with a client socket" )
                index = self._findClientBySocket( sck )
                self._removeClient( index )
    #END _updateErrors( )
#END NetServer
