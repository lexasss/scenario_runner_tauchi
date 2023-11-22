import threading

from typing import Optional, Callable, Set, Any
from socket import socket

from net.simple_socket_server import SimpleSocketServer

PORT = 46228

class TcpServer:
    def __init__(self) -> None:
        self._clients: Set[socket] = set()
        self._cb: Optional[Callable[[str], None]]
        
        socket_server = SimpleSocketServer(port = PORT)

        @socket_server.on('connect')
        def on_connect(sock: socket, peer: Any): # pyright: ignore[ reportUnusedFunction ]
            print(f'TCS: connection from {peer}')
            self._clients.add(sock)
            # socket_server.send(sock, bytes('What is your name?\r\n', 'utf-8'))

        @socket_server.on('disconnect')
        def on_disconnect(sock: socket, peer: Any): # pyright: ignore[ reportUnusedFunction ]
            print(f'TCS: {peer} disconnected')
            self._clients.remove(sock)

        @socket_server.on('message')
        def on_message(sock: socket, peer: Any, message: bytes): # pyright: ignore[ reportUnusedFunction ]
            request = message.decode().rstrip('\r\n')
            print(f'TCS: request from {peer}: {request}')
            if self._cb:
                self._cb(request)
            
        self._server = socket_server
        self._thread: Optional[threading.Thread] = None
        
    def start(self, cb: Optional[Callable[[str], None]] = None) -> None:
        self._cb = cb
        self._server.initialize()
        
        if self._thread is None:
            self._thread = threading.Thread(target = self._start)
            self._thread.start()
        
    def send(self, data: str) -> None:
        if self._thread:
            for client in self._clients:
                self._server.send(client, data.encode())
        
    def close(self) -> None:
        if self._thread:
            self._server.server_socket.close()
            self._thread.join()
            self._thread = None
            
    # Internal
            
    def _start(self) -> None:
        print('TCS: started')

        self._server.run()

        print('TCS: closed')
