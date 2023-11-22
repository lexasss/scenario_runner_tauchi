# coding=utf8
"""
Based on pip/simple_socket_server:

Simple TCP socket server with select
Copyright (c) 2023 webtoucher
Distributed under the BSD 3-Clause license. See LICENSE for more info.
"""

import queue
import select
import socket
import time

from abc import ABCMeta
from event_bus import EventBus
from typing import Tuple, List, Dict, Any


class _Singleton(ABCMeta):
    """Metaclass for singleton."""

    def __call__(cls, *args: Any, **kwargs: Any):
        if not hasattr(cls, '_instance'):
            cls._instance = super().__call__(*args, **kwargs)
        return cls._instance


class SimpleSocketServerException(socket.error):
    def __init__(self, message: str, error: Any):
        super().__init__(message, error)


class SimpleSocketServer(EventBus, metaclass=_Singleton):
    def __init__(self, host: str = '0.0.0.0', port: int = 6666, max_conn: int = 5):
        super().__init__()

        self.__host = host
        self.__port = port
        self.__max_conn = max_conn

        self.__inputs: List[socket.socket] = []
        self.__outputs: List[socket.socket] = []
        self.__messages: Dict[socket.socket, queue.Queue[bytes]] = {}
        self.__clients: Dict[socket.socket, Tuple[str, int]] = {}
        
        self.__initialized = False

    def initialize(self):
        if self.__initialized:
            return
        
        self.__inputs = []
        self.__outputs = []
        self.__messages = {}
        self.__clients = {}

        self.__initialize()
        
    def run(self):
        if not self.__initialized:
            raise SimpleSocketServerException(
                'Socket is not initialized',
                '__initialized = False',
            )

        while self.server_socket.fileno() >= 0:
            sock_to_read, sock_to_write, sock_errors = select.select(
                self.__inputs,
                self.__outputs,
                self.__inputs,
                0.1,
            )
            try:
                self.__read_socket(sock_to_read)
                self.__write_socket(sock_to_write)
            except:
                break
            
            self.__exception_socket(sock_errors)
            time.sleep(0.1)

    def send(self, sock: socket.socket, message: bytes):
        if self.__messages[sock]:
            self.__messages[sock].put(message)
            if sock not in self.__outputs:
                self.__outputs.append(sock)


    # Internal
    
    def __initialize(self):
        self.server_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM,
            socket.IPPROTO_TCP,
        )
        server_fd = self.server_socket.fileno()
        if server_fd < 0:
            self.__initialized = False
            raise SimpleSocketServerException(
                'Error with creating sockets',
                'server_fd < 0',
            )
        self.server_socket.setblocking(False)
        self.server_socket.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1,
        )
        self.server_socket.bind((self.__host, self.__port))
        self.server_socket.listen(self.__max_conn)
        self.__inputs.append(self.server_socket)
        self.__initialized = True
        self.emit('start', self.__host, self.__port)

    def __read_socket(self, sockets_to_read: List[socket.socket]):
        for sock in sockets_to_read:
            if sock is self.server_socket:
                self.__server_socket(sock)
            else:
                self.__receive_message(sock)

    def __server_socket(self, server_socket: socket.socket):
        client_socket, client_address = server_socket.accept()
        self.__clients[client_socket] = client_address
        client_socket.setblocking(False)
        self.__inputs.append(client_socket)
        self.__messages[client_socket] = queue.Queue()
        self.emit('connect', client_socket, client_address)

    def __receive_message(self, sock: socket.socket):
        try:
            data_from_client = sock.recv(1024)
            if data_from_client:
                self.emit('message', sock, self.__clients[sock], data_from_client)
        except ConnectionResetError:
            self.__delete_socket_connection(sock)

    def __write_socket(self, socket_to_write: List[socket.socket]):
        for sock in socket_to_write:
            echo_message = ''.encode()
            try:
                if sock.fileno() > 0:
                    echo_message = self.__messages[sock].get_nowait()
                else:
                    self.__delete_socket_connection(sock)
                    continue
            except queue.Empty:
                self.__outputs.remove(sock)
            try:
                if echo_message:
                    sock.send(echo_message)
            except BrokenPipeError as err:
                self.emit('error', sock, self.__clients[sock], err)
                pass
            except ConnectionResetError:
                self.__delete_socket_connection(sock)

    def __exception_socket(self, socket_errors: Any):
        for sock in socket_errors:
            self.__delete_socket_connection(sock)
            print('trying to delete server socket')
            if sock is self.server_socket:
                self.__inputs = []
                self.__outputs = []
                self.__messages = {}
                self.__clients = {}
                self.__initialized = False

    def __delete_socket_connection(self, sock: socket.socket):
        if sock in self.__inputs:
            self.__inputs.remove(sock)
        self.__messages.pop(sock, None)
        if sock in self.__outputs:
            self.__outputs.remove(sock)
        self.emit('disconnect', sock, self.__clients[sock])
        sock.close()


if __name__ == '__main__':
    pass