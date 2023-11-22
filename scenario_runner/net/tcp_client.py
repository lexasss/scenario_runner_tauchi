from typing import Optional, Callable

from net.tcp_server import PORT
from net.simple_socket_client import SimpleSocketClient

class TcpClient:
    def __init__(self, host: str) -> None:
        self._host = host
        self._cb: Optional[Callable[[str], None]] = None

        socket_client = SimpleSocketClient(host, PORT)

        @socket_client.on('message')
        def on_message(message: bytes): # pyright: ignore[ reportUnusedFunction ]
            request = message.decode().rstrip('\r\n')
            print(f'TCC: message from server: {request}')
            if self._cb:
                self._cb(request)
        
        self._client = socket_client
        
    def connect(self, cb: Callable[[str], None]) -> None:
        self._cb = cb
        self._client.connect(timeout = 20.0)
        print(f'TCC: connected to {self._host}:{PORT}')

    def close(self) -> None:
        self._client.disconnect()
        print('TCC: disconnected')

    def send(self, data: str) -> None:
        self._client.send(data.encode()) # if you don't need an answer

    def request(self, req: str) -> Optional[str]:
        print(f'TCC: sent {req}')
        answer = self._client.ask(req.encode())
        if answer is None:
            return None
        
        answer = answer.decode().rstrip('\r\n')
        print(f'TCC: got {answer}')
        return answer

    @staticmethod
    def can_connect(host: str) -> bool:
        try:
            return SimpleSocketClient.test_connection(host, PORT)
        except:
            return False
        
        return True