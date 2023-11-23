class Response:
    actor_id: int
    error: str
    def has_error(self) -> bool: ...
