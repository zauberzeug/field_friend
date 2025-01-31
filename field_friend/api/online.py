from nicegui import app


class Online:

    def __init__(self) -> None:
        @app.get('/api/online')
        def connected():
            return {'online': True}
