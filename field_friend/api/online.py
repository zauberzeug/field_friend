from nicegui import app


class Online:
    """API endpoints for checking the robot's online status."""

    def __init__(self) -> None:
        @app.get('/api/online')
        def connected():
            return {'online': True}
