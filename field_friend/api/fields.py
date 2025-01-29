from fastapi import Request, Response
from nicegui import app

from field_friend.automations import Field
from field_friend.system import System


class Fields:

    def __init__(self, system: System) -> None:
        self.system = system

        @app.get('/api/fields')
        def fields():
            return self.system.field_provider.fields

        @app.post('/api/fields')
        async def add_field(request: Request):
            try:
                field_data = await request.json()
                new_field = Field.from_dict(field_data)
                self.system.field_provider.create_field(new_field)
                return Response(content={'status': 'ok'}, status_code=200)
            except Exception as e:
                return Response(content={'status': 'error', 'message': str(e)}, status_code=400)
