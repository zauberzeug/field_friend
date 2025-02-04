import shapely.geometry
from fastapi import Request, Response, status
from fastapi.responses import JSONResponse
from nicegui import app

from field_friend.automations import Field
from field_friend.system import System


class Fields:

    def __init__(self, system: System) -> None:
        self.system = system

        @app.get('/api/fields')
        def fields():
            fields = {
                field.id: {
                    **field.to_dict(),
                    'author_id': self.system.robot_id,
                    'outline': field.outline_as_tuples
                } for field in self.system.field_provider.fields
            }
            return fields

        @app.post('/api/fields')
        async def add_field(request: Request):
            try:
                field_data = await request.json()
                field_data.pop('outline', None)
                field_data.pop('author_id', None)
                print(f'field_data🔴: {field_data}')
                new_field = Field.from_dict(field_data)
                print(f'new_field🔵: {new_field}')
                if 'id' in field_data and self.system.field_provider.get_field(field_data['id']):
                    print(f'updating field🔴: {field_data}')
                    self.system.field_provider.update_field_parameters(
                        field_id=field_data['id'],
                        name=field_data['name'],
                        row_count=field_data['row_count'],
                        row_spacing=field_data['row_spacing'],
                        outline_buffer_width=field_data['outline_buffer_width'],
                        bed_count=field_data['bed_count'],
                        bed_spacing=field_data['bed_spacing'],
                        bed_crops=field_data['bed_crops']
                    )
                else:
                    self.system.field_provider.create_field(new_field)
                return JSONResponse(content={'status': 'ok'}, status_code=status.HTTP_200_OK)
            except Exception as e:
                return JSONResponse(content={'status': 'error', 'message': str(e)}, status_code=status.HTTP_400_BAD_REQUEST)

        @app.post('/api/fields/inside')
        async def check_robot_in_field(request: Request):
            try:
                data = await request.json()
                field_id = data.get('field_id')
                if not field_id:
                    return Response(
                        content={'status': 'error', 'message': 'Invalid request. Requires field_id'},
                        status_code=400
                    )
                field = self.system.field_provider.get_field(field_id)
                if not field:
                    return Response(
                        content={'status': 'error', 'message': 'Field not found'},
                        status_code=500
                    )
                if not self.system.robot_locator.pose:
                    return Response(
                        content={'status': 'error', 'message': 'Robot position not available'},
                        status_code=500
                    )
                robot_position = self.system.robot_locator.pose.point
                field_polygon = field.shapely_polygon()
                is_inside = field_polygon.contains(shapely.geometry.Point(robot_position.x, robot_position.y))
                bed_id = None
                if is_inside:
                    row = min(field.rows, key=lambda r: r.line_segment().line.foot_point(
                        robot_position).distance(robot_position))  # nearest row
                    row_index = int(row.id.split('_')[-1]) - 1  # -1 because row IDs start at 1
                    bed_id = int(row_index // field.row_count) + 1  # +1 because bed IDs start at 1
                return Response(
                    content={'status': 'ok', 'inside': is_inside, 'beds': [bed_id]},
                    status_code=200
                )
            # TODO specify the exception
            except Exception as e:
                return Response(
                    content={'status': 'error', 'message': str(e)},
                    status_code=400
                )
