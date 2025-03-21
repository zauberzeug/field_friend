from fastapi import Request, status
from fastapi.responses import JSONResponse
from nicegui import app

from field_friend.system import System


class Automation:
    def __init__(self, system: System) -> None:
        self.system = system

        @app.get('/api/automation/field_navigation/status')
        async def get_field_navigation_status():
            distance_to_end = self.system.robot_locator.pose.point.distance(
                self.system.field_navigation.end_point) if self.system.field_navigation.end_point else None
            return JSONResponse(
                status_code=status.HTTP_200_OK,
                content={
                    'automation_running': self.system.automator.is_running,
                    'state': f'{self.system.field_navigation._state.name}',  # pylint: disable=protected-access
                    'current_row': f'{self.system.field_navigation.current_row.name}' if self.system.field_navigation.current_row else None,
                    'field': f'{self.system.field_navigation.field.name}' if self.system.field_navigation.field else None,
                    'distance_to_end': distance_to_end,
                    'allowed_to_turn': self.system.field_navigation.allowed_to_turn
                }
            )

        @app.post('/api/automation/field_navigation/start')
        async def start_field_navigation(request: Request):
            try:
                request_data = await request.json()
                if 'field_id' not in request_data or 'beds' not in request_data:
                    return JSONResponse(
                        status_code=status.HTTP_400_BAD_REQUEST,
                        content={'error': 'Missing required fields: field_id and beds'}
                    )
                # Set up automation
                self.system.current_navigation = self.system.field_navigation
                self.system.field_navigation.field_id = request_data['field_id']
                self.system.field_provider.select_field(request_data['field_id'])
                self.system.field_provider.only_specific_beds = True
                # TODO currently only one bed is supported
                self.system.field_provider.selected_beds = [int(bed) for bed in request_data['beds']]
                self.system.automator.start()
                return JSONResponse(
                    status_code=status.HTTP_200_OK,
                    content={'status': 'automation started'}
                )
            except ValueError as e:
                return JSONResponse(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    content={'error': f'Invalid input: {e!s}'}
                )
            except Exception as e:
                return JSONResponse(
                    status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                    content={'error': f'Server error: {e!s}'}
                )

        @app.post('/api/automation/field_navigation/confirm_turn')
        async def confirm_turn():
            self.system.field_navigation.allowed_to_turn = True
            return JSONResponse(
                status_code=status.HTTP_200_OK,
                content={'status': 'turn confirmed'}
            )

        @app.post('/api/automation/start')
        async def start_automation():
            if not self.system.automator.is_running:
                self.system.automator.start()
                return JSONResponse(
                    status_code=status.HTTP_200_OK,
                    content={'status': 'automation started'}
                )
            return JSONResponse(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                content={'error': 'Automation is already running'}
            )

        @app.post('/api/automation/pause')
        def pause_automation():
            if not self.system.automator.is_running:
                return JSONResponse(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    content={'error': 'Automation is not running'}
                )
            self.system.automator.pause(because='API call')
            return JSONResponse(
                status_code=status.HTTP_200_OK,
                content={'status': 'automation paused'}
            )

        @app.post('/api/automation/stop')
        def stop_automation():
            if not self.system.automator.is_running and not self.system.automator.is_paused:
                return JSONResponse(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    content={'error': 'Automation is not running'}
                )
            self.system.automator.stop(because='API call')
            return JSONResponse(
                status_code=status.HTTP_200_OK,
                content={'status': 'automation stopped'}
            )

        @app.post('/api/automation/resume')
        def resume_automation():
            if not self.system.automator.is_paused:
                return JSONResponse(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    content={'error': 'Automation is not paused'}
                )
            self.system.automator.resume()
            return JSONResponse(
                status_code=status.HTTP_200_OK,
                content={'status': 'automation resumed'}
            )
