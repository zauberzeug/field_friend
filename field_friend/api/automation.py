from fastapi import Request
from nicegui import app

from field_friend.system import System


class Automation:
    def __init__(self, system: System) -> None:
        self.system = system

        @app.post('/api/automation/start')
        async def start_automation(request: Request):
            # TODO add error handling
            self.system.current_navigation = self.system.field_navigation
            field_data = await request.json()
            self.system.field_navigation.field_id = field_data['field_id']
            self.system.field_provider.only_specific_beds = field_data['only_specific_beds']
            self.system.field_provider.selected_beds = field_data['selected_beds']
            self.system.automator.start()
            return {'automation_started': True}

        @app.post('/api/automation/pause')
        def pause_automation():
            assert self.system.automator.is_running
            self.system.automator.pause(because='API call')
            return {'automation_paused': True}

        @app.post('/api/automation/stop')
        def stop_automation():
            assert self.system.automator.is_running
            self.system.automator.stop(because='API call')
            return {'automation_stopped': True}
