import logging
import os

import httpx
import rosys
from dotenv import load_dotenv
from nicegui import ui

load_dotenv('.env')

TELTONIKA_ROUTER_URL = 'http://192.168.42.1/api'
ADMIN_PASSWORD = os.environ.get('TELTONIKA_PASSWORD')


class TeltonikaRouter:
    def __init__(self) -> None:
        super().__init__()

        self.current_connection: str = 'disconnected'
        self.CONNECTION_CHANGED = rosys.event.Event()

        self.log = logging.getLogger('hardware.teltonika_router')

        self.client = httpx.AsyncClient(headers={'Content-Type': 'application/json'}, timeout=10.0)
        self.auth_token: str = ''
        self.token_time: float = 0.0
        self.connection_check_running = False
        if ADMIN_PASSWORD:
            self.log.info('Connecting to Teltonika router...')
            rosys.on_repeat(self.get_current_connection, 1.0)
        else:
            msg = 'The admin password for the Teltonika router is not set. Please set it in the .env file.'
            self.log.warning(msg)
            ui.label(msg).classes('text-xl')
            return

    async def get_current_connection(self) -> None:
        if self.connection_check_running is True:
            return
        self.connection_check_running = True
        if rosys.time() - self.token_time > 4 * 60:
            await self._get_token()
        self.log.debug('Getting internet connection info...')
        try:
            response = await self.client.get(f'{TELTONIKA_ROUTER_URL}/failover/status',
                                             headers={'Authorization': f'Bearer {self.auth_token}'})
            response.raise_for_status()
        except httpx.RequestError:
            self.log.exception(f'Getting Internet Connection Info: failed')
            self.connection_check_running = False
            return
        self.log.debug('Getting Internet Connection Info: success')
        up_connection = 'disconnected'
        for key, value in response.json()['data'].items():
            if value.get('status') == 'online':
                up_connection = key
                break
        print(up_connection)
        if up_connection == 'wan':
            if self.current_connection != 'ether':
                self.current_connection = 'ether'
                self.CONNECTION_CHANGED.emit()
        elif 'ifWan' in up_connection:
            if self.current_connection != 'wifi':
                self.current_connection = 'wifi'
                self.CONNECTION_CHANGED.emit()
        elif up_connection == 'mob1s1a1' or up_connection == 'mob1s2a1':
            if self.current_connection != 'mobile':
                self.current_connection = 'mobile'
                self.CONNECTION_CHANGED.emit()
        else:
            if self.current_connection != 'disconnected':
                self.current_connection = 'disconnected'
                self.CONNECTION_CHANGED.emit()
        self.connection_check_running = False
        return

    async def _get_token(self) -> None:
        try:
            self.log.info('Getting authentication token for Teltonika router...')
            response = await self.client.post(f'{TELTONIKA_ROUTER_URL}/login',
                                              json={'username': 'admin', 'password': ADMIN_PASSWORD})
            response.raise_for_status()
        except httpx.RequestError as e:
            self.log.error(f'Getting authentication token for Teltonika router: failed, {e}')
            self.auth_token = ''
            self.token_time = 0.0
        self.log.info('Getting authentication token for Teltonika router: success')
        self.auth_token = response.json()['data']['token']
        self.token_time = rosys.time()
