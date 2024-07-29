import logging
import os
from typing import TYPE_CHECKING

import httpx
from dotenv import load_dotenv

import rosys

if TYPE_CHECKING:
    from system import System
load_dotenv('.env')

TELTONIKA_ROUTER_URL = 'http://192.168.42.1/api'
ADMIN_PASSWORD = os.environ.get('TELTONIKA_PASSWORD')


class TeltonikaRouter:
    def __init__(self, system: 'System') -> None:
        super().__init__()

        self.log = logging.getLogger('hardware.teltonika_router')

        self.client = httpx.AsyncClient(headers={'Content-Type': 'application/json'}, timeout=10.0)
        self.auth_token: str = ''
        self.token_time: float = 0.0
        self.wifi_id: str = ''

        if ADMIN_PASSWORD:
            self.log.info('Connecting to Teltonika router...')
            rosys.on_repeat(self.step, 5.0)
        else:
            self.log.error('No Teltonika password found in environment')

    async def step(self) -> None:
        if rosys.time() - self.token_time > 4 * 60:
            await self._get_token()
        if not self.wifi_id:
            await self._get_wifi_id()
        # TODO add a request. In this example the wifi cams are requested
        # try:
        #     self.log.debug('Connecting to router...')
        #     response = await self.client.get(f'{TELTONIKA_ROUTER_URL}/wireless/interfaces/status/{self.wifi_id}',
        #                                      headers={'Authorization': f'Bearer {self.auth_token}'})
        #     response.raise_for_status()
        # except httpx.RequestError:
        #     self.log.exception(f'Getting connection information for WiFi Cams: failed (WiFi ID: {self.wifi_id})')
        #     return
        # self.log.debug('Getting connection information for WiFi Cams: success')

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

    async def _get_wifi_id(self) -> None:
        if not self.auth_token:
            return
        try:
            self.log.info('Getting WiFi network ID...')
            response = await self.client.get(f'{TELTONIKA_ROUTER_URL}/wireless/interfaces/status/',
                                             headers={'Authorization': f'Bearer {self.auth_token}'})
            response.raise_for_status()
        except httpx.RequestError as e:
            self.log.error(f'Getting WiFi network ID: failed, {e}')
            return
        for interface in response.json()['data']:
            if interface['ssid'] == 'WiFi Cameras':
                self.wifi_id = interface['id']
                self.log.info(f'Getting WiFi network ID: {self.wifi_id}')
                break
        else:
            self.log.error('Getting WiFi network ID: not found')
