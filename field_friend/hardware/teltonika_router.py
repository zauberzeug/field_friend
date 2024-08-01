import logging
import os
from typing import TYPE_CHECKING

import httpx
import rosys
from dotenv import load_dotenv

load_dotenv('.env')

TELTONIKA_ROUTER_URL = 'http://192.168.42.1/api'
ADMIN_PASSWORD = os.environ.get('TELTONIKA_PASSWORD')


class TeltonikaRouter:
    def __init__(self) -> None:
        super().__init__()

        self.current_connection: str | None = None
        self.CONNECTION_CHANGED = rosys.event.Event()

        self.log = logging.getLogger('hardware.teltonika_router')

        self.client = httpx.AsyncClient(headers={'Content-Type': 'application/json'}, timeout=10.0)
        self.auth_token: str = ''
        self.token_time: float = 0.0

        if ADMIN_PASSWORD:
            self.log.info('Connecting to Teltonika router...')
            # rosys.on_repeat(self.get_status_info, 5.0)
        else:
            self.log.error('No Teltonika password found in environment')

    async def get_status_info(self) -> None:
        if rosys.time() - self.token_time > 4 * 60:
            await self._get_token()
        self.log.debug('Getting status...')
        print('Getting status...')
        try:
            response = await self.client.get(f'{TELTONIKA_ROUTER_URL}/interfaces/basic/status',
                                             headers={'Authorization': f'Bearer {self.auth_token}'})
            response.raise_for_status()
        except httpx.RequestError:
            self.log.exception(f'Getting Status Info: failed')
            return
        self.log.debug('Getting Status Info: success')
        print(f'Status: {response.json()}')
        # FIXME: is this correct? Is 'is_up' the correct key?
        for connection in response.json()['data']:
            if connection['is_up'] and connection['area_type'] == 'wan':
                if self.current_connection != connection['name']:
                    self.current_connection = connection['name']
                    self.CONNECTION_CHANGED.emit()
                    print(f'Active Connection: {connection["name"]}')

    async def get_sim_cards_info(self) -> None:
        if rosys.time() - self.token_time > 4 * 60:
            await self._get_token()
        mobile_id_list = ['mob1s1a1', 'mob1s2a1']
        self.log.debug('Getting status of SIM-Cards...')
        print('Getting status of SIM-Cards...')
        for mobile_id in mobile_id_list:
            try:
                response = await self.client.get(f'{TELTONIKA_ROUTER_URL}/interfaces/basic/status/{mobile_id}',
                                                 headers={'Authorization': f'Bearer {self.auth_token}'})
                response.raise_for_status()
            except httpx.RequestError:
                self.log.exception(f'Getting SIM-Cards Info: failed')
                return
            self.log.debug('Getting SIM-Cards Info: success')
            print(f'SIM {mobile_id} status: {response.json()}')

    # TODO: is this function necessary?
    async def set_sim_card_activation(self, mobile_id: str, activation: bool) -> None:
        if rosys.time() - self.token_time > 4 * 60:
            await self._get_token()
        self.log.debug('Setting SIM-Cards activation...')
        try:
            response = await self.client.post(f'{TELTONIKA_ROUTER_URL}/interfaces/basic/status/{mobile_id}',
                                              headers={'Authorization': f'Bearer {self.auth_token}'}, data={'up': activation})
            response.raise_for_status()
        except httpx.RequestError:
            self.log.exception(f'Setting SIM-Cards Activation: failed')
            return
        self.log.debug('Setting SIM-Cards Activation: success')
        print(f'SIM {mobile_id} status: {response.json()}')

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
