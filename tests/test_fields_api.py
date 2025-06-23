from unittest.mock import Mock

import pytest
from fastapi.testclient import TestClient
from nicegui import app
from rosys.geometry import GeoPoint, GeoReference

from field_friend.api.fields import Fields
from field_friend.automations.field_provider import FieldProvider

# Global field provider for all tests to ensure consistency
_global_field_provider = FieldProvider()


@pytest.fixture
def mock_system():
    geo_ref = GeoReference(GeoPoint.from_degrees(lat=51.983159, lon=7.434212))
    GeoReference.update_current(geo_ref)

    system = Mock()
    system.robot_id = 'test_robot'
    system.field_provider = _global_field_provider
    system.robot_locator = Mock()
    system.robot_locator.pose = None
    return system


@pytest.fixture(autouse=True)
def clear_fields():
    _global_field_provider.clear_fields()
    yield
    _global_field_provider.clear_fields()


@pytest.fixture
def api_client(mock_system):
    Fields(mock_system)
    return TestClient(app)


@pytest.fixture
def valid_field_data():
    return {
        'id': 'test_field',
        'name': 'Test Field',
        'first_row_start': {'lat': 51.983159, 'lon': 7.434212},
        'first_row_end': {'lat': 51.983159, 'lon': 7.434312},
        'row_spacing': 0.5,
        'row_count': 5,
        'outline_buffer_width': 1,
        'bed_count': 1,
        'bed_spacing': 1,
        'bed_crops': {'0': None}
    }


def test_get_fields_returns_empty_dict_when_no_fields(api_client):
    response = api_client.get('/api/fields')

    assert response.status_code == 200
    assert response.json() == {}


def test_post_field_creates_field_successfully(api_client, valid_field_data):
    response = api_client.post('/api/fields', json=valid_field_data)

    assert response.status_code == 200
    assert response.json() == {'status': 'ok'}

    response = api_client.get('/api/fields')

    assert response.status_code == 200
    fields_data = response.json()
    assert len(fields_data) == 1
    assert 'test_field' in fields_data

    field_data = fields_data['test_field']
    assert field_data['id'] == 'test_field'
    assert field_data['name'] == 'Test Field'
    assert field_data['author_id'] == 'test_robot'
    assert field_data['row_spacing'] == 0.5
    assert field_data['row_count'] == 5
    assert 'outline' in field_data
    assert isinstance(field_data['outline'], list)


def test_post_field_with_existing_id_updates_field(api_client, valid_field_data):
    # Create initial field
    response = api_client.post('/api/fields', json=valid_field_data)
    assert response.status_code == 200
    assert response.json() == {'status': 'ok'}

    # Update field with same ID but different data
    updated_field_data = valid_field_data.copy()
    updated_field_data['name'] = 'Updated Field Name'
    updated_field_data['row_count'] = 10
    updated_field_data['row_spacing'] = 0.75

    response = api_client.post('/api/fields', json=updated_field_data)

    assert response.status_code == 200
    assert response.json() == {'status': 'ok'}

    # Verify field was updated, not duplicated
    response = api_client.get('/api/fields')
    assert response.status_code == 200
    fields_data = response.json()
    assert len(fields_data) == 1

    field_data = fields_data['test_field']
    assert field_data['name'] == 'Updated Field Name'
    assert field_data['row_count'] == 10
    assert field_data['row_spacing'] == 0.75


def test_post_field_strips_outline_and_author_id_from_request(api_client, valid_field_data):
    # Add outline and author_id to request data
    field_data_with_extra = valid_field_data.copy()
    field_data_with_extra['outline'] = [[1, 2], [3, 4]]
    field_data_with_extra['author_id'] = 'different_robot'

    response = api_client.post('/api/fields', json=field_data_with_extra)

    assert response.status_code == 200
    assert response.json() == {'status': 'ok'}

    # Verify field was created and outline/author_id were processed correctly
    response = api_client.get('/api/fields')
    assert response.status_code == 200
    fields_data = response.json()

    field_data = fields_data['test_field']
    # author_id should be from system.robot_id, not from request
    assert field_data['author_id'] == 'test_robot'
    # outline should be computed, not from request
    assert isinstance(field_data['outline'], list)
    assert field_data['outline'] != [[1, 2], [3, 4]]


def test_post_field_with_invalid_data_returns_error(api_client):
    invalid_field_data = {
        'id': 'invalid_field',
        'name': 'Invalid Field',
        # Missing required fields
    }

    response = api_client.post('/api/fields', json=invalid_field_data)

    assert response.status_code == 400
    error_data = response.json()
    assert error_data['status'] == 'error'
    assert isinstance(error_data['message'], str)

    # Verify no field was created
    response = api_client.get('/api/fields')
    assert response.status_code == 200
    assert response.json() == {}


def test_post_fields_inside_with_invalid_field_returns_error(api_client):
    response = api_client.post('/api/fields/inside', json='nonexistent_field')

    assert response.status_code == 500
    error_data = response.json()
    assert error_data['status'] == 'error'
    assert error_data['message'] == 'Field not found'


def test_post_fields_inside_with_empty_request_returns_error(api_client):
    response = api_client.post('/api/fields/inside', json='')

    assert response.status_code == 400
    error_data = response.json()
    assert error_data['status'] == 'error'
    assert 'Invalid request' in error_data['message']


def test_post_field_without_id_generates_id_automatically(api_client):
    field_data_without_id = {
        'name': 'Field Without ID',
        'first_row_start': {'lat': 51.983159, 'lon': 7.434212},
        'first_row_end': {'lat': 51.983159, 'lon': 7.434312},
        'row_spacing': 0.5,
        'row_count': 5,
        'outline_buffer_width': 1,
        'bed_count': 1,
        'bed_spacing': 1,
        'bed_crops': {'0': None}
    }

    response = api_client.post('/api/fields', json=field_data_without_id)

    assert response.status_code == 200
    assert response.json() == {'status': 'ok'}

    response = api_client.get('/api/fields')

    assert response.status_code == 200
    fields_data = response.json()
    assert len(fields_data) == 1

    field_id = next(iter(fields_data.keys()))
    field_data = fields_data[field_id]

    assert field_data['name'] == 'Field Without ID'
    assert field_data['id'] == field_id
    assert 'outline' in field_data
