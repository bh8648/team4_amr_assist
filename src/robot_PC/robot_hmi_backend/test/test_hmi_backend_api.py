from fastapi import HTTPException
import pytest

from robot_hmi_backend import hmi_backend_node as backend


class FakeRosNode:
    def __init__(self, state='FOLLOWING'):
        self.state = state
        self.commands = []
        self.destinations = [{
            'destination_id': 'DEST-A', 'destination_name': 'A 구역',
            'position_x': 1.0, 'position_y': 5.0,
        }]

    def status_payload(self):
        return {
            'state': self.state,
            'goal_type': 'TO_DESTINATION' if self.state == 'TRANSPORTING' else '',
            'goal_completed': self.state == 'TRANSPORTING',
        }

    def destination_payload(self):
        return self.destinations

    def publish_command(self, command, destination_id=''):
        self.commands.append((command, destination_id))


@pytest.mark.parametrize(
    ('endpoint', 'state', 'expected_command'),
    [
        (backend.pause_task, 'FOLLOWING', 'PAUSE'),
        (backend.resume_task, 'PAUSED', 'RESUME'),
        (backend.return_to_dock, 'FOLLOWING', 'RETURN_TO_DOCK'),
        (backend.return_to_dock, 'PAUSED', 'RETURN_TO_DOCK'),
    ],
)
def test_hmi_endpoint_publishes_expected_task_command(monkeypatch, endpoint, state, expected_command):
    fake = FakeRosNode(state)
    monkeypatch.setattr(backend, 'ros_node', fake)
    assert endpoint() == {'accepted': True}
    assert fake.commands == [(expected_command, '')]


def test_start_delivery_uses_registered_destination(monkeypatch):
    fake = FakeRosNode('FOLLOWING')
    monkeypatch.setattr(backend, 'ros_node', fake)
    response = backend.start_delivery(backend.DestinationRequest(destination_id='DEST-A'))
    assert response == {'accepted': True, 'destination_id': 'DEST-A'}
    assert fake.commands == [('START_TRANSPORT', 'DEST-A')]


def test_return_to_dock_rejects_docked_robot(monkeypatch):
    monkeypatch.setattr(backend, 'ros_node', FakeRosNode('DOCKED'))
    with pytest.raises(HTTPException) as error:
        backend.return_to_dock()
    assert error.value.status_code == 409


def test_delivery_complete_publishes_confirmation_after_arrival(monkeypatch):
    fake = FakeRosNode('TRANSPORTING')
    monkeypatch.setattr(backend, 'ros_node', fake)
    assert backend.complete_delivery() == {'accepted': True}
    assert fake.commands == [('DELIVERY_CONFIRMED', '')]
