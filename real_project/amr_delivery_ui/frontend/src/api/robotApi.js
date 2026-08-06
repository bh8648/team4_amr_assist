const BASE = import.meta.env.VITE_API_BASE || `http://${window.location.hostname}:8000`;
const ROBOT_ID = '5';

async function req(path, opts) {
  const token = sessionStorage.getItem('hmi_auth_token');
  const res = await fetch(`${BASE}${path}`, {
    headers: {
      'Content-Type': 'application/json',
      ...(token ? { Authorization: `Bearer ${token}` } : {}),
    },
    ...opts,
  });
  if (!res.ok) {
    if (res.status === 401 && path !== '/api/auth/login') {
      sessionStorage.removeItem('hmi_auth_token');
      window.dispatchEvent(new Event('hmi-auth-expired'));
    }
    throw new Error(`${path} -> ${res.status}`);
  }
  return res.json();
}

export const robotApi = {
  login: (username, password) => req('/api/auth/login', { method: 'POST', body: JSON.stringify({ username, password }) }),
  robotId: ROBOT_ID,
  getRobots: () => req('/api/robots'),
  checkConnection: () => req('/api/robots'),
  getOverviewEvents: () => req('/api/overview/events'),
  getVisionDetections: () => req('/api/vision/detections'),
  getState: (id = ROBOT_ID) => req(`/api/robot/${id}/state`),
  getMap: () => req('/api/map'),
  getDatabaseTables: () => req('/api/database/tables'),
  getDatabaseTable: (table, limit = 100) => req(`/api/database/table/${encodeURIComponent(table)}?limit=${limit}`),
  getTask: (id = ROBOT_ID) => req(`/api/robot/${id}/task`),
  getEvents: (id = ROBOT_ID) => req(`/api/robot/${id}/events`),
  setMode: (mode, id = ROBOT_ID) => req(`/api/robot/${id}/mode`, { method: 'POST', body: JSON.stringify({ mode }) }),
  sendGoal: (x, y, id = ROBOT_ID) => req(`/api/robot/${id}/goal`, { method: 'POST', body: JSON.stringify({ x, y }) }),
  cancelDelivery: (id = ROBOT_ID, returnTo = 'CURRENT') => req(`/api/robot/${id}/cancel`, { method: 'POST', body: JSON.stringify({ return_to: returnTo }) }),
  cancelTask: (id = ROBOT_ID) => req(`/api/robot/${id}/cancel`, { method: 'POST', body: JSON.stringify({ return_to: 'CURRENT' }) }),
  setEstop: (active, id = ROBOT_ID) => req(`/api/robot/${id}/estop`, { method: 'POST', body: JSON.stringify({ active }) }),
  teleop: (linear, angular, id = ROBOT_ID) => req(`/api/robot/${id}/teleop`, { method: 'POST', body: JSON.stringify({ linear, angular }) }),
  setDock: (dock, id = ROBOT_ID) => req(`/api/robot/${id}/dock`, { method: 'POST', body: JSON.stringify({ dock }) }),
};
