const BASE = import.meta.env.VITE_API_BASE || `http://${window.location.hostname}:8000`;

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
  getRobots: () => req('/api/robots'),
  getMap: () => req('/api/map'),
  getDestinations: () => req('/api/destinations'),
  getDatabaseTables: () => req('/api/database/tables'),
  getDatabaseTable: (table, limit = 100) => req(`/api/database/table/${encodeURIComponent(table)}?limit=${limit}`),
  cancelTask: (id) => req(`/api/robot/${id}/cancel`, { method: 'POST', body: JSON.stringify({ return_to: 'CURRENT' }) }),
  setEstop: (active, id) => req(`/api/robot/${id}/estop`, { method: 'POST', body: JSON.stringify({ active }) }),
  teleop: (linear, angular, id) => req(`/api/robot/${id}/teleop`, { method: 'POST', body: JSON.stringify({ linear, angular }) }),
  // 방향키 속도 전송과 분리된 텔레옵 활성화 토글 API
  setTeleopMode: (active, id) => req(`/api/robot/${id}/teleop/mode`, { method: 'POST', body: JSON.stringify({ active }) }),
  setDock: (dock, id) => req(`/api/robot/${id}/dock`, { method: 'POST', body: JSON.stringify({ dock }) }),
};
