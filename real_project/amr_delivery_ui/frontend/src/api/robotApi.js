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
  getDatabaseTables: () => req('/api/database/tables'),
  getDatabaseTable: (table, limit = 100) => req(`/api/database/table/${encodeURIComponent(table)}?limit=${limit}`),
  cancelTask: (id) => req(`/api/robot/${id}/cancel`, { method: 'POST', body: JSON.stringify({ return_to: 'CURRENT' }) }),
  setEstop: (active, id) => req(`/api/robot/${id}/estop`, { method: 'POST', body: JSON.stringify({ active }) }),
  teleop: (linear, angular, id) => req(`/api/robot/${id}/teleop`, { method: 'POST', body: JSON.stringify({ linear, angular }) }),
  setDock: (dock, id) => req(`/api/robot/${id}/dock`, { method: 'POST', body: JSON.stringify({ dock }) }),
  // ===== 배송모드 (임시 — 로봇 부착 UI가 생기면 이 두 줄을 지우면 됨) =====
  getDestinations: () => req('/api/database/table/destinations'),
  startTransport: (id, destinationId) => req(`/api/robot/${id}/transport`, { method: 'POST', body: JSON.stringify({ destination_id: destinationId }) }),
};
