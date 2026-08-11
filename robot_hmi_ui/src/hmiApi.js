const env = import.meta.env || {}
const locationValue = globalThis.location
const protocol = locationValue?.protocol === 'https:' ? 'https:' : 'http:'
const host = locationValue?.hostname || '127.0.0.1'
const apiPort = env.VITE_HMI_API_PORT || '8005'
export const DEFAULT_API_BASE = (env.VITE_HMI_API_URL || `${protocol}//${host}:${apiPort}`).replace(/\/$/, '')

export function createHmiApi(baseUrl = DEFAULT_API_BASE, fetchImpl = globalThis.fetch) {
  const base = baseUrl.replace(/\/$/, '')

  async function request(path, options = {}) {
    const response = await fetchImpl(`${base}${path}`, {
      headers: { 'Content-Type': 'application/json', ...(options.headers || {}) },
      ...options,
    })
    const payload = await response.json().catch(() => ({}))
    if (!response.ok) throw new Error(payload.detail || `HMI 요청 실패 (${response.status})`)
    return payload
  }

  return {
    getStatus: () => request('/api/status'),
    getDestinations: () => request('/api/destinations'),
    startDelivery: (destinationId) => request('/api/delivery/start', {
      method: 'POST',
      body: JSON.stringify({ destination_id: destinationId }),
    }),
    pauseTask: () => request('/api/task/pause', { method: 'POST' }),
    resumeTask: () => request('/api/task/resume', { method: 'POST' }),
    completeDelivery: () => request('/api/delivery/complete', { method: 'POST' }),
    returnToDock: () => request('/api/return-to-dock', { method: 'POST' }),
    statusWebSocketUrl: () => `${base.replace(/^http/, 'ws')}/ws/status`,
  }
}

export const hmiApi = createHmiApi()
