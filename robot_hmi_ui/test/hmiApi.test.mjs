import assert from 'node:assert/strict'
import test from 'node:test'
import { createHmiApi } from '../src/hmiApi.js'

test('HMI 상태·목적지·버튼 API가 올바른 경로를 호출한다', async () => {
  const calls = []
  const fetchMock = async (url, options = {}) => {
    calls.push([url, options.method || 'GET', options.body || null])
    return { ok: true, json: async () => ({ accepted: true }) }
  }
  const api = createHmiApi('http://robot-pc:8000/', fetchMock)

  await api.getStatus()
  await api.getDestinations()
  await api.startDelivery('DEST-A')
  await api.pauseTask()
  await api.resumeTask()
  await api.completeDelivery()
  await api.returnToDock()

  assert.deepEqual(calls, [
    ['http://robot-pc:8000/api/status', 'GET', null],
    ['http://robot-pc:8000/api/destinations', 'GET', null],
    ['http://robot-pc:8000/api/delivery/start', 'POST', JSON.stringify({ destination_id: 'DEST-A' })],
    ['http://robot-pc:8000/api/task/pause', 'POST', null],
    ['http://robot-pc:8000/api/task/resume', 'POST', null],
    ['http://robot-pc:8000/api/delivery/complete', 'POST', null],
    ['http://robot-pc:8000/api/return-to-dock', 'POST', null],
  ])
  assert.equal(api.statusWebSocketUrl(), 'ws://robot-pc:8000/ws/status')
})

test('백엔드 오류 detail을 사용자 오류로 전달한다', async () => {
  const api = createHmiApi('http://robot-pc:8000', async () => ({
    ok: false, status: 409, json: async () => ({ detail: '현재 상태에서는 정상 복귀할 수 없습니다.' }),
  }))
  await assert.rejects(api.returnToDock(), /현재 상태에서는 정상 복귀할 수 없습니다/)
})
