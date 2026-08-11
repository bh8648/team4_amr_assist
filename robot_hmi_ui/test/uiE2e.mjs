import assert from 'node:assert/strict'
import { spawn } from 'node:child_process'
import http from 'node:http'
import { fileURLToPath } from 'node:url'
import path from 'node:path'
import WebSocket from 'ws'

const robotId = process.env.HMI_E2E_ROBOT_ID || 'robot5'
const portSeed = process.pid % 10000
const apiPort = 20000 + portSeed
const frontendPort = 32000 + portSeed
const debugPort = 44000 + portSeed
const frontendDir = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..')
const commands = []
let state = 'FOLLOWING'
let goalCompleted = true

const payload = () => ({
  robot_id: robotId,
  online: true,
  battery: 82,
  position_x: -1.5,
  position_y: -2.0,
  task_id: 'TASK_E2E',
  state,
  previous_state: state === 'PAUSED' ? 'FOLLOWING' : '',
  goal_type: state === 'RETURNING' ? 'TO_DOCK' : state === 'TRANSPORTING' ? 'TO_DESTINATION' : 'TO_WORKER',
  goal_completed: goalCompleted,
  destination_id: state === 'TRANSPORTING' ? 'DEST-A' : '',
  detail: 'E2E 테스트 상태',
  error_code: null,
})

const server = http.createServer((request, response) => {
  response.setHeader('Access-Control-Allow-Origin', '*')
  response.setHeader('Access-Control-Allow-Headers', 'Content-Type')
  response.setHeader('Access-Control-Allow-Methods', 'GET,POST,OPTIONS')
  if (request.method === 'OPTIONS') { response.writeHead(204).end(); return }
  if (request.method === 'GET' && request.url === '/api/status') {
    response.setHeader('Content-Type', 'application/json')
    response.end(JSON.stringify(payload()))
    return
  }
  if (request.method === 'GET' && request.url === '/api/destinations') {
    response.setHeader('Content-Type', 'application/json')
    response.end(JSON.stringify([{ destination_id: 'DEST-A', destination_name: 'A 구역', position_x: 1, position_y: 5 }]))
    return
  }
  if (request.method === 'POST') {
    commands.push(request.url)
    if (request.url === '/api/task/pause') state = 'PAUSED'
    if (request.url === '/api/task/resume') state = 'FOLLOWING'
    if (request.url === '/api/delivery/start') {
      state = 'TRANSPORTING'
      goalCompleted = false
      setTimeout(() => { goalCompleted = true }, 300)
    }
    if (request.url === '/api/delivery/complete' || request.url === '/api/return-to-dock') {
      state = 'RETURNING'
      goalCompleted = false
    }
    response.setHeader('Content-Type', 'application/json')
    response.end(JSON.stringify({ accepted: true }))
    return
  }
  response.writeHead(404).end()
})

const delay = (milliseconds) => new Promise((resolve) => setTimeout(resolve, milliseconds))

async function waitUntil(check, timeoutMs = 10000) {
  const deadline = Date.now() + timeoutMs
  while (Date.now() < deadline) {
    if (await check()) return
    await delay(100)
  }
  throw new Error('E2E 조건 대기 시간 초과')
}

class CdpClient {
  constructor(url) {
    this.nextId = 1
    this.pending = new Map()
    this.socket = new WebSocket(url)
    this.ready = new Promise((resolve, reject) => {
      this.socket.once('open', resolve)
      this.socket.once('error', reject)
    })
    this.socket.on('message', (raw) => {
      const message = JSON.parse(raw.toString())
      if (!message.id || !this.pending.has(message.id)) return
      const { resolve, reject } = this.pending.get(message.id)
      this.pending.delete(message.id)
      if (message.error) reject(new Error(message.error.message))
      else resolve(message.result)
    })
  }

  async send(method, params = {}) {
    await this.ready
    const id = this.nextId++
    const result = new Promise((resolve, reject) => this.pending.set(id, { resolve, reject }))
    this.socket.send(JSON.stringify({ id, method, params }))
    return result
  }

  async evaluate(expression) {
    const result = await this.send('Runtime.evaluate', { expression, returnByValue: true, awaitPromise: true })
    if (result.exceptionDetails) throw new Error(result.exceptionDetails.text)
    return result.result.value
  }

  close() { this.socket.close() }
}

let viteProcess
let chromeProcess
let cdp

try {
  await new Promise((resolve) => server.listen(apiPort, '127.0.0.1', resolve))
  const viteExecutable = path.join(frontendDir, 'node_modules', '.bin', 'vite')
  viteProcess = spawn(viteExecutable, [
    '--host', '127.0.0.1', '--port', String(frontendPort), '--strictPort',
  ], {
    cwd: frontendDir,
    stdio: ['ignore', 'pipe', 'pipe'],
    env: {
      ...process.env,
      VITE_ROBOT_ID: robotId,
      VITE_HMI_API_URL: `http://127.0.0.1:${apiPort}`,
    },
  })
  await waitUntil(async () => {
    try { return (await fetch(`http://127.0.0.1:${frontendPort}`)).ok } catch { return false }
  })

  chromeProcess = spawn('google-chrome', [
    '--headless=new', '--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage',
    `--remote-debugging-port=${debugPort}`, '--remote-allow-origins=*',
    `--user-data-dir=/tmp/${robotId}-hmi-chrome-${process.pid}`,
    `http://127.0.0.1:${frontendPort}`,
  ], { stdio: 'ignore' })

  let target
  await waitUntil(async () => {
    try {
      const targets = await (await fetch(`http://127.0.0.1:${debugPort}/json`)).json()
      target = targets.find((item) => item.type === 'page' && item.url.includes(`127.0.0.1:${frontendPort}`))
      return Boolean(target)
    } catch { return false }
  })
  cdp = new CdpClient(target.webSocketDebuggerUrl)
  await cdp.send('Runtime.enable')
  await waitUntil(() => cdp.evaluate("document.body.innerText.includes('작업자 동행 중')"))
  await waitUntil(() => cdp.evaluate("document.body.innerText.includes('82%') && document.body.innerText.includes('-1.50')"))

  const click = (text, exact = false) => cdp.evaluate(`(() => {
    const button = [...document.querySelectorAll('button')].find((item) => ${exact ? 'item.textContent.trim() ===' : 'item.textContent.includes('}${JSON.stringify(text)}${exact ? '' : ')'});
    if (!button || button.disabled) return false;
    button.click(); return true;
  })()`)
  const hasEnabledButton = (text) => cdp.evaluate(`
    [...document.querySelectorAll('button')].some((item) => item.textContent.includes(${JSON.stringify(text)}) && !item.disabled)
  `)

  assert.equal(await click('작업 현황'), true)
  await waitUntil(() => hasEnabledButton('작업 일시정지'))
  assert.equal(await click('작업 일시정지'), true)
  assert.equal(await click('작업 일시정지', true), true)
  await waitUntil(() => commands.includes('/api/task/pause'))
  await waitUntil(() => hasEnabledButton('작업 재개'))

  assert.equal(await click('작업 재개'), true)
  await waitUntil(() => commands.includes('/api/task/resume'))
  await waitUntil(() => hasEnabledButton('작업 일시정지'))

  assert.equal(await click('배송 보내기'), true)
  await waitUntil(() => hasEnabledButton('A 구역'))
  assert.equal(await click('A 구역'), true)
  assert.equal(await click('이 목적지로 보내기', true), true)
  assert.equal(await click('배송 시작', true), true)
  await waitUntil(() => commands.includes('/api/delivery/start'))
  await waitUntil(() => cdp.evaluate("document.body.innerText.includes('수취 확인 대기')"))
  assert.equal(await click('수취 확인 · 배송 완료', true), true)
  await waitUntil(() => commands.includes('/api/delivery/complete'))
  await waitUntil(() => cdp.evaluate("document.body.innerText.includes('배송 완료가 전달되었습니다')"))
  assert.equal(await click('확인', true), true)

  // 정상 배송 완료 복귀와 별개로, FOLLOWING 중 즉시 도킹 버튼도 검증한다.
  state = 'FOLLOWING'
  goalCompleted = true
  await waitUntil(() => hasEnabledButton('추종 종료 · 도킹하러 가기'))

  assert.equal(await click('추종 종료 · 도킹하러 가기'), true)
  assert.equal(await click('도킹하러 가기', true), true)
  await waitUntil(() => commands.includes('/api/return-to-dock'))

  assert.deepEqual(commands, [
    '/api/task/pause',
    '/api/task/resume',
    '/api/delivery/start',
    '/api/delivery/complete',
    '/api/return-to-dock',
  ])
  console.log(`${robotId} Chrome 버튼 E2E 통과: ${commands.join(' -> ')}`)
} finally {
  cdp?.close()
  chromeProcess?.kill('SIGTERM')
  viteProcess?.kill('SIGTERM')
  await new Promise((resolve) => server.close(resolve))
}
