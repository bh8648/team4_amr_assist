import { useEffect, useMemo, useState } from 'react';
import './WorkerApp.css';
import { hmiApi } from './hmiApi.js';

const STEPS = [
  ['ASSIGNED', '작업자 접근'],
  ['FOLLOWING', '작업자 탐색/동행'],
  ['DELIVERING', '배송 중'],
  ['WAITING_FOR_UNLOAD', '수취 대기'],
];

const STATE_TEXT = {
  IDLE: '도킹 완료 · 작업 대기',
  DOCKED: '도킹 완료 · 작업 대기',
  ASSIGNED: '작업자 위치로 이동 중',
  FOLLOWING: '작업자 탐색/동행 중',
  DELIVERING: '배송 중',
  TRANSPORTING: '배송 중',
  WAITING_FOR_UNLOAD: '수취 대기',
  COMPLETED: '작업 완료',
  PAUSED: '일시정지',
  RETURNING: '도킹 위치로 복귀 중',
  CANCELED: '작업 취소됨',
  ERROR: '오류 발생',
};

const EMPTY_STATUS = {
  robot_id: '', online: false, battery: null, position_x: null, position_y: null,
  task_id: '', state: 'DOCKED', previous_state: '', goal_type: '',
  goal_completed: false, destination_id: '', detail: '', error_code: null,
};

function toUiState(status) {
  if (status.state === 'DOCKED') return 'IDLE';
  if (status.state === 'TRANSPORTING' && status.goal_type === 'TO_DESTINATION') {
    return status.goal_completed ? 'WAITING_FOR_UNLOAD' : 'DELIVERING';
  }
  return status.state;
}

function Modal({ title, children, confirmLabel, danger = false, hideCancel = false, onCancel, onConfirm, busy = false }) {
  return <div className="worker-modal-backdrop" role="presentation" onMouseDown={(event) => event.target === event.currentTarget && onCancel()}>
    <section className="worker-modal" role="dialog" aria-modal="true" aria-labelledby="worker-modal-title">
      <button className="worker-modal-close" aria-label="닫기" onClick={onCancel}>×</button>
      <span className={`worker-modal-icon ${danger ? 'danger' : ''}`}>{danger ? 'Ⅱ' : '✓'}</span>
      <h2 id="worker-modal-title">{title}</h2>
      <div className="worker-modal-copy">{children}</div>
      <footer className={hideCancel ? 'single' : ''}>{!hideCancel && <button className="secondary" onClick={onCancel}>취소</button>}<button className={danger ? 'danger' : ''} disabled={busy} onClick={onConfirm}>{busy ? '요청 중…' : confirmLabel}</button></footer>
    </section>
  </div>;
}

function WorkerHeader({ connected }) {
  return <header className="worker-header">
    <div className="worker-brand"><span>AMR</span><div><strong>작업자 웹 화면</strong><small>WORKER ASSIST</small></div></div>
    <div className={`worker-connection ${connected ? '' : 'offline'}`}><i />{connected ? 'HMI 백엔드 연결됨' : 'HMI 백엔드 연결 끊김'}</div>
  </header>;
}

function RobotSummary({ robot, task, connected }) {
  const battery = robot.battery == null ? '—' : `${Math.round(robot.battery)}%`;
  const x = robot.x == null ? '—' : Number(robot.x).toFixed(2);
  const y = robot.y == null ? '—' : Number(robot.y).toFixed(2);
  const online = connected && robot.online;
  return <section className="worker-robot-summary">
    <div><small>ASSIGNED ROBOT</small><h2>{robot.id.toUpperCase()}</h2><p className={online ? '' : 'offline'}><i /> {online ? 'ONLINE' : connected ? 'STATUS LOST' : 'SERVER OFFLINE'}</p></div>
    <dl><div><dt>배터리</dt><dd>{battery}</dd></div><div><dt>현재 위치</dt><dd>X {x} · Y {y}</dd></div><div><dt>현재 상태</dt><dd>{STATE_TEXT[task.state] || task.state}</dd></div></dl>
  </section>;
}

function HomeView({ robot, task, connected, onNavigate, onReturn }) {
  const active = !['IDLE', 'CANCELED', 'COMPLETED'].includes(task.state);
  return <div className="worker-view">
    <header className="worker-view-title"><small>WORKER HOME</small><h1>현재 로봇과 작업을 확인하세요</h1><p>HMI 백엔드에서 수신한 최신 상태를 표시합니다.</p></header>
    <RobotSummary robot={robot} task={task} connected={connected} />
    <button className={`worker-current-task ${active ? 'active' : ''}`} onClick={() => onNavigate('task')}>
      <span><small>CURRENT TASK</small><strong>{active ? `${task.destination?.name || robot.id.toUpperCase()} · ${STATE_TEXT[task.state] || task.state}` : '현재 진행 중인 작업이 없습니다'}</strong><em>{active ? `TASK ${task.id || '-'} · 상세 현황 보기` : '새 작업자 호출을 기다리고 있습니다.'}</em></span><b>›</b>
    </button>
    <div className="worker-menu-grid">
      <button onClick={() => onNavigate('delivery')}><span>＋</span><strong>배송 보내기</strong><small>등록된 목적지 선택</small></button>
      <button onClick={() => onNavigate('task')}><span>▤</span><strong>작업 현황</strong><small>상태·위치·진행 단계</small></button>
      <button className="return-home" disabled={!task.canReturn} onClick={onReturn}><span>↩</span><strong>{task.state === 'RETURNING' ? '복귀 진행 중' : '추종 종료 · 도킹하러 가기'}</strong><small>{task.canReturn ? '현재 작업 종료 후 도킹 위치로 복귀' : '현재 상태에서는 복귀할 수 없습니다'}</small></button>
    </div>
  </div>;
}

function DeliveryView({ destinations, selected, canStart, busy, onSelect, onRequest }) {
  return <div className="worker-view">
    <header className="worker-view-title"><small>NEW DELIVERY</small><h1>어디로 보낼까요?</h1><p>{canStart ? '목적지 구역을 선택하세요.' : '작업자 추종 중(FOLLOWING)에 배송을 시작할 수 있습니다.'}</p></header>
    <div className="worker-destination-grid">{destinations.map((destination) => <button key={destination.id} className={selected?.id === destination.id ? 'selected' : ''} onClick={() => onSelect(destination)}><span>{destination.name}</span><strong>{destination.description}</strong><small>{destination.id} · X {destination.x} / Y {destination.y}</small></button>)}</div>
    {!destinations.length && <section className="worker-empty-destinations">HMI 백엔드에서 등록 목적지를 기다리고 있습니다.</section>}
    <section className="worker-selection">
      <div><small>선택한 목적지</small><strong>{selected ? `${selected.name} · ${selected.description}` : '목적지를 선택하세요'}</strong><span>{selected ? `${selected.id} · (${selected.x}, ${selected.y})` : '선택 전에는 요청할 수 없습니다.'}</span></div>
      <button disabled={!selected || !canStart || busy} onClick={onRequest}>{busy ? '요청 중…' : '이 목적지로 보내기'}</button>
    </section>
  </div>;
}

function TaskView({ robot, task, connected, busy, onPause, onResume, onReturn, onComplete, onNavigate }) {
  if (task.state === 'IDLE') return <div className="worker-empty-task"><span>▤</span><h1>현재 진행 중인 작업이 없습니다</h1><p>작업자 호출이 배정되면 이 화면에 표시됩니다.</p><button onClick={() => onNavigate('delivery')}>배송지 확인</button></div>;
  const displayedState = task.state === 'PAUSED' ? task.previousState : task.state;
  const returning = displayedState === 'RETURNING';
  const currentStep = Math.max(0, STEPS.findIndex(([state]) => state === displayedState));
  const progress = returning ? 50 : [10, 30, 65, 90][currentStep];
  const mode = displayedState === 'ASSIGNED' ? ['→', '작업자에게 이동 중', '전달받은 작업자 위치로 이동하고 있습니다.'] : displayedState === 'FOLLOWING' ? ['◎', '작업자 탐색/동행 중', '작업자를 탐색하고, 감지되면 따라 이동합니다.'] : displayedState === 'DELIVERING' ? ['→', '목적지 배송 중', '선택한 목적지로 물품을 운반하고 있습니다.'] : displayedState === 'WAITING_FOR_UNLOAD' ? ['✓', '수취 확인 대기', '목적지에 도착했습니다. 수취 확인을 눌러 주세요.'] : displayedState === 'RETURNING' ? ['↩', '도킹 위치로 복귀 중', '현재 작업을 종료하고 도킹 위치로 이동하고 있습니다.'] : displayedState === 'ERROR' ? ['!', '오류 발생', task.detail] : ['Ⅱ', STATE_TEXT[task.state] || task.state, task.detail || '현재 작업 상태를 확인하세요.'];
  return <div className="worker-view">
    <header className="worker-view-title task"><small>TASK {task.id || '-'} · {task.rawState}</small><h1>{task.destination?.name || robot.id.toUpperCase()} · {STATE_TEXT[task.state] || task.state}</h1><p>{task.detail || 'HMI 백엔드의 최신 작업 상태를 기준으로 표시합니다.'}</p></header>
    <section className={`worker-mode worker-mode-${String(displayedState).toLowerCase()}`}><span>{mode[0]}</span><div><small>CURRENT MODE</small><strong>{mode[1]}</strong><p>{mode[2]}</p></div></section>
    {returning ? <section className="worker-return-status"><span>↩</span><div><small>RETURN TO DOCK</small><strong>현재 작업을 종료하고 도킹 위치로 복귀하고 있습니다</strong><p>로봇의 이동 경로를 확보하고 도착할 때까지 기다려 주세요.</p></div></section> : <div className="worker-stepper">{STEPS.map(([state, label], index) => <div key={state} className={`${index < currentStep ? 'done' : ''} ${index === currentStep ? 'current' : ''}`}><i>{index < currentStep ? '✓' : index + 1}</i><span>{label}</span></div>)}</div>}
    <div className="worker-task-grid">
      <section><small>실시간 로봇 상태</small><dl><div><dt>현재 위치</dt><dd>{robot.x == null || robot.y == null ? '좌표 수신 대기' : `X ${Number(robot.x).toFixed(2)} · Y ${Number(robot.y).toFixed(2)}`}</dd></div><div><dt>{returning ? '복귀 위치' : '목적지'}</dt><dd>{returning ? '도킹 위치' : task.destination ? `${task.destination.name} (${task.destination.x}, ${task.destination.y})` : '아직 선택하지 않음'}</dd></div><div><dt>목표 유형</dt><dd>{task.goalType || '-'}</dd></div><div><dt>로봇 연결</dt><dd>{connected && robot.online ? '정상' : '확인 필요'}</dd></div></dl></section>
      <section className="worker-progress"><small>작업 진행률</small><strong>{progress}%</strong><div><i style={{ width: `${progress}%` }} /></div><p>현재 state · <b>{task.rawState}</b>{task.errorCode ? <><br />오류 코드 · <b>{task.errorCode}</b></> : null}</p></section>
    </div>
    <section className="worker-next-action">
      <div><small>NEXT ACTION</small><strong>{task.state === 'RETURNING' ? '로봇이 도킹 위치로 이동하고 있습니다' : task.state === 'FOLLOWING' ? '배송 목적지를 선택하거나 작업자 추종을 계속하세요' : task.state === 'DELIVERING' ? '로봇이 목적지로 이동하고 있습니다' : task.state === 'WAITING_FOR_UNLOAD' ? '수령자가 물품을 확인해 주세요' : task.state === 'PAUSED' ? '작업이 멈춰 있습니다' : '현재 로봇 상태를 확인하세요'}</strong></div>
      {task.state === 'FOLLOWING' && <button className="primary" onClick={() => onNavigate('delivery')}>배송 목적지 선택</button>}
      {task.state === 'WAITING_FOR_UNLOAD' && <button className="primary" disabled={busy} onClick={onComplete}>수취 확인 · 배송 완료</button>}
      {task.state === 'PAUSED' && <button className="resume" disabled={busy} onClick={onResume}>▶ 작업 재개</button>}
    </section>
    <div className="worker-task-actions">{task.canPause && <button className="pause" onClick={onPause}>Ⅱ 작업 일시정지</button>}{task.canReturn && <button className="return" onClick={onReturn}>↩ 추종 종료 · 도킹하러 가기</button>}</div>
  </div>;
}

export default function WorkerApp() {
  const configuredRobotId = (import.meta.env.VITE_ROBOT_ID || window.location.pathname.split('/').filter(Boolean).at(-1) || 'robot5').toLowerCase();
  const [tab, setTab] = useState('home');
  const [selectedDestination, setSelectedDestination] = useState(null);
  const [destinations, setDestinations] = useState([]);
  const [status, setStatus] = useState(EMPTY_STATUS);
  const [modal, setModal] = useState(null);
  const [connected, setConnected] = useState(false);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState('');

  const refreshStatus = async () => {
    const payload = await hmiApi.getStatus();
    setStatus(payload); setConnected(true); setError('');
  };

  const refreshDestinations = async () => {
    const payload = await hmiApi.getDestinations();
    setDestinations(payload.map((item) => ({ id: item.destination_id, name: item.destination_name, description: '등록 목적지', x: item.position_x, y: item.position_y })));
  };

  useEffect(() => {
    let active = true;
    const loadStatus = async () => { try { const payload = await hmiApi.getStatus(); if (active) { setStatus(payload); setConnected(true); setError(''); } } catch (requestError) { if (active) { setConnected(false); setError(requestError.message); } } };
    const loadDestinations = async () => { try { const payload = await hmiApi.getDestinations(); if (active) setDestinations(payload.map((item) => ({ id: item.destination_id, name: item.destination_name, description: '등록 목적지', x: item.position_x, y: item.position_y }))); } catch (requestError) { if (active) setError(requestError.message); } };
    loadStatus(); loadDestinations();
    const statusTimer = window.setInterval(loadStatus, 2000);
    const destinationTimer = window.setInterval(loadDestinations, 5000);
    const socket = new WebSocket(hmiApi.statusWebSocketUrl());
    socket.onopen = () => active && setConnected(true);
    socket.onmessage = (event) => { if (!active) return; try { setStatus(JSON.parse(event.data)); setConnected(true); setError(''); } catch { setError('상태 메시지 형식이 올바르지 않습니다.'); } };
    socket.onerror = () => active && setConnected(false);
    socket.onclose = () => active && setConnected(false);
    return () => { active = false; window.clearInterval(statusTimer); window.clearInterval(destinationTimer); socket.close(); };
  }, []);

  const currentDestination = destinations.find((item) => item.id === status.destination_id) || (selectedDestination?.id === status.destination_id ? selectedDestination : null);
  const state = toUiState(status);
  const previousState = status.previous_state === 'TRANSPORTING' ? (status.goal_completed ? 'WAITING_FOR_UNLOAD' : 'DELIVERING') : toUiState({ ...status, state: status.previous_state || 'FOLLOWING' });
  const robot = { id: status.robot_id || configuredRobotId, online: status.online, battery: status.battery, x: status.position_x, y: status.position_y };
  const task = {
    id: status.task_id, state, rawState: status.state, previousState, destination: currentDestination,
    goalType: status.goal_type, detail: status.detail, errorCode: status.error_code,
    canPause: ['ASSIGNED', 'FOLLOWING', 'TRANSPORTING', 'RETURNING'].includes(status.state),
    canReturn: ['ASSIGNED', 'FOLLOWING', 'TRANSPORTING', 'PAUSED'].includes(status.state),
  };
  const activeTab = useMemo(() => tab, [tab]);
  const robotMismatch = status.robot_id && status.robot_id.toLowerCase() !== configuredRobotId;

  const execute = async (action, afterSuccess) => {
    setBusy(true); setError('');
    try {
      await action();
      setModal(null);
      if (afterSuccess) afterSuccess();
      window.setTimeout(() => refreshStatus().catch(() => {}), 150);
    } catch (requestError) {
      setError(requestError.message);
    } finally {
      setBusy(false);
    }
  };

  return <div className="worker-app">
    <WorkerHeader connected={connected} />
    <main>
      {(error || robotMismatch) && <div className="worker-alert">{robotMismatch ? `이 화면은 ${configuredRobotId}, 연결된 백엔드는 ${status.robot_id}입니다.` : error}</div>}
      {activeTab === 'home' && <HomeView robot={robot} task={task} connected={connected} onNavigate={setTab} onReturn={() => setModal('return')} />}
      {activeTab === 'delivery' && <DeliveryView destinations={destinations} selected={selectedDestination} canStart={status.state === 'FOLLOWING'} busy={busy} onSelect={setSelectedDestination} onRequest={() => setModal('destination')} />}
      {activeTab === 'task' && <TaskView robot={robot} task={task} connected={connected} busy={busy} onPause={() => setModal('pause')} onResume={() => execute(hmiApi.resumeTask)} onReturn={() => setModal('return')} onComplete={() => execute(hmiApi.completeDelivery, () => setModal('completed'))} onNavigate={setTab} />}
    </main>
    <nav className="worker-bottom-nav" aria-label="작업자 화면 메뉴">
      <button className={activeTab === 'home' ? 'active' : ''} onClick={() => setTab('home')}><span>⌂</span>홈</button>
      <button className={activeTab === 'delivery' ? 'active' : ''} onClick={() => setTab('delivery')}><span>＋</span>배송 보내기</button>
      <button className={activeTab === 'task' ? 'active' : ''} onClick={() => setTab('task')}><span>▤</span>작업 현황{state !== 'IDLE' && <i />}</button>
    </nav>
    {modal === 'destination' && <Modal title="이 목적지로 배송을 시작할까요?" confirmLabel="배송 시작" busy={busy} onCancel={() => setModal(null)} onConfirm={() => execute(() => hmiApi.startDelivery(selectedDestination.id), () => setTab('task'))}><b>{selectedDestination?.name}</b><br />목표 좌표 ({selectedDestination?.x}, {selectedDestination?.y})</Modal>}
    {modal === 'pause' && <Modal title="현재 작업을 일시정지할까요?" confirmLabel="작업 일시정지" danger busy={busy} onCancel={() => setModal(null)} onConfirm={() => execute(hmiApi.pauseTask)}>중앙 Task Manager에 <b>PAUSE</b> 명령을 보냅니다.<br />이 기능은 물리 비상정지가 아닙니다.</Modal>}
    {modal === 'return' && <Modal title="추종을 종료하고 도킹하러 갈까요?" confirmLabel="도킹하러 가기" busy={busy} onCancel={() => setModal(null)} onConfirm={() => execute(hmiApi.returnToDock, () => setTab('task'))}>현재 진행 중인 작업을 종료하고 도킹 위치로 이동합니다.<br /><b>로봇의 이동 경로를 확인해 주세요.</b></Modal>}
    {modal === 'completed' && <Modal title="배송 완료가 전달되었습니다" confirmLabel="확인" hideCancel onCancel={() => setModal(null)} onConfirm={() => setModal(null)}>중앙 Task Manager에 수취 확인 명령을 전송했습니다.</Modal>}
  </div>;
}
