import { useEffect, useRef, useState } from 'react';
import './App.css';
import { robotApi } from './api/robotApi';
import { renderFleetMap, worldToCanvas } from './mapRenderer';

// DB의 최종 상태(COMPLETED/CANCELED)도 HMI에서 한글로 표시한다.
const LABELS = { AVAILABLE: '대기', IDLE: '대기', ASSIGNED: '작업자에게 이동', FOLLOWING: '작업자 추종', TRANSPORTING: '배송 중', RETURNING: '복귀 중', PAUSED: '일시정지', DOCKED: '도킹 완료', COMPLETED: '작업 완료', CANCELED: '작업 취소', ERROR: '오류' };
const ACTIVE = new Set(['ASSIGNED', 'FOLLOWING', 'TRANSPORTING', 'RETURNING', 'PAUSED']);
const TASK_LABELS = { ASSIGNED: '작업자에게 이동', FOLLOWING: '작업자 추종', TRANSPORTING: '배송 중', RETURNING: '복귀 중', PAUSED: '일시정지', DOCKED: '완료', COMPLETED: '완료', CANCELED: '취소', ERROR: '오류' };
// 도킹·언도킹·텔레옵 버튼을 열어 주는 유일한 작업 상태 목록
const MANUAL_CONTROL = new Set(['CANCELED', 'ERROR']);

function LoginScreen({ onLogin }) {
  const [username, setUsername] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [submitting, setSubmitting] = useState(false);
  const submit = async (event) => {
    event.preventDefault();
    setSubmitting(true);
    setError('');
    try {
      const result = await robotApi.login(username, password);
      sessionStorage.setItem('hmi_auth_token', result.token);
      onLogin();
    } catch {
      setError('아이디 또는 비밀번호가 올바르지 않습니다.');
    } finally {
      setSubmitting(false);
    }
  };
  return <main className="login-screen"><form className="login-card" onSubmit={submit}><div className="login-brand"><span>AMR</span><div><h1>중앙 관리 제어</h1><small>SECURE OPERATOR LOGIN</small></div></div><label>아이디<input autoFocus autoComplete="username" value={username} onChange={(event) => setUsername(event.target.value)} placeholder="아이디 입력" /></label><label>비밀번호<input type="password" autoComplete="current-password" value={password} onChange={(event) => setPassword(event.target.value)} placeholder="비밀번호 입력" /></label>{error && <p className="login-error">{error}</p>}<button disabled={submitting || !username || !password}>{submitting ? '인증 중…' : '로그인'}</button><small className="login-help">인가된 관리자만 접근할 수 있습니다.</small></form></main>;
}

function taskInfo(robot) {
  const task = robot.current_task;
  const state = task?.state || task?.status;
  const active = ACTIVE.has(state || robot.mode);
  return { task, state, active };
}

function FleetMap({ robots, selectedId, onSelect }) {
  const canvasRef = useRef(null);
  const [map, setMap] = useState(null);
  useEffect(() => {
    let alive = true;
    const refreshMap = () => robotApi.getMap()
      .then((data) => alive && setMap(data))
      .catch(() => alive && setMap(null));
    refreshMap();
    const timer = setInterval(refreshMap, 3000);
    return () => { alive = false; clearInterval(timer); };
  }, []);
  useEffect(() => { if (map) renderFleetMap(canvasRef.current, map, robots, selectedId); }, [map, robots, selectedId]);
  const click = (event) => {
    if (!map) return;
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const x = (event.clientX - rect.left) * canvas.width / rect.width;
    const y = (event.clientY - rect.top) * canvas.height / rect.height;
    const robot = robots.find((item) => { const point = worldToCanvas(map, item.pose_x, item.pose_y); return Math.hypot(point.x - x, point.y - y) < 24; });
    if (robot) onSelect(robot.robot_id);
  };
  return <section className="map-workspace"><header><div><strong>내비게이션 지도</strong><small>MAP FRAME · LIVE</small></div><div className="map-tabs">{robots.map((robot) => <button key={robot.robot_id} className={selectedId === robot.robot_id ? 'active' : ''} onClick={() => onSelect(robot.robot_id)}>{robot.robot_id.toUpperCase()}</button>)}</div></header><div className="map-stage">{map ? <canvas ref={canvasRef} onClick={click} aria-label="AMR 실시간 위치 지도" /> : <span>운영 지도를 연결하는 중입니다.</span>}</div><footer><i /> 지도와 로봇 좌표는 DB 최신 상태를 기준으로 갱신됩니다.</footer></section>;
}

function LaptopCameraTile() {
  const videoRef = useRef(null);
  const [cameraState, setCameraState] = useState('loading');
  const [message, setMessage] = useState('카메라 권한을 요청하고 있습니다.');
  const [devices, setDevices] = useState([]);
  const [selectedDevice, setSelectedDevice] = useState('');
  useEffect(() => {
    let stream;
    let alive = true;
    const connect = async () => {
      if (!navigator.mediaDevices?.getUserMedia) {
        setCameraState('error');
        setMessage(window.isSecureContext ? '이 브라우저에서는 카메라 기능을 지원하지 않습니다.' : '카메라는 localhost 또는 HTTPS 주소에서만 사용할 수 있습니다. http://localhost:5173으로 접속해 주세요.');
        return;
      }
      try {
        setCameraState('loading');
        setMessage('카메라 장치를 연결하고 있습니다.');
        stream = await navigator.mediaDevices.getUserMedia({
          video: {
            width: { ideal: 1280 },
            height: { ideal: 720 },
            ...(selectedDevice ? { deviceId: { exact: selectedDevice } } : { facingMode: 'user' }),
          },
          audio: false,
        });
        if (!alive) { stream.getTracks().forEach((track) => track.stop()); return; }
        const cameras = (await navigator.mediaDevices.enumerateDevices()).filter((device) => device.kind === 'videoinput');
        setDevices(cameras);
        const activeDevice = stream.getVideoTracks()[0]?.getSettings().deviceId;
        // DroidCam/가상 카메라 대신 실제 내장·USB 웹캠을 우선 사용한다.
        if (!selectedDevice) {
          const virtualCamera = /droid|virtual|loopback|obs/i;
          const physicalCamera = cameras.find((camera) => !virtualCamera.test(camera.label));
          const preferredDevice = physicalCamera?.deviceId || activeDevice || cameras[0]?.deviceId;
          if (preferredDevice) setSelectedDevice(preferredDevice);
        }
        videoRef.current.srcObject = stream;
        await videoRef.current.play();
        setCameraState('live');
      } catch (error) {
        if (!alive) return;
        setCameraState('error');
        const errors = {
          NotAllowedError: '카메라 권한이 거부되었습니다. 브라우저 주소창에서 카메라 권한을 허용해 주세요.',
          NotFoundError: '사용 가능한 웹캠을 찾지 못했습니다. OS에서 카메라 장치를 확인해 주세요.',
          NotReadableError: '웹캠이 다른 프로그램에서 사용 중이거나 장치를 열 수 없습니다.',
          OverconstrainedError: '선택한 카메라를 현재 설정으로 열 수 없습니다.',
        };
        setMessage(errors[error.name] || `카메라 연결 실패: ${error.message}`);
      }
    };
    connect();
    return () => {
      alive = false;
      stream?.getTracks().forEach((track) => track.stop());
    };
  }, [selectedDevice]);
  const visibleDevices = devices.filter((device) => !/droid|virtual|loopback|obs/i.test(device.label));
  return <article className="video-tile"><header><div><strong>노트북 웹캠</strong><small>LOCAL CAMERA · 실시간 화면</small></div><span className={cameraState === 'live' ? '' : 'waiting'}><i />{cameraState === 'live' ? 'LIVE' : 'WAIT'}</span></header><div><video ref={videoRef} autoPlay muted playsInline style={{ display: cameraState === 'error' ? 'none' : 'block', width: '100%', height: '100%', objectFit: 'cover' }} />{cameraState !== 'live' && <span><b>{cameraState === 'loading' ? '카메라 연결 중' : '카메라를 연결할 수 없습니다'}</b><small>{message}</small></span>}</div><footer>{visibleDevices.length > 0 ? <label>실제 카메라 선택 <select value={selectedDevice} onChange={(event) => setSelectedDevice(event.target.value)}>{visibleDevices.map((device, index) => <option value={device.deviceId} key={device.deviceId}>CAM {index} · {device.label || `웹캠 ${index}`}</option>)}</select></label> : '실제 웹캠을 찾지 못했습니다. DroidCam/가상 카메라는 목록에서 제외됩니다.'}</footer></article>;
}

function VisionWorkspace() {
  return <section className="vision-workspace"><header><div><strong>웹캠 영상 관제</strong><small>LOCAL CAMERA · LIVE VIEW</small></div><span><i />1 CHANNEL</span></header><div className="video-grid webcam-only"><LaptopCameraTile /></div><footer><i /> 현재 HMI를 실행한 노트북의 웹캠 화면입니다.</footer></section>;
}

function DatabaseWorkspace() {
  const [tables, setTables] = useState([]);
  const [selectedTable, setSelectedTable] = useState('robot_status_logs');
  const [tableData, setTableData] = useState({ columns: [], rows: [] });
  const [error, setError] = useState('');
  useEffect(() => {
    let alive = true;
    const refreshTables = () => robotApi.getDatabaseTables().then((data) => {
      if (!alive) return;
      setTables(data);
      if (data.length && !data.some((table) => table.name === selectedTable)) setSelectedTable(data[0].name);
    }).catch(() => alive && setError('DB 테이블 목록을 불러오지 못했습니다.'));
    refreshTables();
    const timer = setInterval(refreshTables, 3000);
    return () => { alive = false; clearInterval(timer); };
  }, [selectedTable]);
  useEffect(() => {
    let alive = true;
    const refreshRows = () => robotApi.getDatabaseTable(selectedTable).then((data) => {
      if (!alive) return;
      setTableData(data);
      setError('');
    }).catch(() => alive && setError(`${selectedTable} 데이터를 불러오지 못했습니다.`));
    refreshRows();
    const timer = setInterval(refreshRows, 1500);
    return () => { alive = false; clearInterval(timer); };
  }, [selectedTable]);
  return <section className="database-workspace"><header><div><strong>데이터베이스 관제</strong><small>AMR.DB · READ ONLY · LIVE</small></div><span><i />{tableData.rows.length} ROWS</span></header><nav>{tables.map((table) => <button key={table.name} className={selectedTable === table.name ? 'active' : ''} onClick={() => setSelectedTable(table.name)}><strong>{table.name}</strong><small>{table.count}건</small></button>)}</nav><div className="database-table-wrap">{error ? <div className="database-empty">{error}</div> : <table><thead><tr>{tableData.columns.map((column) => <th key={column}>{column}</th>)}</tr></thead><tbody>{tableData.rows.map((row, index) => <tr key={`${selectedTable}-${index}`}>{tableData.columns.map((column) => <td key={column}>{row[column] == null ? <span className="db-null">NULL</span> : String(row[column])}</td>)}</tr>)}</tbody></table>}{!error && tableData.rows.length === 0 && <div className="database-empty">저장된 데이터가 없습니다.</div>}</div><footer><i /> 최근 데이터 최대 100건을 읽기 전용으로 표시합니다.</footer></section>;
}

function RobotPanel({ robot, selected, onSelect }) {
  const { task, state, active } = taskInfo(robot);
  const batteryLevel = robot.battery <= 20 ? 'battery-critical' : robot.battery <= 40 ? 'battery-warning' : '';
  return <button className={`rail-robot ${selected ? 'selected' : ''} ${batteryLevel} ${robot.estopped ? 'paused' : ''}`} onClick={() => onSelect(robot.robot_id)}>
    <div className="rail-robot-head"><div><i /><strong>{robot.robot_id.toUpperCase()}</strong></div><span>{robot.estopped ? '일시정지' : LABELS[robot.mode] || robot.mode}</span></div>
    <div className="rail-task"><small>현재 작업</small><strong>{active ? task ? `TASK ${task.task_id}` : LABELS[robot.mode] : '자동 배정 대기'}</strong><span>{active ? TASK_LABELS[state] || state || LABELS[robot.mode] : '할당된 작업 없음'}</span></div>
    <div className="rail-metrics"><span><small>BATTERY</small><b>{Math.round(robot.battery)}%</b></span><span><small>POSITION</small><b>{robot.pose_x.toFixed(2)}, {robot.pose_y.toFixed(2)}</b></span></div>
    <div className="mini-battery"><i style={{ width: `${robot.battery}%` }} /></div>
  </button>;
}

export default function App() {
  const [authenticated, setAuthenticated] = useState(() => !!sessionStorage.getItem('hmi_auth_token'));
  const [robots, setRobots] = useState([]);
  const [selectedId, setSelectedId] = useState(null);
  const [connected, setConnected] = useState(false);
  const [theme, setTheme] = useState(() => localStorage.getItem('theme') || 'light');
  const [busy, setBusy] = useState(false);
  const [notice, setNotice] = useState('');
  const [view, setView] = useState('map');
  // null이면 비활성화, robot_id가 있으면 해당 로봇의 키보드 텔레옵이 활성화된 상태다.
  const [teleopRobotId, setTeleopRobotId] = useState(null);
  useEffect(() => {
    const expire = () => setAuthenticated(false);
    window.addEventListener('hmi-auth-expired', expire);
    return () => window.removeEventListener('hmi-auth-expired', expire);
  }, []);
  useEffect(() => { document.documentElement.dataset.theme = theme; localStorage.setItem('theme', theme); }, [theme]);
  useEffect(() => {
    if (!authenticated) return undefined;
    let alive = true;
    const refresh = async () => { try { const data = await robotApi.getRobots(); if (!alive) return; setRobots(data); setConnected(true); setSelectedId((id) => id || data[0]?.robot_id || null); } catch { if (alive) setConnected(false); } };
    refresh(); const timer = setInterval(refresh, 1000); return () => { alive = false; clearInterval(timer); };
  }, [authenticated]);
  const selected = robots.find((robot) => robot.robot_id === selectedId);
  const manualReady = !!selected && MANUAL_CONTROL.has(selected.mode);

  // 텔레옵 활성화 중에는 방향키 조합을 100 ms마다 cmd_vel로 갱신한다.
  useEffect(() => {
    if (!teleopRobotId) return undefined;
    const pressed = new Set();
    const directions = new Set(['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight']);
    const send = () => {
      const linear = (pressed.has('ArrowUp') ? 0.18 : 0) + (pressed.has('ArrowDown') ? -0.15 : 0);
      const angular = (pressed.has('ArrowLeft') ? 0.8 : 0) + (pressed.has('ArrowRight') ? -0.8 : 0);
      robotApi.teleop(linear, angular, teleopRobotId).catch(() => {});
    };
    const keydown = (event) => { if (directions.has(event.key)) { event.preventDefault(); pressed.add(event.key); send(); } };
    const keyup = (event) => { if (directions.has(event.key)) { event.preventDefault(); pressed.delete(event.key); send(); } };
    const stop = () => { pressed.clear(); send(); };
    const timer = window.setInterval(() => pressed.size && send(), 100);
    window.addEventListener('keydown', keydown);
    window.addEventListener('keyup', keyup);
    window.addEventListener('blur', stop);
    return () => {
      window.clearInterval(timer);
      window.removeEventListener('keydown', keydown);
      window.removeEventListener('keyup', keyup);
      window.removeEventListener('blur', stop);
      robotApi.teleop(0, 0, teleopRobotId).catch(() => {});
    };
  }, [teleopRobotId]);

  // 선택 로봇이 바뀌거나 허용 상태를 벗어나면 텔레옵을 자동으로 해제한다.
  useEffect(() => {
    if (!teleopRobotId || (selectedId === teleopRobotId && manualReady)) return;
    robotApi.setTeleopMode(false, teleopRobotId).catch(() => {});
    setTeleopRobotId(null);
  }, [manualReady, selectedId, teleopRobotId]);

  if (!authenticated) return <LoginScreen onLogin={() => setAuthenticated(true)} />;
  const execute = async (action, message) => { setBusy(true); try { await action(); setNotice(message); } catch (error) { setNotice(`실행 실패 · ${error.message}`); } finally { setBusy(false); setTimeout(() => setNotice(''), 2300); } };
  const logout = () => {
    if (teleopRobotId) robotApi.setTeleopMode(false, teleopRobotId).catch(() => {});
    setTeleopRobotId(null);
    sessionStorage.removeItem('hmi_auth_token');
    setAuthenticated(false);
    setRobots([]);
    setSelectedId(null);
  };
  const selectedTask = selected ? taskInfo(selected) : null;
  const teleopActive = teleopRobotId === selectedId;
  const dockStatusKnown = !!selected?.dock_status_known;
  // 토글 API가 성공한 뒤에만 화면의 활성화 상태를 변경한다.
  const toggleTeleop = async () => {
    if (!selected) return;
    const next = !teleopActive;
    await execute(() => robotApi.setTeleopMode(next, selected.robot_id).then(() => setTeleopRobotId(next ? selected.robot_id : null)), next ? '텔레옵 활성화 · 방향키로 조종하세요' : '텔레옵 비활성화');
  };

  return <div className="operations-app">
    <header className="app-header"><div className="title"><span>AMR</span><div><h1>중앙 관리 제어</h1><small>관리자 대시보드</small></div></div><div className="view-toggle"><button className={view === 'map' ? 'active' : ''} onClick={() => setView('map')}>지도 관제</button><button className={view === 'video' ? 'active' : ''} onClick={() => setView('video')}>영상 관제</button><button className={view === 'database' ? 'active' : ''} onClick={() => setView('database')}>DB 관제</button></div><div className="global-status"><span className={connected ? '' : 'offline'}><i />{connected ? 'DB · GATEWAY 연결' : '연결 확인 중'}</span><b>{robots.filter((robot) => !robot.estopped).length}/{robots.length} 운용 가능</b><button onClick={() => setTheme(theme === 'light' ? 'dark' : 'light')}>{theme === 'light' ? '라이트' : '다크'}</button><button className="logout-button" onClick={logout}>로그아웃</button></div></header>

    <main className="dashboard-layout">
      <aside className="robot-rail"><header><strong>로봇 현황</strong><small>자동 배정 시스템</small></header><div>{robots.map((robot) => <RobotPanel key={robot.robot_id} robot={robot} selected={selectedId === robot.robot_id} onSelect={setSelectedId} />)}</div><footer><i /> 로봇을 선택하면 우측 제어가 전환됩니다.</footer></aside>

      {view === 'map' ? <FleetMap robots={robots} selectedId={selectedId} onSelect={setSelectedId} /> : view === 'video' ? <VisionWorkspace /> : <DatabaseWorkspace />}

      <aside className="control-rail">
        {selected ? <>
          <section className="selected-unit"><div><small>SELECTED UNIT</small><h2>{selected.robot_id.toUpperCase()}</h2><p>{LABELS[selected.mode] || selected.mode} · X {selected.pose_x.toFixed(2)}, Y {selected.pose_y.toFixed(2)}</p></div><span className={manualReady ? 'ready' : ''}>{selectedTask.active ? '작업 수행 중' : manualReady ? '수동 제어 가능' : selected.mode === 'DOCKED' ? '도킹 상태' : '수동 제어 잠금'}</span></section>

          <section className="quick-actions"><header><strong>작업·안전 제어</strong></header><div><button className="cancel" disabled={busy || !selectedTask.active} onClick={() => window.confirm(`${selected.robot_id.toUpperCase()} 작업을 취소할까요?`) && execute(() => robotApi.cancelTask(selected.robot_id), '작업 취소 요청 완료')}>작업 취소</button><button className={`pause ${selected.estopped ? 'resume' : ''}`} disabled={busy} onClick={() => execute(() => robotApi.setEstop(!selected.estopped, selected.robot_id), selected.estopped ? '운행 재개 요청 완료' : '일시정지 요청 완료')}>{selected.estopped ? '운행 재개' : '일시정지'}</button></div></section>

          {/* 수동 도킹 버튼은 CANCELED/ERROR 상태에서만 활성화한다. */}
          <section className="dock-box"><header><strong>도킹 제어</strong><small>{dockStatusKnown ? 'Create3 실제 상태' : '도킹 상태 수신 대기'}</small></header><div><button disabled={busy || !manualReady || !dockStatusKnown || selected.docked} onClick={() => execute(() => robotApi.setDock(true, selected.robot_id), '도킹 요청 완료')}>도킹</button><button disabled={busy || !manualReady || !dockStatusKnown || !selected.docked} onClick={() => execute(() => robotApi.setDock(false, selected.robot_id), '언도킹 요청 완료')}>언도킹</button></div></section>

          {/* 기존 방향 버튼 대신 키보드 텔레옵을 켜고 끄는 단일 토글만 제공한다. */}
          <section className="teleop-box"><header><div><strong>키보드 텔레옵</strong><small>{teleopActive ? '방향키 ↑ ↓ ← → 로 조종' : '취소·오류 상태에서만 활성화'}</small></div><span className={teleopActive ? 'ready' : ''}>{teleopActive ? 'ACTIVE' : manualReady ? 'READY' : 'LOCKED'}</span></header><button className={`teleop-toggle ${teleopActive ? 'active' : ''}`} disabled={busy || (!manualReady && !teleopActive)} onClick={toggleTeleop}>{teleopActive ? '텔레옵 비활성화' : '텔레옵 활성화'}</button></section>

          <section className="task-queue"><header><strong>현재 작업</strong><small>{robots.filter((robot) => taskInfo(robot).active).length}건</small></header>{robots.map((robot) => { const info = taskInfo(robot); return <div key={robot.robot_id}><i className={info.active ? '' : 'idle'} /><span><strong>{robot.robot_id.toUpperCase()}</strong><small>{info.active ? TASK_LABELS[info.state] || info.state || LABELS[robot.mode] : '배정 대기'}</small></span><b>{info.active ? '진행' : '대기'}</b></div>; })}</section>
        </> : <div className="empty">로봇 상태를 불러오는 중입니다.</div>}
      </aside>
    </main>
    {notice && <div className="toast">{notice}</div>}
  </div>;
}
