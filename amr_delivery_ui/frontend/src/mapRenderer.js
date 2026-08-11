const CELL = 5;
const FRAME = { left: 44, top: 20, right: 18, bottom: 34 };

export function worldToCanvas(map, x, y) {
  return {
    x: FRAME.left + ((x - map.origin[0]) / map.resolution) * CELL,
    y: FRAME.top + (map.height - (y - map.origin[1]) / map.resolution) * CELL,
  };
}

export function renderFleetMap(canvas, map, robots, selectedId, destinations = []) {
  canvas.width = FRAME.left + map.width * CELL + FRAME.right;
  canvas.height = FRAME.top + map.height * CELL + FRAME.bottom;
  const ctx = canvas.getContext('2d');
  ctx.fillStyle = '#eef2f4'; ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = '#fbfcfc'; ctx.fillRect(FRAME.left, FRAME.top, map.width * CELL, map.height * CELL);
  for (let y = 0; y < map.height; y++) for (let x = 0; x < map.width; x++) {
    const cell = map.cells[(map.height - 1 - y) * map.width + x];
    if (cell !== 0) {
      ctx.fillStyle = cell === 1 ? '#46535b' : '#d4dadd';
      ctx.fillRect(FRAME.left + x * CELL, FRAME.top + y * CELL, CELL, CELL);
    }
  }
  const meter = Math.max(1, Math.round(1 / map.resolution));
  ctx.strokeStyle = 'rgba(8,127,115,.15)'; ctx.lineWidth = 1;
  for (let x = 0; x <= map.width; x += meter) {
    const px = FRAME.left + x * CELL; ctx.beginPath(); ctx.moveTo(px, FRAME.top); ctx.lineTo(px, FRAME.top + map.height * CELL); ctx.stroke();
  }
  for (let y = 0; y <= map.height; y += meter) {
    const py = FRAME.top + y * CELL; ctx.beginPath(); ctx.moveTo(FRAME.left, py); ctx.lineTo(FRAME.left + map.width * CELL, py); ctx.stroke();
  }
  ctx.fillStyle = '#087f73'; ctx.font = 'bold 10px sans-serif'; ctx.fillText('MAP FRAME · LIVE', FRAME.left, 13);
  ctx.fillStyle = '#70808a'; ctx.font = '9px sans-serif'; ctx.fillText(`origin ${map.origin[0]}, ${map.origin[1]} · ${map.resolution} m/cell`, FRAME.left, canvas.height - 8);
  destinations.forEach((destination) => {
    const worldX = Number(destination.position_x);
    const worldY = Number(destination.position_y);
    if (!Number.isFinite(worldX) || !Number.isFinite(worldY)) return;
    const point = worldToCanvas(map, worldX, worldY);
    if (point.x < FRAME.left || point.x > FRAME.left + map.width * CELL
        || point.y < FRAME.top || point.y > FRAME.top + map.height * CELL) return;

    const name = String(destination.destination_name || destination.destination_id || '목적지');
    ctx.font = 'bold 10px sans-serif';
    const labelWidth = Math.ceil(ctx.measureText(name).width) + 12;
    const labelX = Math.min(point.x + 10, canvas.width - FRAME.right - labelWidth);
    const labelY = Math.max(FRAME.top + 2, point.y - 18);
    ctx.fillStyle = 'rgba(38,31,66,.88)';
    ctx.fillRect(labelX, labelY, labelWidth, 16);
    ctx.fillStyle = '#fff';
    ctx.fillText(name, labelX + 6, labelY + 11);

    ctx.fillStyle = '#7856d8';
    ctx.beginPath();
    ctx.moveTo(point.x, point.y - 8);
    ctx.lineTo(point.x + 7, point.y);
    ctx.lineTo(point.x, point.y + 8);
    ctx.lineTo(point.x - 7, point.y);
    ctx.closePath();
    ctx.fill();
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 2; ctx.stroke();
  });
  robots.forEach((robot) => {
    const point = worldToCanvas(map, robot.pose_x, robot.pose_y);
    const selected = robot.robot_id === selectedId;
    ctx.fillStyle = selected ? '#ef7a22' : '#087f73';
    ctx.beginPath(); ctx.arc(point.x, point.y, selected ? 11 : 9, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(point.x, point.y); ctx.lineTo(point.x + 14, point.y); ctx.stroke();
    ctx.fillStyle = '#17212a'; ctx.font = `bold ${selected ? 11 : 10}px sans-serif`;
    ctx.fillText(`${robot.robot_id.toUpperCase()}  X ${robot.pose_x.toFixed(2)} · Y ${robot.pose_y.toFixed(2)}`, point.x + 16, point.y + 4);
  });
}
