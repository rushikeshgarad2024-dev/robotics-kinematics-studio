const canvas = document.getElementById('robotCanvas');
const ctx = canvas.getContext('2d');

const origin = { x: 150, y: 400 };
const L1 = 160, L2 = 130, L3 = 80;

let state = {
  mode: 'fk',
  t1: 45 * Math.PI / 180,
  t2: -30 * Math.PI / 180,
  t3: 20 * Math.PI / 180,
  target: { x: 250, y: 200 }
};

function solveIK(tx, ty) {
  // Translate to origin coordinates
  const dx = tx - origin.x;
  const dy = origin.y - ty;
  const dist = Math.sqrt(dx * dx + dy * dy);

  if (dist > (L1 + L2 + L3 - 10)) return; // Out of reach

  const targetDist = Math.min(dist, L1 + L2 - 5);
  const cos2 = (targetDist * targetDist - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  const sin2 = Math.sqrt(Math.max(0, 1 - cos2 * cos2));
  state.t2 = -Math.atan2(sin2, cos2);

  const k1 = L1 + L2 * cos2;
  const k2 = L2 * sin2;
  state.t1 = Math.atan2(dy, dx) - Math.atan2(k2, k1);
  state.t3 = - (state.t1 + state.t2) * 0.5;
}

function render() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // Draw Grid & Workspace Limit
  ctx.strokeStyle = '#1e293b';
  ctx.lineWidth = 1;
  for (let x = 0; x < canvas.width; x += 40) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, canvas.height); ctx.stroke(); }
  for (let y = 0; y < canvas.height; y += 40) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(canvas.width, y); ctx.stroke(); }

  // Draw Workspace Boundary Arc
  ctx.strokeStyle = 'rgba(168, 85, 247, 0.2)';
  ctx.lineWidth = 2;
  ctx.setLineDash([6, 6]);
  ctx.beginPath();
  ctx.arc(origin.x, origin.y, L1 + L2 + L3, 0, Math.PI * 2);
  ctx.stroke();
  ctx.setLineDash([]);

  // Joint calculations
  const j1 = { x: origin.x, y: origin.y };
  const a1 = -state.t1;
  const j2 = { x: j1.x + L1 * Math.cos(a1), y: j1.y + L1 * Math.sin(a1) };

  const a2 = a1 - state.t2;
  const j3 = { x: j2.x + L2 * Math.cos(a2), y: j2.y + L2 * Math.sin(a2) };

  const a3 = a2 - state.t3;
  const ee = { x: j3.x + L3 * Math.cos(a3), y: j3.y + L3 * Math.sin(a3) };

  // Draw Links
  // Link 1
  ctx.strokeStyle = '#3b82f6';
  ctx.lineWidth = 10;
  ctx.beginPath(); ctx.moveTo(j1.x, j1.y); ctx.lineTo(j2.x, j2.y); ctx.stroke();

  // Link 2
  ctx.strokeStyle = '#a855f7';
  ctx.lineWidth = 8;
  ctx.beginPath(); ctx.moveTo(j2.x, j2.y); ctx.lineTo(j3.x, j3.y); ctx.stroke();

  // Link 3 (End Effector)
  ctx.strokeStyle = '#ec4899';
  ctx.lineWidth = 6;
  ctx.beginPath(); ctx.moveTo(j3.x, j3.y); ctx.lineTo(ee.x, ee.y); ctx.stroke();

  // Draw Joints
  [j1, j2, j3, ee].forEach((j, idx) => {
    ctx.fillStyle = idx === 3 ? '#fbbf24' : '#fff';
    ctx.strokeStyle = '#0f172a';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(j.x, j.y, idx === 3 ? 6 : 9, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  });

  // Draw Base Platform
  ctx.fillStyle = '#334155';
  ctx.fillRect(origin.x - 30, origin.y, 60, 20);

  // Update telemetry
  const posX = (ee.x - origin.x).toFixed(1);
  const posY = (origin.y - ee.y).toFixed(1);
  document.getElementById('eePos').textContent = `(${posX}, ${posY}) mm`;
  document.getElementById('eeRadius').textContent = `${Math.sqrt(posX*posX + posY*posY).toFixed(1)} mm`;
}

// Event handlers
document.getElementById('theta1Range').addEventListener('input', e => {
  state.t1 = +e.target.value * Math.PI / 180;
  document.getElementById('theta1Val').textContent = `${e.target.value}°`;
  render();
});
document.getElementById('theta2Range').addEventListener('input', e => {
  state.t2 = +e.target.value * Math.PI / 180;
  document.getElementById('theta2Val').textContent = `${e.target.value}°`;
  render();
});
document.getElementById('theta3Range').addEventListener('input', e => {
  state.t3 = +e.target.value * Math.PI / 180;
  document.getElementById('theta3Val').textContent = `${e.target.value}°`;
  render();
});

document.getElementById('modeSelect').addEventListener('change', e => {
  state.mode = e.target.value;
  document.getElementById('fkControls').style.display = state.mode === 'fk' ? 'block' : 'none';
  document.getElementById('ikControls').style.display = state.mode === 'ik' ? 'block' : 'none';
});

canvas.addEventListener('click', e => {
  if (state.mode === 'ik') {
    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    solveIK(x, y);
    render();
  }
});

render();
