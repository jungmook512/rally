// 훈련 단계 설명 그림 — 위에서 내려다본 책상 (위 = 로봇 쪽, 아래 = 환자 쪽). 위치는 설명용이며 로봇 측정값이 아니다.
// renderTrainingScene(mode, stageIndex, status, opts) → SVG 문자열. 단계마다 팔 자세·손 모양·물체·화살표가 달라진다.
const SCENE_STAGES = {
  1: ['손 시작 위치 확인', '공 보내기', '받기 관측', '손 이탈 대기', '공 회수', '시도 기록'],
  3: ['준비', '던지기·받기', '결과 기록'],
  5: ['손·물체 확인', '물체 집기', '물체 건네기', '물체 돌려받기', '시도 기록'],
  7: ['손 확인', '저항 훈련', '시도 기록'],
};
const SCENE_STATUS = {paused:'휴식 중', ended:'세션 종료', stopped:'안전 정지'};
const sceneEscape = value => String(value).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));

const TOP_DEFS = `<defs>
  <filter id="t-sh" x="-30%" y="-30%" width="160%" height="160%"><feDropShadow dx="3" dy="6" stdDeviation="5" flood-color="#2b2418" flood-opacity=".28"/></filter>
  <filter id="t-sh2" x="-30%" y="-30%" width="160%" height="160%"><feDropShadow dx="1.5" dy="3" stdDeviation="2.5" flood-color="#2b2418" flood-opacity=".3"/></filter>
  <linearGradient id="t-wood" x1="0" y1="0" x2="1" y2="1"><stop stop-color="#f3f5f1"/><stop offset="1" stop-color="#dde4dd"/></linearGradient>
  <linearGradient id="t-white" x2="0" y2="1"><stop stop-color="#c9d2d8"/><stop offset=".25" stop-color="#ffffff"/><stop offset=".6" stop-color="#eef2f4"/><stop offset="1" stop-color="#b8c3ca"/></linearGradient>
  <radialGradient id="t-joint" cx=".38" cy=".35" r=".75"><stop stop-color="#ffffff"/><stop offset=".6" stop-color="#e6ebee"/><stop offset="1" stop-color="#aab6be"/></radialGradient>
  <radialGradient id="t-cap" cx=".4" cy=".35" r=".8"><stop stop-color="#7d8892"/><stop offset="1" stop-color="#3e474f"/></radialGradient>
  <radialGradient id="t-base" cx=".45" cy=".4" r=".7"><stop stop-color="#6c7680"/><stop offset="1" stop-color="#353d44"/></radialGradient>
  <linearGradient id="t-metal" x2="0" y2="1"><stop stop-color="#8e979f"/><stop offset=".45" stop-color="#d9dee2"/><stop offset="1" stop-color="#7b858d"/></linearGradient>
  <radialGradient id="t-ball" cx=".35" cy=".32" r=".75"><stop stop-color="#fff7cf"/><stop offset=".35" stop-color="#f4cd4f"/><stop offset="1" stop-color="#c08f18"/></radialGradient>
  <linearGradient id="t-skin" gradientUnits="userSpaceOnUse" x1="-32" y1="0" x2="32" y2="0"><stop stop-color="#e2aa84"/><stop offset=".4" stop-color="#f5d3b6"/><stop offset=".65" stop-color="#f1c9a8"/><stop offset="1" stop-color="#dba07a"/></linearGradient>
  <radialGradient id="t-palm" gradientUnits="userSpaceOnUse" cx="0" cy="8" r="34"><stop stop-color="#f2ccb1"/><stop offset=".55" stop-color="#f7dac6"/><stop offset="1" stop-color="#efc6a6"/></radialGradient>
  <linearGradient id="t-pad" x2="0" y2="1"><stop stop-color="#9cc3ae"/><stop offset="1" stop-color="#6f9c86"/></linearGradient>
  <linearGradient id="t-rod" x2="1" y2="0"><stop stop-color="#b98b57"/><stop offset=".45" stop-color="#e2bf8a"/><stop offset="1" stop-color="#a87a48"/></linearGradient>
  <linearGradient id="t-curl" x2="0" y2="1"><stop stop-color="#9a6446" stop-opacity=".45"/><stop offset="1" stop-color="#9a6446" stop-opacity="0"/></linearGradient>
  <linearGradient id="t-sleeve" x2="0" y2="1"><stop stop-color="#4f6f88"/><stop offset=".45" stop-color="#7896ad"/><stop offset="1" stop-color="#4a6880"/></linearGradient>
  <radialGradient id="t-shirt" cx=".5" cy=".2" r=".9"><stop stop-color="#87a3b8"/><stop offset="1" stop-color="#4c6a82"/></radialGradient>
  <radialGradient id="t-hair" cx=".45" cy=".4" r=".7"><stop stop-color="#5a4636"/><stop offset="1" stop-color="#271e17"/></radialGradient>
  <linearGradient id="t-rail" x2="1" y2="0"><stop stop-color="#b9c3c9"/><stop offset=".5" stop-color="#f4f7f8"/><stop offset="1" stop-color="#a7b2b9"/></linearGradient>
  <radialGradient id="t-view" cx=".5" cy="0" r="1"><stop stop-color="#6fa8d6" stop-opacity=".35"/><stop offset="1" stop-color="#6fa8d6" stop-opacity=".05"/></radialGradient>
</defs>`;
const r1 = v => v.toFixed(1);
function tCapsule([x1, y1], [x2, y2], w, fill, extra = '') {
  // 둥근 막대 (팔·링크). 폭 방향으로 음영이 지도록 회전한 rect 로
  const len = Math.hypot(x2 - x1, y2 - y1), ang = Math.atan2(y2 - y1, x2 - x1) * 180 / Math.PI;
  return `<g transform="translate(${r1(x1)} ${r1(y1)}) rotate(${r1(ang)})"><rect x="${-w / 2}" y="${-w / 2}" width="${r1(len + w)}" height="${w}" rx="${w / 2}" fill="${fill}" ${extra}/></g>`;
}
function topTable() {
  const grain = [60, 120, 175, 240, 300].map((y, i) => `<path d="M160 ${y}Q480 ${y + (i % 2 ? 8 : -8)} 800 ${y}" stroke="#c7d1c8" stroke-width="1" opacity=".35" fill="none"/>`).join('');
  return `<rect width="960" height="410" fill="#f8faf7"/>
    <rect x="150" y="22" width="660" height="336" rx="10" fill="url(#t-wood)" filter="url(#t-sh)"/>${grain}`;
}
// 오른손 손등 윤곽 한 줄 (손목 왼쪽 → 엄지 → 검지·중지·약지·새끼 → 손날 → 팔뚝). 손목 = (0, 40), 손가락 = 위
const HAND_PATH = 'M-18 40C-20 32-22 26-25 20C-33 12-41 1-45-9C-47-15-42-20-37-15C-33-9-28-3-24 1'
  + 'C-25-12-24-30-23-46C-22-54-13-55-12-49C-12-38-12-24-11.5-12'
  + 'C-11.5-28-11-45-10-55C-9-62 0-62 0.5-55C1-44 1.5-28 1.5-11'
  + 'C2-26 2.5-40 3-49C4-56 12.5-56 13-49C13.5-38 13.5-24 13.8-8'
  + 'C14-18 14.5-28 15.5-35C16.5-42 23.5-42 24-35C24.5-23 25.5-10 25.5 0'
  + 'C25.5 14 22 28 19 40C21 80 24 140 26 210H-26C-24 140-21 80-18 40Z';
// 손 모양 3가지 (모두 오른손, 손목 = (0, 40), 손가락 = 위)
//   back: 손등 (쉬거나 비켜 있을 때) · palm: 손바닥이 위, 살짝 오므림 (공 받기·들기) · grip: 손등 쪽에서 본, 막대를 감싸 쥔 손 (막대는 y = -30 을 가로지름)
function topHand([x, y], ang = -10, {pose = 'back', fade = false, holding = '', under = ''} = {}) {
  let body;
  if (pose === 'palm') {
    // 손등 윤곽을 좌우로 뒤집고(엄지가 오른쪽) 손가락을 조금 짧게 — 오므린 손바닥
    body = `<g transform="scale(-1 .9)"><path d="${HAND_PATH}" fill="url(#t-palm)" stroke="#b97e5c" stroke-opacity=".5" stroke-width="1.2" stroke-linejoin="round" filter="url(#t-sh)"/>
      <path d="M-22-4C-10 0 6-2 22-9M-21 7C-8 5 6 9 15 16M-15 0C-21 10-21 24-14 36" stroke="#c98a68" stroke-width="1.3" fill="none" opacity=".55"/>
      <path d="M-21-24h7M-21-36h7M-9-28h8M-9-42h8M3-25h8M3-38h8M15.5-16h7M15.5-27h7" stroke="#c98a68" stroke-width="1" opacity=".45"/>
      <ellipse cx="0" cy="8" rx="17" ry="15" fill="#a0603c" opacity=".03"/></g>`;
  } else if (pose === 'grip') {
    // 막대를 감싸 쥔 손: 손등 윤곽에서 손가락 끝을 잘라 막대 너머로 말려 들어간 것처럼 (끝으로 갈수록 어둡게). 엄지는 막대 아래라 안 보임
    const cut = `grip-${Math.round(x)}-${Math.round(y)}`;
    // 손가락마다 끝을 둥글게 (말려 내려가는 마디), 엄지 쪽 손바닥 가장자리는 곡선으로
    const tips = [[-17.5, -36, 6], [-5, -38, 6.2], [7.5, -37, 6], [19.6, -32, 5.2]].map(([cx, cy, r]) => `<circle cx="${cx}" cy="${cy}" r="${r}"/>`).join('');
    body = `<clipPath id="${cut}"><path d="M-25 220V36C-26 24-26 10-24-2V-36H26V220Z"/>${tips}</clipPath>
      <g clip-path="url(#${cut})"><path d="${HAND_PATH}" fill="url(#t-skin)" stroke="#b97e5c" stroke-opacity=".55" stroke-width="1.2" stroke-linejoin="round"/>
        <rect x="-27" y="-45" width="54" height="16" fill="url(#t-curl)"/></g>
      <g fill="none" stroke="#b97e5c" stroke-opacity=".55" stroke-width="1.2">${[[-17.5, -36, 6], [-5, -38, 6.2], [7.5, -37, 6], [19.6, -32, 5.2]]
        .map(([cx, cy, r]) => `<path d="M${cx - r} ${cy}A${r} ${r} 0 0 1 ${cx + r} ${cy}"/>`).join('')}<path d="M-25 36C-26 24-26 10-24-2"/></g>
      <path d="M-20-4q3 2 6 0M-8-10q3 2 6 0M4-9q3 2 6 0M15.5-3q3 2 5.5 0" stroke="#c98d6b" stroke-width="1.1" fill="none" opacity=".55"/>
      <path d="M-16 4L-9 30M-5 0L-2 30M6 1L4 30M17 6L9 30" stroke="#d9a685" stroke-width="1.4" opacity=".25"/>`;
  } else {
    const nails = [[-17.5, -47, -2], [-4.8, -55, 0], [8, -49, 2], [19.7, -35, 4]]
      .map(([nx, ny, r]) => `<rect x="${nx - 3.6}" y="${ny}" width="7.2" height="8" rx="3.4" transform="rotate(${r} ${nx} ${ny})" fill="#f9e4d8" opacity=".45"/>`).join('');
    body = `<path d="${HAND_PATH}" fill="url(#t-skin)" stroke="#b97e5c" stroke-opacity=".55" stroke-width="1.2" stroke-linejoin="round" filter="url(#t-sh)"/>${nails}
      <path d="M-20-4q3 2 6 0M-8-10q3 2 6 0M4-9q3 2 6 0M15.5-3q3 2 5.5 0" stroke="#c98d6b" stroke-width="1.1" fill="none" opacity=".55"/>
      <path d="M-17-30q3 1.5 5 0M-6-36q3 1.5 5 0M5-32q3 1.5 5 0M16-22q2.5 1.2 5 0" stroke="#c98d6b" stroke-width=".9" fill="none" opacity=".45"/>
      <path d="M-16 4L-9 30M-5 0L-2 30M6 1L4 30M17 6L9 30" stroke="#d9a685" stroke-width="1.4" opacity=".25"/>`;
  }
  return `<g transform="translate(${x} ${y}) rotate(${ang})" opacity="${fade ? .4 : 1}" class="hand hand-${pose}">${under}${body}${holding}</g>`;
}
// 그리퍼가 든 가로 막대(손목 기준 y = 24, 기울기 tilt°) 위 x = at 지점을 손이 쥔다 (손가락이 막대와 직각)
function topRodGrip([wx, wy], tilt, at) {
  const t = tilt * Math.PI / 180, c = Math.cos(t), sn = Math.sin(t);
  const [px, py] = [wx + at * c - 24 * sn, wy + at * sn + 24 * c];   // 손목 좌표 (at, 24) 를 기울여 옮긴 점
  return topHand([px - 30 * sn, py + 30 * c], tilt, {pose: 'grip'});  // 손 기준 (0, -30) 이 그 점에 오게
}
const topHeldRod = (from, to, tilt = 0) => `<g transform="rotate(${tilt} 0 24)"><rect x="${from}" y="16" width="${to - from}" height="16" rx="7" fill="url(#t-rod)" stroke="#8a6438" stroke-width="1" filter="url(#t-sh2)"/></g>`;
function topPatient(hand, opts = {}) {
  // 사람은 그리지 않고 손만 (10/12)
  return `<g class="patient">${topHand(hand, opts.ang ?? -10, opts)}</g>`;
}
function topArm([wx, wy], {grip = 'open', tool = ''} = {}) {
  const L = 118, [sx, sy] = [480, 64];
  let dx = wx - sx, dy = wy - sy, d = Math.hypot(dx, dy);
  if (d > 2 * L - 2) { const k = (2 * L - 2) / d; dx *= k; dy *= k; wx = sx + dx; wy = sy + dy; d = 2 * L - 2; }
  const a = Math.atan2(dy, dx), off = Math.acos(Math.min(1, d / (2 * L)));
  const e = [sx + L * Math.cos(a + off), sy + L * Math.sin(a + off)];
  const toolAng = 0;   // 그리퍼·손목 카메라는 환자 쪽(아래)을 본다
  const jaw = grip === 'closed' ? 6 : 12;
  const joint = ([x, y], r) => `<circle cx="${r1(x)}" cy="${r1(y)}" r="${r}" fill="url(#t-joint)" stroke="#9aa7b0" stroke-width="1"/>
    <circle cx="${r1(x)}" cy="${r1(y)}" r="${r * .52}" fill="url(#t-cap)"/>`;
  return `<g class="robot" filter="url(#t-sh)">
    <circle cx="${sx}" cy="${sy}" r="42" fill="url(#t-base)"/>
    ${[0, 90, 180, 270].map(t => `<circle cx="${r1(sx + 35 * Math.cos(t * Math.PI / 180 + .78))}" cy="${r1(sy + 35 * Math.sin(t * Math.PI / 180 + .78))}" r="2.6" fill="#9ca6ae"/>`).join('')}
    ${tCapsule([sx, sy], e, 36, 'url(#t-white)')}${tCapsule(e, [wx, wy], 30, 'url(#t-white)')}
    ${joint([sx, sy], 27)}${joint(e, 22)}
    <g transform="translate(${r1(wx)} ${r1(wy)}) rotate(${r1(toolAng)})">
      <rect x="-20" y="-6" width="40" height="22" rx="5" fill="url(#t-metal)" stroke="#5f6870" stroke-width="1"/>
      <rect x="-15" y="-15" width="30" height="9" rx="3" fill="#1f262b"/><circle cx="-8" cy="-10.5" r="2.4" fill="#4c7ea6"/><circle cx="8" cy="-10.5" r="2.4" fill="#4c7ea6"/>
      <rect x="${-jaw - 4}" y="14" width="8" height="20" rx="2.5" fill="url(#t-metal)" stroke="#5f6870" stroke-width=".8"/>
      <rect x="${jaw - 4}" y="14" width="8" height="20" rx="2.5" fill="url(#t-metal)" stroke="#5f6870" stroke-width=".8"/>${tool}</g>
    ${joint([wx, wy], 16)}</g>`;
}
const topBall = ([x, y], cls = '') => `<g class="${cls}"><circle cx="${x}" cy="${y}" r="13" fill="url(#t-ball)" filter="url(#t-sh2)"/>
  <path d="M${x - 9} ${y + 2}Q${x} ${y - 6} ${x + 9} ${y + 3}" stroke="#d9a92a" stroke-width="1.4" fill="none" opacity=".8"/></g>`;
const topArrow = (d, cls = '') => `<path d="${d}" class="flow ${cls}" fill="none" stroke="#e08a0b" stroke-width="3.5" stroke-linecap="round" stroke-dasharray="8 7" marker-end="url(#scene-arrow)" opacity=".9"/>`;
function topNote([x, y], text, anchor = 'start') {
  const w = text.length * 14 + 22, x0 = anchor === 'middle' ? x - w / 2 : anchor === 'end' ? x - w : x;
  return `<g filter="url(#t-sh2)"><rect x="${x0}" y="${y - 19}" width="${w}" height="28" rx="14" fill="#fffdf8"/></g>
    <text x="${x0 + w / 2}" y="${y}" text-anchor="middle" class="note">${sceneEscape(text)}</text>`;
}
const topRing = ([x, y], r = 34) => `<circle cx="${x}" cy="${y}" r="${r}" fill="#f2a93b" fill-opacity=".12" stroke="#e08a0b" stroke-width="2.5" stroke-dasharray="6 6"/>`;
const topView = ([x, y], [tx, ty], w = 50) => `<path d="M${x - 10} ${y + 14}L${x + 10} ${y + 14}L${tx + w} ${ty}L${tx - w} ${ty}Z" fill="url(#t-view)"/>`;
const topRecord = `<g transform="translate(716 210)"><circle r="25" fill="#e5f4ea" stroke="#2f7d5b" stroke-width="2" filter="url(#t-sh2)"/>
  <path d="M-10 1L-3 8L11-8" fill="none" stroke="#2f7d5b" stroke-width="4" stroke-linecap="round" stroke-linejoin="round"/>
  <text y="46" text-anchor="middle" class="note">기록</text></g>`;
function topRack(n) {
  return `<g filter="url(#t-sh2)"><rect x="660" y="44" width="108" height="36" rx="8" fill="#9fb3a8"/><rect x="666" y="50" width="96" height="24" rx="6" fill="#7f978a"/></g>
    ${Array.from({length: n}, (_, i) => topBall([690 + i * 28, 62])).join('')}<text x="714" y="100" text-anchor="middle" class="note small">거치대</text>`;
}
function topLanes() {
  // 레인 2개 (10/12 #142: 레인 3 은 모드 3 패드 거치대 자리라 제거). 공은 가운데 x 480 레인으로, 거치대 쪽이 레인 1
  return [480, 630].map(x => `<rect x="${x - 26}" y="126" width="52" height="222" fill="#d9e3d6" opacity=".7"/>
    <g filter="url(#t-sh2)"><rect x="${x - 30}" y="126" width="7" height="222" rx="3" fill="url(#t-rail)"/><rect x="${x + 23}" y="126" width="7" height="222" rx="3" fill="url(#t-rail)"/></g>`).join('');
}

// 레인 번호 → 그림 x. 0 = 거치대 쪽 레인(630), 1 = 가운데(480) — table_layout 과 같은 순서(레인 1 이 거치대 가까운 쪽). 모르면 가운데
const LANE_X = {0: 630, 1: 480};
function topMode1(stage, lane = null) {
  // 공이 굴러오는 쪽 손은 손바닥이 위 (받을 준비). 회수부터는 손을 옆으로 빼서 손등
  const L = Object.hasOwn(LANE_X, lane) ? LANE_X[lane] : 480;
  const right = L >= 600;                                  // 거치대 쪽 레인이면 메모를 왼쪽에 (화면 밖으로 안 나가게)
  const note = ([dx, y], text) => right ? topNote([L - dx, y], text, 'end') : topNote([L + dx, y], text);
  const palm = {pose: 'palm'}, handAt = [L, 330], view = [L, 152], away = [690, 336];
  const s = {arm: [560, 124], back: topLanes() + topRack(2), patient: topPatient(handAt, palm)};
  if (stage === 0) Object.assign(s, {arm: view, back: topLanes() + topRack(2) + topRing([L, 314], 44) + topView(view, [L, 318]),
    front: note([60, 290], '손바닥을 위로, 레인 끝에')});
  if (stage === 1) Object.assign(s, {arm: [L, 120], back: topLanes() + topRack(1),
    front: topArrow(`M${L} 162V250`) + topBall([L, 272], 'ball-roll') + note([26, 214], '공을 굴려 보냄')});
  if (stage === 2) Object.assign(s, {arm: view, back: topLanes() + topRack(1) + topView(view, [L, 318]),
    front: topBall([L + 1, 318]) + note([60, 250], '손목 카메라로 받기 확인')});
  if (stage === 3) Object.assign(s, {arm: view, back: topLanes() + topRack(1) + topView(view, [L, 306]), patient: topPatient(away),
    front: topBall([L, 296]) + topArrow(`M${L + 35} 318L640 328`) + note([right ? 30 : 120, 286], '손을 빼면 회수 시작')});
  if (stage === 4) Object.assign(s, {arm: [L, 236], grip: 'closed', back: topLanes() + topRack(1), patient: topPatient(away, {fade: true}),
    front: topBall([L, 266]) + topArrow(`M${L + 28} 226Q${Math.round((L + 686) / 2) + 20} 160 686 92`, 'flow-run') + note([80, 214], '공을 거치대로')});
  if (stage === 5) Object.assign(s, {arm: [560, 124], back: topLanes() + topRack(2), patient: topPatient(away, {fade: true}), front: topRecord});
  return s;
}

function topMode3(stage) {
  // 패드(벨크로)를 든 로봇. 던지기 전 손은 손바닥 위에 공을 올림 (아래로 던질 준비), 던진 뒤엔 빈 손바닥
  const pad = '<rect x="-46" y="16" width="92" height="44" rx="10" fill="url(#t-pad)" stroke="#4f7a66" stroke-width="1.5"/>'
    + '<path d="M-38 27H38M-38 38H38M-38 49H38" stroke="#5f8b75" stroke-width="1.2" opacity=".6"/>';
  const ready = [560, 150], moved = [620, 182], hand = [420, 330];
  const s = {arm: ready, grip: 'closed', tool: pad, patient: topPatient(hand, {pose: 'palm', ang: 8, holding: topBall([0, 8])})};
  if (stage === 0) s.front = topNote([470, 270], '공을 손바닥에 올리고 대기');
  if (stage === 1) Object.assign(s, {arm: moved,
    back: `<rect x="514" y="166" width="92" height="44" rx="10" fill="none" stroke="#6f9c86" stroke-width="2" stroke-dasharray="6 5"/>`,
    patient: topPatient(hand, {pose: 'palm', ang: 8}),
    front: `<path d="M426 318Q470 210 612 214" fill="none" stroke="#c9a347" stroke-width="2.5" stroke-dasharray="4 6"/>` + topBall([612, 214], 'ball-throw')
      + topArrow('M566 196L598 204') + topNote([700, 290], '패드가 예측 지점으로', 'middle')});
  if (stage === 2) Object.assign(s, {arm: moved, tool: pad + topBall([0, 38]), patient: topPatient(hand, {pose: 'palm', ang: 8}),
    front: topRecord + topNote([640, 340], '측면 카메라로 패드 위 공 확인', 'middle')});
  return s;
}
function topMode5(stage) {
  // 막대: 책상 위 → 로봇이 집음 → 기울여 건넴(환자가 막대 끝을 감싸 쥠) → 환자가 막대 끝을 그리퍼 사이로 넣어 돌려줌
  const onTable = `<g transform="translate(430 236)"><rect x="-56" y="-8" width="112" height="16" rx="7" fill="url(#t-rod)" stroke="#8a6438" stroke-width="1" filter="url(#t-sh2)"/></g>`;
  const s = {arm: [560, 124], back: onTable, patient: topPatient([600, 336])};
  if (stage === 0) Object.assign(s, {arm: [470, 150], back: topView([470, 150], [430, 236], 70) + onTable + topRing([430, 236], 62),
    front: topNote([520, 300], '물체와 손 위치 확인')});
  if (stage === 1) Object.assign(s, {arm: [430, 196], grip: 'closed', tool: topHeldRod(-56, 56), back: '',
    front: topArrow('M350 262L350 228', 'flow-run') + topNote([340, 296], '물체를 집어 듦', 'end')});
  if (stage === 2) Object.assign(s, {arm: [440, 200], grip: 'closed', tool: topHeldRod(-24, 124, -15), back: '', patient: '',
    front: topRodGrip([440, 200], -15, 98) + topNote([600, 320], '기울여 건넴 → 막대 끝을 쥠', 'middle')});
  if (stage === 3) Object.assign(s, {arm: [440, 200], grip: 'open', tool: topHeldRod(-18, 124), back: '', patient: '',
    front: topRodGrip([440, 200], 0, 98) + topArrow('M640 266L580 266', 'flow-run') + topNote([620, 320], '막대 끝을 그리퍼 사이로', 'middle')});
  if (stage === 4) Object.assign(s, {front: topRecord, patient: topPatient([600, 336], {fade: true})});
  return s;
}
function topMode7(stage) {
  // 팔씨름형: 로봇이 든 손잡이를 환자가 감싸 쥐고 민다. 로봇은 버티다 물러날 뿐 환자 쪽으로 밀지 않음
  const handle = topHeldRod(-46, 46);
  const s = {arm: [480, 206], grip: 'closed', tool: handle, patient: topPatient([480, 344], {ang: 0})};
  if (stage === 0) Object.assign(s, {back: topView([480, 206], [480, 330], 46), front: topRing([480, 318], 44) + topNote([540, 304], '손이 보이는지 확인')});
  if (stage === 1) Object.assign(s, {patient: '', front: topRodGrip([480, 206], 0, 0) + topArrow('M570 330L570 262', 'flow-run') + topNote([590, 300], '환자가 밀기')
    + `<path d="M390 232L390 192" stroke="#8aa0ad" stroke-width="4" stroke-linecap="round" stroke-dasharray="6 6" marker-end="url(#scene-arrow-grey)"/>` + topNote([378, 214], '로봇은 버티다 물러남', 'end')});
  if (stage === 2) Object.assign(s, {patient: topPatient([480, 344], {ang: 0, fade: true}), front: topRecord});
  return s;
}
const TOP_BUILDERS = {1: topMode1, 3: topMode3, 5: topMode5, 7: topMode7};

function renderTrainingScene(mode, stageIndex, status, opts = {}) {
  if (!Object.hasOwn(SCENE_STAGES, mode)) return '';
  const stages = SCENE_STAGES[mode];
  const preview = !!opts.preview;
  const active = (preview || ['running', 'pause_pending'].includes(status)) && stageIndex >= 0 && stageIndex < stages.length;
  const stage = active ? stageIndex : -1;
  // 움직임(애니메이션)은 실행 중인 알려진 단계에서만. 정지·휴식·끊김이면 멈춘 그림
  const motion = active && ((mode === 1 && [1, 4].includes(stage)) || (mode === 3 && stage === 1) || (mode === 5 && [1, 3].includes(stage)) || (mode === 7 && stage === 1));
  const s = TOP_BUILDERS[mode](stage, opts.lane ?? null);   // 모드 1: brain 이 고른 레인(0·1)으로 공을 굴린다
  const body = `${topTable()}${s.back || ''}${s.patient || ''}${topArm(s.arm, {grip: s.grip, tool: s.tool || ''})}${s.front || ''}`;
  // 그림 위 표시는 미리보기·정지·휴식일 때만 (진행 중 단계 이름은 진행 트랙·현재 단계 칸에 이미 있음)
  const stepName = active ? `${stage + 1}. ${stages[stage]}` : '';
  const label = preview ? stepName : active ? '' : (SCENE_STATUS[status] || '');
  const tone = status === 'stopped' && !preview ? '#a71d28' : preview ? '#2b6f6a' : '#1f5752';
  const pill = label ? `<g transform="translate(100 0)"><rect x="20" y="16" width="${40 + label.length * 16 + (preview ? 64 : 0)}" height="36" rx="18" fill="#fff" stroke="${tone}" stroke-width="2"/>
    <text x="36" y="40" class="pill" style="fill:${tone}">${preview ? '미리보기 · ' : ''}${sceneEscape(label)}</text></g>` : '';
  const frame = status === 'stopped' && !preview ? '<rect x="113" y="3" width="734" height="404" rx="10" fill="none" stroke="#a71d28" stroke-width="6"/>' : '';
  const veil = !active ? '<rect width="960" height="410" fill="#f8faf7" opacity=".35"/>' : '';
  return `<svg xmlns="http://www.w3.org/2000/svg" viewBox="110 0 740 410" role="img" aria-label="${sceneEscape(stepName || label || stages.join(' → '))} · 위에서 본 설명 그림" data-scene-mode="${mode}" data-scene-stage="${stage}" data-scene-status="${sceneEscape(status)}" class="${motion ? 'scene-moving' : 'scene-still'}">
    <style>
      text{font-family:system-ui,'Noto Sans KR',sans-serif;font-weight:650}.pill{font-size:17px}.note{font-size:15px;fill:#3a4a55}.note.small{font-size:13px;fill:#6c6352}
      .scene-moving .flow{animation:scene-flow 1s linear infinite}
      .scene-moving .ball-roll{animation:scene-roll 2.4s ease-in infinite}
      .scene-moving .ball-throw{animation:scene-throw 2.4s ease-out infinite}
      @keyframes scene-flow{to{stroke-dashoffset:-18}}
      @keyframes scene-roll{0%{transform:translateY(-130px);opacity:0}10%{opacity:1}75%,100%{transform:none}}
      @keyframes scene-throw{0%{transform:translate(-186px,104px);opacity:0}10%{opacity:1}50%{transform:translate(-100px,10px) scale(1.4)}85%,100%{transform:none}}
      .ball-throw{transform-box:fill-box;transform-origin:center}
      @media(prefers-reduced-motion:reduce){.flow,.ball-roll,.ball-throw{animation:none!important}}
    </style>
    <defs><marker id="scene-arrow" viewBox="0 0 10 10" refX="7" refY="5" markerWidth="4" markerHeight="4" orient="auto-start-reverse"><path d="M1 1L9 5L1 9Z" fill="#e08a0b"/></marker>
      <marker id="scene-arrow-grey" viewBox="0 0 10 10" refX="7" refY="5" markerWidth="4" markerHeight="4" orient="auto-start-reverse"><path d="M1 1L9 5L1 9Z" fill="#8aa0ad"/></marker></defs>
    ${TOP_DEFS}${body}${veil}${frame}${pill}
  </svg>`;
}
