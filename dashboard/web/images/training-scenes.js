// 훈련 단계 설명 그림 (환자 자리에서 로봇을 바라본 모습). 위치는 설명용이며 로봇 측정값이 아니다.
// renderTrainingScene(mode, stageIndex, status, opts) → SVG 문자열. 단계마다 팔 자세·손·물체·화살표가 달라진다.
const SCENE_STAGES = {
  1: ['손 시작 위치 확인', '공 보내기', '받기 관측', '손 이탈 대기', '공 회수', '시도 기록'],
  3: ['준비', '던지기·받기', '결과 기록'],
  5: ['손·물체 확인', '물체 집기', '물체 건네기', '물체 돌려받기', '시도 기록'],
  7: ['손 확인', '저항 훈련', '시도 기록'],
};
const SCENE_STATUS = {paused:'휴식 중', ended:'세션 종료', stopped:'안전 정지'};
const SHOULDER = [480, 92];

// 원근: u = 좌우(-1~1), v = 먼 쪽 0 ~ 가까운 쪽 1
const sceneAt = (u, v) => [480 + u * (180 + 250 * v), 150 + 232 * v];
const sceneEscape = value => String(value).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));

function sceneArm([wx, wy], tool = '', grip = 'open') {
  // 어깨 → 팔꿈치 → 손목 2관절. 팔꿈치는 위쪽 해
  const L = 122, [sx, sy] = SHOULDER;
  let dx = wx - sx, dy = wy - sy, d = Math.hypot(dx, dy);
  if (d > 2 * L - 4) { const k = (2 * L - 4) / d; dx *= k; dy *= k; wx = sx + dx; wy = sy + dy; d = 2 * L - 4; }
  const base = Math.atan2(dy, dx), off = Math.acos(Math.min(1, d / (2 * L)));
  const e1 = [sx + L * Math.cos(base - off), sy + L * Math.sin(base - off)];
  const e2 = [sx + L * Math.cos(base + off), sy + L * Math.sin(base + off)];
  const [ex, ey] = e1[1] < e2[1] ? e1 : e2;
  const link = (x1, y1, x2, y2) => `<path d="M${x1} ${y1}L${x2} ${y2}" stroke="#93a6b2" stroke-width="27" stroke-linecap="round"/>
    <path d="M${x1} ${y1}L${x2} ${y2}" stroke="#eef2f4" stroke-width="21" stroke-linecap="round"/>`;
  const joint = (x, y, r) => `<circle cx="${x}" cy="${y}" r="${r}" fill="#41698a"/><circle cx="${x}" cy="${y}" r="${r * .45}" fill="#eef2f4"/>`;
  const fingers = grip === 'closed' ? 'M-9 14V30M9 14V30' : 'M-15 14L-17 30M15 14L17 30';
  return `<g class="robot">
    <path d="M444 150H516L528 166H432Z" fill="#a9b8c1" stroke="#7d909c"/>
    <rect x="458" y="102" width="44" height="50" rx="8" fill="#e2e8eb" stroke="#93a6b2" stroke-width="2"/>
    ${link(sx, sy, ex, ey)}${link(ex, ey, wx, wy)}
    ${joint(sx, sy, 16)}${joint(ex, ey, 14)}${joint(wx, wy, 12)}
    <g transform="translate(${wx.toFixed(1)} ${wy.toFixed(1)})">
      <rect x="-17" y="-26" width="34" height="11" rx="3" fill="#2f4352"/><circle cx="-8" cy="-20.5" r="3" fill="#7fb0cf"/><circle cx="8" cy="-20.5" r="3" fill="#7fb0cf"/>
      <rect x="-19" y="2" width="38" height="13" rx="3" fill="#3a4f5e"/>
      <path d="${fingers}" stroke="#2f4352" stroke-width="6" stroke-linecap="round"/>
      ${tool}
    </g></g>`;
}
function sceneHand([x, y], {fade = false, scale = 1, rotate = 0, holding = ''} = {}) {
  // 환자 오른손을 위에서 본 모습 (손가락이 로봇 쪽). 팔뚝은 화면 아래로
  return `<g transform="translate(${x} ${y}) rotate(${rotate}) scale(${scale})" opacity="${fade ? .38 : 1}" class="hand">
    <path d="M-20 26L-26 140H26L20 26Z" fill="#efc7a4" stroke="#c99472" stroke-width="2"/>
    <rect x="-25" y="-16" width="50" height="46" rx="18" fill="#f4d2b3" stroke="#c99472" stroke-width="2"/>
    ${[-17, -6, 5, 16].map((fx, i) => `<rect x="${fx - 4.5}" y="${-38 + Math.abs(i - 1.5) * 4}" width="9" height="28" rx="4.5" fill="#f4d2b3" stroke="#c99472" stroke-width="1.6"/>`).join('')}
    <rect x="-37" y="0" width="10" height="24" rx="5" transform="rotate(-28 -32 12)" fill="#f4d2b3" stroke="#c99472" stroke-width="1.6"/>
    ${holding}
  </g>`;
}
const sceneBall = ([x, y], r = 13, cls = '') => `<g class="${cls}"><ellipse cx="${x}" cy="${y + r * .9}" rx="${r}" ry="${r * .32}" fill="#5b5a4a" opacity=".18"/>
  <circle cx="${x}" cy="${y}" r="${r}" fill="#e5bb4f" stroke="#a8862f" stroke-width="1.6"/><path d="M${x - r * .45} ${y - r * .55}Q${x} ${y - r * .85} ${x + r * .4} ${y - r * .5}" fill="none" stroke="#fbeec4" stroke-width="2"/></g>`;
const sceneArrow = (d, cls = '') => `<path d="${d}" class="flow ${cls}" fill="none" stroke="#d48a12" stroke-width="4" stroke-linecap="round" stroke-dasharray="10 8" marker-end="url(#scene-arrow)"/>`;
const sceneNote = ([x, y], text, anchor = 'start') => `<text x="${x}" y="${y}" text-anchor="${anchor}" class="note">${sceneEscape(text)}</text>`;
const sceneCone = ([cx, cy], [tx, ty], w = 70) => `<path d="M${cx - 6} ${cy - 18}L${cx + 6} ${cy - 18}L${tx + w} ${ty}L${tx - w} ${ty}Z" fill="#7fb0cf" opacity=".16"/>`;
const sceneRecord = `<g transform="translate(800 120)"><circle r="30" fill="#e2f2e8" stroke="#3f8a67" stroke-width="3"/><path d="M-12 1L-3 10L14 -9" fill="none" stroke="#3f8a67" stroke-width="5" stroke-linecap="round" stroke-linejoin="round"/>
  <text y="56" text-anchor="middle" class="note">DB 에 기록</text></g>`;
const sceneRing = ([x, y], r = 46) => `<ellipse cx="${x}" cy="${y}" rx="${r}" ry="${r * .62}" fill="none" stroke="#d48a12" stroke-width="3" stroke-dasharray="7 6"/>`;

function sceneTable() {
  const [a, b, c, d] = [sceneAt(-1, 0), sceneAt(1, 0), sceneAt(1, 1.1), sceneAt(-1, 1.1)];
  return `<rect width="960" height="410" fill="#f3f6f4"/><path d="M0 150H960" stroke="#dfe6e2"/>
    <path d="M${a}L${b}L${c}L${d}Z" fill="#ece5d6" stroke="#cbbfa6" stroke-width="1.5"/>`;
}
function sceneLanes(active) {
  return [-0.5, 0, 0.5].map(u => {
    const p = (du, v) => sceneAt(u + du, v);
    const strip = `M${p(-.11, .06)}L${p(.11, .06)}L${p(.11, 1.02)}L${p(-.11, 1.02)}Z`;
    const rails = `M${p(-.11, .06)}L${p(-.11, 1.02)}M${p(.11, .06)}L${p(.11, 1.02)}`;
    return `<path d="${strip}" fill="${u === 0 && active ? '#dcebe2' : '#e6eee6'}"/><path d="${rails}" stroke="#93ad9c" stroke-width="3"/>`;
  }).join('');
}
function sceneRack(balls) {
  return `<g><rect x="250" y="146" width="96" height="26" rx="6" fill="#d9cfbb" stroke="#a99c80" stroke-width="1.5"/>
    ${Array.from({length: balls}, (_, i) => sceneBall([272 + i * 26, 150], 10)).join('')}
    <text x="298" y="194" text-anchor="middle" class="note small">거치대</text></g>`;
}

// 모드별 단계 그림: arm = 손목 위치, grip, tool(손목에 붙는 것), back(로봇 뒤에 그릴 것), front(앞)
function sceneMode1(stage) {
  const lane = v => sceneAt(0, v);
  const hand0 = [480, 352], view = [480, 168];
  const s = {arm: [560, 160], grip: 'open', back: sceneRack(2), front: sceneHand(hand0)};
  if (stage === 0) Object.assign(s, {arm: view, back: sceneRack(2) + sceneCone(view, [480, 340]) + sceneRing([480, 345], 62),
    front: sceneHand(hand0) + sceneNote([560, 330], '손을 레인 끝에')});
  if (stage === 1) Object.assign(s, {arm: lane(.1), back: sceneRack(1), front: sceneHand(hand0)
    + sceneArrow(`M${lane(.2)}L${lane(.68)}`) + sceneBall(lane(.78), 13, 'ball-roll') + sceneNote([540, 250], '공을 굴려 보냄')});
  if (stage === 2) Object.assign(s, {arm: view, back: sceneRack(1) + sceneCone(view, [480, 330]),
    front: sceneBall(lane(.8), 13) + sceneHand([480, 342]) + sceneNote([560, 300], '손목 카메라로 받기 확인')});
  if (stage === 3) Object.assign(s, {arm: view, back: sceneRack(1) + sceneCone(view, [480, 330]),
    front: sceneBall(lane(.8), 13) + sceneHand([700, 360], {fade: true}) + sceneArrow('M520 352L640 360') + sceneNote([600, 316], '손을 빼면 회수 시작', 'middle')});
  if (stage === 4) Object.assign(s, {arm: [480, 236], grip: 'closed', back: sceneRack(1),
    front: sceneBall([480, 262], 12) + sceneArrow('M455 230Q380 190 340 180', 'flow-run') + sceneHand([700, 360], {fade: true}) + sceneNote([380, 250], '공을 거치대로', 'end')});
  if (stage === 5) Object.assign(s, {arm: [560, 160], back: sceneRack(2), front: sceneHand([700, 360], {fade: true}) + sceneRecord});
  return s;
}
function sceneMode3(stage) {
  const pad = '<rect x="-38" y="30" width="76" height="54" rx="12" fill="#8fb3a0" stroke="#4f7a66" stroke-width="3"/><path d="M-26 44H26M-26 57H26M-26 70H26" stroke="#6f9a85" stroke-width="2"/>';
  const ballOnPad = sceneBall([0, 57], 12);
  const s = {arm: [560, 160], tool: pad, front: sceneHand([400, 352], {holding: sceneBall([0, -30], 13)})};
  if (stage === 1) Object.assign(s, {arm: [612, 178], back: `<rect x="522" y="190" width="76" height="54" rx="12" fill="none" stroke="#8fb3a0" stroke-width="2" stroke-dasharray="6 5"/>`,
    front: sceneHand([400, 352]) + `<path d="M400 312Q470 150 600 230" fill="none" stroke="#c9a347" stroke-width="2.5" stroke-dasharray="4 6"/>`
      + sceneBall([600, 232], 12, 'ball-throw') + sceneArrow('M570 218L596 230') + sceneNote([660, 300], '패드가 예측 지점으로', 'middle')});
  if (stage === 2) Object.assign(s, {arm: [612, 178], tool: pad + ballOnPad, front: sceneHand([400, 352]) + sceneRecord + sceneNote([700, 300], '패드 위 공을 측면 카메라로 확인', 'middle')});
  return s;
}
function sceneMode5(stage) {
  const rod = (rot = 0) => `<g transform="rotate(${rot})"><rect x="-9" y="22" width="18" height="70" rx="8" fill="#c9a76a" stroke="#977a44" stroke-width="2"/></g>`;
  const rodOnTable = `<g transform="translate(480 262) rotate(90)">${rod()}</g>`;
  const s = {arm: [560, 160], back: rodOnTable, front: sceneHand([600, 360])};
  if (stage === 0) Object.assign(s, {arm: [520, 160], back: sceneCone([520, 160], [480, 262], 80) + rodOnTable + sceneRing([480, 262], 70),
    front: sceneHand([600, 360]) + sceneNote([580, 240], '물체·손 위치 확인')});
  if (stage === 1) Object.assign(s, {arm: [480, 196], grip: 'closed', tool: rod(), back: '', front: sceneHand([600, 360])
    + sceneArrow('M520 290L520 240', 'flow-run') + sceneNote([540, 300], '물체를 집어 듦')});
  if (stage === 2) Object.assign(s, {arm: [470, 230], grip: 'closed', tool: rod(-25), back: '',
    front: sceneHand([530, 352], {rotate: -12}) + sceneArrow('M500 300L520 318') + sceneNote([600, 260], '기울여 건넴 → 손으로 잡기')});
  if (stage === 3) Object.assign(s, {arm: [480, 200], grip: 'open', back: '',
    front: sceneHand([490, 330], {holding: `<g transform="translate(0 -60) rotate(-10)">${rod()}</g>`}) + sceneArrow('M540 300L520 236', 'flow-run') + sceneNote([560, 270], '막대를 그리퍼 사이로')});
  if (stage === 4) Object.assign(s, {arm: [560, 160], front: sceneHand([600, 360], {fade: true}) + sceneRecord});
  return s;
}
function sceneMode7(stage) {
  const handle = '<rect x="-10" y="16" width="20" height="58" rx="9" fill="#b98a5a" stroke="#86613a" stroke-width="2"/>';
  const s = {arm: [480, 214], grip: 'closed', tool: handle, front: sceneHand([480, 360], {fade: stage === 2})};
  if (stage === 0) Object.assign(s, {back: sceneCone([480, 214], [480, 330], 60), front: sceneHand([480, 345]) + sceneRing([480, 330], 58) + sceneNote([560, 330], '손이 보이는지 확인')});
  if (stage === 1) Object.assign(s, {front: sceneHand([480, 318]) + sceneArrow('M560 330L560 262', 'flow-run') + sceneNote([580, 300], '환자가 밀기')
    + `<path d="M400 232L400 196" stroke="#8aa0ad" stroke-width="4" stroke-linecap="round" stroke-dasharray="6 6" marker-end="url(#scene-arrow-grey)"/>` + sceneNote([388, 214], '로봇은 버티다 물러남', 'end')});
  if (stage === 2) s.front += sceneRecord;
  return s;
}
const SCENE_BUILDERS = {1: sceneMode1, 3: sceneMode3, 5: sceneMode5, 7: sceneMode7};

function renderTrainingScene(mode, stageIndex, status, opts = {}) {
  if (!Object.hasOwn(SCENE_STAGES, mode)) return '';
  const stages = SCENE_STAGES[mode];
  const preview = !!opts.preview;
  const active = (preview || ['running', 'pause_pending'].includes(status)) && stageIndex >= 0 && stageIndex < stages.length;
  const stage = active ? stageIndex : -1;
  // 움직임(애니메이션)은 실행 중인 알려진 단계에서만. 정지·휴식·끊김이면 멈춘 그림
  const motion = active && ((mode === 1 && [1, 4].includes(stage)) || (mode === 3 && stage === 1) || (mode === 5 && [1, 3].includes(stage)) || (mode === 7 && stage === 1));
  const s = SCENE_BUILDERS[mode](stage);
  const label = active ? `${stage + 1}. ${stages[stage]}` : (SCENE_STATUS[status] || '');
  const tone = status === 'stopped' && !preview ? '#a71d28' : preview ? '#6b4fa0' : '#2f6b5b';
  const pill = label ? `<g><rect x="20" y="16" width="${46 + label.length * 19 + (preview ? 74 : 0)}" height="40" rx="20" fill="#fff" stroke="${tone}" stroke-width="2"/>
    <text x="40" y="43" class="pill" style="fill:${tone}">${preview ? '미리보기 · ' : ''}${sceneEscape(label)}</text></g>` : '';
  const frame = status === 'stopped' && !preview ? '<rect x="3" y="3" width="954" height="404" rx="10" fill="none" stroke="#a71d28" stroke-width="6"/>' : '';
  const veil = !active ? '<rect width="960" height="410" fill="#f3f6f4" opacity=".35"/>' : '';
  return `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 960 410" role="img" aria-label="${sceneEscape(label || stages.join(' → '))} · 환자 자리에서 본 설명 그림" data-scene-mode="${mode}" data-scene-stage="${stage}" data-scene-status="${sceneEscape(status)}" class="${motion ? 'scene-moving' : 'scene-still'}">
    <style>
      text{font-family:system-ui,'Noto Sans KR',sans-serif;font-weight:650}.pill{font-size:21px}.note{font-size:17px;fill:#5b4a2a}.note.small{font-size:14px;fill:#7a6d55}
      .scene-moving .flow{animation:scene-flow 1s linear infinite}
      .scene-moving .ball-roll{animation:scene-roll 2.4s ease-in infinite}
      .scene-moving .ball-throw{animation:scene-throw 2.4s ease-out infinite}
      @keyframes scene-flow{to{stroke-dashoffset:-18}}
      @keyframes scene-roll{0%{transform:translateY(-150px);opacity:0}10%{opacity:1}75%,100%{transform:none}}
      @keyframes scene-throw{0%{transform:translate(-200px,80px);opacity:0}10%{opacity:1}45%{transform:translate(-110px,-60px)}80%,100%{transform:none}}
      @media(prefers-reduced-motion:reduce){.flow,.ball-roll,.ball-throw{animation:none!important}}
    </style>
    <defs><marker id="scene-arrow" viewBox="0 0 10 10" refX="7" refY="5" markerWidth="4" markerHeight="4" orient="auto-start-reverse"><path d="M1 1L9 5L1 9Z" fill="#d48a12"/></marker>
      <marker id="scene-arrow-grey" viewBox="0 0 10 10" refX="7" refY="5" markerWidth="4" markerHeight="4" orient="auto-start-reverse"><path d="M1 1L9 5L1 9Z" fill="#8aa0ad"/></marker></defs>
    ${sceneTable()}${mode === 1 ? sceneLanes(active) : ''}${s.back || ''}${sceneArm(s.arm, s.tool || '', s.grip)}${s.front || ''}${veil}${frame}${pill}
  </svg>`;
}
