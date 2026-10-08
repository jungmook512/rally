// Illustrations explain reported stages; positions are not robot telemetry.
function sceneLink(x1, y1, x2, y2, radius) {
  const length = Math.hypot(x2-x1,y2-y1);
  const angle = Math.atan2(y2-y1,x2-x1)*180/Math.PI;
  return `<g transform="translate(${x1} ${y1}) rotate(${angle})">
    <path d="M0 ${-radius}L${length} ${-radius+3}Q${length+9} 0 ${length} ${radius-3}L0 ${radius}Z" fill="url(#metal)" stroke="#8d9ca6" stroke-width="1.2"/>
    <path d="M12 ${-radius+6}L${length-12} ${-radius+9}" stroke="#fff" stroke-width="2" opacity=".85"/>
    <path d="M10 ${radius-6}L${length-9} ${radius-8}" stroke="#afbdc5" stroke-width="1"/>
    <path d="M${length*.66} ${-radius+2}V${radius-2}" stroke="#a9b7c0" stroke-width="1"/>
    <rect x="${length*.34}" y="-5" width="28" height="10" rx="2" fill="#e3e9ec" stroke="#afbcc3" stroke-width=".7"/>
  </g>`;
}
function sceneJoint(x,y,r) {
  return `<g transform="translate(${x} ${y})"><circle r="${r+3}" fill="#bcc8cf" stroke="#778c9b"/>
    <circle r="${r}" fill="url(#blue-ring)" stroke="#365e7b"/>
    <circle r="${r-5}" fill="url(#metal)" stroke="#a2b1bc"/><circle r="${r-10}" fill="#e6edf0" stroke="#c1ccd2"/>
    ${[45,135,225,315].map(a=>`<g transform="rotate(${a})"><circle cy="${r-3}" r="1.5" fill="#516979"/><path d="M-1 ${r-3}h2" stroke="#cfd9df" stroke-width=".6"/></g>`).join('')}
    <path d="M-7 -3H7M-4 1H4" stroke="#869aa8" stroke-width=".8"/>
  </g>`;
}
const SCENE_DEFS=`<defs>
  <linearGradient id="room" x2="0" y2="1"><stop stop-color="#eef1f3"/><stop offset="1" stop-color="#f6f5f0"/></linearGradient>
  <linearGradient id="metal" x2="0" y2="1"><stop stop-color="#fdfefe"/><stop offset=".33" stop-color="#f2f5f6"/><stop offset=".72" stop-color="#dbe3e8"/><stop offset="1" stop-color="#aebec9"/></linearGradient>
  <linearGradient id="blue-ring" x2="1" y2="1"><stop stop-color="#779bb2"/><stop offset=".5" stop-color="#496f8b"/><stop offset="1" stop-color="#304f69"/></linearGradient>
  <linearGradient id="tabletop" x2=".2" y2="1"><stop stop-color="#e8e3d8"/><stop offset="1" stop-color="#d5cdbd"/></linearGradient>
  <marker id="arrow" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path d="M1 1L9 5L1 9" fill="none" stroke="#5c8074" stroke-width="1.6"/></marker>
</defs>`;
// Patient viewpoint: the robot is across the table; these are illustrative positions.
const SCENE_ROOM=`<rect width="960" height="410" rx="12" fill="url(#room)"/>
  <path d="M0 180H960" stroke="#dce3e5"/>
  <path d="M305 190L655 190L1082 410H-122Z" fill="url(#tabletop)" stroke="#b4b1a7" stroke-width="1.2"/>
  <path d="M350 210L187 410M412 210L340 410M548 210L620 410M610 210L773 410" stroke="#c5bcaa" stroke-width=".8" opacity=".45"/>
  <ellipse cx="486" cy="209" rx="93" ry="12" fill="#647d83" opacity=".12"/>`;
function frontRobot(mode) {
  return `<path d="M428 191H550L569 210H409Z" fill="#b9c5cc" stroke="#7e939f"/>
    <path d="M409 210H569V215H409Z" fill="#8b9fae" stroke="#728b9c"/>
    <g fill="#4a6475"><ellipse cx="422" cy="207" rx="3" ry="1.8"/><ellipse cx="556" cy="207" rx="3" ry="1.8"/></g>
    <path d="M450 164V194Q484 208 526 194V164" fill="url(#metal)" stroke="#8c9eaa"/>
    <ellipse cx="488" cy="165" rx="38" ry="12" fill="#eef3f5" stroke="#91a5b2"/>
    <path d="M451 184Q488 197 525 184" fill="none" stroke="#4e7593" stroke-width="5"/>
    ${sceneLink(488,165,549,105,29)}${sceneJoint(549,105,31)}
    ${sceneLink(549,102,554,40,25)}${sceneJoint(554,40,28)}
    ${sceneLink(550,40,478,52,25)}${sceneJoint(478,52,26)}
    ${sceneLink(478,58,478,109,20)}
    <path d="M460 108L458 138Q478 146 498 138L496 108" fill="url(#metal)" stroke="#8b9daa"/>
    <ellipse cx="478" cy="109" rx="19" ry="7" fill="#dde7ed" stroke="#456b88" stroke-width="4"/>
    <path d="M462 124H494" stroke="#b1c0c9"/><circle cx="464" cy="131" r="1.5" fill="#7c919f"/><circle cx="492" cy="131" r="1.5" fill="#7c919f"/>
    <rect x="448" y="79" width="61" height="19" rx="5" fill="#aebfc9" stroke="#687f90"/>
    <rect x="452" y="83" width="53" height="11" rx="4" fill="#263947"/>
    <circle cx="461" cy="88.5" r="3.5" fill="#63889f"/><circle cx="495" cy="88.5" r="3.5" fill="#63889f"/>
    <circle cx="477" cy="88.5" r="2" fill="#7b919c"/>
    <path d="M452 80Q426 46 453 25Q466 17 483 19" fill="none" stroke="#495c68" stroke-width="3"/>
    <path d="M494 140L497 151H459L462 140Z" fill="#687f8e" stroke="#3c5262"/>
    ${mode===7?'<path d="M460 151Q478 159 496 151" fill="none" stroke="#435c6c" stroke-width="3"/>':`<rect x="460" y="151" width="36" height="18" rx="3" fill="#344956" stroke="#213a4c"/><path d="M464 166L463 184L473 190M492 166L493 184L483 190" fill="none" stroke="#293d4c" stroke-width="5" stroke-linejoin="round"/><path d="M467 171V182M489 171V182" stroke="#91a5b1" stroke-width="1"/>`}
  `;
}
function perspectiveLanes() {
  // Every edge projects to the single vanishing point (480,100).
  return [160,270,425,535,690,800].map((x,i)=>{
    const farX=480+(x-480)*110/290,farRight=480+(x+19-480)*110/290;
    return `<path d="M${farX} 210L${farRight} 210L${x+19} 390H${x}Z" fill="${i===2||i===3?'#a3b8ca':'#aec2b2'}" stroke="#839b92" stroke-width=".7"/>`;
  }).join('');
}
const SCENE_STAGES = {
  1: ['손 시작 위치 확인', '공 보내기', '받기 관측', '손 이탈 대기', '공 회수', '시도 기록'],
  3: ['준비', '던지기 관측', '착지점 예측', '패드 이동', '받기 확인', '결과 기록'],
  5: ['손·물체 확인', '물체 집기', '물체 건네기', '물체 돌려받기', '시도 기록'],
  7: ['손 확인', '저항 훈련', '시도 기록'],
};
const SCENE_STATUS = {paused:'휴식 중', ended:'세션 종료', stopped:'안전 정지', stale:'상태 수신 대기',
  inactive:'선택한 모드 대기', waiting:'동작 확인 대기'};
function renderTrainingScene(mode, stageIndex, status) {
  if (!Object.hasOwn(SCENE_STAGES, mode)) return '';
  const active = ['running', 'pause_pending'].includes(status) && stageIndex >= 0 && stageIndex < SCENE_STAGES[mode].length;
  const stage = active ? stageIndex : -1;
  const motion = active && ((mode === 1 && [1,4].includes(stage)) || (mode === 5 && [1,2,3].includes(stage)));
  const label = active ? SCENE_STAGES[mode][stage] : (SCENE_STATUS[status] || '동작 확인 대기');
  const color = status === 'stopped' ? '#a71d28' : '#316e69';
  let scene='';
  if(mode===1) {
    const ballY=[2,3,4].includes(stage)?314:230;
    const ballClass=stage===1?'ball-send':stage===4?'ball-return':'';
    scene=perspectiveLanes()+`<ellipse cx="480" cy="321" rx="22" ry="9" fill="none" stroke="#748e7b" stroke-width="1.6" stroke-dasharray="5 4"/>
      <g class="${motion?ballClass:''}"><ellipse cx="480" cy="${ballY+12}" rx="13" ry="4" fill="#727b65" opacity=".18"/><circle cx="480" cy="${ballY}" r="13" fill="#d8b865" stroke="#a18a4f" stroke-width="1.2"/><path d="M474 ${ballY-8}Q480 ${ballY-12} 485 ${ballY-6}" fill="none" stroke="#f4e4b9" stroke-width="2"/></g>`;
  }
  if(mode===3) scene=`<path d="M566 309Q576 172 480 187" fill="none" stroke="#8aa193" stroke-width="1.7" stroke-dasharray="6 5"/>
    <circle cx="563" cy="304" r="14" fill="#d8b865" stroke="#a18a4f"/><ellipse cx="480" cy="190" rx="34" ry="11" fill="#7e9790" stroke="#506e65" stroke-width="2"/>`;
  if(mode===5) {
    const rodY=stage===3?282:208;
    const rodClass=stage===2?'rod-give':stage===3?'rod-receive':stage===1?'rod-pick':'';
    scene=`<g class="${motion?rodClass:''}"><ellipse cx="480" cy="${rodY+12}" rx="55" ry="5" fill="#6a756a" opacity=".12"/><rect x="430" y="${rodY}" width="100" height="17" rx="8" fill="#c5af82" stroke="#9d8763" stroke-width="1.2"/><path d="M439 ${rodY+4}H520" stroke="#e9d7b3" stroke-width="1.4"/></g>`;
  }
  if(mode===7&&active&&stage===1) scene='<ellipse cx="482" cy="160" rx="33" ry="30" fill="none" stroke="#7e9b8c" stroke-width="1.6" stroke-dasharray="5 4"/>';
  const veil=active?'':'<rect x="230" y="18" width="500" height="360" rx="20" fill="#edf4f6" opacity=".1"/>';
  const escape=value=>String(value).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  return `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 960 410" role="img" aria-label="${escape(label)} · 환자 정면 시점의 실행 단계 설명. 실측 움직임 아님" data-scene-mode="${mode}" data-scene-stage="${stage}" data-scene-status="${escape(status)}" class="${motion?'scene-moving':'scene-still'}">
    <style>
      text{font-family:system-ui,'Noto Sans KR',sans-serif;font-size:20px;font-weight:600;fill:#375768}.small{font-size:17px;fill:#627985}
      .ball-send{animation:scene-send 1.8s ease-out forwards}.ball-return{animation:scene-return 1.8s ease-in-out forwards}
      .rod-give{animation:scene-give 1.8s ease-in-out forwards}.rod-receive{animation:scene-receive 1.8s ease-in-out forwards}.rod-pick{animation:scene-pick 1.2s ease-out forwards}
      @keyframes scene-send{to{transform:translateY(84px)}}@keyframes scene-return{to{transform:translateY(-84px)}}
      @keyframes scene-give{to{transform:translateY(74px)}}@keyframes scene-receive{to{transform:translateY(-74px)}}@keyframes scene-pick{from{transform:translateY(35px)}to{transform:translateY(0)}}
      @media(prefers-reduced-motion:reduce){.ball-send,.ball-return,.rod-give,.rod-receive,.rod-pick{animation:none}}
    </style>${SCENE_DEFS}${SCENE_ROOM}${frontRobot(mode)}${scene}${veil}
    <rect x="28" y="20" width="264" height="41" rx="9" fill="#fff" opacity=".9"/><text x="43" y="48" style="fill:${color}">${escape(label)}</text>
    <text x="35" y="391" class="small">환자 시점 · 로봇을 정면으로 바라봄</text>
  </svg>`;
}
