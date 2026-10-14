// Run: node dashboard/test/test_mode_pages.cjs (no browser or robot required).
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const html = fs.readFileSync(path.join(__dirname, '../web/index.html'), 'utf8');
const elements = new Map();
function element() {
  return {dataset:{}, children:[], attributes:{}, hidden:false, textContent:'', htmlWrites:0, style:{}, className:'',
    get innerHTML(){return this.markup||'';}, set innerHTML(value){this.markup=value;this.htmlWrites++;},
    parentNode:null, previousElementSibling:null,
    prepend(row){this.children.unshift(row);}, append(row){this.children.push(row);if(row&&typeof row==='object'){row.parentNode=this;row.previousElementSibling=null;}},
    after(node){node.parentNode=this.parentNode;node.previousElementSibling=this;},
    setAttribute(name,value){this.attributes[name]=value;}, removeAttribute(name){delete this.attributes[name];}};
}
const ids = [...html.matchAll(/id="([^"]+)"/g)].map(m => m[1]);
ids.forEach(id => elements.set(id, element()));
// 모드 전환 버튼 (10/14): data-mode 버튼들을 classList 와 함께
const modeButtons = [...html.matchAll(/<button type="button" data-mode="(\d?)">/g)].map(m => {
  const classes = new Set();
  return Object.assign(element(), {dataset:{mode:m[1]}, disabled:false, title:'',
    classList:{toggle(name,on){on?classes.add(name):classes.delete(name);}, contains:name=>classes.has(name)}});
});
elements.get('mode-switch').querySelectorAll = () => modeButtons;
const nav = [...html.matchAll(/<a[^>]*data-page="([^"]+)"[^>]*>/g)].map(match => Object.assign(element(),{dataset:{page:match[1]}}));
const listeners = {};
const intervals = [];
const context = vm.createContext({
  document:{getElementById(id){assert(elements.has(id), id);return elements.get(id);},
    querySelectorAll(selector){assert.equal(selector,'#mode-nav a');return nav;}, createElement:element},
  location:{hash:'#mode1'}, addEventListener(name,callback){listeners[name]=callback;},
  Date, performance, AbortSignal, setInterval(callback){intervals.push(callback);return intervals.length;}, clearInterval(){}, setTimeout(){}, console,
});
context.window = context;
assert(html.includes('src="/images/training-scenes.js"'), 'scene engine must load before page logic');
vm.runInContext(fs.readFileSync(path.join(__dirname,'../web/images/training-scenes.js'),'utf8'),context);
vm.runInContext(html.split('<script>')[1].split('</script>')[0].replace('load().then(connect);',''),context);
function run(code){return vm.runInContext(code,context);}
const get = id => elements.get(id);
for(const mode of [1,3,5,7]) {
  run(`location.hash='#mode${mode}';selectPage();`);
  assert.match(get('page-title').textContent,new RegExp(`모드 ${mode}`));
  const picture = get('mode-diagram').innerHTML;
  assert.match(picture, /<svg/);
  assert(picture.includes(`data-scene-mode="${mode}"`));
  assert.match(picture, /aria-label=["'][^"']+["']/);
  assert.equal(get('trials-view').hidden,true);
  assert.equal(get('events-view').hidden,true);
  assert.equal(nav.find(a=>a.attributes['aria-current']==='page').dataset.page,'mode');
}
// 모드 1 레인은 2개 (#142 레인 3 제거) — 레일 2개 × 레인 2개
assert.equal((run('topLanes()').match(/url\(#t-rail\)/g) || []).length, 4, 'mode 1 scene must draw exactly 2 lanes');
// brain 이 고른 레인으로 공이 굴러간다 (control.lane 0 → 거치대 쪽 x 630, 1 → 가운데 x 480, 없으면 가운데)
const ballAt = (lane) => { const svg = run(`renderTrainingScene(1,1,'running',{lane:${lane}})`); const m = svg.match(/class="ball-roll"><circle cx="(\d+)"/); return m && Number(m[1]); };
assert.equal(ballAt(0), 630, 'lane 0 rolls on the rack-side lane');
assert.equal(ballAt(1), 480, 'lane 1 rolls on the middle lane');
assert.equal(ballAt(null), 480, 'unknown lane keeps the middle lane');
run("location.hash='#mode1';selectPage();renderState({time:new Date().toISOString(),safety:{level:'normal'}});renderControl({mode:1,phase:'running',bt_node:'밀기',updated_at:Date.now()/1000,trial_index:1,lane:0});");
assert(get('mode-diagram').innerHTML.includes('class="ball-roll"><circle cx="630"'), 'page scene follows control.lane');
run("renderControl({...control,lane:1});");
assert(get('mode-diagram').innerHTML.includes('class="ball-roll"><circle cx="480"'), 'lane change redraws the scene');
// 세션 리포트 마크다운 → HTML (표·제목·목록·참고 문장), 본문의 HTML 은 글자로
const rep = run(String.raw`renderReportMd("## 세션 요약 (취소)\n\n| 모드 | 시도 |\n|---|---|\n| 5 건네고 받기 | 4 |\n\n지표에서 뺀 시도:\n- 모드 5: interrupted 1회 <b>x</b>\n\n### 지식 그래프 근거\n\n_이 해석은 참고용입니다._")`);
assert(rep.includes('<h4>세션 요약 (취소)</h4>') && rep.includes('<h5>지식 그래프 근거</h5>'), 'headings');
assert(rep.includes('<th>모드</th>') && rep.includes('<td>4</td>') && !rep.includes('---'), 'table without separator row');
assert(rep.includes('<li>모드 5: interrupted 1회 &lt;b&gt;x&lt;/b&gt;</li>'), 'list items escaped');
assert(rep.includes('<p class="note"><em>이 해석은 참고용입니다.</em></p>'), 'closing note');
run("location.hash='#mode1';selectPage();renderControl({mode:1,phase:'running',bt_node:'손 이탈 확인 대기',updated_at:Date.now()/1000,trial_index:2,difficulty:1});");
assert.equal(get('step-title').textContent,'손 이탈 대기');
assert.match(get('steps').innerHTML,/aria-current="step"/);
run("renderControl({...control,phase:'paused',can_resume:true});");
assert.equal(get('step-title').textContent,'휴식 중');
assert.equal(get('resume').disabled,false);
assert.doesNotMatch(get('steps').innerHTML,/aria-current="step"/);
run("renderControl(null);");
assert.equal(get('resume').disabled,true);
assert.equal(get('step-title').textContent,'대기 중');
run("renderControl({mode:1,phase:'running',bt_node:'밀기 1',updated_at:Date.now()/1000-5});");
assert.doesNotMatch(get('steps').innerHTML,/aria-current="step"/);
// Scene changes follow observed stages; stale or stopped states must never animate.
const moving = () => /class=["'][^"']*\bscene-moving\b/.test(get('mode-diagram').innerHTML);
run("renderState({time:new Date().toISOString(),safety:{level:'normal'}});renderControl({mode:1,phase:'running',bt_node:'밀기 1',updated_at:Date.now()/1000});");
assert(moving(), 'known running stage should animate');
const pushScene = get('mode-diagram').innerHTML;
const writes = get('mode-diagram').htmlWrites;
run('renderControl({...control,updated_at:Date.now()/1000});');
assert.equal(get('mode-diagram').innerHTML,pushScene);
assert.equal(get('mode-diagram').htmlWrites,writes,'same state must preserve the SVG DOM and animation');
run("renderControl({...control,bt_node:'공 회수 1'});");
assert.notEqual(get('mode-diagram').innerHTML,pushScene);
assert(moving());
run("renderControl({...control,phase:'pause_pending'});");
assert(moving(), 'pause_pending completes the current trial');
for (const phase of ['paused','ended']) {
  run(`renderControl({...control,phase:'${phase}'});`);
  assert(!moving(), `${phase} must be static`);
}
run("renderControl({...control,phase:'running',bt_node:'알 수 없는 단계'});");
assert(!moving(), 'unknown stage must be static');
run("renderControl({...control,bt_node:'밀기 1'});renderState({time:new Date().toISOString(),safety:{level:'stop'}});");
assert(!moving(), 'fresh STOP must immediately freeze the illustration');
run("renderState({time:new Date().toISOString(),safety:{level:'normal'}});");
assert(moving());
run('stateTime=Date.now()-3000;');
intervals.forEach(callback=>callback());
assert(!moving(), 'stale safety must freeze without another control update');
run("renderState({time:new Date().toISOString(),safety:{level:'normal'}});renderControl({...control,updated_at:Date.now()/1000-3});");
assert(!moving(), 'stale session must be static');
run("renderControl({...control,mode:5,updated_at:Date.now()/1000});");
assert(!moving(), 'viewing an inactive mode must be static');
run("location.hash='#mode3';selectPage();renderControl({...control,mode:3,phase:'running',bt_node:'알 수 없는 단계'});");
assert(!moving());
assert.match(get('mode-diagram').innerHTML,/data-scene-stage="-1"/);
run("location.hash='#mode1';selectPage();");
run("addTrial({trial_id:'one',mode:1},false);addTrial({trial_id:'five',mode:5},false);addEvent({kind:'safety_change',level:'normal'},false,true);");
const firstTrial = get('trials').children[1];
const firstEvent = get('events').children[0];
function visit(hash) { context.location.hash=hash; listeners.hashchange(); }
// 스케치 배치 (10/12): 모드 화면이면 훈련 상태·카메라 칸이 모드 격자 안으로, 전체 현황이면 원래 자리로
// 단계 미리보기 (10/12): 단계를 누르면 그 단계 그림, 실시간 상태와 별개. 페이지를 옮기면 끝
visit('#mode5');
run("$('steps').onclick({target:{closest:()=>({dataset:{i:'2'}})}});");
assert.match(get('mode-diagram').innerHTML,/data-scene-stage="2"/);
assert.match(get('mode-diagram').innerHTML,/미리보기/);
assert.match(get('steps').innerHTML,/class="[^"]*preview/);
run("$('steps-play').onclick();");                  // 미리보기 중이면 실시간으로
assert.equal(run('previewStage'),null);
assert.doesNotMatch(get('mode-diagram').innerHTML,/미리보기/);
run("$('steps-play').onclick();");                  // 처음부터 자동 재생
assert.match(get('mode-diagram').innerHTML,/data-scene-stage="0"/);
visit('#mode1');
assert.equal(run('previewStage'),null);
visit('#mode5');
assert.equal(get('control-bar').parentNode, get('mode-view'));
assert.equal(get('camera-view').parentNode, get('camera-slot'));
assert.equal(get('steps-panel').parentNode, get('card-slot-5'));
assert.equal(get('card-5').className, 'mode-card selected');
assert.equal(get('camera-view').hidden, false);
visit('#overview');
assert.equal(get('control-bar').previousElementSibling, get('control-home'));
assert.equal(get('camera-view').previousElementSibling, get('camera-home'));
visit('#trials');
assert.equal(get('camera-view').hidden, true);
for (const [hash,title,trialVisible,eventVisible] of [
  ['#trials','최근 시도',true,false], ['#events','사건 기록',false,true], ['#overview','전체 현황',false,false],
]) {
  visit(hash);
  assert.equal(get('page-title').textContent,title);
  assert.equal(get('trials-view').hidden,!trialVisible);
  assert.equal(get('events-view').hidden,!eventVisible);
  assert.equal(get('mode-view').hidden,true);
  assert.equal(run('selectedMode'),null);
  assert.equal(nav.filter(a=>a.attributes['aria-current']==='page').length,1);
  assert.equal(nav.find(a=>a.attributes['aria-current']==='page').dataset.page,hash.slice(1));
}
visit('#trials');
assert.equal(get('trials').children.filter(r=>!r.hidden).length,2);
run("addTrial({trial_id:'seven',mode:7},true);addTrial({trial_id:'seven',mode:7},true);");
assert.equal(get('trials').children.filter(r=>!r.hidden).length,3);
visit('#mode1');
assert.equal(get('trials-view').hidden,true);
assert.equal(get('events-view').hidden,true);
run("addTrial({trial_id:'three',mode:3},true);addEvent({kind:'safety_change',level:'slow'},true,false);");
visit('#trials'); // hashchange also serves browser back/forward navigation.
assert.equal(get('trials').children.filter(r=>!r.hidden).length,4);
assert(get('trials').children.includes(firstTrial),'tab changes must preserve existing rows');
visit('#events');
assert.equal(get('events').children.length,2);
assert(get('events').children.includes(firstEvent),'tab changes must preserve existing events');
visit('#not-a-page');
assert.equal(run('selectedPage'),'overview');
assert.equal(get('page-title').textContent,'전체 현황');
assert.equal(get('trials-view').hidden,true);
assert.equal(get('events-view').hidden,true);
for (const [level,label] of [['normal','정상'],['slow','느리게'],['stop','정지']]) {
  run(`renderState({time:new Date().toISOString(),safety:{level:'${level}',reason:'가상 상태'}});`);
  assert.equal(get('safety').className,level);
  assert(get('safety').innerHTML.includes(label));
}
run("renderControl({phase:'running',session_id:'session',updated_at:Date.now()/1000,proposal:{proposal_id:'p',prompt:'휴식 제안',status:'pending',remaining_s:20}});");
assert.equal(get('proposal-panel').hidden,false);
assert.equal(get('proposal-accept').disabled,false);
run("renderControl({...control,proposal:{...control.proposal,status:'rejected'}});");
assert.equal(get('proposal-accept').disabled,true);
run("renderControl({...control,proposal:{...control.proposal,status:'pending',remaining_s:0}});");
assert.equal(get('proposal-accept').disabled,true);
run("renderControl(null);");
assert.equal(get('proposal-panel').hidden,true);

for (const [source,label] of [['test','테스트'],['llm','AI 제안'],['unknown','출처 미확인']]) {
  context.proposalSource=source;
  run("renderControl({phase:'running',session_id:'source-session',updated_at:Date.now()/1000,proposal:{proposal_id:proposalSource,source:proposalSource,prompt:'혹시 피곤하진 않으세요? 조금 쉬었다 할까요?',status:'pending',remaining_s:20}});");
  assert.equal(get('proposal-source').textContent,label);
  assert.equal(get('proposal-accept').disabled,false);
  assert.equal(get('proposal-reject').disabled,false);
  for (const [status,message] of [['accepted','수락됨'],['rejected','거절됨'],['expired','시간 만료']]) {
    run(`renderControl({...control,proposal:{...control.proposal,status:'${status}'}});`);
    assert.equal(get('proposal-source').textContent,label);
    assert(get('proposal-status').textContent.includes(message));
    assert.equal(get('proposal-accept').disabled,true);
    assert.equal(get('proposal-reject').disabled,true);
  }
}
run('renderControl(null)');

async function testProposalResponses() {
  let socket, calls = 0, finish;
  context.WebSocket = class { constructor() { socket = this; } };
  context.fetch = (url, options) => {
    calls++;
    assert.equal(url, '/api/session/proposal/respond');
    assert.equal(JSON.parse(options.body).decision, 'accept');
    return new Promise(resolve => { finish = resolve; });
  };
  run('connect()');
  visit('#trials');
  socket.onmessage({data:JSON.stringify({type:'event',data:{kind:'trial',trial_id:'live',mode:5,trial_index:5,success:true}})});
  assert.equal(get('trials').children.filter(r=>!r.hidden).length,5);
  visit('#events');
  assert.equal(get('events').children.length,3);
  visit('#overview');
  const snapshot = id => ({phase:'running', session_id:'session', updated_at:Date.now()/1000,
    proposal:{proposal_id:id, source:'llm', prompt:'휴식 제안', status:'pending', remaining_s:20}});
  const receive = data => socket.onmessage({data:JSON.stringify({type:'session',data})});
  receive(snapshot('timeout'));
  const click = get('proposal-accept').onclick();
  await get('proposal-accept').onclick();
  await get('proposal-reject').onclick();
  assert.equal(calls, 1, 'pending clicks must not send another request');
  receive(snapshot('timeout'));
  assert.equal(get('proposal-accept').disabled, true);
  finish({ok:false,json:async()=>({detail:'응답 미확인 — 자동 재시도하지 않고 상태를 확인해 주세요'})});
  await click;
  assert.equal(get('proposal-panel').hidden, false);
  assert.match(get('proposal-message').textContent, /응답 미확인/);
  assert.equal(get('proposal-accept').disabled, true);
  receive(snapshot('timeout'));
  assert.match(get('proposal-message').textContent, /응답 미확인/);
  assert.equal(calls, 1, 'state updates must not retry the request');
  socket.onclose();
  assert.equal(get('proposal-panel').hidden, false);
  assert.equal(get('proposal-accept').disabled, true);
  receive(snapshot('timeout'));
  assert.match(get('proposal-message').textContent, /응답 미확인/);
  receive({...snapshot('timeout'),updated_at:Date.now()/1000-3});
  assert.equal(get('proposal-accept').disabled, true);
  receive({...snapshot('timeout'),proposal:{...snapshot('timeout').proposal,remaining_s:0}});
  assert.equal(get('proposal-accept').disabled, true);
  receive(snapshot('new'));
  assert.equal(get('proposal-message').textContent, '');
  assert.equal(get('proposal-accept').disabled, false);

  context.fetch = async () => { calls++; throw new Error('network down'); };
  await get('proposal-accept').onclick();
  assert.equal(get('proposal-panel').hidden, false);
  assert.match(get('proposal-message').textContent, /응답 결과 미확인/);
  receive(snapshot('new'));
  assert.match(get('proposal-message').textContent, /응답 결과 미확인/);
  assert.equal(calls, 2);

  context.fetch = () => { calls++; return new Promise(resolve => { finish = resolve; }); };
  const oldClick = get('proposal-accept').onclick();
  receive(snapshot('replacement'));
  finish({ok:true,json:async()=>({message:'이전 제안 승인'})});
  await oldClick;
  receive(snapshot('replacement'));
  assert.equal(get('proposal-message').textContent, '', 'late responses must not label a new proposal');
  assert.equal(calls, 3);
  context.fetch = async () => { calls++; return {ok:true,json:async()=>({message:'휴식 예약 승인'})}; };
  await get('proposal-accept').onclick();
  receive({...snapshot('replacement'),phase:'pause_pending',proposal:{...snapshot('replacement').proposal,status:'accepted'}});
  assert.equal(get('proposal-message').textContent, '휴식 예약 승인');
  assert.equal(get('proposal-accept').disabled, true);
  assert.equal(calls, 4);
  receive({phase:'running',session_id:'next-session',updated_at:Date.now()/1000,proposal:null});
  assert.equal(get('proposal-panel').hidden, true, 'new sessions must clear old proposal results');
  assert.equal(get('proposal-message').textContent, '');
  assert.equal(get('proposal-prompt').textContent, '');
  receive(snapshot('old-session-request'));
  context.fetch = () => { calls++; return new Promise(resolve => { finish = resolve; }); };
  const previousSessionClick = get('proposal-accept').onclick();
  receive({phase:'running',session_id:'third-session',updated_at:Date.now()/1000,proposal:null});
  finish({ok:true,json:async()=>({message:'이전 세션 승인'})});
  await previousSessionClick;
  assert.equal(get('proposal-panel').hidden, true);
  assert.equal(get('proposal-message').textContent, '', 'old session replies must remain hidden');
  // 모드 전환: 진행 중일 때만 보이고, 지금 모드·손 대기·꺼진 모드 3 은 못 누름. 확인 창을 거쳐 POST, brain 답을 그대로 표시
  const btn = m => modeButtons.find(b => b.dataset.mode === String(m ?? ''));
  receive({phase:'paused',session_id:'s-mode',mode:5,updated_at:Date.now()/1000,proposal:null});
  assert.equal(get('mode-switch').hidden, true);
  receive({phase:'running',session_id:'s-mode',mode:5,mode3_enabled:false,hand_wait:'',updated_at:Date.now()/1000,proposal:null,pending_switch:{mode:7}});
  assert.equal(get('mode-switch').hidden, false);
  assert(btn(5).classList.contains('on') && btn(5).disabled);
  assert.equal(btn(3).disabled, true);
  assert.equal(btn(1).disabled, false);
  assert(btn(7).classList.contains('pending') && !btn(1).classList.contains('pending'));
  let sent = null, asked = 0;
  context.confirm = () => { asked++; return false; };
  await get('mode-switch').onclick({target:{closest:()=>btn(1)}});
  assert.equal(sent, null, 'cancelled confirm must not send');
  context.confirm = () => { asked++; return true; };
  context.fetch = async (url, options) => { sent = {url, body:JSON.parse(options.body)}; return {ok:true,json:async()=>({message:'모드 1 공 굴리기(으)로 전환 접수'})}; };
  await get('mode-switch').onclick({target:{closest:()=>btn(null)}});
  assert.deepEqual(sent, {url:'/api/session/mode', body:{session_id:'s-mode', mode:null}});
  assert.equal(asked, 2);
  assert.match(get('control-message').textContent, /접수/);
  receive({phase:'running',session_id:'s-mode',mode:5,mode3_enabled:true,hand_wait:'hand_near',updated_at:Date.now()/1000,proposal:null,pending_switch:null});
  assert.equal(btn(1).disabled, true, 'hand wait blocks switching');
  console.log('PASS: dynamic stage scenes, STOP/stale freeze, unchanged scene DOM, record tabs, hash navigation, live records, state freshness, resume gating, trials, async proposal clicks, mode switch buttons, errors and WebSocket recovery');
}
testProposalResponses().catch(error => { console.error(error); process.exitCode = 1; });
