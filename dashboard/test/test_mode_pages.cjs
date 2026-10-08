// Run: node dashboard/test/test_mode_pages.cjs (no browser or robot required).
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const html = fs.readFileSync(path.join(__dirname, '../web/index.html'), 'utf8');
const elements = new Map();
function element() {
  return {dataset:{}, children:[], hidden:false, textContent:'', innerHTML:'',
    prepend(row){this.children.unshift(row);}, setAttribute(){}, removeAttribute(){}};
}
const ids = [...html.matchAll(/id="([^"]+)"/g)].map(m => m[1]);
ids.forEach(id => elements.set(id, element()));
const context = vm.createContext({
  document:{getElementById(id){assert(elements.has(id), id);return elements.get(id);},
    querySelectorAll(){return [];}, createElement:element},
  location:{hash:'#mode1'}, window:{addEventListener(){}},
  Date, performance, AbortSignal, setInterval(){}, setTimeout(){}, console,
});
vm.runInContext(html.split('<script>')[1].split('</script>')[0].replace('load().then(connect);',''),context);
function run(code){return vm.runInContext(code,context);}
const get = id => elements.get(id);
for(const mode of [1,3,5,7]) {
  run(`location.hash='#mode${mode}';selectPage();`);
  assert.match(get('page-title').textContent,new RegExp(`모드 ${mode}`));
  assert.match(get('mode-diagram').innerHTML,/<svg/);
}
run("location.hash='#mode1';selectPage();renderControl({mode:1,phase:'running',bt_node:'손 이탈 확인 대기',updated_at:Date.now()/1000,trial_index:2,difficulty:1});");
assert.equal(get('step-title').textContent,'손 이탈 대기');
assert.match(get('steps').innerHTML,/aria-current="step"/);
run("renderControl({...control,phase:'paused',can_resume:true});");
assert.equal(get('step-title').textContent,'휴식 중');
assert.equal(get('resume').disabled,false);
assert.doesNotMatch(get('steps').innerHTML,/aria-current="step"/);
run("renderControl(null);");
assert.equal(get('resume').disabled,true);
assert.equal(get('step-title').textContent,'상태 수신 대기');
run("renderControl({mode:1,phase:'running',bt_node:'밀기 1',updated_at:Date.now()/1000-5});");
assert.doesNotMatch(get('steps').innerHTML,/aria-current="step"/);
run("addTrial({trial_id:'one',mode:1},false);addTrial({trial_id:'five',mode:5},false);");
assert.equal(get('trials').children.filter(r=>!r.hidden).length,1);
run("location.hash='#mode3';selectPage();");
assert.equal(get('trials').children.filter(r=>!r.hidden).length,0);
assert.equal(get('trials-empty').hidden,false);
run("location.hash='#overview';selectPage();");
assert.equal(get('trials').children.filter(r=>!r.hidden).length,2);
assert.equal(get('mode-view').hidden,true);
run("renderControl({phase:'running',session_id:'session',updated_at:Date.now()/1000,proposal:{proposal_id:'p',prompt:'휴식 제안',status:'pending',remaining_s:20}});");
assert.equal(get('proposal-panel').hidden,false);
assert.equal(get('proposal-accept').disabled,false);
run("renderControl({...control,proposal:{...control.proposal,status:'rejected'}});");
assert.equal(get('proposal-accept').disabled,true);
run("renderControl({...control,proposal:{...control.proposal,status:'pending',remaining_s:0}});");
assert.equal(get('proposal-accept').disabled,true);
run("renderControl(null);");
assert.equal(get('proposal-panel').hidden,true);

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
  const snapshot = id => ({phase:'running', session_id:'session', updated_at:Date.now()/1000,
    proposal:{proposal_id:id, prompt:'휴식 제안', status:'pending', remaining_s:20}});
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
  console.log('PASS: mode pages, state freshness, resume gating, trials, async proposal clicks, errors and WebSocket recovery');
}
testProposalResponses().catch(error => { console.error(error); process.exitCode = 1; });
