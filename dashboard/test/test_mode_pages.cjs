// Run: node dashboard/test/test_mode_pages.cjs (no browser or robot required).
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const html = fs.readFileSync(path.join(__dirname, '../web/index.html'), 'utf8');
const elements = new Map();
function element() {
  return {dataset:{}, children:[], attributes:{}, hidden:false, textContent:'', htmlWrites:0,
    get innerHTML(){return this.markup||'';}, set innerHTML(value){this.markup=value;this.htmlWrites++;},
    prepend(row){this.children.unshift(row);}, append(row){this.children.push(row);},
    setAttribute(name,value){this.attributes[name]=value;}, removeAttribute(name){delete this.attributes[name];}};
}
const ids = [...html.matchAll(/id="([^"]+)"/g)].map(m => m[1]);
ids.forEach(id => elements.set(id, element()));
const nav = [...html.matchAll(/<a[^>]*data-page="([^"]+)"[^>]*>/g)].map(match => Object.assign(element(),{dataset:{page:match[1]}}));
const listeners = {};
const intervals = [];
const context = vm.createContext({
  document:{getElementById(id){assert(elements.has(id), id);return elements.get(id);},
    querySelectorAll(selector){assert.equal(selector,'#mode-nav a');return nav;}, createElement:element},
  location:{hash:'#mode1'}, addEventListener(name,callback){listeners[name]=callback;},
  Date, performance, AbortSignal, setInterval(callback){intervals.push(callback);}, setTimeout(){}, console,
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
  assert.equal(nav.find(a=>a.attributes['aria-current']==='page').dataset.page,String(mode));
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
  console.log('PASS: dynamic stage scenes, STOP/stale freeze, unchanged scene DOM, record tabs, hash navigation, live records, state freshness, resume gating, trials, async proposal clicks, errors and WebSocket recovery');
}
testProposalResponses().catch(error => { console.error(error); process.exitCode = 1; });
