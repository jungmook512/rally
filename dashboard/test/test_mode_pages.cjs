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
  Date, performance, setInterval(){}, console,
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
console.log('PASS: mode pages, phase highlighting, stale state, resume gating, trial filtering');
