(() => {
  'use strict';
  const C=FieldCore,$=id=>document.getElementById(id),copy=o=>JSON.parse(JSON.stringify(o));
  const esc=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  let state=C.defaults(),mode='robot',selected=null,drag=null,measure=[],preview=null,playing=false,animation=0,history=[];
  try { const saved=localStorage.getItem('biobuzz-field-v1');if(saved)state=C.validate(JSON.parse(saved)); } catch(e) { console.warn('Saved plan could not be loaded',e); }
  const scale=()=>state.unit==='cm'?2.54:1;
  const value=n=>Number((n*scale()).toFixed(3));
  const fmt=n=>(n*scale()).toFixed(2);
  const screen=p=>({x:p.x,y:state.field.h-p.y});
  const polygon=p=>p.map(q=>{const s=screen(q);return s.x+','+s.y;}).join(' ');
  const getPath=(o,path)=>path.split('.').reduce((a,k)=>a[k],o);
  function setPath(o,path,v){const keys=path.split('.'),key=keys.pop();keys.reduce((a,k)=>a[k],o)[key]=v;}
  function remember(){history.push(copy(state));if(history.length>40)history.shift();}
  function persist(){try{localStorage.setItem('biobuzz-field-v1',JSON.stringify(state));}catch(e){notify('浏览器未允许本地存储，请使用“保存方案”保留文件。');}}
  function notify(message){$('notice').textContent=message;}
  function change(fn){remember();stopPreview();try{fn();C.validate(state);persist();render();}catch(e){state=history.pop();notify(e.message);render();}}
  function stopPreview(){playing=false;cancelAnimationFrame(animation);preview=null;$('preview').textContent='播放预览';}
  function setMode(next){mode=next;drag=null;if(next==='measure')measure=[];render();}
  function text(x,y,label,cls='object-label',anchor='middle'){return `<text x="${x}" y="${state.field.h-y}" class="${cls}" text-anchor="${anchor}">${esc(label)}</text>`;}
  function line(a,b,attrs){const p=screen(a),q=screen(b);return `<line x1="${p.x}" y1="${p.y}" x2="${q.x}" y2="${q.y}" ${attrs}/>`;}
  function rect(o,attrs){return `<rect x="${o.x}" y="${state.field.h-o.y-o.h}" width="${o.w}" height="${o.h}" ${attrs}/>`;}
  function drawRobot(p,kind='bot',color='#277c89',opacity=1){
    const r=state.robot,poly=C.robotPolygon(p,r),c=Math.cos(C.rad(p.h)),s=Math.sin(C.rad(p.h));
    const center={x:p.x-r.offsetX*c+r.offsetY*s,y:p.y-r.offsetX*s-r.offsetY*c};
    const front={x:center.x+r.length*.41*c,y:center.y+r.length*.41*s},ref=screen(p);
    return `<g class="${kind}" opacity="${opacity}"><polygon points="${polygon(poly)}" fill="${color}" fill-opacity=".17" stroke="${color}" stroke-width=".5"/>${line(center,front,`stroke="${color}" stroke-width=".55" marker-end="url(#arrow)"`)}<circle cx="${ref.x}" cy="${ref.y}" r=".9" fill="${color}" stroke="white" stroke-width=".35"/></g>`;
  }
  function renderMap(info){
    const f=state.field,svg=$('field');svg.setAttribute('viewBox',`-14 -12 ${f.w+28} ${f.h+28}`);
    let s='<defs><marker id="arrow" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="context-stroke"/></marker><clipPath id="field-clip"><rect width="'+f.w+'" height="'+f.h+'"/></clipPath></defs>';
    s+=`<rect width="${f.w}" height="${f.h}" fill="#e2e4e1" stroke="#66747a" stroke-width=".8"/>`;
    for(let y=0;y<6;y++)for(let x=0;x<6;x++)s+=rect({x:x*f.w/6,y:y*f.h/6,w:f.w/6,h:f.h/6},`fill="${(x+y)%2?'#dce0de':'#e4e7e4'}"`);
    if(state.reference)s+=`<image href="manual-field.png" width="${f.w}" height="${f.h}" opacity=".58" preserveAspectRatio="none" clip-path="url(#field-clip)"/>`;
    for(let i=0;i<=6;i++){
      s+=line({x:i*f.w/6,y:0},{x:i*f.w/6,y:f.h},'stroke="#8f9d9d" stroke-width=".16" stroke-dasharray=".6 .7"');
      s+=line({x:0,y:i*f.h/6},{x:f.w,y:i*f.h/6},'stroke="#8f9d9d" stroke-width=".16" stroke-dasharray=".6 .7"');
      if(i<6){s+=text((i+.5)*f.w/6,-4,String.fromCharCode(65+i),'tick');s+=text(-4,(i+.5)*f.h/6,String(i+1),'tick');}
    }
    s+=text(f.w/2,-9,'观众侧 / AUDIENCE','tick');
    const red='fill="#d66f79" fill-opacity=".26" stroke="#ba5260" stroke-width=".4"',blue='fill="#5f83c6" fill-opacity=".25" stroke="#4f70b2" stroke-width=".4"';
    s+=rect({x:0,y:f.h*3/4-11.5,w:11,h:23},red)+text(5.5,f.h*3/4,'装载','object-label');
    s+=rect({x:f.w-11,y:f.h/4-11.5,w:11,h:23},blue)+text(f.w-5.5,f.h/4,'装载','object-label');
    s+=rect({x:0,y:0,w:23,h:2},red)+rect({x:f.w-23,y:f.h-2,w:23,h:2},blue);
    const box={x:(f.w-f.frameW)/2,y:(f.h-f.frameD)/2,w:f.frameW,h:f.frameD};
    s+=rect(box,`fill="#ba8c4e" fill-opacity="${f.blockCenter?'.18':'.04'}" stroke="#a36c31" stroke-width=".5" stroke-dasharray="1.2 .8"`);
    s+=text(f.w/2,f.h/2,'中央框架 '+f.frameW+' × '+f.frameD+' in');
    for(const dx of [-12.75,12.75])s+=line({x:f.w/2+dx,y:box.y},{x:f.w/2+dx,y:box.y+box.h},'stroke="#997f5f" stroke-width=".2" stroke-dasharray=".7 1"');
    for(const o of C.obstacles({...f,blockCenter:false}))s+=rect(o,'fill="#c49339" fill-opacity=".55" stroke="#80692d" stroke-width=".35"');
    if(state.sweep)for(const p of info.samples)s+=drawRobot(p,'',C.collision(p,state)?'#bb5547':'#6b8590',.22);
    for(let i=1;i<state.route.length;i++)s+=line(state.route[i-1],state.route[i],`stroke="${info.problems.some(p=>p.segment===i)?'#be5145':'#344e5b'}" stroke-width=".8" stroke-linecap="round"`);
    const origin=state.frame,osp=screen(origin);
    const axis=(a,label,color)=>{const end={x:origin.x+14*Math.cos(C.rad(a)),y:origin.y+14*Math.sin(C.rad(a))};return line(origin,end,`stroke="${color}" stroke-width=".4" marker-end="url(#arrow)"`)+text(end.x+1,end.y+1,label,'point-label');};
    s+=`<circle cx="${osp.x}" cy="${osp.y}" r="1" fill="white" stroke="#52595c" stroke-width=".35"/>`+axis(origin.angle,'+X','#8f444b')+axis(origin.angle+90,'+Y','#46756d')+text(origin.x-1,origin.y-2,'O','point-label','end');
    s+=drawRobot(state.robot);
    for(const kind of ['route','marks'])state[kind].forEach((p,i)=>{const q=screen(p),chosen=selected&&selected.kind===kind&&selected.index===i;
      s+=`<g class="waypoint" data-kind="${kind}" data-index="${i}" role="button" tabindex="0" aria-label="${esc(p.name)}"><title>${esc(p.name)}</title><circle cx="${q.x}" cy="${q.y}" r="${chosen?1.7:1.3}" fill="${kind==='route'?'#344e5b':'#b86029'}" stroke="white" stroke-width=".4"/>${text(p.x,p.y+(kind==='route'&&i%2?-4:3),kind==='route'?String(i):p.name,'point-label')}</g>`;
    });
    if(selected&&selected.kind==='route'&&state.route[selected.index])s+=drawRobot(state.route[selected.index],'','#a96725',.7);
    if(measure.length){const a=screen(measure[0]);s+=`<circle cx="${a.x}" cy="${a.y}" r="1" fill="#a04a4a"/>`;if(measure.length===2){s+=line(measure[0],measure[1],'stroke="#a04a4a" stroke-width=".4" stroke-dasharray="1 1"');s+=text((measure[0].x+measure[1].x)/2,(measure[0].y+measure[1].y)/2+2,fmt(C.dist(...measure))+' '+state.unit,'point-label');}}
    s+='<g id="preview-bot">'+(preview?drawRobot(preview,'','#825d82',.8):'')+'</g>';
    svg.innerHTML=s;
  }
  function renderLists(){
    for(const kind of ['route','marks']){
      const list=$(kind==='route'?'route-list':'mark-list');
      list.innerHTML=state[kind].map((p,i)=>{const q=C.local(p,state.frame);return `<div role="button" tabindex="0" class="point-row ${selected&&selected.kind===kind&&selected.index===i?'selected':''}" data-kind="${kind}" data-index="${i}"><span class="point-index">${kind==='route'?i:'R'}</span><span><strong>${esc(p.name)}</strong><small>${fmt(q.x)}, ${fmt(q.y)} ${state.unit} · ${q.h.toFixed(1)}°</small></span></div>`;}).join('');
    }
    const item=selected&&state[selected.kind][selected.index];
    $('point-editor').hidden=!item;
    if(item){const p=C.local(item,state.frame);$('edit-title').textContent=selected.kind==='route'?'编辑路线点':'编辑参考点';$('point-name').value=item.name;$('point-x').value=value(p.x);$('point-y').value=value(p.y);$('point-h').value=Number(p.h.toFixed(2));$('move-up').disabled=selected.kind!=='route'||selected.index===0;}
  }
  function render(){
    const info=C.analyze(state),p=C.local(state.robot,state.frame),hit=C.collision(state.robot,state);
    document.querySelectorAll('[data-key]').forEach(el=>{const v=getPath(state,el.dataset.key);el.value=el.hasAttribute('data-unit')?value(v):v;});
    $('unit').value=state.unit;$('center-block').checked=state.field.blockCenter;$('reference').checked=state.reference;$('sweep').checked=state.sweep;
    $('robot-x').value=value(p.x);$('robot-y').value=value(p.y);$('robot-h').value=Number(p.h.toFixed(2));
    $('robot-status').className='status'+(hit?' bad':'');$('robot-status').textContent=hit?'外形碰撞：'+hit:'外形通过当前二维净空检查';
    $('point-count').textContent=state.route.length+' 点';$('distance').textContent=fmt(info.length)+' '+state.unit;$('duration').textContent=info.seconds.toFixed(1)+' s';
    $('route-status').className='status'+(info.problems.length?' bad':'');$('route-status').textContent=state.route.length<2?'在场地图添加至少两个机器人路线点':info.problems.length?info.problems.map(p=>'第 '+p.segment+' 段：'+p.reason).join('；'):'当前采样未发现二维碰撞';
    $('undo').disabled=!history.length;
    document.querySelectorAll('[data-mode]').forEach(b=>{b.classList.toggle('active',b.dataset.mode===mode);b.setAttribute('aria-pressed',b.dataset.mode===mode);});
    const hints={robot:'点击或拖动机器人；朝向可在左侧输入',route:'点击添加机器人路线点；拖动已有路点可调整',mark:'点击标记射击框等参考点；参考点不参与机器人碰撞检查',origin:'点击设置坐标原点，现有机器人和路线的物理位置保持不变',axis:'点击原点以外的位置，定义 +X 指向',measure:'依次点击两个位置，计算距离'};
    $('mode-hint').textContent=hints[mode];renderMap(info);renderLists();
  }
  function eventPoint(e){const svg=$('field'),p=new DOMPoint(e.clientX,e.clientY).matrixTransform(svg.getScreenCTM().inverse());return {x:p.x,y:state.field.h-p.y};}
  $('field').addEventListener('pointerdown',e=>{
    if(e.button!==0)return;stopPreview();const p=eventPoint(e),item=e.target.closest('[data-kind]');
    if(item&&!['origin','axis','measure'].includes(mode)){selected={kind:item.dataset.kind,index:Number(item.dataset.index)};remember();drag={...selected};$('field').setPointerCapture(e.pointerId);render();return;}
    if(p.x<0||p.y<0||p.x>state.field.w||p.y>state.field.h)return;
    if(mode==='origin'){change(()=>{state.frame.x=p.x;state.frame.y=p.y;});setMode('robot');}
    else if(mode==='axis'){if(C.dist(p,state.frame)<.1){notify('请选一个离原点更远的位置。');return;}change(()=>state.frame.angle=C.deg(Math.atan2(p.y-state.frame.y,p.x-state.frame.x)));setMode('robot');}
    else if(mode==='route'||mode==='mark'){const kind=mode==='route'?'route':'marks';change(()=>{state[kind].push({...p,h:state.robot.h,name:kind==='route'?'P'+state.route.length:'REF_'+(state.marks.length+1)});selected={kind,index:state[kind].length-1};});}
    else if(mode==='measure'){if(measure.length===2)measure=[];measure.push(p);render();}
    else{remember();state.robot.x=p.x;state.robot.y=p.y;drag={kind:'robot'};$('field').setPointerCapture(e.pointerId);persist();render();}
  });
  $('field').addEventListener('pointermove',e=>{const p=eventPoint(e),q=C.local(p,state.frame);$('cursor').textContent=`X ${fmt(q.x)} · Y ${fmt(q.y)} ${state.unit}`;if(!drag)return;const item=drag.kind==='robot'?state.robot:state[drag.kind][drag.index];item.x=Math.max(0,Math.min(state.field.w,p.x));item.y=Math.max(0,Math.min(state.field.h,p.y));persist();render();});
  for(const event of ['pointerup','pointercancel'])$('field').addEventListener(event,()=>{drag=null;});
  document.querySelectorAll('[data-mode]').forEach(b=>b.onclick=()=>setMode(b.dataset.mode));
  document.querySelectorAll('[data-key]').forEach(el=>el.addEventListener('change',()=>{const n=Number(el.value)/(el.hasAttribute('data-unit')?scale():1);change(()=>setPath(state,el.dataset.key,n));}));
  $('unit').onchange=e=>change(()=>state.unit=e.target.value);
  for(const [id,key]of [['center-block','field.blockCenter'],['reference','reference'],['sweep','sweep']])$(id).onchange=e=>change(()=>setPath(state,key,e.target.checked));
  function setRobot(){const p={x:Number($('robot-x').value)/scale(),y:Number($('robot-y').value)/scale(),h:Number($('robot-h').value)};change(()=>Object.assign(state.robot,C.world(p,state.frame)));}
  for(const id of ['robot-x','robot-y','robot-h'])$(id).onchange=setRobot;
  $('zero-here').onclick=()=>change(()=>state.frame={x:state.robot.x,y:state.robot.y,angle:state.robot.h});
  document.querySelectorAll('[data-wall]').forEach(b=>b.onclick=()=>change(()=>state.robot=C.snap(state,b.dataset.wall)));
  $('start-here').onclick=()=>change(()=>{const p={x:state.robot.x,y:state.robot.y,h:state.robot.h,name:'START'};if(state.route.length)state.route[0]=p;else state.route.push(p);selected={kind:'route',index:0};});
  document.querySelectorAll('.point-list').forEach(list=>{
    const choose=e=>{const row=e.target.closest('[data-kind]');if(row){selected={kind:row.dataset.kind,index:Number(row.dataset.index)};render();}};
    list.onclick=choose;list.onkeydown=e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();choose(e);}};
  });
  $('field').addEventListener('keydown',e=>{const el=e.target.closest('[data-kind]');if(el&&(e.key==='Enter'||e.key===' ')){e.preventDefault();selected={kind:el.dataset.kind,index:Number(el.dataset.index)};render();$('point-name').focus();}});
  function editPoint(){if(!selected)return;const p={x:Number($('point-x').value)/scale(),y:Number($('point-y').value)/scale(),h:Number($('point-h').value)},name=$('point-name').value.trim()||'POINT';change(()=>Object.assign(state[selected.kind][selected.index],C.world(p,state.frame),{name}));}
  for(const id of ['point-name','point-x','point-y','point-h'])$(id).onchange=editPoint;
  $('delete-point').onclick=()=>{if(selected)change(()=>{state[selected.kind].splice(selected.index,1);selected=null;});};
  $('move-up').onclick=()=>{if(selected&&selected.kind==='route'&&selected.index>0)change(()=>{const i=selected.index;[state.route[i-1],state.route[i]]=[state.route[i],state.route[i-1]];selected.index--;});};
  $('clear-route').onclick=()=>change(()=>{state.route=[];selected=null;stopPreview();});
  $('undo').onclick=()=>{if(history.length){stopPreview();state=history.pop();selected=null;persist();render();}};
  $('auto-plan').onclick=()=>{try{if(state.route.length<2)throw Error('先添加起点与终点。');const result=C.plan(state.route[0],state.route.at(-1),state);change(()=>{state.route=result;selected=null;});notify('已按保守圆包络绕障，中间路点已替换。仍需在实地验证净空与停车表现。');}catch(e){notify(e.message);}};
  function pointAt(t){const points=state.route;if(points.length<2)return null;const lengths=points.slice(1).map((p,i)=>C.dist(points[i],p)),sum=lengths.reduce((a,b)=>a+b,0);let goal=t*sum;
    if(sum===0){const full=(points.length-1)*t,i=Math.min(points.length-2,Math.floor(full)),u=full-i,a=points[i],b=points[i+1];return {x:a.x,y:a.y,h:C.wrap(a.h+C.wrap(b.h-a.h)*u)};}
    for(let i=0;i<lengths.length;i++){if(goal<=lengths[i]||i===lengths.length-1){const a=points[i],b=points[i+1],u=lengths[i]?goal/lengths[i]:0;return {x:a.x+(b.x-a.x)*u,y:a.y+(b.y-a.y)*u,h:C.wrap(a.h+C.wrap(b.h-a.h)*u)};}goal-=lengths[i];}return points.at(-1);
  }
  function updatePreview(t){preview=pointAt(t);$('progress').value=Math.round(t*1000);const g=$('preview-bot');if(g)g.innerHTML=preview?drawRobot(preview,'','#825d82',.8):'';}
  $('progress').oninput=e=>{playing=false;cancelAnimationFrame(animation);$('preview').textContent='播放预览';updatePreview(Number(e.target.value)/1000);};
  $('preview').onclick=()=>{if(playing){stopPreview();render();return;}if(state.route.length<2){notify('先添加至少两个路线点。');return;}playing=true;$('preview').textContent='停止预览';const start=performance.now(),duration=Math.max(2,C.analyze(state).seconds)*1000;const tick=now=>{if(!playing)return;const t=Math.min(1,(now-start)/duration);updatePreview(t);if(t<1)animation=requestAnimationFrame(tick);else{playing=false;$('preview').textContent='播放预览';}};animation=requestAnimationFrame(tick);};
  function download(name,body,type){const url=URL.createObjectURL(new Blob([body],{type})),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
  $('save').onclick=()=>download('biobuzz-plan.json',JSON.stringify(state,null,2),'application/json');
  $('load').onclick=()=>$('file').click();
  $('file').onchange=async e=>{const file=e.target.files[0];if(!file)return;try{if(file.size>2e6)throw Error('方案文件超过 2 MB。');const loaded=C.validate(JSON.parse(await file.text()));change(()=>{state=loaded;selected=null;stopPreview();});notify('方案已载入。');}catch(error){notify('载入失败：'+error.message);}e.target.value='';};
  $('export-positions').onclick=()=>download('Position-snippet.java',C.positions(state),'text/plain');
  $('export-route').onclick=()=>{try{download('PlannedRoute.java',C.java(state),'text/plain');notify('已导出英寸/弧度的 Pedro 2.1.2 路线。运行前将机器人放在导出的 START 位姿。');}catch(e){notify(e.message);}};
  document.addEventListener('keydown',e=>{if(e.key==='Escape'){stopPreview();setMode('robot');}});
  render();
  // 仅暴露状态副本，便于离线验证；不允许外部脚本直接修改真实方案。
  window.fieldPlanner={snapshot:()=>copy(state),analyze:()=>C.analyze(state)};
})();
