(function (root) {
  'use strict';
  const rad = d => d * Math.PI / 180;
  const deg = r => r * 180 / Math.PI;
  const wrap = a => ((a + 180) % 360 + 360) % 360 - 180;
  const dist = (a,b) => Math.hypot(a.x-b.x,a.y-b.y);
  function defaults() { return {version:1,unit:'in',frame:{x:0,y:0,angle:0},field:{w:144,h:144,frameW:49.46,frameD:38.95,blockCenter:true,flowerDepth:4.8,clearance:1},robot:{x:24,y:24,h:0,length:18,width:18,offsetX:0,offsetY:0},route:[],marks:[],speed:30,reference:true,sweep:false}; }
  function local(p,f) { const c=Math.cos(rad(f.angle)),s=Math.sin(rad(f.angle)),x=p.x-f.x,y=p.y-f.y;return {x:x*c+y*s,y:-x*s+y*c,h:wrap((p.h||0)-f.angle)}; }
  function world(p,f) { const c=Math.cos(rad(f.angle)),s=Math.sin(rad(f.angle));return {x:f.x+p.x*c-p.y*s,y:f.y+p.x*s+p.y*c,h:wrap((p.h||0)+f.angle)}; }
  function robotPolygon(p,r) { const c=Math.cos(rad(p.h)),s=Math.sin(rad(p.h));return [[-1,-1],[1,-1],[1,1],[-1,1]].map(([sx,sy])=>{const x=sx*r.length/2-r.offsetX,y=sy*r.width/2-r.offsetY;return {x:p.x+x*c-y*s,y:p.y+x*s+y*c};}); }
  function obstacles(f) {
    const a=[];
    if(f.blockCenter)a.push({id:'中央框架包络',x:(f.w-f.frameW)/2,y:(f.h-f.frameD)/2,w:f.frameW,h:f.frameD});
    const d=f.flowerDepth,t=4.29;
    a.push({id:'西花朵近似包络',x:0,y:f.h/3-t/2,w:d,h:t},{id:'东花朵近似包络',x:f.w-d,y:2*f.h/3-t/2,w:d,h:t},{id:'北花朵近似包络',x:f.w/3-t/2,y:f.h-d,w:t,h:d},{id:'南花朵近似包络',x:2*f.w/3-t/2,y:0,w:t,h:d});return a;
  }
  function rectPolygon(r,m=0) { return [{x:r.x-m,y:r.y-m},{x:r.x+r.w+m,y:r.y-m},{x:r.x+r.w+m,y:r.y+r.h+m},{x:r.x-m,y:r.y+r.h+m}]; }
  function overlap(a,b) {
    for(const poly of [a,b])for(let i=0;i<poly.length;i++){
      const p=poly[i],q=poly[(i+1)%poly.length],nx=-(q.y-p.y),ny=q.x-p.x;
      const pa=a.map(v=>v.x*nx+v.y*ny),pb=b.map(v=>v.x*nx+v.y*ny);
      if(Math.max(...pa)<Math.min(...pb)-1e-9||Math.max(...pb)<Math.min(...pa)-1e-9)return false;
    }return true;
  }
  function collision(p,state) {
    const poly=robotPolygon(p,state.robot),f=state.field,m=f.clearance;
    if(poly.some(v=>v.x<m||v.y<m||v.x>f.w-m||v.y>f.h-m))return '围墙 / 留边';
    for(const o of obstacles(f))if(overlap(poly,rectPolygon(o,m)))return o.id;
    return '';
  }
  function snap(state,wall){const p=state.robot,poly=robotPolygon(p,p),m=state.field.clearance;let x=p.x,y=p.y;
    if(wall==='left')x+=m-Math.min(...poly.map(p=>p.x));
    if(wall==='right')x+=state.field.w-m-Math.max(...poly.map(p=>p.x));
    if(wall==='bottom')y+=m-Math.min(...poly.map(p=>p.y));
    if(wall==='top')y+=state.field.h-m-Math.max(...poly.map(p=>p.y));
    return {...p,x,y};
  }
  function legSamples(a,b) {
    const turn=wrap(b.h-a.h),n=Math.max(1,Math.ceil(dist(a,b)/0.5),Math.ceil(Math.abs(turn)/2));
    return Array.from({length:n+1},(_,i)=>({x:a.x+(b.x-a.x)*i/n,y:a.y+(b.y-a.y)*i/n,h:wrap(a.h+turn*i/n)}));
  }
  function analyze(state) {
    let length=0;const problems=[],samples=[];
    for(let i=1;i<state.route.length;i++){
      length+=dist(state.route[i-1],state.route[i]);
      const leg=legSamples(state.route[i-1],state.route[i]);
      const bad=leg.find(p=>collision(p,state));
      if(bad)problems.push({segment:i,reason:collision(bad,state)});
      samples.push(...leg.filter((p,j)=>j%Math.max(1,Math.floor(leg.length/12))===0));
    }return {length,seconds:length/state.speed,problems,samples};
  }
  function radius(r) {return Math.max(...robotPolygon({x:0,y:0,h:0},r).map(p=>Math.hypot(p.x,p.y)));}
  function plan(start,end,state) {
    const f=state.field,R=radius(state.robot)+f.clearance,obs=obstacles(f),step=2;
    const free=p=>p.x>=R&&p.y>=R&&p.x<=f.w-R&&p.y<=f.h-R&&!obs.some(o=>p.x>=o.x-R&&p.x<=o.x+o.w+R&&p.y>=o.y-R&&p.y<=o.y+o.h+R);
    const visible=(a,b)=>{const n=Math.ceil(dist(a,b)/.5);for(let i=0;i<=n;i++){const t=n?i/n:0;if(!free({x:a.x+(b.x-a.x)*t,y:a.y+(b.y-a.y)*t}))return false;}return true;};
    if(!free(start)||!free(end))throw Error('起点或终点不满足保守圆包络的净空。可调整位置，或手动布线并检查旋转外形。');
    if(visible(start,end))return [{...start},{...end}];
    const nx=Math.floor(f.w/step)+1,ny=Math.floor(f.h/step)+1,point=id=>({x:(id%nx)*step,y:Math.floor(id/nx)*step});
    function nearest(p){let best=-1,score=Infinity;for(let y=Math.max(0,Math.round(p.y/step)-3);y<Math.min(ny,Math.round(p.y/step)+4);y++)for(let x=Math.max(0,Math.round(p.x/step)-3);x<Math.min(nx,Math.round(p.x/step)+4);x++){const id=y*nx+x,q=point(id),d=dist(p,q);if(d<score&&free(q)&&visible(p,q)){score=d;best=id;}}if(best<0)throw Error('起终点无法接入 2 英寸规划网格，请稍微移动后重试。');return best;}
    const first=nearest(start),last=nearest(end),g=new Float64Array(nx*ny).fill(Infinity),parents=new Int32Array(nx*ny).fill(-1),closed=new Uint8Array(nx*ny),heap=[];
    function push(v){heap.push(v);let i=heap.length-1;while(i){let p=(i-1)>>1;if(heap[p].f<=v.f)break;heap[i]=heap[p];i=p;}heap[i]=v;}
    function pop(){const v=heap[0],t=heap.pop();if(heap.length){let i=0;while(i*2+1<heap.length){let j=i*2+1;if(j+1<heap.length&&heap[j+1].f<heap[j].f)j++;if(heap[j].f>=t.f)break;heap[i]=heap[j];i=j;}heap[i]=t;}return v.id;}
    g[first]=0;push({id:first,f:dist(point(first),point(last))});
    while(heap.length){const id=pop();if(closed[id])continue;closed[id]=1;if(id===last)break;const p=point(id),x=id%nx,y=Math.floor(id/nx);
      for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){
        if(!dx&&!dy)continue;const xx=x+dx,yy=y+dy;if(xx<0||yy<0||xx>=nx||yy>=ny)continue;
        const next=yy*nx+xx,q=point(next);if(closed[next]||!free(q)||!visible(p,q))continue;
        const score=g[id]+step*Math.hypot(dx,dy);if(score<g[next]){g[next]=score;parents[next]=id;push({id:next,f:score+dist(q,point(last))});}
      }
    }
    if(!closed[last])throw Error('未找到满足圆包络净空的路径。机器人尺寸、障碍包络或留边可能过大。');
    const raw=[end];for(let i=last;i!==-1;i=parents[i])raw.push(point(i));raw.push(start);raw.reverse();
    const simplified=[raw[0]];let at=0;while(at<raw.length-1){let next=raw.length-1;while(next>at+1&&!visible(raw[at],raw[next]))next--;simplified.push(raw[next]);at=next;}
    const total=simplified.slice(1).reduce((s,p,i)=>s+dist(simplified[i],p),0);let moved=0;
    return simplified.map((p,i)=>{if(i)moved+=dist(simplified[i-1],p);return {...p,h:wrap(start.h+wrap(end.h-start.h)*(total?moved/total:0)),name:i===0?start.name:i===simplified.length-1?end.name:'绕障'+i};});
  }
  const num=n=>{const x=Math.abs(n)<.0000005?0:n;return x.toFixed(4).replace(/\.?0+$/,'');};
  const poseCode=p=>`new Pose(${num(p.x)}, ${num(p.y)}, Math.toRadians(${num(p.h)}))`;
  function java(state) {
    if(state.route.length<2)throw Error('至少需要两个路线点。');
    if(analyze(state).problems.length)throw Error('路线存在碰撞，请先调整。');
    const points=state.route.map(p=>local(p,state.frame));
    let text='package org.firstinspires.ftc.teamcode.Auto;\n\nimport com.pedropathing.follower.Follower;\nimport com.pedropathing.geometry.BezierLine;\nimport com.pedropathing.geometry.Pose;\nimport com.pedropathing.paths.PathChain;\n\n// 坐标单位 inch；heading 为弧度。起点需与真实机器人一致。\npublic final class PlannedRoute {\n    public static final Pose START = '+poseCode(points[0])+';\n\n    public static PathChain build(Follower follower) {\n        return follower.pathBuilder()\n';
    for(let i=1;i<points.length;i++){const a=points[i-1],b=points[i];text+=`                .addPath(new BezierLine(${poseCode(a)}, ${poseCode(b)}))\n                .setLinearHeadingInterpolation(Math.toRadians(${num(a.h)}), Math.toRadians(${num(b.h)}))\n`;}
    return text+'                .build();\n    }\n}\n';
  }
  function positions(state){const all=[{...state.robot,name:'ROBOT_POSE'},...state.marks,...state.route];const used=new Set();return '// inch；朝向逆时针为正；Math.toRadians 的输入为度。\n'+all.map((p,i)=>{let name=(p.name||'POINT_'+i).replace(/[^a-zA-Z0-9_]/g,'_');if(!/^[a-zA-Z_]/.test(name))name='P_'+name;if(used.has(name))name+='_'+i;used.add(name);return 'public static Pose '+name+' = '+poseCode(local(p,state.frame))+';';}).join('\n');}
  function validate(s){
    if(!s||s.version!==1||!s.field||!s.frame||!s.robot||!Array.isArray(s.route)||!Array.isArray(s.marks))throw Error('不是有效的场地方案。');
    const finite=o=>Object.values(o).every(v=>typeof v!=='number'||Number.isFinite(v));
    if(!finite(s.field)||!finite(s.robot)||!finite(s.frame)||!Number.isFinite(s.speed)||s.speed<=0||s.route.length>300||s.marks.length>300)throw Error('方案包含无效数值或点数过多。');
    for(const [o,k,min,max]of [[s.field,'w',48,240],[s.field,'h',48,240],[s.field,'frameW',1,100],[s.field,'frameD',1,100],[s.field,'flowerDepth',0,20],[s.field,'clearance',0,12],[s.robot,'length',1,60],[s.robot,'width',1,60]])if(!Number.isFinite(o[k])||o[k]<min||o[k]>max)throw Error('尺寸超出允许范围：'+k);
    for(const p of [s.robot,...s.route,...s.marks])if(!['x','y','h'].every(k=>Number.isFinite(p[k]))||p.name&&typeof p.name!=='string')throw Error('坐标点格式有误。');
    if(!['x','y','angle'].every(k=>Number.isFinite(s.frame[k]))||!Number.isFinite(s.robot.offsetX)||!Number.isFinite(s.robot.offsetY))throw Error('坐标系/定位点偏移有误。');
    return s;
  }
  root.FieldCore={rad,deg,wrap,dist,defaults,local,world,robotPolygon,rectPolygon,obstacles,collision,snap,legSamples,analyze,plan,java,positions,validate};
  if(typeof module!=='undefined')module.exports=root.FieldCore;
})(typeof window!=='undefined'?window:globalThis);
