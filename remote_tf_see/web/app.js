(() => {
  const canvas = document.getElementById('scene'), ctx = canvas.getContext('2d');
  const $ = id => document.getElementById(id);
  let state = {transforms: [], target: null}, yaw = -0.72, pitch = 0.48, zoom = 190;
  let dragging = false, lastX = 0, lastY = 0, dpr = 1;

  const quatMul = (a,b) => [
    a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],
    a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
    a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],
    a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]
  ];
  const quatRotate = (q,v) => {
    const n = Math.hypot(...q) || 1, u=q.map(x=>x/n), p=[...v,0];
    return quatMul(quatMul(u,p),[-u[0],-u[1],-u[2],u[3]]).slice(0,3);
  };
  const compose = (a,b) => ({
    p: a.p.map((v,i)=>v+quatRotate(a.q,b.p)[i]), q: quatMul(a.q,b.q)
  });
  const worldPoses = transforms => {
    const byChild = new Map(transforms.map(t=>[t.child,t])), frames = new Set();
    transforms.forEach(t => {frames.add(t.parent);frames.add(t.child)});
    const memo = new Map(), visiting = new Set();
    function solve(frame) {
      if(memo.has(frame)) return memo.get(frame);
      if(visiting.has(frame)) return null;
      visiting.add(frame); const edge=byChild.get(frame); let value;
      if(!edge) value={p:[0,0,0],q:[0,0,0,1]};
      else { const parent=solve(edge.parent); value=parent?compose(parent,{p:edge.translation,q:edge.rotation}):null; }
      visiting.delete(frame); if(value) memo.set(frame,value); return value;
    }
    frames.forEach(solve); return memo;
  };
  const project = p => {
    const cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch);
    const x=cy*p[0]-sy*p[1], y=sy*p[0]+cy*p[1], z=p[2];
    const yy=cp*y-sp*z, zz=sp*y+cp*z, perspective=1/(1+Math.max(-.8,zz*.08));
    return [canvas.clientWidth/2+x*zoom*perspective,canvas.clientHeight/2-yy*zoom*perspective,zz];
  };
  function line(a,b,color,width=1,alpha=1,dash=[]) {
    const p=project(a),q=project(b); ctx.save();ctx.globalAlpha=alpha;ctx.strokeStyle=color;ctx.lineWidth=width*dpr;ctx.setLineDash(dash.map(x=>x*dpr));ctx.beginPath();ctx.moveTo(p[0]*dpr,p[1]*dpr);ctx.lineTo(q[0]*dpr,q[1]*dpr);ctx.stroke();ctx.restore();
  }
  function axes(pose,length=.09,strong=false) {
    const colors=['#fa5c69','#55d68b','#58a6ff'];
    for(let i=0;i<3;i++){const unit=[0,0,0];unit[i]=length;const v=quatRotate(pose.q,unit);line(pose.p,pose.p.map((x,j)=>x+v[j]),colors[i],strong?3:2,strong?1:.88)}
    const s=project(pose.p);ctx.fillStyle=strong?'#ffca6a':'#d7e2ed';ctx.beginPath();ctx.arc(s[0]*dpr,s[1]*dpr,(strong?4:2.5)*dpr,0,Math.PI*2);ctx.fill();
  }
  function grid() {
    const size=1,step=.1;for(let i=-size;i<=size+.001;i+=step){const major=Math.abs(i)<.001;line([-size,i,0],[size,i,0],major?'#3a5367':'#1c2a35',major?1.4:.7,major?.8:.55);line([i,-size,0],[i,size,0],major?'#3a5367':'#1c2a35',major?1.4:.7,major?.8:.55)}
  }
  function render() {
    resize();ctx.clearRect(0,0,canvas.width,canvas.height);grid();
    const poses=worldPoses(state.transforms), edges=[];
    state.transforms.forEach(t=>{const a=poses.get(t.parent),b=poses.get(t.child);if(a&&b)edges.push([a.p,b.p])});
    edges.sort((a,b)=>project(a[1])[2]-project(b[1])[2]).forEach(e=>line(e[0],e[1],'#52677a',1.2,.7,[3,3]));
    [...poses.entries()].sort((a,b)=>project(a[1].p)[2]-project(b[1].p)[2]).forEach(([name,pose])=>{axes(pose,.075);const p=project(pose.p);ctx.fillStyle='#91a2b5';ctx.font=`${10*dpr}px ui-monospace,monospace`;ctx.fillText(name,(p[0]+5)*dpr,(p[1]-5)*dpr)});
    if(state.target){const parent=poses.get(state.target.frame);if(parent){const target=compose(parent,{p:state.target.position,q:state.target.rotation});line(parent.p,target.p,'#ffca6a',1.5,.75,[4,4]);axes(target,.14,true);const p=project(target.p);ctx.fillStyle='#ffca6a';ctx.font=`600 ${11*dpr}px ui-monospace,monospace`;ctx.fillText('target_pose',(p[0]+7)*dpr,(p[1]-8)*dpr)}}
    if(state.planned_path){const parent=poses.get(state.planned_path.frame),raw=state.planned_path.points||[];if(parent&&raw.length){const path=raw.map(v=>compose(parent,{p:v.position,q:v.rotation}));for(let i=1;i<path.length;i++)line(path[i-1].p,path[i].p,'#b78cff',3,.9);path.forEach((v,i)=>{if(i%Math.max(1,Math.floor(path.length/25))===0){const p=project(v.p);ctx.fillStyle='#d1b6ff';ctx.beginPath();ctx.arc(p[0]*dpr,p[1]*dpr,1.7*dpr,0,Math.PI*2);ctx.fill()}});axes(path[path.length-1],.11,true)}}
    if(state.current_pose){const parent=poses.get(state.current_pose.frame);if(parent){const current=compose(parent,{p:state.current_pose.position,q:state.current_pose.rotation});const p=project(current.p);ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(p[0]*dpr,p[1]*dpr,5*dpr,0,Math.PI*2);ctx.fill();ctx.strokeStyle='#b78cff';ctx.lineWidth=2*dpr;ctx.stroke()}}
    requestAnimationFrame(render);
  }
  function resize(){dpr=Math.min(devicePixelRatio||1,2);const w=Math.round(canvas.clientWidth*dpr),h=Math.round(canvas.clientHeight*dpr);if(canvas.width!==w||canvas.height!==h){canvas.width=w;canvas.height=h}}
  function fmt(v){return v.map(x=>(+x).toFixed(4)).join(', ')}
  function updatePanel(){
    const parents=new Set(state.transforms.map(t=>t.parent)),children=new Set(state.transforms.map(t=>t.child));$('frames').textContent=new Set([...parents,...children]).size;$('roots').textContent=[...parents].filter(x=>!children.has(x)).length;
    if(state.target){const t=state.target;$('pose').innerHTML=`<dt>Frame</dt><dd>${escapeHtml(t.frame)}</dd><dt>Position</dt><dd>[${fmt(t.position)}]</dd><dt>Quaternion</dt><dd>[${fmt(t.rotation)}]</dd>`;const sec=Math.max(0,Date.now()/1000-t.received_at);$('age').textContent=sec<2?'实时':`${sec.toFixed(0)} 秒前`}
    updatePlan();
    buildTree(parents,children);
  }
  function updatePlan(){const status=state.task_status||'unknown',path=state.planned_path,points=path?.points||[];$('plan-state').textContent=status;$('plan-points').textContent=points.length;const card=document.querySelector('.plan-card');card.className='card plan-card '+(status.startsWith('failed')?'failed':status==='completed'||status==='planned_only'?'success':status.includes('executing')||status.includes('planning')?'active':'');let progress='—';if(points.length&&state.current_pose){const world=worldPoses(state.transforms),pp=world.get(path.frame),cp=world.get(state.current_pose.frame);if(pp&&cp){const current=compose(cp,{p:state.current_pose.position,q:state.current_pose.rotation}).p,pts=points.map(v=>compose(pp,{p:v.position,q:v.rotation}).p);let best=0,dist=Infinity;pts.forEach((p,i)=>{const d=Math.hypot(p[0]-current[0],p[1]-current[1],p[2]-current[2]);if(d<dist){dist=d;best=i}});progress=`${Math.round(best/Math.max(1,pts.length-1)*100)}%`}}$('plan-progress').textContent=progress}
  function escapeHtml(s){return String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]))}
  function buildTree(parents,children){
    const kids=new Map();state.transforms.forEach(t=>{if(!kids.has(t.parent))kids.set(t.parent,[]);kids.get(t.parent).push(t.child)});const roots=[...parents].filter(x=>!children.has(x));
    if(!roots.length){$('tree').innerHTML='<p class="empty">等待 /tf 或 /tf_static</p>';return}
    const seen=new Set(), node=n=>{if(seen.has(n))return '';seen.add(n);const sub=(kids.get(n)||[]).sort().map(node).join('');return `<li><span class="node">${escapeHtml(n)}</span>${sub?`<ul>${sub}</ul>`:''}</li>`};$('tree').innerHTML=`<ul>${roots.sort().map(node).join('')}</ul>`;
  }
  function fit(){const poses=[...worldPoses(state.transforms).values()];if(!poses.length){zoom=190;return}let max=.3;poses.forEach(v=>{max=Math.max(max,Math.hypot(...v.p))});zoom=Math.max(30,Math.min(550,Math.min(canvas.clientWidth,canvas.clientHeight)*.38/max))}
  canvas.addEventListener('pointerdown',e=>{dragging=true;lastX=e.clientX;lastY=e.clientY;canvas.setPointerCapture(e.pointerId)});canvas.addEventListener('pointermove',e=>{if(!dragging)return;yaw+=(e.clientX-lastX)*.008;pitch=Math.max(-1.45,Math.min(1.45,pitch+(e.clientY-lastY)*.008));lastX=e.clientX;lastY=e.clientY});canvas.addEventListener('pointerup',()=>dragging=false);canvas.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(20,Math.min(1000,zoom*Math.exp(-e.deltaY*.001)))},{passive:false});canvas.addEventListener('dblclick',()=>{yaw=-.72;pitch=.48;fit()});$('fit').addEventListener('click',fit);
  const stream=new EventSource('/events');stream.onopen=()=>{$('status').textContent='实时连接';$('status').parentElement.className='connection live'};stream.onerror=()=>{$('status').textContent='连接中断，正在重试';$('status').parentElement.className='connection error'};stream.onmessage=e=>{try{state=JSON.parse(e.data);updatePanel()}catch(err){console.error(err)}};
  render();setInterval(updatePanel,1000);
})();
