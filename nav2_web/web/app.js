(() => {
  const canvas = document.getElementById('map');
  const ctx = canvas.getContext('2d');
  const $ = id => document.getElementById(id);
  const fixedFrame = document.body.dataset.fixedFrame || 'map';
  let live = {transforms: [], plan: null, local_plan: null, goal: null, footprints: {}, navigation: {}, bt: {}};
  let selectedGoal = null;
  let grids = {}, images = {}, versions = {}, loadedGridVersions = {};
  let center = [0, 0], scale = 70, fitted = false, dragging = false;
  let dragStart = [0, 0], goalStart = null, goalPreview = null, dpr = 1;

  const qmul = (a, b) => [
    a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],
    a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
    a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],
    a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]
  ];
  const rotate = (q, v) => {
    const n = Math.hypot(...q) || 1, u = q.map(x => x / n), p = [...v, 0];
    return qmul(qmul(u, p), [-u[0], -u[1], -u[2], u[3]]).slice(0, 3);
  };
  const compose = (a, b) => {
    const r = rotate(a.q, b.p);
    return {p: a.p.map((v, i) => v + r[i]), q: qmul(a.q, b.q)};
  };
  const yaw = q => Math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]*q[1]+q[2]*q[2]));
  const screen = p => [canvas.clientWidth/2+(p[0]-center[0])*scale, canvas.clientHeight/2-(p[1]-center[1])*scale];
  const world = p => [center[0]+(p[0]-canvas.clientWidth/2)/scale, center[1]-(p[1]-canvas.clientHeight/2)/scale];

  function poses() {
    const edges = new Map(live.transforms.map(t => [t.child, t]));
    const memo = new Map([[fixedFrame, {p: [0, 0, 0], q: [0, 0, 0, 1]}]]), visiting = new Set();
    function solve(frame) {
      if (memo.has(frame)) return memo.get(frame);
      if (visiting.has(frame)) return null;
      visiting.add(frame);
      const edge = edges.get(frame), parent = edge && solve(edge.parent);
      const value = parent ? compose(parent, {p: edge.translation, q: edge.rotation}) : null;
      visiting.delete(frame);
      if (value) memo.set(frame, value);
      return value;
    }
    live.transforms.forEach(t => solve(t.child));
    return memo;
  }

  function inFixedFrame(record, transforms) {
    if (!record) return null;
    if (!record.frame || record.frame === fixedFrame) return {p: record.position, q: record.rotation};
    const origin = transforms.get(record.frame);
    return origin ? compose(origin, {p: record.position, q: record.rotation}) : null;
  }

  function resize() {
    dpr = Math.min(devicePixelRatio || 1, 2);
    const width = Math.round(canvas.clientWidth*dpr), height = Math.round(canvas.clientHeight*dpr);
    if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
  }

  function drawLine(a, b, color, width=2, alpha=1, dash=[]) {
    const p = screen(a), q = screen(b);
    ctx.save(); ctx.globalAlpha = alpha; ctx.strokeStyle = color; ctx.lineWidth = width*dpr;
    ctx.setLineDash(dash.map(v => v*dpr)); ctx.beginPath(); ctx.moveTo(p[0]*dpr, p[1]*dpr);
    ctx.lineTo(q[0]*dpr, q[1]*dpr); ctx.stroke(); ctx.restore();
  }

  function gridPose(grid, transforms=null) {
    const [z, w] = grid.origin.rotation, angle = 2*Math.atan2(z, w);
    const local = {p: [grid.origin.position[0], grid.origin.position[1], 0], q: [0, 0, Math.sin(angle/2), Math.cos(angle/2)]};
    if (!transforms || !grid.frame || grid.frame === fixedFrame) return local;
    const framePose = transforms.get(grid.frame);
    return framePose ? compose(framePose, local) : null;
  }

  function drawGrid(grid, image, alpha, transforms) {
    if (!grid || !image) return;
    const origin = gridPose(grid, transforms); if (!origin) return; const p = screen(origin.p);
    ctx.save(); ctx.translate(p[0]*dpr, p[1]*dpr); ctx.rotate(-yaw(origin.q)); ctx.scale(1, -1);
    ctx.globalAlpha = alpha; ctx.imageSmoothingEnabled = false;
    ctx.drawImage(image, 0, 0, grid.width*grid.resolution*scale*dpr, grid.height*grid.resolution*scale*dpr);
    ctx.restore();
  }

  async function makeImages(name, grid) {
    const version = grid.data;
    if (versions[name] === version) return;
    versions[name] = version;
    const compressed = Uint8Array.from(atob(grid.data), c => c.charCodeAt(0));
    const raw = new Uint8Array(await new Response(new Blob([compressed]).stream().pipeThrough(new DecompressionStream('gzip'))).arrayBuffer());
    const build = painter => {
      const pixels = new ImageData(grid.width, grid.height);
      for (let i=0; i<raw.length; i++) painter(raw[i]-1, pixels.data, i*4);
      const offscreen = document.createElement('canvas');
      offscreen.width = grid.width; offscreen.height = grid.height;
      offscreen.getContext('2d').putImageData(pixels, 0, 0);
      return offscreen;
    };
    if (name === 'map') {
      images.map = build((value, data, offset) => {
        const color = value < 0 ? 100 : value >= 65 ? 25 : value <= 20 ? 238 : 165;
        data[offset]=color; data[offset+1]=color; data[offset+2]=color; data[offset+3]=255;
      });
      if (!fitted) fit();
      return;
    }
    const inflationColor = name === 'global_costmap' ? [255, 168, 76] : [77, 184, 255];
    images[`${name}_inflation`] = build((value, data, offset) => {
      if (value <= 0 || value >= 90) return;
      const alpha = Math.round(30 + value/89*150);
      data[offset]=inflationColor[0]; data[offset+1]=inflationColor[1]; data[offset+2]=inflationColor[2]; data[offset+3]=alpha;
    });
    images[`${name}_obstacles`] = build((value, data, offset) => {
      if (value < 90) return;
      data[offset]=255; data[offset+1]=55; data[offset+2]=105; data[offset+3]=235;
    });
  }

  function transformPoint(point, frame, transforms) {
    if (!frame || frame === fixedFrame) return point;
    const origin = transforms.get(frame);
    return origin ? compose(origin, {p: point, q: [0, 0, 0, 1]}).p : null;
  }

  function drawPath(path, color, width, transforms) {
    if (!path || path.points.length < 2) return;
    for (let i=1; i<path.points.length; i++) {
      const a = transformPoint(path.points[i-1].position, path.frame, transforms);
      const b = transformPoint(path.points[i].position, path.frame, transforms);
      if (a && b) drawLine(a, b, color, width, .95);
    }
  }

  function drawParticles(cloud, transforms) {
    if (!cloud) return;
    ctx.save(); ctx.fillStyle = '#bd8cff'; ctx.globalAlpha = .55;
    for (const particle of cloud.points) {
      const p = transformPoint(particle.position, cloud.frame, transforms);
      if (!p) continue;
      const s = screen(p); ctx.fillRect(s[0]*dpr-1*dpr, s[1]*dpr-1*dpr, 2*dpr, 2*dpr);
    }
    ctx.restore();
  }

  function drawCovariance(record, transforms) {
    const pose = inFixedFrame(record, transforms);
    if (!pose || !record.covariance) return;
    const [xx, xy, yx, yy] = record.covariance;
    const trace = xx+yy, delta = Math.sqrt(Math.max(0, (xx-yy)**2+4*xy*yx));
    const major = Math.sqrt(Math.max(0, (trace+delta)/2))*2, minor = Math.sqrt(Math.max(0, (trace-delta)/2))*2;
    if (!Number.isFinite(major+minor)) return;
    const angle = .5*Math.atan2(xy+yx, xx-yy), p = screen(pose.p);
    ctx.save(); ctx.translate(p[0]*dpr, p[1]*dpr); ctx.rotate(-angle);
    ctx.strokeStyle='#bd8cff'; ctx.fillStyle='#bd8cff22'; ctx.lineWidth=2*dpr;
    ctx.beginPath(); ctx.ellipse(0, 0, Math.max(major*scale*dpr, 2), Math.max(minor*scale*dpr, 2), 0, 0, Math.PI*2);
    ctx.fill(); ctx.stroke(); ctx.restore();
  }

  function drawFootprint(footprint, transforms) {
    if (!footprint || footprint.points.length < 3) return;
    const points = footprint.points.map(p => transformPoint(p, footprint.frame, transforms)).filter(Boolean);
    if (points.length < 3) return;
    ctx.save(); ctx.strokeStyle='#4fd18b'; ctx.fillStyle='#4fd18b22'; ctx.lineWidth=2*dpr;
    ctx.beginPath(); points.forEach((point, index) => { const p=screen(point); index ? ctx.lineTo(p[0]*dpr,p[1]*dpr) : ctx.moveTo(p[0]*dpr,p[1]*dpr); });
    ctx.closePath(); ctx.fill(); ctx.stroke(); ctx.restore();
  }

  function drawRobot(pose) {
    if (!pose) return;
    const p=screen(pose.p); ctx.save(); ctx.translate(p[0]*dpr,p[1]*dpr); ctx.rotate(-yaw(pose.q));
    ctx.fillStyle='#4fd18b'; ctx.strokeStyle='#effff7'; ctx.lineWidth=2*dpr;
    ctx.beginPath(); ctx.moveTo(16*dpr,0); ctx.lineTo(-10*dpr,-9*dpr); ctx.lineTo(-6*dpr,0); ctx.lineTo(-10*dpr,9*dpr); ctx.closePath();
    ctx.fill(); ctx.stroke(); ctx.restore();
  }

  function drawGoal(goal) {
    if (!goal) return;
    const p=screen(goal.position), angle=yaw(goal.rotation); ctx.save(); ctx.translate(p[0]*dpr,p[1]*dpr); ctx.rotate(-angle);
    ctx.strokeStyle='#ffd166'; ctx.fillStyle='#ffd166'; ctx.lineWidth=3*dpr; ctx.beginPath(); ctx.arc(0,0,9*dpr,0,Math.PI*2); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(16*dpr,0); ctx.lineTo(5*dpr,-6*dpr); ctx.lineTo(5*dpr,6*dpr); ctx.closePath(); ctx.fill(); ctx.restore();
  }

  function drawPreview() {
    if (!goalPreview) return;
    const p=screen(goalPreview.p); ctx.save(); ctx.translate(p[0]*dpr,p[1]*dpr); ctx.rotate(-goalPreview.a);
    ctx.strokeStyle='#ffd166'; ctx.setLineDash([5*dpr,4*dpr]); ctx.lineWidth=2*dpr; ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(28*dpr,0); ctx.stroke(); ctx.restore();
  }

  function fit() {
    const grid=grids.map; if (!grid) return;
    const origin=gridPose(grid), corners=[[0,0,0],[grid.width*grid.resolution,0,0],[0,grid.height*grid.resolution,0],[grid.width*grid.resolution,grid.height*grid.resolution,0]].map(p=>compose(origin,{p,q:[0,0,0,1]}).p);
    const xs=corners.map(p=>p[0]), ys=corners.map(p=>p[1]), width=Math.max(...xs)-Math.min(...xs), height=Math.max(...ys)-Math.min(...ys);
    center=[(Math.max(...xs)+Math.min(...xs))/2,(Math.max(...ys)+Math.min(...ys))/2];
    scale=Math.max(8,Math.min(500,(canvas.clientWidth-70)/Math.max(width,.1),(canvas.clientHeight-70)/Math.max(height,.1))); fitted=true;
  }

  function updateScaleBar() {
    const candidates=[.1,.2,.5,1,2,5,10,20,50,100], metres=candidates.find(v=>v*scale>=55)||100;
    $('scale-bar').style.width=`${metres*scale}px`; $('scale-label').textContent=`${metres} m`;
  }

  function render() {
    resize(); ctx.fillStyle='#171b1e'; ctx.fillRect(0,0,canvas.width,canvas.height);
    const transforms=poses(), robot=transforms.get('base_footprint')||transforms.get('base_link')||inFixedFrame(live.amcl_pose,transforms);
    if ($('show-map').checked) drawGrid(grids.map,images.map,.94,transforms);
    if ($('show-inflation').checked) {
      drawGrid(grids.global_costmap,images.global_costmap_inflation,.72,transforms);
      drawGrid(grids.local_costmap,images.local_costmap_inflation,.82,transforms);
    }
    if ($('show-obstacles').checked) drawGrid(grids.local_costmap,images.local_costmap_obstacles,.92,transforms);
    if ($('show-particles').checked) drawParticles(live.particles,transforms);
    if ($('show-covariance').checked) drawCovariance(live.amcl_pose,transforms);
    if ($('show-global-plan').checked) drawPath(live.plan,'#ff5d6c',3,transforms);
    if ($('show-local-plan').checked) drawPath(live.local_plan,'#55b7ff',3,transforms);
    if ($('show-footprint').checked) drawFootprint(live.footprints?.local||live.footprints?.global,transforms);
    drawRobot(robot); drawGoal(selectedGoal||live.goal); drawPreview(); updateScaleBar();
    $('robot-frame').textContent=robot?(transforms.has('base_footprint')?'base_footprint':transforms.has('base_link')?'base_link':'AMCL pose'):'—';
    if (robot) { $('robot-x').textContent=`${robot.p[0].toFixed(2)} m`; $('robot-y').textContent=`${robot.p[1].toFixed(2)} m`; $('robot-yaw').textContent=`${(yaw(robot.q)*180/Math.PI).toFixed(1)}°`; }
    requestAnimationFrame(render);
  }

  async function loadMaps() {
    try {
      const response=await fetch('/api/map',{cache:'no-store'}); if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const data=await response.json();
      for (const [name,grid] of Object.entries(data)) if (grid) {
        grids[name]=grid; await makeImages(name,grid);
        loadedGridVersions[name]=live.grid_versions?.[name]??loadedGridVersions[name]??0;
        if (name==='map') { $('map-size').textContent=`${(grid.width*grid.resolution).toFixed(1)} × ${(grid.height*grid.resolution).toFixed(1)} m`; $('resolution').textContent=`${(grid.resolution*100).toFixed(1)} cm`; }
      }
      setStream('map',Boolean(grids.map),'实时');
    } catch (error) { console.error('Map update failed',error); }
  }

  function showGoal(goal) {
    if (!goal) return;
    $('goal-x').textContent=`${goal.position[0].toFixed(2)} m`; $('goal-y').textContent=`${goal.position[1].toFixed(2)} m`;
    $('goal-yaw').textContent=`${(yaw(goal.rotation)*180/Math.PI).toFixed(1)}°`; $('send-goal').disabled=false;
  }
  function setStream(id,ready,text) { $(`${id}-state`).textContent=ready?text:'等待'; $(`${id}-dot`).className=ready?'live':''; }
  function formatDuration(value) {
    if (value === null || value === undefined || !Number.isFinite(value)) return '—';
    const seconds=Math.max(0,Math.round(value)); return seconds<60?`${seconds} s`:`${Math.floor(seconds/60)}m ${seconds%60}s`;
  }
  function updateStatus() {
    const nav=live.navigation||{}, bt=live.bt||{}, covariance=live.amcl_pose?.covariance;
    $('nav-status').textContent=live.status; $('bt-stage').textContent=bt.stage||'等待行为树'; $('bt-status').textContent=bt.status||'IDLE';
    $('recovery-behavior').textContent=bt.recovery||'—'; $('distance-remaining').textContent=Number.isFinite(nav.distance_remaining)?`${nav.distance_remaining.toFixed(2)} m`:'—';
    $('eta').textContent=formatDuration(nav.estimated_time_remaining); $('navigation-time').textContent=formatDuration(nav.navigation_time);
    $('recovery-count').textContent=nav.recoveries??0; $('replan-count').textContent=nav.replans??0;
    $('localization-sigma').textContent=covariance?`${Math.sqrt(Math.max(0,covariance[0])).toFixed(2)} / ${Math.sqrt(Math.max(0,covariance[3])).toFixed(2)} m`:'—';
    setStream('tf',live.transforms.length>0,'实时');
    const planPoints=(live.plan?.points?.length||0), localPoints=(live.local_plan?.points?.length||0);
    setStream('plan',planPoints+localPoints>0,`${planPoints} / ${localPoints} 点`);
    setStream('amcl',Boolean(live.amcl_pose||live.particles),live.particles?`${live.particles.total} 粒子`:'位姿');
    setStream('footprint',Boolean(live.footprints?.local||live.footprints?.global),'实时');
  }
  function setZoom(factor) { scale=Math.max(5,Math.min(1000,scale*factor)); }

  canvas.addEventListener('pointerdown',event=>{
    if(event.button===2){fetch('/api/cancel',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});return;}
    dragging=true; goalStart=world([event.offsetX,event.offsetY]); dragStart=[event.clientX,event.clientY]; canvas.setPointerCapture(event.pointerId);
  });
  canvas.addEventListener('pointermove',event=>{
    if(!dragging)return; const dx=event.clientX-dragStart[0],dy=event.clientY-dragStart[1];
    if(Math.hypot(dx,dy)>8)goalPreview={p:[goalStart[0],goalStart[1],0],a:Math.atan2(-dy,dx)};
  });
  canvas.addEventListener('pointerup',()=>{
    if(!dragging)return; dragging=false; const angle=goalPreview?.a||0;
    selectedGoal={frame:fixedFrame,position:[goalStart[0],goalStart[1],0],rotation:[0,0,Math.sin(angle/2),Math.cos(angle/2)]}; showGoal(selectedGoal); goalPreview=null;
  });
  canvas.addEventListener('contextmenu',event=>event.preventDefault());
  canvas.addEventListener('wheel',event=>{event.preventDefault();setZoom(Math.exp(-event.deltaY*.001));},{passive:false});
  $('send-goal').onclick=async()=>{
    if(!selectedGoal)return; const button=$('send-goal'); button.disabled=true; $('nav-status').textContent='发送中';
    try { const response=await fetch('/api/goal',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({x:selectedGoal.position[0],y:selectedGoal.position[1],yaw:yaw(selectedGoal.rotation),frame:selectedGoal.frame})});
      if(!response.ok)throw new Error(`HTTP ${response.status}`); const result=await response.json(); if(!result.accepted)throw new Error('Nav2 action 不可用'); selectedGoal=null;
    } catch(error) {$('nav-status').textContent=`发送失败: ${error.message}`;} finally {button.disabled=false;}
  };
  $('cancel-goal').onclick=()=>fetch('/api/cancel',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});
  $('zoom-in').onclick=()=>setZoom(1.25); $('zoom-out').onclick=()=>setZoom(.8); $('fit').onclick=fit;

  const stream=new EventSource('/events');
  stream.onopen=()=>{$('connection').className='connection live';$('connection').lastElementChild.textContent='实时连接';};
  stream.onerror=()=>{$('connection').className='connection error';$('connection').lastElementChild.textContent='连接中断，正在重试';};
  stream.onmessage=event=>{try{live=JSON.parse(event.data);if(!selectedGoal)showGoal(live.goal);updateStatus();if(Object.entries(live.grid_versions||{}).some(([name,version])=>loadedGridVersions[name]!==version))loadMaps();}catch(error){console.error(error);}};
  loadMaps(); render();
})();
