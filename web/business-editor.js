(() => {
  'use strict';
  let points=[], networks=[], pointSnapshot=null, routeSnapshot=null, steps=[];
  let pointMap=null, routeMap=null, busy=false, picking=false, origin=null, meta=null, projection=null;
  const image=new Image(), canvas=$('businessMap'), ctx=canvas.getContext('2d'), pending=new Map();
  let view={scale:1,panX:0,panY:0};
  const message=text=>{$('editorMessage').textContent=text;};
  const currentMap=()=>app.state?.active_map;
  const stamp=map=>map?`${map.map_id}:${map.revision}`:'';
  const selectedRoute=()=>app.routes.find(item=>item.route_id===app.selectedRouteId);
  window.previewIsCurrent=()=>{
    const plan=app.recordedPreview, route=selectedRoute(), map=currentMap();
    return Boolean(plan && map && route && plan.map_id===map.map_id && plan.map_revision===map.revision &&
      plan.route_id===route.route_id && plan.route_revision===route.revision && plan.loop_count===Number($('loopCount').value));
  };
  function request(type, data) {
    return new Promise((resolve,reject)=>{
      const id=requestId(type), timer=setTimeout(()=>{pending.delete(id);reject(new Error('响应超时，请刷新核对保存结果'));},45000);
      pending.set(id,{resolve,reject,timer});
      if (!sendControl(type,{...data,request_id:id})) { clearTimeout(timer);pending.delete(id);reject(new Error('连接已断开')); }
    });
  }
  const onMessage=app.ws.onmessage;
  app.ws.onmessage=event=>{
    onMessage(event);
    let data;try { data=JSON.parse(event.data); } catch (_) { return; }
    if (data.type==='command_result' && pending.has(data.request_id)) {
      const entry=pending.get(data.request_id); clearTimeout(entry.timer);pending.delete(data.request_id);
      if (data.ok) entry.resolve(data.data); else entry.reject(new Error(data.message || data.code));
    }
  };
  async function operation(action) {
    if (busy) return;
    busy=true;refreshBusiness();
    try { await action(); } catch(error) {message(error.message);}
    finally {busy=false;refreshBusiness();}
  }
  function options(select, items, key, empty) {
    const previous=select.value;
    select.replaceChildren(); if (empty!==undefined) select.add(new Option(empty,''));
    for (const item of items) select.add(new Option(item.name,item[key]));
    if (items.some(item=>item[key]===previous)) select.value=previous;
  }
  function newPoint() {
    pointSnapshot=null; pointMap=currentMap()?{...currentMap()}:null;
    $('editPoint').value=''; $('editPointName').value=''; $('editPointX').value=''; $('editPointY').value='';
    $('editPointYaw').value=0; $('editPointInspect').checked=false; $('editPointPreset').value=1; $('editPointPrompt').value='';draw();refreshBusiness();
  }
  function newRoute() {
    routeSnapshot=null;routeMap=currentMap()?{...currentMap()}:null;
    $('editRoute').value='';$('editRouteName').value='';$('editRouteMode').value='FREE_NAV';steps=[];renderSteps();refreshBusiness();
  }
  $('editPoint').onchange=()=>{
    const point=points.find(item=>item.point_id===$('editPoint').value);
    if (!point) return newPoint();
    pointSnapshot={...point};pointMap={...currentMap()};
    for (const [field,key] of [['Name','name'],['X','x'],['Y','y'],['Preset','ptz_preset'],['Prompt','recognition_prompt']]) $('editPoint'+field).value=point[key] ?? '';
    $('editPointYaw').value=(point.yaw*180/Math.PI).toFixed(2);$('editPointInspect').checked=Boolean(point.inspection_enabled);draw();refreshBusiness();
  };
  $('editRoute').onchange=()=>{
    const route=app.routes.find(item=>item.route_id===$('editRoute').value);
    if (!route) return newRoute();
    routeSnapshot={...route};routeMap={...currentMap()};steps=route.steps.map(item=>({...item}));
    $('editRouteName').value=route.name;$('editRouteMode').value=route.navigation_mode || 'FREE_NAV';
    $('editNetwork').value=route.network_id || '';renderSteps();refreshBusiness();
  };
  function renderSteps() {
    const host=$('editSteps');host.replaceChildren();
    steps.forEach((step,index)=>{
      const row=document.createElement('div');row.className='route-step';
      const name=document.createElement('span');name.textContent=`${index+1}. ${points.find(item=>item.point_id===step.point_id)?.name || step.point_name || step.point_id}`;
      const dwell=document.createElement('input');dwell.type='number';dwell.min=0;dwell.max=86400;dwell.step=.1;dwell.value=step.dwell_seconds;dwell.setAttribute('aria-label',`第 ${index+1} 点停留秒数`);
      dwell.oninput=()=>{step.dwell_seconds=Number(dwell.value);};row.append(name,dwell);
      for (const [label,offset] of [['上移',-1],['下移',1],['移除',0]]) {
        const button=document.createElement('button');button.type='button';button.className='btn gray';button.textContent=label;
        button.disabled=offset===-1?index===0:offset===1?index===steps.length-1:false;
        button.onclick=()=>{if (offset) [steps[index],steps[index+offset]]=[steps[index+offset],steps[index]];else steps.splice(index,1);renderSteps();};row.append(button);
      }
      host.append(row);
    });
  }
  $('addStep').onclick=()=>{const point=points.find(item=>item.point_id===$('addStepPoint').value);if (point) {steps.push({point_id:point.point_id,point_name:point.name,dwell_seconds:0});renderSteps();}};
  $('newPoint').onclick=newPoint;$('newRoute').onclick=newRoute;
  function mapFields(map) {
    if (!map || stamp(map)!==stamp(currentMap())) throw new Error('地图已变化，请重新选择资料或新增');
    return {map_id:map.map_id,map_revision:Number(map.revision)};
  }
  $('savePoint').onclick=()=>operation(async()=>{
    if (!$('editPointX').value.trim() || !$('editPointY').value.trim()) throw new Error('请填写坐标或在地图上选点');
    await request('point_save',{...mapFields(pointMap),point_id:pointSnapshot?.point_id || '',expected_revision:pointSnapshot?.revision || 0,
      name:$('editPointName').value,x:Number($('editPointX').value),y:Number($('editPointY').value),yaw:Number($('editPointYaw').value)*Math.PI/180,
      inspection_enabled:$('editPointInspect').checked,ptz_preset:$('editPointInspect').checked?Number($('editPointPreset').value):0,recognition_prompt:$('editPointPrompt').value});
    newPoint(); await reload();message('点位已保存；引用它的路线版本也会更新。');
  });
  $('deletePoint').onclick=()=>operation(async()=>{
    if (!pointSnapshot || !confirm(`删除点位“${pointSnapshot.name}”？被路线引用的点位会拒绝删除。`)) return;
    await request('point_delete',{...mapFields(pointMap),point_id:pointSnapshot.point_id,expected_revision:pointSnapshot.revision});newPoint();await reload();message('点位已删除');
  });
  $('saveRoute').onclick=()=>operation(async()=>{
    const fixed=$('editRouteMode').value==='RECORDED_NETWORK', network=networks.find(item=>item.network_id===$('editNetwork').value);
    if (fixed && (!network || !network.published)) throw new Error('请选择已发布的当前路网');
    await request('route_save',{...mapFields(routeMap),route_id:routeSnapshot?.route_id || '',expected_revision:routeSnapshot?.revision || 0,
      name:$('editRouteName').value,navigation_mode:$('editRouteMode').value,network_id:fixed?network.network_id:'',network_revision:fixed?network.revision:0,
      steps:steps.map(step=>({point_id:step.point_id,dwell_seconds:step.dwell_seconds}))});
    newRoute();await reload();message('路线已保存，可在上方选择并执行。');
  });
  $('deleteRoute').onclick=()=>operation(async()=>{
    if (!routeSnapshot || !confirm(`删除路线“${routeSnapshot.name}”？历史报告仍会保留。`)) return;
    await request('route_delete',{...mapFields(routeMap),route_id:routeSnapshot.route_id,expected_revision:routeSnapshot.revision});newRoute();await reload();message('路线已删除');
  });
  $('btnPreview').onclick=()=>operation(async()=>{
    const route=selectedRoute(),loops=Number($('loopCount').value);
    if (!route?.requires_preview || !Number.isInteger(loops) || loops<0 || loops>100000) throw new Error('请选择固定路线并填写有效圈数');
    $('previewMessage').textContent='正在检查道路与自动接入路径…';
    app.recordedPreview=await request('route_preview',{...mapFields(currentMap()),route_id:route.route_id,loop_count:loops});
    $('previewMessage').textContent=`预览完成，共 ${app.recordedPreview.blocks.length} 个行驶段；绿色前进，橙色倒车。请确认后执行。`;
    message('固定路线预览完成');draw();updateUi();
  });
  $('loopCount').addEventListener('input',()=>{app.recordedPreview=null;updateUi();draw();});
  $('editRouteMode').onchange=()=>refreshBusiness();
  window.refreshBusiness=()=>{
    const ready=isOwner() && Boolean(app.state?.map_sync_ok) && !taskRunning() && !app.telemetry?.patrol?.active && !busy;
    for (const id of ['savePoint','saveRoute']) $(id).disabled=!ready;
    $('deletePoint').disabled=!ready || !pointSnapshot;$('deleteRoute').disabled=!ready || !routeSnapshot;
    $('networkField').hidden=$('editRouteMode').value!=='RECORDED_NETWORK';
    $('editPointPreset').disabled=!$('editPointInspect').checked;
    $('pickEditPoint').disabled=!projection;
    if (busy) {$('btnPreview').disabled=true;$('btnStart').disabled=true;}
  };
  $('editPointInspect').onchange=()=>refreshBusiness();
  async function reload() {
    await loadState();const mapping=currentMap();if (!mapping) return;
    const current=stamp(mapping), [pointResponse,networkResponse]=await Promise.all([
      fetch(`/api/local/maps/${encodeURIComponent(mapping.map_id)}/points`,{cache:'no-store'}),
      fetch(`/api/local/maps/${encodeURIComponent(mapping.map_id)}/networks`,{cache:'no-store'})]);
    if (!pointResponse.ok || !networkResponse.ok) throw new Error('点位或路网资料读取失败');
    const p=await pointResponse.json(),n=await networkResponse.json();if (current!==stamp(currentMap())) return;
    points=p.items || [];networks=n.items || [];
    options($('editPoint'),points,'point_id','新增点位');options($('addStepPoint'),points,'point_id');
    options($('editRoute'),app.routes,'route_id','新增路线');options($('editNetwork'),networks.filter(item=>item.published && item.map_revision===mapping.revision),'network_id','请选择路网');
    if (!pointMap) pointMap={...mapping};if (!routeMap) routeMap={...mapping};
    renderSteps();
    try {
      const next=await fetchSiteMapMeta();if (siteMapVersion(next)!==siteMapVersion(meta)) {await loadSiteMap(image,next);meta=next;projection=new SiteMapProjection(meta);}
      draw();
    } catch(error) {meta=projection=null;$('previewMessage').textContent=error.message;draw();}
    refreshBusiness();
  }
  $('refreshEditor').onclick=()=>reload().then(()=>message('资料已刷新；编辑中的字段保留，版本冲突时请重新选择。')).catch(error=>message(error.message));
  function draw() {
    const width=canvas.clientWidth || 600;canvas.width=width;canvas.height=Math.min(400,width*.75);ctx.clearRect(0,0,canvas.width,canvas.height);
    if (!projection || !meta || !image.complete) return;
    const scale=Math.min(canvas.width/meta.width,canvas.height/meta.height);
    view={scale,panX:(canvas.width-meta.width*scale)/2,panY:(canvas.height-meta.height*scale)/2};
    ctx.drawImage(image,view.panX,view.panY,meta.width*scale,meta.height*scale);
    if (previewIsCurrent()) for (const block of app.recordedPreview.blocks) {
      ctx.strokeStyle=block.direction===1?'#16c79a':'#ffa45c';ctx.lineWidth=3;ctx.beginPath();
      block.samples.forEach((sample,index)=>{const [x,y]=projection.worldToCanvas(sample.x,sample.y,view);if (index) ctx.lineTo(x,y);else ctx.moveTo(x,y);});ctx.stroke();
    }
    for (const point of points) {const [x,y]=projection.worldToCanvas(point.x,point.y,view);ctx.fillStyle='#176be8';ctx.beginPath();ctx.arc(x,y,4,0,Math.PI*2);ctx.fill();ctx.fillText(point.name,x+7,y-7);}
    if ($('editPointX').value!=='' && $('editPointY').value!=='' && stamp(pointMap)===stamp(currentMap())) {
      const [x,y]=projection.worldToCanvas(Number($('editPointX').value),Number($('editPointY').value),view),yaw=projection.worldYawToScreen(Number($('editPointYaw').value)*Math.PI/180);
      ctx.strokeStyle='#f3bd4a';ctx.lineWidth=3;ctx.beginPath();ctx.arc(x,y,7,0,Math.PI*2);ctx.moveTo(x,y);ctx.lineTo(x+25*Math.cos(yaw),y+25*Math.sin(yaw));ctx.stroke();
    }
  }
  function world(event) {const bounds=canvas.getBoundingClientRect();return projection.canvasToWorld(event.clientX-bounds.left,event.clientY-bounds.top,view);}
  $('pickEditPoint').onclick=()=>{picking=!picking;canvas.style.cursor=picking?'crosshair':'default';$('previewMessage').textContent=picking?'在地图按下并拖动设置点位与朝向。':'已退出地图选点。';if (picking) canvas.scrollIntoView({block:'center',behavior:'smooth'});};
  canvas.onpointerdown=event=>{if (!picking || !projection) return;canvas.setPointerCapture(event.pointerId);origin=world(event);pointMap={...currentMap()};$('editPointX').value=origin[0].toFixed(3);$('editPointY').value=origin[1].toFixed(3);draw();};
  canvas.onpointermove=event=>{if (!origin) return;const [x,y]=world(event);if (Math.hypot(x-origin[0],y-origin[1])>.02) $('editPointYaw').value=(Math.atan2(y-origin[1],x-origin[0])*180/Math.PI).toFixed(1);draw();};
  canvas.onpointerup=canvas.onpointercancel=()=>{origin=null;picking=false;canvas.style.cursor='default';};
  for (const id of ['editPointX','editPointY','editPointYaw']) $(id).addEventListener('input',draw);
  addEventListener('resize',draw);
  reload().catch(error=>message(error.message));
})();
