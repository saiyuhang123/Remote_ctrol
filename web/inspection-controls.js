/* Map targets and robot audio share the console's existing owned connection. */
(() => {
  'use strict';
  let points = [], selected = null, picking = false, pickOrigin = null;
  let targetStamp = '', pointsStamp = '', pending = '', assets = [], homeStamp = '';
  let lastAudio = '', lastTask = '', lastPointsLoad = 0;
  let capturePending='';
  $('manualCapture').onclick=()=>{
    const id=requestId('capture');
    if (sendControl('capture',{request_id:id})) {
      capturePending=id;$('captureMessage').textContent='正在抓拍…';refresh();
      setTimeout(()=>{if (capturePending===id) {capturePending='';$('captureMessage').textContent='抓拍响应超时，请检查抓拍节点';refresh();}},10000);
    }
  };
  const stamp = () => app.mapMeta ? `${app.mapMeta.map_id}:${app.mapMeta.revision}` : '';
  const pointMessage = text => { $('pointMessage').textContent = text; };
  function markTarget() {
    targetStamp = stamp(); drawMap();
  }
  function useCoordinates() {
    selected = null; $('targetPoint').value = ''; markTarget();
  }
  for (const id of ['targetX','targetY','targetYaw','targetPreset','targetPrompt','targetInspect']) {
    $(id).addEventListener('input', useCoordinates);
  }
  $('targetPoint').onchange = () => {
    selected = points.find(item => item.point_id === $('targetPoint').value) || null;
    if (!selected) return;
    $('targetX').value = selected.x; $('targetY').value = selected.y;
    $('targetYaw').value = (selected.yaw*180/Math.PI).toFixed(1);
    $('targetInspect').checked = Boolean(selected.inspection_enabled);
    $('targetPreset').value = selected.ptz_preset || 1;
    $('targetPrompt').value = selected.recognition_prompt || '';
    markTarget(); pointMessage(`已选择：${selected.name}`);
  };
  $('pickTarget').onclick = () => {
    picking = !picking; app.dragging = false;
    $('pickTarget').textContent = picking ? '退出地图选点' : '地图选点与朝向';
    canvas.style.cursor = picking ? 'crosshair' : 'grab';
    if (picking) pointMessage('点击地图选择目标；按住并拖动设置到达朝向。');
  };
  function world(event) {
    const box = canvas.getBoundingClientRect();
    return app.projection.canvasToWorld(event.clientX-box.left, event.clientY-box.top, app.view);
  }
  canvas.addEventListener('pointerdown', event => {
    if (!picking || !app.mapReady || !app.projection) return;
    event.preventDefault(); event.stopImmediatePropagation();
    canvas.setPointerCapture(event.pointerId);
    const [x,y] = world(event); pickOrigin = [x,y];
    $('targetX').value = x.toFixed(3); $('targetY').value = y.toFixed(3);
    useCoordinates();
  }, true);
  canvas.addEventListener('pointermove', event => {
    if (!pickOrigin) return;
    event.stopImmediatePropagation();
    const [x,y] = world(event), [ox,oy] = pickOrigin;
    if (Math.hypot(x-ox,y-oy) > .03) $('targetYaw').value = (Math.atan2(y-oy,x-ox)*180/Math.PI).toFixed(1);
    drawMap();
  }, true);
  function finishPick(event) {
    if (!pickOrigin) return;
    event.stopImmediatePropagation(); pickOrigin = null; picking = false;
    $('pickTarget').textContent = '地图选点与朝向'; canvas.style.cursor = 'grab';
    pointMessage(`目标 ${$('targetX').value}, ${$('targetY').value} m · 朝向 ${$('targetYaw').value}°`);
  }
  canvas.addEventListener('pointerup', finishPick, true);
  canvas.addEventListener('pointercancel', finishPick, true);
  window.renderInspectionTarget = () => {
    if (!targetStamp || targetStamp !== stamp() || !app.projection) return;
    const x = Number($('targetX').value), y = Number($('targetY').value), yaw = Number($('targetYaw').value)*Math.PI/180;
    const [px,py] = app.projection.worldToCanvas(x,y,app.view);
    ctx.save(); ctx.translate(px,py); ctx.rotate(app.projection.worldYawToScreen(yaw));
    ctx.strokeStyle = '#ffd166'; ctx.fillStyle = '#ffd166'; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.arc(0,0,9,0,2*Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(26,0); ctx.lineTo(18,-6); ctx.moveTo(26,0); ctx.lineTo(18,6); ctx.stroke();
    ctx.restore();
  };
  $('pointStart').onclick = () => {
    if (!app.mapReady || !app.mapMeta || targetStamp !== stamp()) return pointMessage('地图已更新，请重新选择目标。');
    const data = {type:'point_start', request_id:requestId('point'), session_id:app.session,
      map_id:app.mapMeta.map_id, map_revision:app.mapMeta.revision};
    if (selected) Object.assign(data, {point_id:selected.point_id, point_revision:selected.revision});
    else {
      const x=Number($('targetX').value), y=Number($('targetY').value), degrees=Number($('targetYaw').value);
      const preset=Number($('targetPreset').value), enabled=$('targetInspect').checked;
      if (![x,y,degrees].every(Number.isFinite) || (enabled && (!Number.isInteger(preset) || preset<1 || preset>255)))
        return pointMessage('请填写有效坐标、朝向及云台预置位。');
      Object.assign(data, {x,y,yaw:degrees*Math.PI/180,inspection_enabled:enabled,
        ptz_preset:enabled?preset:0, recognition_prompt:$('targetPrompt').value});
    }
    if (sendRaw(data)) { pending=data.request_id; pointMessage('正在提交到点巡检任务…'); refresh(); }
    else pointMessage('连接已断开，请等待重连。');
  };
  $('pointCancel').onclick = () => sendControl('route_cancel');
  $('pointHome').onclick = () => sendControl('return_home', {map_id:app.mapMeta.map_id,map_revision:app.mapMeta.revision});
  const previousMessage = app.ws.onmessage;
  app.ws.onmessage = event => {
    previousMessage(event);
    let data; try { data=JSON.parse(event.data); } catch (_) { return; }
    if (data.type==='command_result' && data.command==='point_start' && data.request_id===pending) {
      pointMessage(data.message || data.code); if (!data.ok) pending='';
    }
    if (data.type==='telemetry' && data.task && data.task.kind==='point' && data.task.running) pending='';
    if (data.type==='task_finished' && data.kind==='point') {
      pending=''; pointMessage((data.success?'巡检完成：':'任务结束：')+(data.message || data.code || ''));
    }
    if (data.type==='command_result' && ['audio_play','audio_stop'].includes(data.command))
      $('audioMessage').textContent=data.message || data.code;
    if (data.type==='command_result' && data.command==='capture' && data.request_id===capturePending) {
      $('captureMessage').textContent=data.message || data.code;
      if (data.code!=='PENDING' || !data.ok) capturePending='';
      const path=data.data?.image;
      if (data.ok && typeof path==='string' && /^captures\/[0-9-]+\/[\w.-]+\.jpg$/.test(path)) {
        $('capturePreview').hidden=false;$('capturePreview').href='/'+path;$('capturePreview').querySelector('img').src='/'+path;
      }
    }
    refresh();
  };
  async function loadPoints() {
    if (!app.mapReady || !app.mapMeta || !app.mapMeta.map_id) return;
    const requested = stamp();
    try {
      const [response, localResponse] = await Promise.all([
        fetch(`/api/local/maps/${encodeURIComponent(app.mapMeta.map_id)}/points`, {cache:'no-store'}),
        fetch('/api/local/state', {cache:'no-store'})]);
      const data = await response.json(); if (!response.ok) throw new Error(data.detail || '点位读取失败');
      const local = await localResponse.json();
      if (requested!==stamp()) return;
      homeStamp = localResponse.ok && local.map_sync_ok && local.home &&
        local.active_map?.map_id===app.mapMeta.map_id && local.active_map?.revision===app.mapMeta.revision ? requested : '';
      points=Array.isArray(data.items)?data.items:[]; pointsStamp=requested;
      const previous=$('targetPoint').value;
      $('targetPoint').replaceChildren(new Option('在地图上选点或输入坐标',''));
      points.forEach(point => $('targetPoint').add(new Option(`${point.name}${point.inspection_enabled?' · 抓拍识别':''}`,point.point_id)));
      if (points.some(point=>point.point_id===previous)) $('targetPoint').value=previous;
      if (selected && !points.some(point=>point.point_id===selected.point_id && point.revision===selected.revision)) {
        selected=null; targetStamp=''; pointMessage('巡检点已更新，请重新选择。');
      }
    } catch(error) { homeStamp=''; pointMessage(error.message); }
  }
  async function loadAudio(preferred='') {
    const response=await fetch('/api/audio/files',{cache:'no-store'}), data=await response.json();
    if (!response.ok) throw new Error(data.detail || '音频列表读取失败');
    assets=Array.isArray(data.items)?data.items:[];
    const previous=preferred || $('audioSelect').value;
    $('audioSelect').replaceChildren(new Option(assets.length?'请选择音频':'暂无音频',''));
    assets.forEach(item=>$('audioSelect').add(new Option(`${item.name} · ${Number(item.duration_seconds).toFixed(1)}s`,item.filename)));
    $('audioSelect').value=assets.some(item=>item.filename===previous)?previous:(assets[0]?.filename || '');
    refresh();
  }
  $('audioUpload').onclick=async()=>{
    const file=$('audioFile').files[0];
    if (!file) { $('audioMessage').textContent='请先选择音频文件。'; return; }
    if (file.size>20*1024*1024) { $('audioMessage').textContent='音频不能超过 20MB。'; return; }
    $('audioUpload').disabled=true; $('audioMessage').textContent='正在上传并检查音频…';
    try {
      const form=new FormData(); form.append('file',file);
      const response=await fetch('/api/audio/files',{method:'POST',body:form}), data=await response.json();
      if (!response.ok) throw new Error(data.detail || '上传失败');
      await loadAudio(data.filename); $('audioMessage').textContent=`已上传：${data.name}`;
    } catch(error) { $('audioMessage').textContent=error.message; }
    finally { $('audioUpload').disabled=false; }
  };
  $('audioPlay').onclick=()=>sendControl('audio_play',{filename:$('audioSelect').value});
  $('audioStop').onclick=()=>sendControl('audio_stop');
  function refresh() {
    const t=app.telemetry || {}, control=t.control || {}, task=t.task || {}, audio=t.audio || {};
    const fresh=Date.now()-app.telemetryReceivedAt<3000;
    const owner=isOwner() && fresh && app.ws.readyState===1;
    $('manualCapture').disabled=!owner || !t.capture?.available || Boolean(capturePending);
    if (!t.capture?.available && !capturePending) $('captureMessage').textContent='抓拍节点未就绪或视频没有新鲜画面';
    const pointBusy=task.running && task.kind==='point';
    const ready=app.mapReady && targetStamp===stamp() && $('targetX').value!=='' && $('targetY').value!=='';
    $('pointStart').textContent=$('targetInspect').checked?'到达并巡检':'一键到达';
    $('pointStart').disabled=!owner || !ready || Boolean(pending) || task.running || t.patrol?.active || control.estop_known!==true || control.estopped!==false;
    $('pointCancel').disabled=!owner || !(pointBusy || pending);
    $('pointHome').disabled=!owner || !app.mapReady || homeStamp!==stamp() || task.kind==='return' || Boolean(task.pending_return);
    $('pointHome').title=homeStamp!==stamp()?'请先在本地设置当前地图的待命返航点':'';
    $('pickTarget').disabled=!app.mapReady;
    $('targetPreset').disabled=!$('targetInspect').checked;
    $('audioPlay').disabled=!owner || !audio.available || audio.intercom_active || !$('audioSelect').value;
    $('audioStop').disabled=!owner || !audio.available;
    const audioKey=JSON.stringify([audio.state,audio.request_id,audio.message,audio.available,fresh]);
    if (audioKey!==lastAudio) {
      lastAudio=audioKey;
      const title={idle:'播放器就绪',queued:'等待播报',playing:'机器人正在播放',done:'播报完成',stopped:'已停止',error:'播放失败'}[audio.state] || '';
      $('audioMessage').textContent=!fresh?'连接已断开，等待状态更新':!audio.available?'机器人音频播放器未启动或离线':
        title+(audio.message && audio.message!==title?' · '+audio.message:'');
    }
    const taskKey=JSON.stringify(t.patrol || {});
    if (pointBusy && taskKey!==lastTask) { lastTask=taskKey; pointMessage(t.patrol?.message || '正在前往巡检点'); }
    if ((!pointsStamp || pointsStamp!==stamp() || Date.now()-lastPointsLoad>15000) && app.mapReady) {
      lastPointsLoad=Date.now(); pointsStamp=stamp(); loadPoints();
    }
  }
  loadAudio().catch(error=>$('audioMessage').textContent=error.message);
  setInterval(refresh,250); refresh();
})();
