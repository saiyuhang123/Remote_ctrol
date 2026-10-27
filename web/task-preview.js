// Read-only task map and fixed-route preview; configuration lives in the desktop UI.
(() => {
  'use strict';
  let points = [], meta = null, projection = null, mapStamp = '';
  let busy = false, loading = false, mapError = false;
  const image = new Image(), canvas = $('businessMap'), ctx = canvas.getContext('2d');
  const pending = new Map();
  const message = text => { $('previewMessage').textContent = text; };
  const currentMap = () => app.state?.active_map;
  const stamp = map => map ? `${map.map_id}:${map.revision}` : '';
  const selectedRoute = () => app.routes.find(item => item.route_id === app.selectedRouteId);

  window.previewIsCurrent = () => {
    const plan = app.recordedPreview, route = selectedRoute(), map = currentMap();
    return Boolean(plan && map && route && plan.map_id === map.map_id && plan.map_revision === map.revision &&
      plan.route_id === route.route_id && plan.route_revision === route.revision && plan.loop_count === Number($('loopCount').value));
  };

  function requestPreview(data) {
    return new Promise((resolve, reject) => {
      const id = requestId('route_preview');
      const timer = setTimeout(() => {
        pending.delete(id); reject(new Error('路线预览响应超时，请重试'));
      }, 45000);
      pending.set(id, {resolve, reject, timer});
      if (!sendControl('route_preview', {...data, request_id:id})) {
        clearTimeout(timer); pending.delete(id); reject(new Error('连接已断开'));
      }
    });
  }
  const onMessage = app.ws.onmessage;
  app.ws.onmessage = event => {
    onMessage(event);
    let data; try { data = JSON.parse(event.data); } catch (_) { return; }
    if (data.type === 'command_result' && pending.has(data.request_id)) {
      const entry = pending.get(data.request_id);
      clearTimeout(entry.timer); pending.delete(data.request_id);
      if (data.ok) entry.resolve(data.data);
      else entry.reject(new Error(data.message || data.code));
    }
  };

  $('btnPreview').onclick = async () => {
    if (busy) return;
    busy = true; app.recordedPreview = null; updateUi();
    try {
      const route = selectedRoute(), map = currentMap(), loops = Number($('loopCount').value);
      if (!map || !route?.requires_preview || !Number.isInteger(loops) || loops < 0 || loops > 100000)
        throw new Error('请选择固定路线并填写有效圈数');
      message('正在检查道路与自动接入路径…');
      app.recordedPreview = await requestPreview({
        map_id:map.map_id, map_revision:Number(map.revision), route_id:route.route_id, loop_count:loops,
      });
      message(window.previewIsCurrent()
        ? `预览完成，共 ${app.recordedPreview.blocks.length} 个行驶段；绿色前进，橙色倒车。请确认后执行。`
        : '路线、地图或圈数已变化，请重新预览。');
    } catch (error) { message(error.message); }
    finally { busy = false; updateUi(); }
  };
  $('loopCount').addEventListener('input', () => { app.recordedPreview = null; updateUi(); });
  window.refreshTaskPreview = () => {
    if (busy) { $('btnPreview').disabled = true; $('btnStart').disabled = true; }
    if (stamp(currentMap()) !== mapStamp && !loading) reload();
    draw();
  };

  async function reload() {
    if (loading) return;
    const mapping = currentMap(), current = stamp(mapping);
    if (!mapping) { points = []; meta = projection = null; mapStamp = ''; draw(); return; }
    loading = true;
    if (current !== mapStamp) { points = []; meta = projection = null; draw(); }
    mapStamp = current;
    try {
      const [response, next] = await Promise.all([
        fetch(`/api/local/maps/${encodeURIComponent(mapping.map_id)}/points`, {cache:'no-store'}),
        fetchSiteMapMeta(),
      ]);
      if (!response.ok) throw new Error('巡检点预览资料读取失败');
      const data = await response.json();
      if (current !== stamp(currentMap())) return;
      if (siteMapVersion(next) !== siteMapVersion(meta)) await loadSiteMap(image, next);
      if (current !== stamp(currentMap())) return;
      points = data.items || []; meta = next; projection = new SiteMapProjection(meta);
      if (mapError && !busy) message('地图已恢复，可选择固定路线进行预览。');
      mapError = false;
    } catch (error) {
      points = []; meta = projection = null; mapError = true;
      if (!busy) message(error.message);
    } finally { loading = false; draw(); }
  }

  function draw() {
    const width = canvas.clientWidth || 600;
    canvas.width = width; canvas.height = Math.min(400, width * .75);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    if (!projection || !meta || !image.complete || stamp(currentMap()) !== mapStamp) return;
    const scale = Math.min(canvas.width / meta.width, canvas.height / meta.height);
    const view = {scale, panX:(canvas.width - meta.width * scale) / 2, panY:(canvas.height - meta.height * scale) / 2};
    ctx.drawImage(image, view.panX, view.panY, meta.width * scale, meta.height * scale);
    if (window.previewIsCurrent()) for (const block of app.recordedPreview.blocks) {
      ctx.strokeStyle = block.direction === 1 ? '#16c79a' : '#ffa45c';
      ctx.lineWidth = 3; ctx.beginPath();
      block.samples.forEach((sample, index) => {
        const [x, y] = projection.worldToCanvas(sample.x, sample.y, view);
        if (index) ctx.lineTo(x, y); else ctx.moveTo(x, y);
      });
      ctx.stroke();
    }
    for (const point of points) {
      const [x, y] = projection.worldToCanvas(point.x, point.y, view);
      ctx.fillStyle = '#176be8'; ctx.beginPath(); ctx.arc(x, y, 4, 0, Math.PI * 2);
      ctx.fill(); ctx.fillText(point.name, x + 7, y - 7);
    }
  }
  addEventListener('resize', draw);
  setInterval(reload, 5000);
  reload();
})();
