(() => {
  'use strict';
  const frame = document.getElementById('view');
  const pages = {monitor:'monitor.html', tasks:'tasks.html', records:'records.html'};
  const viewRevision='lease-20261029-1';
  let socket, session = null, telemetry = null, control = {}, task = {};
  let viewId = '', currentPage = '', reconnectTimer;
  let socketUrl='';
  let returnState = null, returnStateAt = 0, loadingReturnState = false, returnPending = '';
  let resetPending = '';
  const notice = text => {
    const node = document.getElementById('taskNotice');
    node.textContent = text; node.hidden = !text;
  };
  window.RobotConsole={get session(){return session;},get socketUrl(){return socketUrl;}};
  function post(kind, data) {
    if (viewId) frame.contentWindow.postMessage({robotConsole:true, id:viewId, kind, data}, location.origin);
  }
  function send(data) {
    if (!socket || socket.readyState !== WebSocket.OPEN) return false;
    socket.send(JSON.stringify(data)); return true;
  }
  function command(data) {
    if (!['control_claim','estop','estop_release'].includes(data.type)) {
      if (!session) {
        post('data', {type:'command_result',command:data.type,request_id:data.request_id,
          ok:false,code:'NOT_OWNER',message:'控制权已释放，请重新取得'});
        return false;
      }
      data = {...data, session_id:session};
    }
    const sent = send(data);
    if (!sent) post('data', {type:'command_result',command:data.type,request_id:data.request_id,
      ok:false,code:'DISCONNECTED',message:'连接已断开，请等待重连'});
    return sent;
  }
  function sessionState() { post('data', {type:'session_state', session_id:session}); }
  function refreshHeader() {
    dispatchEvent(new Event('robot-console-state'));
    document.getElementById('owner').textContent = control.local_control ? '本地控制优先' :
      session ? '本网页控制中' : control.owner_present ? '其他网页控制中' : '未取得控制权';
    document.getElementById('release').disabled = !session;
    document.getElementById('owner').dataset.owned = Boolean(session);
    document.getElementById('task').textContent = task.running
      ? (task.kind === 'return' ? '正在返回待命点' : task.kind === 'point' ? '到点巡检进行中' : '巡航任务进行中') : '';
    const fresh = Boolean(telemetry) && Date.now()-telemetry.received_at_ms < 3000;
    const connected = socket && socket.readyState === WebSocket.OPEN;
    const map = returnState?.active_map, home = returnState?.home;
    const validHome = Date.now()-returnStateAt < 10000 && returnState?.map_sync_ok && map && home &&
      home.map_id === map.map_id && home.map_revision === map.revision;
    const owner = Boolean(session) && fresh && connected && !control.local_control;
    const returning = Boolean(returnPending || task.pending_return || (task.running && task.kind === 'return'));
    const motionReady = control.estop_known === true && control.estopped === false && !telemetry?.desktop?.pending?.estop_release;
    const guard = telemetry?.patrol_guard || {};
    const button = document.getElementById('returnHome');
    button.disabled = !owner || !motionReady || !validHome || returning || guard.ready !== true;
    button.textContent = returning ? '正在返回…' : '一键返回';
    button.title = !validHome ? '请先在本地“地图与返航设置”保存当前地图待命点' :
      !owner ? '请先取得控制权并等待新鲜状态' : !motionReady ? '请先解除急停并等待确认' :
      '结束当前路线或到点巡检，确认停止后返回待命点';
    document.getElementById('cancelReturn').disabled = !owner || !returning;
    const busy = task.running || telemetry?.patrol?.active || telemetry?.nav?.active_goal_count > 0;
    const reset = document.getElementById('resetPatrol');
    reset.disabled = !owner || !motionReady || busy || !guard.reset_available || guard.ready === true ||
      Boolean(resetPending || guard.reset_pending);
    reset.textContent = resetPending || guard.reset_pending ? '正在复位…' : '复位巡航联锁';
    reset.title = '明确复位：核对旧任务与 Nav2 目标已终止、里程计新鲜且车辆停稳；不会恢复旧任务';
    const guardNotice = document.getElementById('guardNotice');
    guardNotice.hidden = !fresh || guard.ready === true || !guard.available;
    guardNotice.textContent = `巡航联锁未就绪：${guard.fault_code || ''} ${guard.reason || ''}。`+
      '旧任务不会自动恢复；确认停稳后点击“复位巡航联锁”，待就绪后重新开始任务。';
  }
  async function loadReturnState() {
    if (loadingReturnState) return;
    loadingReturnState = true;
    const abort = new AbortController(), timer = setTimeout(() => abort.abort(), 3000);
    try {
      const response = await fetch('/api/local/state', {cache:'no-store', signal:abort.signal});
      if (!response.ok) throw new Error('地图状态不可用');
      returnState = await response.json(); returnStateAt = Date.now();
    } catch (_) { returnState = null; returnStateAt = 0; }
    finally { clearTimeout(timer); loadingReturnState = false; refreshHeader(); }
  }
  function stopInteraction() {
    if (!session) return;
    if (!task.running && control.mode === 'manual') command({type:'cmd_vel',linear:0,angular:0});
    command({type:'ptz_stop'});
  }
  function showPage(force=false) {
    const page = location.hash.slice(1) in pages ? location.hash.slice(1) : 'monitor';
    if (page === currentPage && !force) return;
    if (page !== currentPage) stopInteraction();
    viewId = ''; currentPage = page;
    frame.src = `${pages[page]}?v=${viewRevision}${force ? '&reload='+Date.now() : ''}`;
    document.querySelectorAll('nav a').forEach(link => {
      link.classList.toggle('active', link.dataset.page === page);
      if (link.dataset.page === page) link.setAttribute('aria-current', 'page');
      else link.removeAttribute('aria-current');
    });
  }
  addEventListener('hashchange', () => showPage()); showPage();
  document.querySelector('nav a[data-page="records"]').addEventListener('click', event => {
    if (currentPage==='records') {
      event.preventDefault();
      // Reload only this read-only view; the outer control socket and its
      // task heartbeat remain alive.
      showPage(true);
    }
  });
  addEventListener('message', event => {
    if (event.origin !== location.origin || event.source !== frame.contentWindow || !event.data.robotView) return;
    const message = event.data;
    if (message.kind === 'ready') {
      viewId = message.id;
      post(socket && socket.readyState === WebSocket.OPEN ? 'open' : 'close');
      sessionState();
      if (telemetry) post('data', telemetry);
    } else if (message.id === viewId && message.kind === 'navigate' && message.page in pages) {
      location.hash = message.page;
    } else if (message.id === viewId && message.kind === 'command' && message.data) {
      command(message.data);
    }
  });
  function connect() {
    clearTimeout(reconnectTimer);
    const configuredPort = Number(new URL(location.href).searchParams.get('ws_port') || 8765);
    const port = Number.isInteger(configuredPort) && configuredPort > 0 && configuredPort <= 65535 ? configuredPort : 8765;
    socketUrl=`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.hostname}:${port}`;
    socket = new RobotControlConnection(socketUrl);
    socket.onopen = () => {
      document.getElementById('connection').textContent = '已连接'; post('open');
      document.getElementById('connection').dataset.state = 'connected';
    };
    socket.onmessage = event => {
      let data; try { data = JSON.parse(event.data); } catch (_) { return; }
      const before = session;
      if (data.type === 'command_result' && data.command === 'control_claim' && data.ok) session = data.session_id;
      if (data.type === 'command_result' && data.command === 'control_release' && data.ok) session = null;
      if (data.type === 'telemetry') {
        data.received_at_ms = Date.now();
        telemetry = data; control = data.control || {}; task = data.task || {};
        if (task.running && task.kind === 'return' && task.request_id === returnPending) returnPending = '';
      }
      if (data.type === 'control_state') { control = data.control || {}; task = data.task || task; }
      if ((data.type === 'telemetry' || data.type === 'control_state') && !control.owner_present) session = null;
      if (data.type === 'command_result' && ['return_home','return_cancel'].includes(data.command)) {
        notice(data.message || (data.ok ? '请求已受理' : '返回请求失败：'+(data.code || '未知错误')));
        if (!data.ok && data.request_id === returnPending) returnPending = '';
        if (data.ok && data.command === 'return_cancel') returnPending = '';
      }
      if (data.type === 'command_result' && data.command === 'patrol_reset_interlock') {
        if (data.request_id === resetPending && (!data.ok || data.code !== 'PENDING')) resetPending = '';
        notice(data.message || data.code);
      }
      if (data.type === 'task_finished' && data.kind === 'return') {
        if (data.request_id === returnPending) returnPending = '';
        notice((data.success ? '返回完成：' : '返回结束：')+(data.message || data.code || ''));
      }
      if (before && !session) { returnPending = ''; resetPending = ''; }
      if (before !== session) sessionState();
      post('data', data); refreshHeader();
    };
    socket.onclose = () => {
      session = telemetry = null; control = {}; task = {}; returnPending = ''; resetPending = '';
      document.getElementById('connection').textContent = '已断开，正在重连';
      document.getElementById('connection').dataset.state = 'disconnected';
      sessionState(); post('close'); refreshHeader();
      reconnectTimer = setTimeout(connect, 3000);
    };
    socket.onerror = () => {
      document.getElementById('connection').textContent = '连接异常';
      document.getElementById('connection').dataset.state = 'disconnected';
    };
  }
  document.getElementById('release').onclick = () => command({type:'control_release'});
  document.getElementById('stop').onclick = () => send({type:'estop'});
  document.getElementById('returnHome').onclick = () => {
    refreshHeader();
    if (document.getElementById('returnHome').disabled) return;
    stopInteraction();
    const id = 'home_'+Date.now()+'_'+Math.random().toString(16).slice(2);
    const map = returnState.active_map;
    if (command({type:'return_home',request_id:id,map_id:map.map_id,map_revision:map.revision})) {
      returnPending = id;
      notice('正在结束当前任务，确认停止后返回待命点。'); refreshHeader();
    }
  };
  document.getElementById('cancelReturn').onclick = () => {
    if (command({type:'return_cancel',request_id:'cancel_home_'+Date.now()})) notice('正在取消返回，请等待停车确认。');
  };
  document.getElementById('resetPatrol').onclick = () => {
    refreshHeader(); if (document.getElementById('resetPatrol').disabled) return;
    const id = 'reset_patrol_'+Date.now();
    if (command({type:'patrol_reset_interlock',request_id:id})) {
      resetPending = id; notice('正在核对任务终止和车辆停稳，请等待联锁复位结果。'); refreshHeader();
    }
  };
  setInterval(refreshHeader, 500);
  setInterval(loadReturnState, 5000); loadReturnState();
  document.addEventListener('visibilitychange', () => { if (document.hidden) stopInteraction(); });
  addEventListener('pagehide', () => {
    stopInteraction();
    // Closing or reloading the whole console really gives up the lease.
    if (session) command({type:'control_release'});
    if (socket) socket.close();
  });
  connect();
})();
