(() => {
  'use strict';
  const frame = document.getElementById('view');
  const pages = {monitor:'monitor.html', tasks:'tasks.html', records:'records.html'};
  const viewRevision='records-tabs-20261026-2';
  let socket, session = null, telemetry = null, control = {}, task = {};
  let viewId = '', currentPage = '', reconnectTimer;
  let socketUrl='';
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
    socket = new WebSocket(socketUrl);
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
      }
      if (data.type === 'control_state') { control = data.control || {}; task = data.task || task; }
      if ((data.type === 'telemetry' || data.type === 'control_state') && !control.owner_present) session = null;
      if (before !== session) sessionState();
      post('data', data); refreshHeader();
    };
    socket.onclose = () => {
      session = telemetry = null; control = {}; task = {};
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
  setInterval(() => { if (session) command({type:'control_heartbeat'}); }, 300);
  document.getElementById('release').onclick = () => command({type:'control_release'});
  document.getElementById('stop').onclick = () => send({type:'estop'});
  document.addEventListener('visibilitychange', () => { if (document.hidden) stopInteraction(); });
  addEventListener('pagehide', () => {
    stopInteraction();
    // Closing or reloading the whole console really gives up the lease.
    if (session) command({type:'control_release'});
    if (socket) socket.close();
  });
  connect();
})();
