/* One owned connection, independent of map/video rendering on the UI thread. */
'use strict';
let socket = null, session = '', generation = -1, claimedGeneration = -1;
let lastUiPulse = performance.now(), heartbeatTimer = null;
const UI_TIMEOUT_MS = 5000;
const emit = (kind, data) => postMessage({kind, data});
function receive(raw) {
  let data; try { data = JSON.parse(raw); } catch (_) { return; }
  if (data.type === 'command_result' && data.command === 'control_claim' && data.ok) {
    const revision = data.control_revision;
    if (Number.isInteger(revision) && revision < generation) {
      data = {...data, ok:false, code:'CONTROL_REVOKED', message:'取权期间控制状态已变化，请重新取得控制权'};
    } else {
      session = data.session_id || '';
      claimedGeneration = Number.isInteger(revision) ? revision : -1;
      generation = Math.max(generation, claimedGeneration);
      heartbeat();
    }
  }
  if (data.type === 'telemetry' || data.type === 'control_state') {
    const revision = data.control?.revision;
    // A queued snapshot from before Claim cannot erase the new lease.
    if (Number.isInteger(revision) && revision < generation) return;
    if (Number.isInteger(revision)) generation = revision;
    if (!data.control?.owner_present || data.control?.local_control ||
        (session && claimedGeneration >= 0 && generation !== claimedGeneration)) session = '';
  }
  if (data.type === 'command_result' &&
      ((data.command === 'control_release' && data.ok) || data.code === 'NOT_OWNER' || data.code === 'CONTROL_REVOKED')) session = '';
  emit('message', JSON.stringify(data));
}
function heartbeat() {
  if (!session || socket?.readyState !== WebSocket.OPEN) return;
  if (performance.now()-lastUiPulse > UI_TIMEOUT_MS) {
    socket.send(JSON.stringify({type:'control_release', session_id:session}));
    session = '';
    socket.close(4000, 'Control UI unresponsive');
    return;
  }
  socket.send(JSON.stringify({type:'control_heartbeat', session_id:session}));
}
onmessage = event => {
  const {kind, data} = event.data;
  if (kind === 'pulse') { lastUiPulse = performance.now(); return; }
  if (kind === 'connect') {
    if (socket) return;
    lastUiPulse = performance.now();
    socket = new WebSocket(data);
    socket.onopen = () => emit('open');
    socket.onmessage = event => receive(event.data);
    socket.onerror = () => emit('error');
    socket.onclose = event => {
      session = ''; clearInterval(heartbeatTimer);
      emit('close', {code:event.code, reason:event.reason}); close();
    };
    heartbeatTimer = setInterval(heartbeat, 250);
  } else if (kind === 'send' && socket?.readyState === WebSocket.OPEN) {
    lastUiPulse = performance.now();
    let command; try { command=JSON.parse(data); } catch (_) { return; }
    if (!['control_claim','estop','estop_release'].includes(command.type) &&
        (!session || command.session_id !== session)) {
      emit('message', JSON.stringify({type:'command_result',command:command.type,
        request_id:command.request_id,ok:false,code:'NOT_OWNER',message:'控制权已释放，请重新取得'}));
      return;
    }
    socket.send(data);
  } else if (kind === 'close') {
    clearInterval(heartbeatTimer); session = '';
    if (socket) socket.close();
  }
};
