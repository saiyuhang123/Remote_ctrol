// Shared ownership/unlock UI for monitor and task views.
(() => {
  'use strict';
  const el = id => document.getElementById(id);
  function update(app) {
    const t = app.telemetry || {}, c = t.control || {}, local = t.desktop || {};
    const connected = app.ws?.readyState === 1 && Number.isFinite(t.received_at_ms) && Date.now()-t.received_at_ms < 3000;
    const reasons = [];
    if (!connected) reasons.push('等待网关连接和新鲜状态');
    if (app.session) reasons.push('本网页已取得控制权');
    else if (c.owner_present) reasons.push('其他网页正在控制，请先由该网页释放');
    if (c.local_control) reasons.push('Qt 仍启用了操作，请先取消顶部“启用操作”');
    if (app.unlockPending || local.pending?.estop_release) reasons.push('正在等待本地解除急停反馈');
    el('btnClaim').disabled = reasons.length > 0;
    el('btnClaim').title = reasons.join('；') || '取得本网页控制权';
    const stopHint = c.estop_known !== true ? '急停状态未知：可操作设备，行驶前需解除急停。' :
      c.estopped ? '急停保持中：可操作设备，行驶前需解除急停。' : '急停已解除；行驶和任务需由你明确启动。';
    el('controlHint').textContent = (reasons.join('；') || '可取得控制权。') + ' ' + stopHint;
    const otherOwner = c.owner_present && !app.session;
    const busy = t.task?.running || t.patrol?.active || t.nav?.active_goal_count > 0;
    const pending = app.unlockPending || local.pending?.estop_release;
    el('btnUnlock').disabled = !connected || c.local_control || otherOwner || busy ||
      !local.unlock_available || pending || (c.estop_known === true && c.estopped === false);
    el('btnUnlock').textContent = pending ? '正在解除急停…' : '解除急停（保持静止）';
    el('btnUnlock').title = !local.unlock_available ? '请更新并重新打开 Qt，以提供本地解除急停服务' :
      c.local_control ? '请先在 Qt 关闭“启用操作”' : busy ? '请先结束活动任务并停稳' :
      '由本地控制器检查新鲜里程计、连续停稳和活动目标后执行；不会恢复旧任务';
  }
  function release(app, send, report, changed) {
    if (app.unlockPending) return;
    const id = `unlock_${Date.now()}_${Math.random().toString(16).slice(2)}`;
    app.unlockRequestId = id;
    if (!send({type:'estop_release',request_id:id,session_id:app.session || ''})) {
      report('连接已断开，解除急停未提交', true); return;
    }
    app.unlockPending = true;
    report('正在请求本地检查停稳并解除急停…'); changed();
    setTimeout(() => {
      if (app.unlockPending && app.unlockRequestId === id) {
        app.unlockPending = false; app.unlockRequestId = '';
        report('解除急停等待超时，请核对本地状态后再试', true); changed();
      }
    }, 8000);
  }
  function result(app, data, report, changed) {
    if (data.command !== 'estop_release') return false;
    if (data.request_id !== app.unlockRequestId) return true;
    app.unlockPending = false; app.unlockRequestId = '';
    report(data.message || data.code, !data.ok); changed(); return true;
  }
  window.RobotControlAccess = {update, release, result};
})();
