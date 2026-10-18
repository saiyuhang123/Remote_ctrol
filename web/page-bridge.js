// Page views never own a robot socket. The outer console survives view changes.
(() => {
  'use strict';
  const pages = { 'monitor.html':'monitor', 'tasks.html':'tasks', 'records.html':'records' };
  const page = pages[location.pathname.split('/').pop()] || 'monitor';
  if (window === window.top) {
    location.replace(`/console.html${location.search}#${page}`);
    window.RobotConnection = class { constructor() { this.readyState = 0; } send() {} };
    return;
  }
  document.documentElement.classList.add('embedded');
  const style = document.createElement('style');
  style.textContent = 'html.embedded header nav { display:none !important; }';
  document.head.appendChild(style);
  const id = `${Date.now()}_${Math.random().toString(16).slice(2)}`;
  const connections = new Set();
  const post = (kind, extra = {}) => parent.postMessage({robotView:true, id, kind, ...extra}, location.origin);
  window.RobotConnection = class {
    constructor() {
      this.readyState = 0;
      connections.add(this);
      post('ready');
    }
    send(raw) {
      if (this.readyState === 1) post('command', {data:JSON.parse(raw)});
    }
  };
  addEventListener('message', event => {
    if (event.origin !== location.origin || event.source !== parent ||
        !event.data.robotConsole || event.data.id !== id) return;
    const {kind, data} = event.data;
    for (const connection of connections) {
      if (kind === 'open') {
        const opened = connection.readyState !== 1;
        connection.readyState = 1;
        if (opened && connection.onopen) connection.onopen();
      } else if (kind === 'close') {
        connection.readyState = 0;
        if (connection.onclose) connection.onclose();
      } else if (kind === 'data' && connection.onmessage) {
        connection.onmessage({data:JSON.stringify(data)});
      }
    }
  });
  document.addEventListener('click', event => {
    const anchor = event.target.closest('a[href]');
    if (!anchor || event.button !== 0 || event.ctrlKey || event.metaKey || event.shiftKey || event.altKey) return;
    const url = new URL(anchor.href, location.href);
    const target = url.origin === location.origin && pages[url.pathname.split('/').pop()];
    if (target) { event.preventDefault(); post('navigate', {page:target}); }
  });
  // Records has no control transport, but its navigation still uses the shell.
  post('ready');
})();
