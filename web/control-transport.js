/* The shell retains this transport while its same-origin views are replaced. */
(() => {
  'use strict';
  window.RobotControlConnection = class {
    constructor(url) {
      this.readyState = WebSocket.CONNECTING;
      this.worker = new Worker('control-worker.js?v=lease-20261029-1');
      this.worker.onmessage = event => {
        const {kind,data} = event.data;
        if (kind === 'open') { this.readyState=WebSocket.OPEN; this.onopen?.(); }
        else if (kind === 'message') this.onmessage?.({data});
        else if (kind === 'error') this.onerror?.(new Event('error'));
        else if (kind === 'close') {
          this.readyState=WebSocket.CLOSED; clearInterval(this.pulse);
          this.worker.terminate(); this.onclose?.(data);
        }
      };
      this.worker.onerror = () => {
        this.readyState=WebSocket.CLOSED; clearInterval(this.pulse); this.worker.terminate();
        this.onerror?.(new Event('error')); this.onclose?.({reason:'控制连接工作线程退出'});
      };
      this.pulse = setInterval(() => this.worker.postMessage({kind:'pulse'}),250);
      this.worker.postMessage({kind:'connect',data:url});
    }
    send(data) { if (this.readyState===WebSocket.OPEN) this.worker.postMessage({kind:'send',data}); }
    close() {
      clearInterval(this.pulse);
      this.readyState=WebSocket.CLOSING; this.worker.postMessage({kind:'close'});
    }
  };
})();
