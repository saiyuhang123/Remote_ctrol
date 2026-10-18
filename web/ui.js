// Local, decorative icons. No network fonts, UI framework, or control logic.
(() => {
  'use strict';
  const paths = {
    robot: '<rect x="4" y="7" width="16" height="13" rx="4"/><path d="M12 3v4M9 3h6M1 12v4m22-4v4M8 20v2m8-2v2M9 16h6"/><circle cx="8" cy="12" r="1"/><circle cx="16" cy="12" r="1"/>',
    monitor: '<rect x="3" y="3" width="18" height="14" rx="3"/><path d="M8 21h8m-4-4v4M6 11h3l2-4 3 7 2-3h2"/>',
    route: '<circle cx="6" cy="5" r="2"/><circle cx="18" cy="19" r="2"/><path d="M8 5h8a4 4 0 0 1 0 8H8a3 3 0 0 0 0 6h8"/>',
    records: '<rect x="5" y="4" width="14" height="17" rx="2"/><path d="M9 3h6v4H9zM9 12h6m-6 4h6"/>',
    shield: '<path d="m12 3 8 3v6c0 5-8 9-8 9s-8-4-8-9V6z"/><path d="m8 12 3 3 5-6"/>',
    video: '<rect x="3" y="5" width="13" height="14" rx="3"/><path d="m16 10 5-3v10l-5-3z"/>',
    gas: '<path d="M12 3s-7 7-7 12a7 7 0 0 0 14 0c0-5-7-12-7-12Z"/><path d="M9 16a3 3 0 0 0 3 3"/>',
    compass: '<circle cx="12" cy="12" r="9"/><path d="m16 8-3 5-5 3 3-5z"/>',
    joystick: '<path d="M12 6v8m-4 3h8M5 14l-2 6h18l-2-6h-4M9 14H5"/><circle cx="12" cy="4" r="2"/>',
    ptz: '<circle cx="12" cy="10" r="6"/><circle cx="12" cy="10" r="2"/><path d="M8 16v4h8v-4M5 21h14M3 7 1 5m20 2 2-2"/>',
    scan: '<path d="M8 3H3v5m13-5h5v5M3 16v5h5m13-5v5h-5M7 12h10m-7-4h4m-4 8h4"/>',
    locate: '<circle cx="12" cy="12" r="6"/><circle cx="12" cy="12" r="2"/><path d="M12 2v4m0 12v4M2 12h4m12 0h4"/>',
    gauge: '<path d="M4 18a9 9 0 1 1 16 0M5 10l2 1m-1 5h2m4-12v3m6 3-2 1m2 5h-2m-4 0 4-6"/><circle cx="12" cy="16" r="2"/>',
    arrow: '<path d="M5 12h14m-6-6 6 6-6 6"/>',
    refresh: '<path d="M20 7v5h-5M4 17v-5h5"/><path d="M6 7a7 7 0 0 1 11-2l3 3M4 16l3 3a7 7 0 0 0 11-2"/>',
    stop: '<path d="m8 3-5 5v8l5 5h8l5-5V8l-5-5Z"/><path d="M12 7v6m0 3v1"/>',
    home: '<path d="m3 10 9-7 9 7m-16-1v12h14V9M9 21v-8h6v8"/>',
    play: '<path d="m8 4 12 8-12 8z"/>',
    face: '<path d="M8 3H3v5m13-5h5v5M3 16v5h5m13-5v5h-5M8 9v1m8-1v1m-8 5c2 2 6 2 8 0"/>',
    person: '<circle cx="12" cy="6" r="3"/><path d="M5 21v-3a7 7 0 0 1 14 0v3"/>',
    plate: '<rect x="2" y="6" width="20" height="12" rx="3"/><path d="M6 10v4m4-4v4m4-4v4m4-4v4"/>',
  };
  document.querySelectorAll('[data-icon]').forEach(node => {
    if (!paths[node.dataset.icon]) return;
    node.innerHTML = `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.65" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${paths[node.dataset.icon]}</svg>`;
  });
})();
