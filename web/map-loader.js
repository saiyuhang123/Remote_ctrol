// Shared Nav2 occupancy-map loader and affine world<->image projection.
// Nav2 origin yaw is retained. The source raster row 0 is the north/top edge
// of the map-local frame, so v = height - local_y/resolution.
async function fetchSiteMapMeta() {
  const url = `/api/map/meta?ts=${Date.now()}`;
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 5000);
  let response, data;
  try {
    response = await fetch(url, { cache: 'no-store', signal:controller.signal });
    data = await response.json();
  } finally { clearTimeout(timeout); }
  if (response.status === 404) {
    throw new Error(data.detail || '尚未配置实机地图');
  }
  if (!response.ok) throw new Error('网页地图配置加载失败');
  if (!Number.isInteger(data.width) || data.width <= 0 ||
      !Number.isInteger(data.height) || data.height <= 0 ||
      !Number.isFinite(data.resolution) || data.resolution <= 0 ||
      !Array.isArray(data.origin) || data.origin.length < 3 ||
      !data.origin.slice(0, 3).every(Number.isFinite) ||
      typeof data.image !== 'string' || !/^[\w.-]+\.(png|jpe?g)$/i.test(data.image)) {
    throw new Error('网页地图配置无效');
  }
  if (data.sync_valid === false) throw new Error(data.sync_error || '网页地图与本地地图版本不一致');
  return data;
}

function siteMapVersion(meta) {
  return meta ? JSON.stringify([meta.map_id, meta.revision, meta.sha256, meta.width,
    meta.height, meta.resolution, meta.origin]) : '';
}

async function loadSiteMap(image, meta = null) {
  const data = meta || await fetchSiteMapMeta();
  const version = encodeURIComponent(`${data.map_id || ''}-${data.revision || 0}-${String(data.sha256 || '').slice(0, 12)}`);
  await new Promise((resolve, reject) => {
    const finish = error => {
      clearTimeout(timeout); image.onload = image.onerror = null;
      if (error) reject(error); else resolve();
    };
    const timeout = setTimeout(() => finish(new Error('网页地图图片加载超时')), 5000);
    image.onload = () => finish();
    image.onerror = () => finish(new Error('网页地图图片加载失败'));
    image.src = `maps/${data.image}?v=${version}`;
  });
  if (image.naturalWidth !== data.width || image.naturalHeight !== data.height) {
    throw new Error('地图图片尺寸与配置不一致');
  }
  return data;
}

class SiteMapProjection {
  constructor(meta) {
    this.meta = meta;
    this.width = meta.width;
    this.height = meta.height;
    this.resolution = Number(meta.resolution);
    this.originX = Number(meta.origin[0]);
    this.originY = Number(meta.origin[1]);
    this.originYaw = Number(meta.origin[2] || 0);
    this.cos = Math.cos(this.originYaw);
    this.sin = Math.sin(this.originYaw);
  }

  worldToLocal(x, y) {
    const dx = Number(x) - this.originX;
    const dy = Number(y) - this.originY;
    return [this.cos * dx + this.sin * dy, -this.sin * dx + this.cos * dy];
  }

  localToWorld(x, y) {
    return [this.originX + this.cos * x - this.sin * y,
            this.originY + this.sin * x + this.cos * y];
  }

  // Image pixels [u,v], v grows downward.
  worldToImage(x, y) {
    const [lx, ly] = this.worldToLocal(x, y);
    return [lx / this.resolution, this.height - ly / this.resolution];
  }

  imageToWorld(u, v) {
    return this.localToWorld(Number(u) * this.resolution,
                             (this.height - Number(v)) * this.resolution);
  }

  worldToCanvas(x, y, view) {
    const [u, v] = this.worldToImage(x, y);
    return [view.panX + u * view.scale, view.panY + v * view.scale];
  }

  canvasToWorld(px, py, view) {
    return this.imageToWorld((px - view.panX) / view.scale,
                             (py - view.panY) / view.scale);
  }

  // World yaw -> canvas screen angle (canvas y points down).
  worldYawToScreen(yaw) { return -(Number(yaw) - this.originYaw); }
}

if (typeof window !== 'undefined') {
  window.loadSiteMap = loadSiteMap;
  window.fetchSiteMapMeta = fetchSiteMapMeta;
  window.siteMapVersion = siteMapVersion;
  window.SiteMapProjection = SiteMapProjection;
}
