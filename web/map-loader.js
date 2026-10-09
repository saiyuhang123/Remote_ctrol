// 两个地图页面共用实测地图；未配置或加载失败时由页面显示原因。
async function loadSiteMap(image) {
  const response = await fetch('maps/meta.json', { cache: 'no-store' });
  if (response.status === 404) throw new Error('尚未配置实机地图');
  if (!response.ok) throw new Error('实机地图配置加载失败');
  const data = await response.json();
  if (!Number.isInteger(data.width) || data.width <= 0 ||
      !Number.isInteger(data.height) || data.height <= 0 ||
      !Number.isFinite(data.resolution) || data.resolution <= 0 ||
      !Array.isArray(data.origin) || data.origin.length < 3 ||
      !data.origin.slice(0, 3).every(Number.isFinite) ||
      typeof data.image !== 'string' || !/^[\w.-]+\.(png|jpe?g)$/i.test(data.image)) {
    throw new Error('实机地图配置无效');
  }
  if (data.origin[2] !== 0) throw new Error('当前地图页面仅支持原点朝向为 0 的地图');
  await new Promise((resolve, reject) => {
    image.onload = resolve;
    image.onerror = () => reject(new Error('实机地图图片加载失败'));
    image.src = `maps/${data.image}`;
  });
  if (image.naturalWidth !== data.width || image.naturalHeight !== data.height) {
    throw new Error('地图图片尺寸与配置不一致');
  }
  return data;
}
