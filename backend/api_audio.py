"""Audio asset upload; playback remains an owned ROS gateway command."""
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import time
import uuid

from fastapi import APIRouter, HTTPException, Request

router = APIRouter(tags=['audio'])
ROOT = Path(__file__).resolve().parents[1]
AUDIO_DIR = Path(os.environ.get('ROBOT_AUDIO_DIR', ROOT/'data/audio')).expanduser().resolve()
EXTENSIONS = ('.mp3', '.wav', '.ogg', '.m4a', '.flac')
MAX_BYTES = 20*1024*1024


@router.get('/api/audio/files')
def audio_files():
    if not AUDIO_DIR.exists():
        return {'items': []}
    items = []
    for path in AUDIO_DIR.glob('*.json'):
        try:
            item = json.loads(path.read_text(encoding='utf-8'))
            asset = (AUDIO_DIR/item['filename']).resolve()
            if asset.parent == AUDIO_DIR and asset.is_file() and asset.suffix in EXTENSIONS:
                items.append(item)
        except (OSError, ValueError, KeyError, TypeError):
            continue
    return {'items': sorted(items, key=lambda item: item.get('uploaded_at_ns', 0), reverse=True)}


@router.post('/api/audio/files')
async def upload_audio(request: Request):
    try:
        length = int(request.headers.get('content-length', '0') or 0)
    except ValueError as error:
        raise HTTPException(422, '上传长度无效') from error
    if length > MAX_BYTES+1024*1024:
        raise HTTPException(413, '音频不能超过 20MB')
    probe = shutil.which('ffprobe')
    if not probe:
        raise HTTPException(503, '请先安装 FFmpeg（包含 ffprobe）')
    form = await request.form()
    upload = form.get('file')
    if upload is None or not hasattr(upload, 'read'):
        raise HTTPException(422, '请选择音频文件')
    name = Path(str(upload.filename or '').replace('\\', '/')).name
    suffix = Path(name).suffix.lower()
    if suffix not in EXTENSIONS:
        await upload.close()
        raise HTTPException(422, '支持 MP3、WAV、OGG、M4A、FLAC 音频')
    AUDIO_DIR.mkdir(parents=True, exist_ok=True)
    temporary = AUDIO_DIR/(uuid.uuid4().hex+'.upload')
    metadata_temp = AUDIO_DIR/(uuid.uuid4().hex+'.tmp')
    size, digest = 0, hashlib.sha256()
    try:
        with temporary.open('wb') as stream:
            while True:
                chunk = await upload.read(1024*1024)
                if not chunk:
                    break
                size += len(chunk)
                if size > MAX_BYTES:
                    raise HTTPException(413, '音频不能超过 20MB')
                digest.update(chunk)
                stream.write(chunk)
        if not size:
            raise HTTPException(422, '音频文件为空')
        try:
            result = subprocess.run([probe, '-v', 'error', '-show_entries',
                'format=duration:stream=codec_type', '-of', 'json', str(temporary)],
                capture_output=True, timeout=10, check=True)
            info = json.loads(result.stdout)
            duration = float(info.get('format', {}).get('duration', 0))
            if not any(item.get('codec_type') == 'audio' for item in info.get('streams', [])):
                raise ValueError('没有音轨')
            if not math.isfinite(duration) or not 0 < duration <= 600:
                raise ValueError('音频时长必须在 10 分钟以内')
        except (ValueError, subprocess.SubprocessError) as error:
            raise HTTPException(422, '音频无效或超过 10 分钟，请转换后重试') from error
        filename = digest.hexdigest()+suffix
        metadata = dict(filename=filename, name=name[:200], size=size,
                        duration_seconds=duration, uploaded_at_ns=time.time_ns())
        metadata_temp.write_text(json.dumps(metadata, ensure_ascii=False), encoding='utf-8')
        os.replace(temporary, AUDIO_DIR/filename)
        os.replace(metadata_temp, AUDIO_DIR/(filename+'.json'))
        return metadata
    finally:
        temporary.unlink(missing_ok=True)
        metadata_temp.unlink(missing_ok=True)
        await upload.close()
