"""音频文件管理：上传/列表/删除（合同：特定时段或区域播放音频文件）。

文件存 web/audio/（页面可直接预览）；只允许音频扩展名；防路径穿越。
"""
import time
from pathlib import Path

from fastapi import APIRouter, HTTPException, UploadFile, File

router = APIRouter(prefix='/api/audio', tags=['audio'])

AUDIO_DIR = Path.home() / 'ros2Project' / 'Remote_ctrol' / 'web' / 'audio'
ALLOWED_EXT = {'.mp3', '.wav', '.ogg', '.m4a', '.aac', '.flac'}
MAX_MB = 20


def _safe_name(name: str) -> str:
    base = Path(name).name          # 剥掉任何路径成分
    if not base or base in ('.', '..'):
        raise HTTPException(400, '文件名非法')
    return base


@router.post('/upload')
async def upload(file: UploadFile = File(...)):
    name = _safe_name(file.filename or '')
    ext = Path(name).suffix.lower()
    if ext not in ALLOWED_EXT:
        raise HTTPException(400, f'不支持的格式 {ext}，允许：{" ".join(ALLOWED_EXT)}')
    data = await file.read()
    if len(data) > MAX_MB * 1024 * 1024:
        raise HTTPException(400, f'文件超过 {MAX_MB}MB')
    AUDIO_DIR.mkdir(parents=True, exist_ok=True)
    # 重名加时间戳
    out = AUDIO_DIR / name
    if out.exists():
        out = AUDIO_DIR / f'{Path(name).stem}_{int(time.time())}{ext}'
    out.write_bytes(data)
    return {'ok': True, 'file': out.name}


@router.get('')
def list_audio():
    AUDIO_DIR.mkdir(parents=True, exist_ok=True)
    items = []
    for f in sorted(AUDIO_DIR.iterdir(), key=lambda p: p.stat().st_mtime, reverse=True):
        if f.suffix.lower() in ALLOWED_EXT:
            st = f.stat()
            items.append({'name': f.name, 'size': st.st_size,
                          'mtime': int(st.st_mtime), 'url': f'/audio/{f.name}'})
    return {'items': items}


@router.delete('/{name}')
def delete_audio(name: str):
    path = AUDIO_DIR / _safe_name(name)
    if not path.exists():
        raise HTTPException(404, '文件不存在')
    path.unlink()
    return {'ok': True}
