"""Read local committed vision media, without ROS imports or arbitrary file paths."""
import json
import os
from pathlib import Path
import re
import sqlite3
from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse

router = APIRouter(prefix='/api/vision', tags=['vision'])


def directory():
    root = Path(os.environ.get('ROBOT_PROJECT_ROOT', Path(__file__).resolve().parents[1]))
    return Path(os.environ.get('VISION_DATA_DIR', root/'data/vision')).expanduser().resolve()


@router.get('/media/{event_id}')
def media(event_id: str):
    if not re.fullmatch(r'[a-f0-9]{64}', event_id):
        raise HTTPException(404, '记录不存在')
    root = directory()
    try:
        with sqlite3.connect((root / 'events.sqlite3').as_uri() + '?mode=ro', uri=True) as connection:
            row = connection.execute('SELECT payload FROM events WHERE event_id=?', (event_id,)).fetchone()
        if row is None:
            raise HTTPException(404, '记录不存在')
        path = Path(json.loads(row[0]).get('image_path', '')).resolve()
        if not path.is_relative_to(root/'media') or not path.is_file():
            raise HTTPException(404, '图片不存在')
        return FileResponse(path, media_type='image/jpeg')
    except (sqlite3.Error, OSError, ValueError):
        raise HTTPException(404, '视觉记录尚未接入') from None
