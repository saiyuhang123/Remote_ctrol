"""Durable manual-capture index, separate from the patrol database."""
from contextlib import closing
import hashlib
import math
from pathlib import Path
import re

from fastapi import APIRouter, HTTPException, Query
from . import vision_store

router = APIRouter(tags=['captures'])
WEB = Path(__file__).resolve().parents[1]/'web'


def connection():
    conn = vision_store.connect()
    conn.execute('CREATE TABLE IF NOT EXISTS captures(capture_id TEXT PRIMARY KEY, stamp REAL NOT NULL, label TEXT NOT NULL, image TEXT NOT NULL)')
    conn.commit()
    return conn


@router.post('/api/records/captures')
def receive_capture(payload: dict):
    image, stamp = payload.get('image'), payload.get('ts')
    if (not isinstance(image, str) or not image.startswith('captures/') or
            not (WEB/image).resolve().is_relative_to((WEB/'captures').resolve()) or
            Path(image).suffix.lower() != '.jpg' or type(stamp) not in (float, int) or not math.isfinite(stamp) or stamp<=0):
        raise HTTPException(422, '抓拍记录无效')
    identifier = payload.get('capture_id') or hashlib.sha256(image.encode()).hexdigest()
    if not isinstance(identifier, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', identifier):
        raise HTTPException(422, '抓拍编号无效')
    label = str(payload.get('label') or '手动抓拍')[:200]
    with closing(connection()) as conn, conn:
        previous = conn.execute('SELECT image FROM captures WHERE capture_id=?', (identifier,)).fetchone()
        if previous and previous['image'] != image:
            raise HTTPException(409, '抓拍编号已被其他记录使用')
        conn.execute('INSERT OR IGNORE INTO captures VALUES(?,?,?,?)', (identifier, stamp, label, image))
    return dict(ok=True, capture_id=identifier)


@router.get('/api/records/captures')
def captures(limit: int = Query(30, ge=1, le=100), offset: int = Query(0, ge=0)):
    with closing(connection()) as conn:
        rows = conn.execute('SELECT * FROM captures ORDER BY stamp DESC,capture_id DESC LIMIT ? OFFSET ?', (limit, offset)).fetchall()
        total = conn.execute('SELECT COUNT(*) FROM captures').fetchone()[0]
    return dict(total=total, items=[dict(row)|dict(image_url='/'+row['image'], available=(WEB/row['image']).is_file()) for row in rows])
