"""FastAPI console, read-only patrol APIs and uploaded media.

Run:
  python3 -m uvicorn backend.main:app --host 0.0.0.0 --port 8888
"""
from pathlib import Path

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

from .api_local import router as local_router
from .api_inspection import router as inspection_router
from .api_audio import router as audio_router
from .api_captures import router as capture_router

WEB_DIR = Path(__file__).resolve().parents[1] / "web"

app = FastAPI(title="巡检机器人网页端", version="0.4.0")
app.include_router(local_router)
app.include_router(inspection_router)
app.include_router(audio_router)
app.include_router(capture_router)
# Only media/incoming events are writable here. Motion stays in the gateway;
# patrol maps, routes, results and reports are written by the patrol executor.

app.mount("/", StaticFiles(directory=str(WEB_DIR), html=True), name="web")
