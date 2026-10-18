"""FastAPI static server and read-only local business API.

Run:
  python3 -m uvicorn backend.main:app --host 0.0.0.0 --port 8888
"""
from pathlib import Path

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

from .api_local import router as local_router

WEB_DIR = Path(__file__).resolve().parents[1] / "web"

app = FastAPI(title="巡检机器人网页端", version="0.2.0")
app.include_router(local_router)
# The web MVP is read-only. Motion, map, route, plan, audio/vision upload and
# event-record writes are not mounted by this server.

app.mount("/", StaticFiles(directory=str(WEB_DIR), html=True), name="web")
