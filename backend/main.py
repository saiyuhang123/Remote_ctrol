"""FastAPI 业务后端主程序。

职责：托管 web/ 静态页面 + /api/* 业务接口 + SQLite 数据层。
运行：python3 -m uvicorn backend.main:app --host 0.0.0.0 --port 8888
（start_nav2_monitor.sh 已集成）
"""
from pathlib import Path

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

from . import db
from . import scheduler
from .api_records import router as records_router
from .api_points import router as points_router
from .api_routes import router as routes_router
from .api_plans import router as plans_router
from .api_audio import router as audio_router

WEB_DIR = Path.home() / 'ros2Project' / 'Remote_ctrol' / 'web'

app = FastAPI(title='巡检机器人业务后端', version='0.1.0')
app.include_router(records_router)
app.include_router(points_router)
app.include_router(routes_router)
app.include_router(plans_router)
app.include_router(audio_router)


@app.on_event('startup')
def _startup():
    db.init_db()
    scheduler.start_scheduler()


@app.get('/api/health')
def health():
    return {'ok': True, 'ts': __import__('time').time()}


# 静态页面挂载在最后（兜底捕获）
app.mount('/', StaticFiles(directory=str(WEB_DIR), html=True), name='web')
