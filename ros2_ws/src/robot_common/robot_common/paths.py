"""实机运行目录：一键启动时由 ROBOT_PROJECT_ROOT 指定。"""
import os
from pathlib import Path

PROJECT_ROOT = Path(os.environ.get('ROBOT_PROJECT_ROOT', str(Path.home() / 'Remote_ctrol')))
