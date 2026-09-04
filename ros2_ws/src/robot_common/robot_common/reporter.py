"""HTTP 上报器：失败存 SQLite 本地缓存，恢复后补传（断网韧性底线）。

用法：
    from robot_common.reporter import HttpReporter
    rep = HttpReporter('http://127.0.0.1:8888', cache_db='/tmp/xxx_cache.db')
    rep.post('/api/records/detections', {...})   # 非阻塞
"""
import json
import queue
import sqlite3
import threading
import time

import requests


class HttpReporter:
    def __init__(self, base_url, cache_db, timeout=2.0):
        self.base_url = base_url.rstrip('/')
        self.cache_db = cache_db
        self.timeout = timeout
        self._q = queue.Queue(maxsize=200)
        self._init_db()
        threading.Thread(target=self._worker, daemon=True).start()

    def _init_db(self):
        with sqlite3.connect(self.cache_db) as c:
            c.execute('''CREATE TABLE IF NOT EXISTS cache(
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                path TEXT, payload TEXT, created_at REAL)''')

    def post(self, path, payload):
        """非阻塞上报；失败自动入本地缓存"""
        try:
            self._q.put_nowait((path, payload))
        except queue.Full:
            self._cache(path, payload)   # 队列满直接落盘

    def _cache(self, path, payload):
        try:
            with sqlite3.connect(self.cache_db) as c:
                c.execute('INSERT INTO cache(path,payload,created_at) VALUES(?,?,?)',
                          (path, json.dumps(payload, ensure_ascii=False), time.time()))
        except Exception:
            pass

    def _send(self, path, payload):
        try:
            r = requests.post(self.base_url + path, json=payload,
                              timeout=self.timeout)
            return r.status_code == 200
        except Exception:
            return False

    def _worker(self):
        while True:
            try:
                path, payload = self._q.get(timeout=1.0)
                if not self._send(path, payload):
                    self._cache(path, payload)
            except queue.Empty:
                pass
            self._retry_cache()

    def _retry_cache(self):
        try:
            with sqlite3.connect(self.cache_db) as c:
                rows = c.execute('SELECT id,path,payload FROM cache LIMIT 3').fetchall()
                for rid, path, payload in rows:
                    if self._send(path, json.loads(payload)):
                        c.execute('DELETE FROM cache WHERE id=?', (rid,))
                    else:
                        return   # 后端仍不可达，下轮再试
        except Exception:
            pass
