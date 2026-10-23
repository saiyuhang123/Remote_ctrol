import json

import pytest

from robot_patrol.reporting import render_html
from robot_patrol.store import PatrolStore, SCHEMA_VERSION
from test_engine import make_store, snapshot


@pytest.mark.parametrize('state,success', [(9,False),(10,True),(11,False),(12,False)])
def test_task_terminal_report_is_persisted_and_last_summary_updates_it(tmp_path, state, success):
    store, mapping = make_store(tmp_path)
    route = snapshot(['A'])
    route.name = '<script>alert(1)</script>'
    store.create_run('run1', 'route', mapping.map_id, 1, 1, route.as_dict(), 1, 1, 1_000_000_000)
    store.finish_run('run1', state, success, 1, 1, '' if success else 'CANCELED', 'done', 3_000_000_000)
    report = json.loads(store._conn.execute('SELECT payload_json FROM inspection_reports').fetchone()[0])
    assert report['run']['state']==state and report['duration_seconds']==2.
    assert report['route']['name']==route.name
    assert '<script>alert' not in render_html(report)
    store.upsert_inspection_summary('run1', '2026-10-22', mapping.map_id, 'route', 2, 3, [{'plate':'京A12345','count':1}])
    updated = json.loads(store._conn.execute('SELECT payload_json FROM inspection_reports').fetchone()[0])
    assert updated['summary']['face_capture_count']==2 and updated['summary']['person_count']==3
    store.close()


def test_schema_v5_history_is_preserved_and_backfilled(tmp_path):
    store, mapping = make_store(tmp_path)
    store.create_run('old', 'route', mapping.map_id, 1, 1, snapshot(['A']).as_dict(), 1, 1, 1)
    store.finish_run('old', 10, True, 1, 1, '', '', 2)
    store._conn.execute('DROP TABLE inspection_reports')
    store._conn.execute('UPDATE schema_version SET version=5')
    store.close()
    reopened = PatrolStore(tmp_path/'patrol.sqlite3')
    assert reopened._conn.execute('SELECT version FROM schema_version').fetchone()[0]==SCHEMA_VERSION
    assert reopened._conn.execute('SELECT run_id FROM inspection_reports').fetchone()[0]=='old'
    assert reopened.get_map(mapping.map_id).name==mapping.name
    assert list(tmp_path.glob('patrol.sqlite3.pre_v5.*.bak'))
    reopened.close()
