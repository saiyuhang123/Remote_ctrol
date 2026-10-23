import math
from types import SimpleNamespace as NS

import pytest

from robot_patrol.models import PatrolError
from robot_patrol.point_task import point_route
from test_engine import make_store


def request(**overrides):
    return NS(**(dict(point_id='',point_revision=0,x=.2,y=.2,yaw=0.,
                      inspection_enabled=True,ptz_preset=7,recognition_prompt='读电压')|overrides))


def test_arbitrary_target_is_ephemeral_and_saved_point_uses_its_revision(tmp_path):
    store,mapping=make_store(tmp_path)
    route=point_route(store,request(),mapping)
    assert len(route.steps)==1 and route.steps[0].inspection_enabled
    assert route.steps[0].ptz_preset==7
    assert not store.list_routes(mapping.map_id) and not store.list_points(mapping.map_id)
    point=store.upsert_point('',mapping.map_id,'电表',.1,.1,.3,'manual',1,
                             inspection_enabled=True,ptz_preset=9,recognition_prompt='读电流')
    route=point_route(store,request(point_id=point.point_id,point_revision=point.revision),mapping)
    assert route.steps[0].ptz_preset==9 and route.steps[0].recognition_prompt=='读电流'
    with pytest.raises(PatrolError,match='版本'):
        point_route(store,request(point_id=point.point_id,point_revision=point.revision+1),mapping)
    store.close()


@pytest.mark.parametrize('values',[{'x':float('nan')},{'y':float('inf')},{'x':100.}, {'ptz_preset':0}])
def test_invalid_target_never_creates_points_or_routes(tmp_path,values):
    store,mapping=make_store(tmp_path)
    with pytest.raises(PatrolError):
        point_route(store,request(**values),mapping)
    assert not store.list_points(mapping.map_id) and not store.list_routes(mapping.map_id)
    store.close()


def test_rotated_map_bounds_and_arrival_without_inspection(tmp_path):
    store,mapping=make_store(tmp_path)
    mapping.origin_x,mapping.origin_y,mapping.origin_yaw=10.,20.,math.pi/2
    route=point_route(store,request(x=9.8,y=20.2,inspection_enabled=False,ptz_preset=0),mapping)
    assert not route.steps[0].inspection_enabled
    with pytest.raises(PatrolError,match='地图范围'):
        point_route(store,request(x=10.2,y=20.2),mapping)
    store.close()
