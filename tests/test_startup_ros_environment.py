"""Exercise the actual launch scripts with fake child programs, never devices."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('domain',[63,80])
@pytest.mark.parametrize('inherited_domain',[None,'244'])
@pytest.mark.parametrize('localhost',[None,'0'])
def test_web_and_desktop_inherit_device_domain_from_fresh_or_wrong_shell(tmp_path,domain,inherited_domain,localhost):
    project=tmp_path/'robot'
    for folder in ['scripts','desktop_ui/scripts','desktop_ui/build','ros2_ws/install','config','mockbin']:
        (project/folder).mkdir(parents=True,exist_ok=True)
    for name in ['scripts/start_robot_monitor.sh','desktop_ui/scripts/run.sh']:
        shutil.copy2(ROOT/name,project/name)
    (project/'config/ros_domain.sh').write_text(f'export ROS_DOMAIN_ID={domain}\n')
    # ROS setup is read-only. Its PATH is shadowed afterwards, so none of the
    # launch script's child programs can start ROS or a real HTTP server.
    mockbin=project/'mockbin'
    (project/'ros2_ws/install/setup.bash').write_text(f'export PATH="{mockbin}:$PATH"\n')
    child=f'''#!{sys.executable}
import json,os,sys,time
from pathlib import Path
p=Path(os.environ['STARTUP_RECORD_DIR'])
record=dict(argv=sys.argv,domain=os.environ.get('ROS_DOMAIN_ID'),localhost=os.environ.get('ROS_LOCALHOST_ONLY'))
(p/(str(os.getpid())+'.json')).write_text(json.dumps(record))
deadline=time.monotonic()+3
while len(list(p.glob('*.json')))<int(os.environ['STARTUP_EXPECTED_CHILDREN']) and time.monotonic()<deadline:time.sleep(.02)
'''
    for name in ['mockbin/ros2','mockbin/python3','desktop_ui/build/robot_desktop']:
        path=project/name;path.write_text(child);path.chmod(0o755)
    env=os.environ.copy()
    for key in ['ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','ROBOT_TLS_CERT','ROBOT_TLS_KEY','ROBOT_PROJECT_ROOT','DESKTOP_BUILD_DIR']:
        env.pop(key,None)
    env.update(ENABLE_VIDEO='0',ENABLE_CAPTURE='0',ENABLE_DETECTOR='0',ENABLE_AUDIO='1',
               ROBOT_PROJECT_ROOT=str(project))
    if inherited_domain is not None:env['ROS_DOMAIN_ID']=inherited_domain
    if localhost is not None:env['ROS_LOCALHOST_ONLY']=localhost
    expected_localhost=localhost or '1'
    for script,expected in [('scripts/start_robot_monitor.sh',3),('desktop_ui/scripts/run.sh',1)]:
        record_dir=tmp_path/('web' if expected==3 else 'desktop');record_dir.mkdir()
        env.update(STARTUP_RECORD_DIR=str(record_dir),STARTUP_EXPECTED_CHILDREN=str(expected))
        result=subprocess.run(['bash',str(project/script)],env=env,cwd=project,
                              capture_output=True,text=True,timeout=8)
        assert result.returncode==0,result.stdout+result.stderr
        records=[json.loads(p.read_text()) for p in record_dir.glob('*.json')]
        assert len(records)==expected
        assert all(r['domain']==str(domain) and r['localhost']==expected_localhost for r in records),records
