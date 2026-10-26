from pathlib import Path
from io import BytesIO
import json
from types import SimpleNamespace as NS

import pytest
import yaml

from ocr_node.deepseek_vision import DeepSeekVisionNode, load_vision_config


def test_load_yaml_vision_config_and_mask_key(tmp_path):
    path = tmp_path / 'vision.yaml'
    path.write_text(yaml.safe_dump({
        'api_key': 'test-api-key-value-1234567890',
        'base_url': 'https://api.deepseek.com',
        'model': 'deepseek-flash',
        'timeout_sec': 42,
        'max_tokens': 777,
        'system_prompt': '只根据图片回答',
    }), encoding='utf-8')
    config = load_vision_config(str(path))
    assert config['model'] == 'deepseek-flash'
    assert config['timeout_sec'] == 42.0
    assert config['max_tokens'] == 777
    assert config['system_prompt'] == '只根据图片回答'


def test_load_legacy_labelled_key_file(tmp_path):
    path = tmp_path / 'legacy'
    path.write_text('姓名：测试\nAPI Key：legacy-api-key-value-1234567890\n', encoding='utf-8')
    config = load_vision_config(str(path))
    assert config['api_key'] == 'legacy-api-key-value-1234567890'
    assert config['base_url'] == 'https://api.deepseek.com'
    assert config['model'] == 'deepseek-flash'


def api_fixture(monkeypatch,content='仪表读数：-0012.30 V',finish='stop'):
    requests=[]
    payload=dict(model='deepseek-flash',choices=[dict(finish_reason=finish,message=dict(content=content))])
    def open_request(request,timeout):
        requests.append(json.loads(request.data))
        return BytesIO(json.dumps(payload).encode())
    monkeypatch.setattr('urllib.request.urlopen',open_request)
    client=NS(_config=dict(base_url='https://api.deepseek.com',api_key='fake-key',
                          model='deepseek-flash',max_tokens=1024,timeout_sec=10.),
              _extract_text=DeepSeekVisionNode._extract_text)
    return client,requests


def test_image_request_returns_final_readout_without_reasoning_only_budget(monkeypatch):
    client,requests=api_fixture(monkeypatch)
    payload=DeepSeekVisionNode._post_chat(client,b'fake-jpeg','image/jpeg','保留小数点和正负号')
    assert DeepSeekVisionNode._extract_text(payload)=='仪表读数：-0012.30 V'
    assert requests[0]['thinking']=={'type':'disabled'}
    content=requests[0]['messages'][-1]['content']
    assert content[0]['text']=='保留小数点和正负号'
    assert content[1]['image_url']['detail']=='high'
    assert content[1]['image_url']['url'].startswith('data:image/jpeg;base64,')


@pytest.mark.parametrize('finish',['length','content_filter','aborted'])
def test_incomplete_numeric_output_is_not_accepted_as_a_measurement(monkeypatch,finish):
    client,_=api_fixture(monkeypatch,content='仪表读数：-0012.',finish=finish)
    with pytest.raises(RuntimeError):
        DeepSeekVisionNode._post_chat(client,b'fake-jpeg','image/jpeg','读数')


@pytest.mark.parametrize('content',[None,'',{},[]])
def test_missing_content_is_not_converted_into_fake_recognition_text(content):
    assert DeepSeekVisionNode._extract_text({'choices':[{'message':{'content':content}}]})==''
