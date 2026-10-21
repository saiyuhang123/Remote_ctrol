from pathlib import Path

import yaml

from ocr_node.deepseek_vision import load_vision_config


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
