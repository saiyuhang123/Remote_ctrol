#!/usr/bin/env python3
"""Offline model integrity/inference checks. Never opens ROS or a camera."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT/'ros2_ws/src/detector_node'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-dir', type=Path, default=PROJECT/'ros2_ws/src/detector_node/models')
    parser.add_argument('--samples-dir', type=Path, help='Optional public face.jpg / bus.jpg / plate.jpg samples; see models/README.md')
    args = parser.parse_args()
    root = args.model_dir.resolve()
    manifest = json.loads((root/'model_manifest.json').read_text())
    for name, entry in manifest['files'].items():
        path = root/name
        if path.stat().st_size != entry['bytes'] or hashlib.sha256(path.read_bytes()).hexdigest() != entry['sha256']:
            raise RuntimeError(f'model checksum mismatch: {name}')
    import transforms3d  # Check the ROS/NumPy compatibility pin too.
    import cv2
    import numpy as np
    from detector_node.detection_pipeline import YunetCpu, YoloCpu, LpdYunetCpu, PlateTextCpu, Pipeline
    cv2.setNumThreads(2)
    face = YunetCpu(root/'face_detection_yunet.onnx')
    person = YoloCpu(root/'yolo26n_openvino_model/yolo26n.xml')
    plate = LpdYunetCpu(root/'license_plate_detection_lpd_yunet_2023mar.onnx')
    text = PlateTextCpu(root/'plate_recognition_hyperlpr3.onnx')
    blank = np.zeros((1080, 1920, 3), np.uint8)
    timings = {}
    for name, model in [('face', face), ('person', person), ('plate', plate)]:
        assert model(blank) == [], name+' unexpectedly detected a blank image'
        samples = []
        for _ in range(3):
            start = time.perf_counter(); model(blank)
            samples.append((time.perf_counter()-start)*1000)
        timings[name] = round(float(np.median(samples)), 1)
    print(json.dumps(dict(model_files='checksums OK', blank_1080p_median_ms=timings,
                          transforms3d=transforms3d.__version__), ensure_ascii=False))
    if args.samples_dir:
        def image(name):
            result = cv2.imread(str(args.samples_dir/name))
            if result is None: raise RuntimeError('cannot read sample '+name)
            return result
        faces, people = face(image('face.jpg')), person(image('bus.jpg'))
        assert faces and people, 'positive face/person samples produced no detections'
        pipeline = Pipeline(plate=text, plate_detector=plate)
        frame = image('plate.jpg')
        assert pipeline.process(frame, person=False, now=1.) == []
        events = pipeline.process(frame, person=False, now=1.3)
        assert len(events) == 1 and events[0]['plate_text'] == '苏ED51712', 'plate sample failed'
        print(json.dumps(dict(sample_faces=len(faces), sample_people=len(people),
                              consecutive_plate_event=events[0]['plate_text']), ensure_ascii=False))
    print('CPU models ready. No ROS node, camera connection or motion command was created.')


if __name__ == '__main__':
    main()
