# Model Files

Place the YOLOv8 face model used by `face_tracking.launch.py` here:

```text
yolov8n-face.pt
face_recognition_sface_2021dec.onnx
```

`yolov8n-face.pt` detects face boxes. `face_recognition_sface_2021dec.onnx`
generates face identity embeddings for target locking and re-identification.

The launch file can also point at another model path with the `yolo_model_path`
or `reid_model_path` parameter.
