from ultralytics import YOLOv10
import os

cur_dir = os.path.dirname(os.path.abspath(__file__))
print(cur_dir)
y10_path = os.path.join(cur_dir, 'best.pt')

model = YOLOv10(y10_path)
model.model.model[-1].export = True
model.model.model[-1].format = 'onnx'
del model.model.model[-1].cv2
del model.model.model[-1].cv3
model.fuse()