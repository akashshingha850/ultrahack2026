import cv2
import torch
import numpy as np
import yaml
from depth_anything_3.api import DepthAnything3

with open("config.yaml") as f:
    cfg = yaml.safe_load(f)

stream_url = cfg["stream"]["input"]

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = DepthAnything3.from_pretrained("depth-anything/DA3NESTED-GIANT-LARGE")
model = model.to(device=device)
model.eval()

cap = cv2.VideoCapture(stream_url)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    prediction = model.inference([rgb])
    depth = prediction.depth[0]  # [H, W] already numpy

    # Normalise for display
    depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_INFERNO)

    cv2.imshow("Depth", depth_color)
    cv2.imshow("RGB", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
