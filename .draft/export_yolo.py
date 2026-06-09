from ultralytics import YOLO

# Load a YOLO26n PyTorch model
model = YOLO("ultrahack2026.pt")

# Export the model to TensorRT
model.export(format="engine", half=True, )  # creates 'yolo26n.engine'

# # Load the exported TensorRT model
# trt_model = YOLO("yolo26n.engine")

# # Run inference
# results = trt_model("https://ultralytics.com/images/bus.jpg")