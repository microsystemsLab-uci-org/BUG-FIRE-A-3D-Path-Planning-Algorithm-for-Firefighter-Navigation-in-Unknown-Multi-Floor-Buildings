'''
To run this code to train and export the YOLOv8 model execute the two lines of code # in the windowns terminal
'''
# cd "c:\Users\Eudald\OneDrive - personalmicrosoftsoftware.uci.edu\NEVERLOST\05_Path_Planning\Emergency signs datasets"
# python emergencySigns_yolov8_py_vscode.py

from pathlib import Path
import os, shutil, glob
import torch
from ultralytics import YOLO
from roboflow import Roboflow

def main():
    ROBOFLOW_API_KEY = os.getenv("ROBOFLOW_API_KEY")
    if not api_key:
        api_key = input("Enter your Roboflow API key: ")

    HOME = Path.cwd()
    RUNS_DIR = HOME / "runs"
    DATASETS_DIR = HOME / "datasets"
    DATASETS_DIR.mkdir(exist_ok=True)

    print(f"Working dir: {HOME}")

    if torch.cuda.is_available():
        gpu_name = torch.cuda.get_device_name(0)
        print(f"✅ GPU detected: {gpu_name}")
        device = "cuda"
    else:
        print("⚠️ No GPU detected, running on CPU")
        device = "cpu"

    api_key = ROBOFLOW_API_KEY
    if not api_key:
        raise RuntimeError("Set ROBOFLOW_API_KEY environment variable first.")

    rf = Roboflow(api_key=api_key)
    project = rf.workspace("emergency-exit-signs").project("emergency-exit-signs-v2")
    dataset = project.version(7).download("yolov8-obb")
    data_yaml = Path(dataset.location) / "data.yaml"

    model = YOLO("yolov8s.pt")
    results = model.train(
        data=str(data_yaml),
        epochs=250,
        imgsz=1024,
        project=str(RUNS_DIR),
        name="train",
        plots=True,
        device=device   # 👈 this makes training use your GPU if available
    )

    best_weights = RUNS_DIR / "train" / "weights" / "best.pt"
    YOLO(str(best_weights)).export(format="onnx", device=device)

    YOLO(str(best_weights)).val(data=str(data_yaml), device=device)

    test_images = Path(dataset.location) / "test" / "images"
    YOLO(str(best_weights)).predict(source=str(test_images), conf=0.25, save=True, device=device)
    print("\nDone. Check results inside:", RUNS_DIR)

if __name__ == '__main__':
    main()