from ultralytics import YOLO


def main():
    # Load a model
    model = YOLO("yolov8n.pt")  # load a pretrained model (recommended for training)

    results = model.train(
        data="marker.yaml",
        epochs=70,
        imgsz=416,  # 416*416
        batch=3,
        device='0',
        name='marker_final'
    )


if __name__ == "__main__":
    main()