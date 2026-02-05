import cv2
from ultralytics import YOLO

model_path = 'E:/pycode/yolo/runs/detect/marker_final2/weights/best.pt'
video_path = 'E:/pycode/yolo/video_real/val1.mp4'
output_path = 'E:/pycode/yolo/output3.mp4'

model = YOLO(model_path)
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("Cannot open video file!")
    exit()

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

fourcc = cv2.VideoWriter_fourcc(*'mp4v')

out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

cv2.namedWindow('Marker detect', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Marker detect', 800, 600)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model.predict(frame, conf=0.3, imgsz=416, verbose=False, iou=0.5)

    annotated_frame = results[0].plot()

    out.write(annotated_frame)

    cv2.imshow('Marker detect', annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
out.release()
cv2.destroyAllWindows()
