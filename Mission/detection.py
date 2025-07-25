import cv2
import time
from datetime import datetime
import logging
from threading import Lock, Event
from ultralytics import YOLO
import numpy as np
from align import mid_coord, init_align

def take_picture():
    global targetDetectionFlag, latest_frame, target_lat, target_lon, target_alt
    logging.info("Taking picture")
    for i in range(4):
        with frame_lock:
            if latest_frame is None:
                logging.warning("No frame available to capture.")
                continue
            frame = latest_frame.copy()

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"image_{timestamp}{i}.jpg"
        cv2.imwrite(filename, frame)
        time.sleep(0.3)
        logging.info(f"Saved image: {filename}")
    # msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    # if msg:
    #     target_lat = msg.lat / 1e7
    #     target_lon = msg.lon / 1e7
    #     target_alt = msg.relative_alt / 1000.0
    # time.sleep(0.3)`
    targetDetectionFlag = False

async def detectionClassify(camera, drone):
    global rtsp_show, targetDetectionFlag, interruption_flag, h_x, h_y, latest_frame, tracker, disasters, objects, shapes, frame_lock, stop_detection, max_counts
    latest_frame = None
    stop_detection = False
    rtsp_show = False
    targetDetectionFlag = False
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    filename = f"stream_{timestamp}.mp4"
    object_count = 0
    frameCount = 0
    targetCount = 0
    start_time = time.time()

    # Shape to cluster mapping
    shape_to_cluster = {
        'cone': 'Cluster 1',
        'cube': 'Cluster 2',
        'circle': 'Cluster 3',
        'triangle': 'Cluster 4',
        'square': 'Cluster 5'
    }

    # Load YOLOv8 model
    model = YOLO('firecone.pt')
    
    # Connect to UDP stream
    cam = cv2.VideoCapture(camera, cv2.CAP_FFMPEG)
    
    if not cam.isOpened():
        logging.error("Failed to open UDP video stream.")
        return

    # Feed resolution
    imW = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    imH = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
    h_x = imW / 2
    h_y = imH / 2
    mid_coord(h_x, h_y)
    record = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(filename, record, 20.0, (imW, imH))

    try:
        while cam.isOpened() and not stop_detection:
            ret, frame = cam.read()
            if not ret:
                logging.warning("No frame received from UDP stream.")
                break

            with frame_lock:
                latest_frame = frame.copy()

            start_prediction = time.time()
            results = model.predict(source=frame, conf=0.75, imgsz=640)
            end_prediction = time.time()
            logging.info(f"YOLO Detection took time {end_prediction - start_prediction:.3f} secs")

            # Reset clusters per frame
            clusters = {
                'cone': [],
                'cube': [],
                'circle': [],
                'triangle': [],
                'square': []
            }

            frameCount2D = 0
            frameCount3D = 0

            for result in results:
                for box in result.boxes:
                    clsID = int(box.cls[0])
                    conf = float(box.conf[0])
                    object_name = model.names[clsID]
                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    if conf < 0.75:
                        continue

                    if object_name in clusters:
                        clusters[object_name].append((x1, y1, x2, y2))

                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
                    label = f"{object_name}:{int(conf * 100)}%"
                    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

                    if object_name in ["fire", "flood"]:
                        with open("disaster_midpoint.txt", "a") as f:
                            f.write(f"{cx}, {cy}\n")
                        logging.info(f"Updated disaster coordinates ({cx}, {cy})")

                    if targetDetectionFlag:
                        with open("pixel.txt", "a") as f:
                            f.write(f"{cx}, {cy}\n")
                        logging.info(f"The coordinates: {cx}, {cy}, {object_name}")
                        targetCount += 1

                    elif object_name in ["cone", "box", "sphere"]:
                        if object_name in objects:
                            frameCount3D += 1
                            cv2.imwrite(f"{frameCount3D}.jpg", frame)
                    elif object_name in ["circle", "triangle", "square"]:
                        if object_name in shapes:
                            frameCount2D += 1
                            cv2.imwrite(f"{frameCount2D}.jpg", frame)

            # Update max counts
            for shape in clusters:
                current_count = len(clusters[shape])
                if current_count > max_counts[shape]:
                    max_counts[shape] = current_count

            # Draw grouped clusters
            for shape_type, boxes in clusters.items():
                if not boxes:
                    continue

                color = (0, 255, 0) if shape_type == 'triangle' else \
                        (255, 0, 0) if shape_type == 'square' else \
                        (0, 0, 255) if shape_type == 'circle' else \
                        (255, 255, 0) if shape_type == 'cone' else \
                        (0, 255, 255)

                for (x1, y1, x2, y2) in boxes:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                coords = np.array(boxes)
                if len(coords) > 0:
                    cx1, cy1 = np.min(coords[:, :2], axis=0)
                    cx2, cy2 = np.max(coords[:, 2:], axis=0)
                    cx1, cy1, cx2, cy2 = map(int, (cx1, cy1, cx2, cy2))
                    cv2.putText(frame, shape_to_cluster[shape_type], (cx1, cy1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
                    cv2.rectangle(frame, (cx1, cy1), (cx2, cy2), color, 1)

            # Display max counts in top-left corner
            display_text = f"Max Counts - Cone: {max_counts['cone']} | Cube: {max_counts['cube']} | Circle: {max_counts['circle']} | Triangle: {max_counts['triangle']} | Square: {max_counts['square']}"
            cv2.putText(frame, display_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # Add Team aeroKLE in top-right corner
            cv2.putText(frame, "Team aeroKLE", (imW - 200, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

            if targetDetectionFlag and targetCount >= 4:
                print("Target Located")
                init_align(drone)
                time.sleep(1)

            if time.time() - start_time >= 2 and targetDetectionFlag:
                targetCount = 0
                start_time = time.time()

            out.write(frame)
            cv2.imshow("aeroKLE-UDP Stream", frame)
            rtsp_show = True

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except Exception as e:
        logging.error(f"YOLO detection failed: {e}")

    finally:
        cam.release()
        out.release()
        cv2.destroyAllWindows()
        logging.info("Detection ended.")