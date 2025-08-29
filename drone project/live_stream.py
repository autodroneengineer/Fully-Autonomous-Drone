import cv2
from flask import Flask, Response

app = Flask(__name__)

camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

def generate_frames():
    while True:
        success, frame = camera.read()
        if not success or frame is None:
            print("[ERROR] Camera frame not captured.")
            continue
        
        if len(frame.shape) == 2:
            print("[INFO] Converting grayscale frame to RGB")
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        frame = cv2.resize(frame, (640, 480))

        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            print("[ERROR] Frame encoding failed.")
            continue

        yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return "<h1>Camera Streaming</h1><img src='/video_feed'>"

if __name__ == '__main__':
    try:
        print("[INFO] Starting Flask Server...")
        app.run(host='0.0.0.0', port=5000, debug=False)
    except Exception as e:
        print(f"[ERROR] Exception occurred: {e}")
    finally:
        print("[INFO] Releasing camera resources...")
        camera.release()
