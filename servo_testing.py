import cv2
import serial
import time

# --- Configuration ---
SERIAL_PORT = 'COM10'  # Change if needed
BAUD_RATE = 9600

# --- Camera frame size ---
WIDTH, HEIGHT = 640, 480
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2

# --- Initial Pan/Tilt Values ---
pan = 0.0
tilt = 0.0
step = 0.1  # Increment per key press

# --- Serial Initialization ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
    time.sleep(2)  # Give Arduino time to reset
    print(f"Serial port {SERIAL_PORT} opened successfully.")
except serial.SerialException as e:
    ser = None
    print(f"ERROR: Could not open serial port {SERIAL_PORT}.")
    print(f"Exception: {e}")

def send_servo_values(pan_val, tilt_val):
    """Send pan/tilt values to Arduino via serial"""
    global ser

    # Map pan/tilt (-1 → 1) to camera center coordinates (pixels)
    cam_x = int(CENTER_X + pan_val * CENTER_X)
    cam_y = int(CENTER_Y - tilt_val * CENTER_Y)  # invert Y: positive tilt = up

    msg = f"{pan_val:.2f} {tilt_val:.2f}\n"
    if ser and ser.is_open:
        try:
            ser.write(msg.encode('utf-8'))
            ser.flush()
        except Exception as e:
            print(f"Serial Write Error: {e}")

    # Print normalized values AND calculated camera center
    print(f"Pan={pan_val:.2f}, Tilt={tilt_val:.2f}, Camera Center=({cam_x},{cam_y})")

def run_manual_control():
    global pan, tilt, step

    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    print("\n--- Manual Control Interface ---")
    print("Use WASD to control Pan/Tilt. ESC to exit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Display pan/tilt values on frame
        cv2.putText(frame, f"PAN: {pan:.2f} (A/D)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, f"TILT: {tilt:.2f} (W/S)", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Draw calculated camera center
        cam_x = int(CENTER_X + pan * CENTER_X)
        cam_y = int(CENTER_Y - tilt * CENTER_Y)
        cv2.drawMarker(frame, (cam_x, cam_y), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(frame, f"Cam Center: ({cam_x},{cam_y})", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Camera View", frame)
        key = cv2.waitKey(100) & 0xFF

        command_sent = False

        if key == 27:  # ESC
            break
        if key == ord('a'):  # left
            pan = max(-1.0, pan - step)
            command_sent = True
        if key == ord('d'):  # right
            pan = min(1.0, pan + step)
            command_sent = True
        if key == ord('w'):  # up
            tilt = min(1.0, tilt + step)
            command_sent = True
        if key == ord('s'):  # down
            tilt = max(-1.0, tilt - step)
            command_sent = True

        if command_sent:
            send_servo_values(pan, tilt)

    cap.release()
    cv2.destroyAllWindows()
    if ser and ser.is_open:
        ser.close()
        print("Serial port closed.")

if __name__ == '__main__':
    run_manual_control()
