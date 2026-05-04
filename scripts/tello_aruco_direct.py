import argparse
import math
import time

import cv2
import numpy as np
from djitellopy import Tello


CAMERA_MATRIX = np.array(
    [
        [921.170702, 0.0, 459.904354],
        [0.0, 919.018377, 351.238301],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float32,
)
DIST_COEFFS = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.0], dtype=np.float32)

MARKER_POINTS = np.array(
    [
        [-0.5, 0.5, 0.0],
        [0.5, 0.5, 0.0],
        [0.5, -0.5, 0.0],
        [-0.5, -0.5, 0.0],
    ],
    dtype=np.float32,
)


def clip(value, lo, hi):
    return int(max(lo, min(hi, value)))


def make_aruco_detector():
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    if hasattr(aruco, "DetectorParameters"):
        parameters = aruco.DetectorParameters()
    else:
        parameters = aruco.DetectorParameters_create()

    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(dictionary, parameters)

        def detect(gray):
            return detector.detectMarkers(gray)

        return detect

    def detect(gray):
        return aruco.detectMarkers(gray, dictionary, parameters=parameters)

    return detect


def estimate_marker_pose(corners, marker_size_m):
    object_points = MARKER_POINTS * marker_size_m
    image_points = corners.reshape(4, 2).astype(np.float32)
    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        CAMERA_MATRIX,
        DIST_COEFFS,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok:
        ok, rvec, tvec = cv2.solvePnP(object_points, image_points, CAMERA_MATRIX, DIST_COEFFS)
    if not ok:
        return None
    return rvec.reshape(3), tvec.reshape(3)


def find_tag(frame, detect_markers, marker_id, marker_size_m):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detect_markers(gray)
    if ids is None or len(ids) == 0:
        return None, corners, ids

    ids_flat = ids.flatten()
    matches = np.where(ids_flat == marker_id)[0]
    if len(matches) == 0:
        return None, corners, ids

    best_idx = int(matches[0])
    pose = estimate_marker_pose(corners[best_idx], marker_size_m)
    if pose is None:
        return None, corners, ids

    rvec, tvec = pose
    marker_corners = corners[best_idx].reshape(4, 2)
    center = marker_corners.mean(axis=0)
    return {"rvec": rvec, "tvec": tvec, "center": center, "corners": marker_corners}, corners, ids


def draw_status(frame, lines):
    y = 28
    for line in lines:
        cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        y += 28


def main():
    parser = argparse.ArgumentParser(description="Direct Tello ArUco follower with native OpenCV window.")
    parser.add_argument("--marker-id", type=int, default=0)
    parser.add_argument("--marker-size", type=float, default=0.15, help="Marker side length in meters.")
    parser.add_argument("--target-dist", type=float, default=0.45, help="Target distance in meters.")
    parser.add_argument("--loss-land-sec", type=float, default=1.0)
    parser.add_argument("--follow-delay", type=float, default=1.0)
    parser.add_argument("--max-fb", type=int, default=28)
    parser.add_argument("--max-lr", type=int, default=22)
    parser.add_argument("--max-ud", type=int, default=18)
    parser.add_argument("--max-yaw", type=int, default=45)
    args = parser.parse_args()

    detect_markers = make_aruco_detector()
    tello = Tello()
    airborne = False
    follow_enabled = False
    pending_follow_time = None
    last_seen = 0.0
    last_rc_time = 0.0
    window_name = "Tello ArUco Direct - e takeoff/follow, f toggle, space stop, q land/quit"

    try:
        tello.connect()
        battery = tello.get_battery()
        temperature = tello.get_temperature()
        print(f"Connected. battery={battery}% temp={temperature}C")
        tello.streamoff()
        tello.streamon()
        frame_reader = tello.get_frame_read()

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 960, 720)

        while True:
            now = time.time()
            frame = frame_reader.frame
            if frame is None:
                time.sleep(0.02)
                continue

            frame = cv2.resize(frame, (960, 720))
            tag, corners, ids = find_tag(frame, detect_markers, args.marker_id, args.marker_size)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            if tag is not None:
                last_seen = now
                cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, tag["rvec"], tag["tvec"], args.marker_size * 0.5)

            if pending_follow_time is not None and now >= pending_follow_time:
                follow_enabled = True
                pending_follow_time = None

            lr = fb = ud = yaw = 0
            status = "FOLLOW" if follow_enabled else "VIEW"

            if follow_enabled and airborne:
                if tag is None:
                    status = "TAG LOST"
                    if last_seen > 0 and now - last_seen >= args.loss_land_sec:
                        print("Tag lost; landing.")
                        tello.send_rc_control(0, 0, 0, 0)
                        tello.land()
                        airborne = False
                        follow_enabled = False
                else:
                    x, y, z = tag["tvec"]
                    cx = tag["center"][0]
                    frame_center_x = frame.shape[1] / 2.0

                    fb = clip((z - args.target_dist) * 95.0, -args.max_fb, args.max_fb)
                    lr = clip(x * 80.0, -args.max_lr, args.max_lr)
                    ud = clip(-y * 75.0, -args.max_ud, args.max_ud)
                    yaw = clip((cx - frame_center_x) * 0.12, -args.max_yaw, args.max_yaw)

                    if abs(z - args.target_dist) < 0.05:
                        fb = 0
                    if abs(x) < 0.06:
                        lr = 0
                    if abs(y) < 0.05:
                        ud = 0
                    if abs(cx - frame_center_x) < 35:
                        yaw = 0

            if airborne and now - last_rc_time >= 0.05:
                tello.send_rc_control(lr, fb, ud, yaw)
                last_rc_time = now

            tag_line = "tag: not found"
            if tag is not None:
                x, y, z = tag["tvec"]
                tag_line = f"tag id={args.marker_id} x={x:.2f} y={y:.2f} z={z:.2f}m target={args.target_dist:.2f}m"

            draw_status(
                frame,
                [
                    f"{status} airborne={airborne} battery={battery}%",
                    tag_line,
                    f"rc lr={lr} fb={fb} ud={ud} yaw={yaw}",
                ],
            )
            cv2.imshow(window_name, frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                if airborne:
                    tello.send_rc_control(0, 0, 0, 0)
                    tello.land()
                    airborne = False
                break
            if key == ord(" "):
                follow_enabled = False
                pending_follow_time = None
                if airborne:
                    tello.send_rc_control(0, 0, 0, 0)
            if key == ord("f"):
                follow_enabled = not follow_enabled
                pending_follow_time = None
            if key == ord("e") and not airborne:
                if tag is None:
                    print("No tag visible yet. Takeoff blocked; put the tag fully in frame first.")
                else:
                    print("Taking off; follow will start after delay.")
                    tello.takeoff()
                    airborne = True
                    pending_follow_time = time.time() + args.follow_delay

    except KeyboardInterrupt:
        pass
    finally:
        try:
            if airborne:
                tello.send_rc_control(0, 0, 0, 0)
                tello.land()
        except Exception:
            pass
        try:
            tello.streamoff()
            tello.end()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
