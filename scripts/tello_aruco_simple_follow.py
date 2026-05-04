from djitellopy import Tello
import cv2
import numpy as np
import time


WIDTH = 640
HEIGHT = 480

MARKER_ID = 0
MARKER_SIZE_M = 0.15
TARGET_DIST_M = 0.60
DEAD_ZONE_X = 65
DIST_DEAD_ZONE_M = 0.05
CENTER_DEAD_ZONE_Y = 55
TAG_CONFIRM_SEC = 5.0
LOST_LAND_SEC = 5.0
TAKEOFF_SETTLE_SEC = 1.0

MAX_YAW_SPEED = 35
MAX_FB_SPEED = 24
MAX_UD_SPEED = 14
YAW_KP = 0.16
FB_KP = 85.0
UD_KP = 0.10

# Set this to True for camera/detection only. False runs the auto takeoff/follow state machine.
TEST_ONLY = False

CAMERA_MATRIX = np.array(
    [
        [921.170702, 0.0, 459.904354],
        [0.0, 919.018377, 351.238301],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float32,
)
CAMERA_MATRIX[0, :] *= WIDTH / 960.0
CAMERA_MATRIX[1, :] *= HEIGHT / 720.0
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


def make_detector():
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(dictionary, parameters)
    return detector


def get_aruco(img, detector):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return None, corners, ids

    ids_flat = ids.flatten()
    matches = np.where(ids_flat == MARKER_ID)[0]
    if len(matches) == 0:
        return None, corners, ids

    index = int(matches[0])
    marker_corners = corners[index].reshape(4, 2).astype(np.float32)
    object_points = MARKER_POINTS * MARKER_SIZE_M
    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        marker_corners,
        CAMERA_MATRIX,
        DIST_COEFFS,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok:
        return None, corners, ids

    cx = int(np.mean(marker_corners[:, 0]))
    cy = int(np.mean(marker_corners[:, 1]))
    x, y, z = tvec.reshape(3)
    return (cx, cy, float(x), float(y), float(z), rvec, tvec), corners, ids


def draw_grid(img):
    center_x = WIDTH // 2
    cv2.line(img, (center_x - DEAD_ZONE_X, 0), (center_x - DEAD_ZONE_X, HEIGHT), (255, 255, 0), 2)
    cv2.line(img, (center_x + DEAD_ZONE_X, 0), (center_x + DEAD_ZONE_X, HEIGHT), (255, 255, 0), 2)
    cv2.line(img, (0, HEIGHT // 2 - CENTER_DEAD_ZONE_Y), (WIDTH, HEIGHT // 2 - CENTER_DEAD_ZONE_Y), (255, 255, 0), 2)
    cv2.line(img, (0, HEIGHT // 2 + CENTER_DEAD_ZONE_Y), (WIDTH, HEIGHT // 2 + CENTER_DEAD_ZONE_Y), (255, 255, 0), 2)
    cv2.circle(img, (center_x, HEIGHT // 2), 5, (0, 0, 255), 5)


def clip(value, limit):
    return int(max(-limit, min(limit, value)))


def main():
    detector = make_detector()

    me = Tello()
    me.connect()
    print("battery:", me.get_battery())
    print("temperature:", me.get_temperature())

    me.streamoff()
    me.streamon()
    frame_read = me.get_frame_read()

    airborne = False
    following = False
    last_seen = 0.0
    first_seen = None
    state = "WAIT_TAG"
    takeoff_time = None

    cv2.namedWindow("Tello ArUco Simple", cv2.WINDOW_NORMAL)

    while True:
        my_frame = frame_read.frame
        if my_frame is None:
            time.sleep(0.02)
            continue

        img = cv2.resize(my_frame, (WIDTH, HEIGHT))
        tag, corners, ids = get_aruco(img, detector)

        lr = 0
        fb = 0
        ud = 0
        yaw = 0
        status = state

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)

        if tag is not None:
            cx, cy, x, y, z, rvec, tvec = tag
            last_seen = time.time()
            cv2.circle(img, (cx, cy), 8, (0, 255, 0), cv2.FILLED)
            cv2.line(img, (WIDTH // 2, HEIGHT // 2), (cx, cy), (0, 0, 255), 2)
            cv2.drawFrameAxes(img, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec, MARKER_SIZE_M * 0.5)

            if not airborne and not TEST_ONLY:
                if first_seen is None:
                    first_seen = time.time()
                seen_for = time.time() - first_seen
                status = f"TAG LOCK {TAG_CONFIRM_SEC - seen_for:.1f}s"
                if seen_for >= TAG_CONFIRM_SEC:
                    print("Tag confirmed for 5 seconds. Taking off.")
                    me.takeoff()
                    airborne = True
                    following = False
                    takeoff_time = time.time()
                    state = "TAKEOFF"
                    status = state

            if airborne and not following and takeoff_time is not None:
                status = "TAKEOFF"
                if time.time() - takeoff_time >= TAKEOFF_SETTLE_SEC:
                    following = True
                    state = "FOLLOW"
                    status = state

            if following:
                status = "FOLLOW"
                x_error_px = cx - WIDTH // 2
                y_error_px = cy - HEIGHT // 2
                dist_error_m = z - TARGET_DIST_M

                if abs(x_error_px) > DEAD_ZONE_X:
                    yaw = clip(x_error_px * YAW_KP, MAX_YAW_SPEED)

                if abs(dist_error_m) > DIST_DEAD_ZONE_M:
                    fb = clip(dist_error_m * FB_KP, MAX_FB_SPEED)

                if abs(y_error_px) > CENTER_DEAD_ZONE_Y:
                    ud = clip(-y_error_px * UD_KP, MAX_UD_SPEED)
        else:
            first_seen = None
            if following:
                status = "TAG LOST"
                if last_seen and time.time() - last_seen > LOST_LAND_SEC:
                    print("Tag lost for 5 seconds. Landing.")
                    me.send_rc_control(0, 0, 0, 0)
                    me.land()
                    airborne = False
                    following = False
                    state = "WAIT_TAG"
                    takeoff_time = None
            else:
                status = "NO TAG"

        draw_grid(img)
        cv2.putText(img, status, (20, 35), cv2.FONT_HERSHEY_COMPLEX, 0.9, (0, 255, 255), 2)
        if tag is not None:
            cv2.putText(
                img,
                f"id={MARKER_ID} z={tag[4]:.2f}m target={TARGET_DIST_M:.2f}m",
                (20, 70),
                cv2.FONT_HERSHEY_COMPLEX,
                0.65,
                (0, 255, 255),
                2,
            )
        cv2.putText(
            img,
            f"rc lr={lr} fb={fb} ud={ud} yaw={yaw} airborne={airborne}",
            (20, HEIGHT - 20),
            cv2.FONT_HERSHEY_COMPLEX,
            0.65,
            (0, 255, 255),
            2,
        )

        if airborne:
            me.send_rc_control(lr, fb, ud, yaw)

        cv2.imshow("Tello ArUco Simple", img)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("e") and not airborne:
            if tag is None:
                print("Put the ArUco tag fully in view before takeoff.")
            else:
                me.takeoff()
                airborne = True
                following = False
                takeoff_time = time.time()
                state = "TAKEOFF"

        if key == ord("f") and airborne:
            following = not following
            state = "FOLLOW" if following else "HOVER"
            if not following:
                me.send_rc_control(0, 0, 0, 0)

        if key == ord(" "):
            following = False
            state = "HOVER" if airborne else "WAIT_TAG"
            if airborne:
                me.send_rc_control(0, 0, 0, 0)

        if key == ord("q"):
            if airborne:
                me.send_rc_control(0, 0, 0, 0)
                me.land()
            break

    me.streamoff()
    me.end()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
