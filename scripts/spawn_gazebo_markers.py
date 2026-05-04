#!/usr/bin/env python3
import pathlib
import math
import subprocess


MODELS = pathlib.Path(
    "/ros2_ws/106a_final_project/drone_simulation/src/tello_ros/tello_gazebo/models"
)

TARGET = "marker_0"
ALL_MARKERS = [f"marker_{index}" for index in range(8)]


def call_service(service, service_type, request):
    return subprocess.run(
        ["ros2", "service", "call", service, service_type, str(request)],
        check=False,
        capture_output=True,
        text=True,
    )


def marker_xml(name):
    model_dir = MODELS / name
    xml = (model_dir / "model.sdf").read_text()
    xml = xml.replace(
        f"model://{name}/materials/scripts",
        f"file://{model_dir}/materials/scripts",
    )
    xml = xml.replace(
        f"model://{name}/materials/textures",
        f"file://{model_dir}/materials/textures",
    )
    return xml


def drone_pose():
    result = subprocess.run(
        "timeout 2 gz topic -e /gazebo/default/pose/info | head -80",
        check=False,
        capture_output=True,
        text=True,
        shell=True,
        executable="/bin/bash",
    )
    lines = result.stdout.splitlines()
    for index, line in enumerate(lines):
        if line.strip() != 'name: "tello_1"':
            continue

        values = {}
        for follow in lines[index : index + 25]:
            stripped = follow.strip()
            for key in ("x", "y", "z", "w"):
                prefix = f"{key}: "
                if stripped.startswith(prefix):
                    values.setdefault(key, []).append(float(stripped[len(prefix) :]))

        if all(key in values for key in ("x", "y", "z", "w")):
            px, py, pz = values["x"][0], values["y"][0], values["z"][0]
            qx, qy, qz, qw = values["x"][1], values["y"][1], values["z"][1], values["w"][0]
            break
    else:
        raise RuntimeError("Could not read tello_1 pose from Gazebo")

    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    return px, py, pz, yaw


def upright_marker_orientation(drone_yaw):
    half_yaw = drone_yaw / 2.0
    half_pitch = -math.pi / 4.0
    qz = (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
    qy = (0.0, math.sin(half_pitch), 0.0, math.cos(half_pitch))

    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qy
    return {
        "x": w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        "y": w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        "z": w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        "w": w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    }


def main():
    for name in ALL_MARKERS:
        call_service("/delete_entity", "gazebo_msgs/srv/DeleteEntity", {"name": name})

    try:
        px, py, pz, yaw = drone_pose()
    except RuntimeError:
        # Fallback for the current Gazebo session if gz transport cannot be
        # captured from Python. These values are refreshed manually from
        # /gazebo/default/pose/info before spawning the tag.
        px, py, pz, yaw = 3.0549419595, -1.0255162262, 1.2648897289, 1.5374
    distance = 1.2
    x = px + math.cos(yaw) * distance
    y = py + math.sin(yaw) * distance
    z = pz
    name = TARGET
    request = {
        "name": name,
        "xml": marker_xml(name),
        "robot_namespace": "",
        "initial_pose": {
            "position": {"x": x, "y": y, "z": z},
            # Rotate the flat marker upright so it faces the drone's current heading.
            "orientation": upright_marker_orientation(yaw),
        },
        "reference_frame": "world",
    }
    result = call_service("/spawn_entity", "gazebo_msgs/srv/SpawnEntity", request)
    ok = "success=True" in result.stdout
    print(
        f"{name}: {'spawned' if ok else 'failed'} upright at "
        f"({x:.3f}, {y:.3f}, {z:.3f}), drone_yaw={yaw:.3f}"
    )
    if not ok:
        print(result.stdout[-1000:])
        print(result.stderr[-1000:])


if __name__ == "__main__":
    main()
