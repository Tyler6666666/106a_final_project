from pathlib import Path

import cv2
import numpy as np


def main():
    out_dir = Path("/ros2_ws/106a_final_project/artifacts")
    out_dir.mkdir(parents=True, exist_ok=True)

    marker_id = 0
    marker_cells = 8  # 6x6 payload plus 1-cell black border on each side.
    cell_px = 200
    marker_px = marker_cells * cell_px
    margin_px = cell_px
    canvas_px = marker_px + 2 * margin_px

    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    marker = np.zeros((marker_px, marker_px), dtype=np.uint8)
    cv2.aruco.drawMarker(dictionary, marker_id, marker_px, marker, 1)

    canvas = np.full((canvas_px, canvas_px), 255, dtype=np.uint8)
    canvas[margin_px : margin_px + marker_px, margin_px : margin_px + marker_px] = marker

    cv2.imwrite(str(out_dir / "aruco_6x6_50_id0_15cm_with_margin.png"), canvas)
    cv2.imwrite(str(out_dir / "aruco_6x6_50_id0_15cm_marker_only.png"), marker)

    marker_cm = 15.0
    cell_cm = marker_cm / marker_cells
    margin_cm = cell_cm
    page_cm = marker_cm + 2 * margin_cm

    small = np.zeros((marker_cells, marker_cells), dtype=np.uint8)
    cv2.aruco.drawMarker(dictionary, marker_id, marker_cells, small, 1)

    rects = []
    for y in range(marker_cells):
        for x in range(marker_cells):
            if small[y, x] == 0:
                rx = margin_cm + x * cell_cm
                ry = margin_cm + y * cell_cm
                rects.append(
                    f'<rect x="{rx:.6f}cm" y="{ry:.6f}cm" '
                    f'width="{cell_cm:.6f}cm" height="{cell_cm:.6f}cm"/>'
                )

    svg = f"""<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{page_cm:.6f}cm" height="{page_cm:.6f}cm" viewBox="0 0 {page_cm:.6f} {page_cm:.6f}">
  <rect width="100%" height="100%" fill="white"/>
  <g fill="black">
    {chr(10).join(rects)}
  </g>
</svg>
"""
    (out_dir / "aruco_6x6_50_id0_15cm_print.svg").write_text(svg)

    html = f"""<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>ArUco DICT_6X6_50 ID 0 - 15cm</title>
<style>
  @page {{ size: letter; margin: 1cm; }}
  body {{ margin: 0; font-family: Arial, sans-serif; }}
  .marker {{ width: {page_cm:.6f}cm; height: {page_cm:.6f}cm; }}
  p {{ font-size: 12pt; margin: 0.4cm 0 0 0; }}
</style>
</head>
<body>
  <img class="marker" src="aruco_6x6_50_id0_15cm_print.svg" />
  <p>DICT_6X6_50, ID 0. Black marker side length: 15cm. Print at 100% scale.</p>
</body>
</html>
"""
    (out_dir / "aruco_6x6_50_id0_15cm_print.html").write_text(html)

    for path in [
        out_dir / "aruco_6x6_50_id0_15cm_print.svg",
        out_dir / "aruco_6x6_50_id0_15cm_with_margin.png",
        out_dir / "aruco_6x6_50_id0_15cm_marker_only.png",
        out_dir / "aruco_6x6_50_id0_15cm_print.html",
    ]:
        print(path)


if __name__ == "__main__":
    main()
