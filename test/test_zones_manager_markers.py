from navegacion_gps.zones_manager import build_zone_markers


def test_build_zone_markers_publishes_deleteall_and_closed_outline() -> None:
    markers = build_zone_markers(
        [
            {
                "id": "zone_alpha",
                "outer_xy": [
                    {"x": 0.0, "y": 0.0},
                    {"x": 2.0, "y": 0.0},
                    {"x": 2.0, "y": 1.0},
                    {"x": 0.0, "y": 1.0},
                ],
                "holes_xy": [],
            }
        ],
        "map",
    )

    assert len(markers.markers) == 2
    assert markers.markers[0].action == markers.markers[0].DELETEALL

    outline = markers.markers[1]
    assert outline.header.frame_id == "map"
    assert outline.type == outline.LINE_STRIP
    assert outline.ns == "zone_outer"
    assert outline.text == "zone_alpha"
    assert len(outline.points) == 5
    assert outline.points[0].x == outline.points[-1].x
    assert outline.points[0].y == outline.points[-1].y


def test_build_zone_markers_includes_hole_marker() -> None:
    markers = build_zone_markers(
        [
            {
                "id": "zone_with_hole",
                "outer_xy": [
                    {"x": 0.0, "y": 0.0},
                    {"x": 3.0, "y": 0.0},
                    {"x": 3.0, "y": 3.0},
                    {"x": 0.0, "y": 3.0},
                ],
                "holes_xy": [
                    [
                        {"x": 1.0, "y": 1.0},
                        {"x": 2.0, "y": 1.0},
                        {"x": 2.0, "y": 2.0},
                        {"x": 1.0, "y": 2.0},
                    ]
                ],
            }
        ],
        "odom",
    )

    assert len(markers.markers) == 3
    assert markers.markers[1].header.frame_id == "odom"
    assert markers.markers[2].ns == "zone_hole_0"
    assert len(markers.markers[2].points) == 5
