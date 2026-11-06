"""Read XYZ without requiring intensity/confidence to share the XYZ datatype."""
import numpy as np
from sensor_msgs_py import point_cloud2


def cloud_xyz(message):
    # Humble read_points_numpy checks *all* field types even with field_names.
    # Odin packs FLOAT32 XYZ, UINT8 intensity, UINT16 confidence and FLOAT32 time
    # into 19 bytes. read_points respects the declared offsets and point stride.
    points = point_cloud2.read_points(
        message, field_names=["x", "y", "z"], skip_nans=True
    )
    return np.stack([points[name] for name in ("x", "y", "z")], axis=-1).reshape(-1, 3).astype(np.float64)
