"""Pure validation shared by the ROS operator adapter and its tests."""
import math


def waypoint_coordinates(data):
    if not isinstance(data, dict):
        raise ValueError('control must be a JSON object')
    if data.get('command') != 'waypoint':
        return None
    if any(isinstance(data.get(k), bool) or not isinstance(data.get(k), (int, float))
           for k in ('x', 'y')):
        raise ValueError('waypoint coordinates must be numeric')
    x, y = float(data['x']), float(data['y'])
    if not all(math.isfinite(v) and abs(v) <= 100 for v in (x, y)):
        raise ValueError('waypoint must be finite and within the scenario extent')
    return x, y
