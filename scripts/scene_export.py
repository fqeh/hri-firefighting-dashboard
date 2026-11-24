"""Static Gazebo scenery as browser-renderable primitives.

The dashboard's web 3D view draws the same SDF geometry Gazebo loads, so the
presenter's GPU renders the world instead of the server's software rasterizer.
Only public static geometry leaves this module: runtime-spawned hazards are not
part of the world files, and moving entities are exported in their own model
frame so the browser places them from live odometry.
"""
import json
import math
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

SHAPES = ('box', 'cylinder', 'sphere', 'ellipsoid', 'mesh')
ENTITY_INCLUDES = ('human', 'robot', 'uav')
HAZARD_TEMPLATES = {'tree': 'hazard_tree', 'fire': 'hazard_fire', 'gas': 'hazard_gas', 'debris': 'hazard_debris', 'smoke': 'hazard_smoke',
                    'fire_smoke': 'hazard_fire_smoke'}
DEFAULT_COLOR = [.7, .7, .7, 1.]


def floats(text, count, default=0.):
    values = [float(v) for v in (text or '').split()]
    return (values + [default] * count)[:count]


def transform(pose):
    """4x4 matrix for an SDF pose: fixed-axis roll, pitch, yaw (R = Rz·Ry·Rx)."""
    x, y, z, roll, pitch, yaw = pose
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
            [-sp, cp * sr, cp * cr, z],
            [0., 0., 0., 1.]]


IDENTITY = transform([0.] * 6)


def compose(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def quaternion(m):
    """Rotation part of a 4x4 matrix as [x, y, z, w] (three.js order)."""
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = .5 / math.sqrt(trace + 1.)
        q = [(m[2][1] - m[1][2]) * s, (m[0][2] - m[2][0]) * s, (m[1][0] - m[0][1]) * s, .25 / s]
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = 2. * math.sqrt(1. + m[0][0] - m[1][1] - m[2][2])
        q = [.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s]
    elif m[1][1] > m[2][2]:
        s = 2. * math.sqrt(1. + m[1][1] - m[0][0] - m[2][2])
        q = [(m[0][1] + m[1][0]) / s, .25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s]
    else:
        s = 2. * math.sqrt(1. + m[2][2] - m[0][0] - m[1][1])
        q = [(m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, .25 * s, (m[1][0] - m[0][1]) / s]
    return [round(v, 5) for v in q]


def rounded(values, digits=3):
    return [round(float(v), digits) for v in values]


class ModelPath:
    """Resolves model:// URIs against an ordered list of model directories."""
    def __init__(self, directories):
        self.directories = [Path(d) for d in directories]

    def file(self, uri):
        if not uri.startswith('model://'):
            raise ValueError(f'Unsupported URI: {uri}')
        relative = uri[len('model://'):]
        for directory in self.directories:
            candidate = directory / relative
            if candidate.is_file():
                return candidate
            if (candidate / 'model.sdf').is_file():
                return candidate / 'model.sdf'
        raise FileNotFoundError(uri)

    def model(self, uri):
        return ET.parse(self.file(uri)).getroot().find('model')


def obj_mesh(path, scale):
    vertices, indices = [], []
    for line in Path(path).read_text().splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == 'v':
            vertices.extend(float(v) * s for v, s in zip(parts[1:4], scale))
        elif parts[0] == 'f':
            corners = [int(p.split('/')[0]) for p in parts[1:]]
            corners = [c - 1 if c > 0 else len(vertices) // 3 + c for c in corners]
            for i in range(1, len(corners) - 1):  # fan triangulation
                indices.extend((corners[0], corners[i], corners[i + 1]))
    return {'shape': 'mesh', 'vertices': rounded(vertices), 'indices': indices}


def primitive(geometry, models):
    shape = next((g for g in geometry if g.tag in SHAPES), None) if geometry is not None else None
    if shape is None:
        return None
    if shape.tag == 'box':
        return {'shape': 'box', 'size': rounded(floats(shape.findtext('size'), 3, 1.))}
    if shape.tag == 'cylinder':
        return {'shape': 'cylinder', 'radius': float(shape.findtext('radius') or 1),
                'length': float(shape.findtext('length') or 1)}
    if shape.tag == 'sphere':
        return {'shape': 'sphere', 'radius': float(shape.findtext('radius') or 1)}
    if shape.tag == 'ellipsoid':
        return {'shape': 'ellipsoid', 'radii': rounded(floats(shape.findtext('radii'), 3, 1.))}
    uri = shape.findtext('uri') or ''
    if not uri.endswith('.obj'):
        return None
    return obj_mesh(models.file(uri), floats(shape.findtext('scale'), 3, 1.))


def material_color(visual):
    material = visual.find('material')
    text = None if material is None else (material.findtext('diffuse') or material.findtext('ambient'))
    color = floats(text, 4, 1.) if text else list(DEFAULT_COLOR)
    color[3] *= 1. - float(visual.findtext('transparency') or 0.)
    return rounded(color)


def visuals(model, frame, models):
    """Every visual of a model (links and nested models) in the given frame."""
    items = []
    for link in model.findall('link'):
        link_frame = compose(frame, transform(floats(link.findtext('pose'), 6)))
        for visual in link.findall('visual'):
            prim = primitive(visual.find('geometry'), models)
            if prim is None:
                continue
            m = compose(link_frame, transform(floats(visual.findtext('pose'), 6)))
            items.append({**prim, 'name': f"{model.get('name')}/{link.get('name')}/{visual.get('name')}",
                          'position': rounded([m[0][3], m[1][3], m[2][3]]), 'quaternion': quaternion(m),
                          'color': material_color(visual)})
    for nested in model.findall('model'):
        items += visuals(nested, compose(frame, transform(floats(nested.findtext('pose'), 6))), models)
    return items


def logical_camera(model):
    sensor = model.find(".//sensor[@type='logical_camera']")
    if sensor is None:
        return None
    camera = sensor.find('logical_camera')
    return {'horizontal_fov': float(camera.findtext('horizontal_fov') or 1),
            'far': float(camera.findtext('far') or 10),
            'pitch': floats(sensor.findtext('pose'), 6)[4]}


def world_scene(world_file, models):
    world = ET.parse(world_file).getroot().find('world')
    scenery, entities, sensors, spawns = [], {}, {}, {}
    for element in world:
        if element.tag == 'model':
            scenery += visuals(element, transform(floats(element.findtext('pose'), 6)), models)
        elif element.tag == 'include':
            model = models.model(element.findtext('uri'))
            name = element.findtext('name') or model.get('name')
            if name in ENTITY_INCLUDES:
                entities[name] = visuals(model, IDENTITY, models)
                # Where the agent starts, so the view can show it before the first odometry.
                pose = floats(element.findtext('pose') or model.findtext('pose'), 6)
                spawns[name] = {'position': rounded(pose[:3]), 'yaw': round(pose[5], 4)}
                camera = logical_camera(model)
                if camera:
                    sensors[name] = camera
            else:
                pose = element.findtext('pose') or model.findtext('pose')
                scenery += visuals(model, transform(floats(pose, 6)), models)
    background = world.findtext('scene/background')
    return {'world': world.get('name'), 'background': rounded(floats(background, 4, 1.)) if background else None,
            'scenery': scenery, 'entities': entities, 'sensors': sensors, 'spawns': spawns}


def hazard_templates(models):
    templates = {}
    for kind, name in HAZARD_TEMPLATES.items():
        try:
            templates[kind] = visuals(models.model(f'model://{name}'), IDENTITY, models)
        except FileNotFoundError:
            continue
    return templates


def export_scene(root, state):
    """Scene for the active scenario; `state` is the session supervisor state."""
    root = Path(root)
    scenario = state.get('scenario', 'forest')
    sim = root / 'ros2_ws/src/hri_sim'
    industrial = root / 'ros2_ws/src/hri_industrial'
    if scenario == 'oil_facility':
        models = ModelPath([industrial / 'models', sim / 'models'])
        scene = world_scene(industrial / 'worlds/oil_facility.sdf', models)
    else:
        sys.path.insert(0, str(root / 'ros2_ws/src/hri_bringup'))
        from hri_bringup.forest_world import prepare_world
        active = root / state.get('output', '') / 'scenario.json' if state.get('output') else None
        override = str(active) if active and active.is_file() else None
        world_file, _, directory = prepare_world(sim, state.get('case', 'F'), state.get('seed', 7), override)
        try:
            models = ModelPath([directory, sim / 'models'])
            scene = world_scene(world_file, models)
        finally:
            shutil.rmtree(directory, ignore_errors=True)
        models = ModelPath([sim / 'models'])
    scene.update(schema_version=1, scenario=scenario, hazards=hazard_templates(models),
                 source='Gazebo SDF world geometry (public, static); hazards and positions come from live telemetry')
    return scene


if __name__ == '__main__':
    print(json.dumps(export_scene(Path(__file__).resolve().parents[1],
                                  {'scenario': sys.argv[1] if len(sys.argv) > 1 else 'forest'}))[:2000])
