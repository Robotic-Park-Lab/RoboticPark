#!/usr/bin/env python3
"""Dev utility (not installed by the package) to generate Webots world
(.wbt) and RViz (.rviz) files for RoboticPark multi-robot formation
experiences, from a robot list -- so that scaling an experience to a
different N doesn't mean hand-editing hundreds of lines of Webots/RViz
text per robot.

RViz generation works by lifting the per-robot Display block (RobotModel
+ local_pose + target_pose [+ goal_pose] + path) out of an existing,
known-good experience file (IROS_AffineFormation_N05.rviz) and cloning
it per robot name via string substitution. Neighbour "Marker" displays
(topology-specific arrows between drones) are intentionally dropped --
they are a nice-to-have, not needed to supervise a run.

Run manually from this directory when adding a new experience:
    python3 generate_formation_assets.py
"""
import copy
from pathlib import Path

import yaml

RVIZ_TEMPLATE = Path(__file__).parent.parent / 'rviz' / 'IROS_AffineFormation_N05.rviz'
WORLDS_DIR = Path(__file__).parent.parent / 'worlds'
RVIZ_DIR = Path(__file__).parent.parent / 'rviz'

WBT_HEADER = """#VRML_SIM R2025a utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/Floor.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/bitcraze/crazyflie/protos/Crazyflie.proto"
"""

KHEPERA_EXTERNPROTO = (
    'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/'
    'projects/robots/k-team/khepera4/protos/Khepera4.proto"\n'
)

WBT_ENV = """EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/appearances/protos/Parquetry.proto"

WorldInfo {{
\tinfo [
\t\t "Robotic Park World. {description}"
\t]
\tbasicTimeStep 20
}}
Viewpoint {{
\tfieldOfView 1
\torientation 0.08257814004826021 0.7347446275807098 -0.6732987323821825 0.7219016991902246
\tposition -6.686523789804856 3.246953734178069 5.0217815008034
}}
TexturedBackground {{
}}
TexturedBackgroundLight {{
}}
DirectionalLight {{
\tambientIntensity 1
\tdirection 6 -16 -10
\tintensity 4
}}
Floor {{
\ttranslation {floor_x} 0 0
\tsize {floor_sx} {floor_sy}
\tappearance Parquetry {{
\t\ttype "chequered"
\t\tIBLStrength 0
\t}}
}}
Solid {{
\trotation 0 0 1 3.14159
\tchildren [
\t\tCadShape {{
\t\t\turl [
\t\t\t\t"meshes/RoboticPark.dae"
\t\t\t]
\t\t}}
\t]
\tcontactMaterial ""
\tboundingObject Mesh {{
\t\turl [
\t\t\t"meshes/RoboticPark_new.dae"
\t\t]
\t}}
\tradarCrossSection 1
}}
"""

CRAZYFLIE_BLOCK = """Crazyflie {{
\ttranslation {x:.3f} {y:.3f} {z:.3f}
\tname "{name}"
\tcontroller "<extern>"
{supervisor}  extensionSlot [
    InertialUnit {{
    }}
  ]
}}
"""

KHEPERA_BLOCK = """Khepera4 {{
\ttranslation {x:.3f} {y:.3f} {z:.3f}
\tname "{name}"
\tcontroller "<extern>"
  turretSlot [
    GPS {{
    }}
    InertialUnit {{
    }}
  ]
}}
"""


def generate_wbt(out_path, description, robots, floor=(-1, 8, 6), needs_khepera_proto=False):
    """robots: list of dicts with keys name, kind ('Crazyflie'|'Khepera4'),
    x, y, z, supervisor (bool, at most one True -- the Webots-side robot
    the mars_supervisor_pkg Supervisor node attaches to)."""
    parts = [WBT_HEADER]
    if needs_khepera_proto:
        parts.append(KHEPERA_EXTERNPROTO)
    parts.append(WBT_ENV.format(
        description=description, floor_x=floor[0], floor_sx=floor[1], floor_sy=floor[2]))
    for r in robots:
        if r['kind'] == 'Crazyflie':
            sup = '  supervisor TRUE\n' if r.get('supervisor') else ''
            parts.append(CRAZYFLIE_BLOCK.format(
                x=r['x'], y=r['y'], z=r['z'], name=r['name'], supervisor=sup))
        elif r['kind'] == 'Khepera4':
            parts.append(KHEPERA_BLOCK.format(x=r['x'], y=r['y'], z=r['z'], name=r['name']))
        else:
            raise ValueError(f"Unknown robot kind: {r['kind']}")
    out_path.write_text('\n'.join(parts))
    print(f"wrote {out_path}")


def _substitute(obj, old, new):
    if isinstance(obj, dict):
        return {k: _substitute(v, old, new) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_substitute(v, old, new) for v in obj]
    if isinstance(obj, str):
        return obj.replace(old, new)
    return obj


def _extract_robot_block(displays, robot_name):
    start = next(
        i for i, d in enumerate(displays)
        if d.get('Class') == 'rviz_default_plugins/RobotModel' and d.get('Name') == robot_name)
    end = start + 1
    while end < len(displays) and displays[end].get('Class') != 'rviz_default_plugins/RobotModel':
        end += 1
    block = displays[start:end]
    keep_suffixes = ('-local_pose', '-target_pose', '-goal_pose', '-path')
    return [
        d for d in block
        if d.get('Name') == robot_name or
        (isinstance(d.get('Name'), str) and d['Name'].endswith(keep_suffixes))
    ]


def generate_rviz(out_path, robot_names, sheep_name=None):
    """robot_names: Crazyflie names to clone from the dron01 template
    block. sheep_name: if given, also clone the khepera01 template block
    (with goal_pose kept) under this name."""
    data = yaml.safe_load(RVIZ_TEMPLATE.read_text())
    displays = data['Visualization Manager']['Displays']

    dron_template = _extract_robot_block(displays, 'dron01')
    shared = [d for d in displays if d.get('Name') in ('Grid', 'RobotModel')]

    new_displays = list(shared)
    for name in robot_names:
        new_displays.extend(_substitute(copy.deepcopy(dron_template), 'dron01', name))

    if sheep_name:
        khepera_template = _extract_robot_block(displays, 'khepera01')
        new_displays.extend(_substitute(copy.deepcopy(khepera_template), 'khepera01', sheep_name))

    data['Visualization Manager']['Displays'] = new_displays
    if sheep_name:
        data['Visualization Manager']['Views']['Current']['Target Frame'] = \
            f'/{sheep_name}/base_link'
    else:
        data['Visualization Manager']['Views']['Current']['Target Frame'] = '<Fixed Frame>'

    out_path.write_text(yaml.dump(data, sort_keys=True, default_flow_style=False))
    print(f"wrote {out_path}")


if __name__ == '__main__':
    import math

    # ---- Herding: circular herder ring around khepera01, N = 7, 9, 13 ----
    for N in (7, 9, 13):
        radius = 1.1
        robots = []
        for k in range(N):
            a = 2 * math.pi * k / N
            robots.append({
                'name': f'dron{k + 1:02d}', 'kind': 'Crazyflie',
                'x': radius * math.cos(a), 'y': radius * math.sin(a), 'z': 0.015,
                'supervisor': (k == 0),
            })
        robots.append({
            'name': 'khepera01', 'kind': 'Khepera4', 'x': 0.0, 'y': 0.0, 'z': 0.015,
        })
        generate_wbt(
            WORLDS_DIR / f'Herding_N{N:02d}.wbt',
            f'Herding control. N herders: {N}.',
            robots, needs_khepera_proto=True)
        generate_rviz(
            RVIZ_DIR / f'Herding_N{N:02d}.rviz',
            [f'dron{k + 1:02d}' for k in range(N)], sheep_name='khepera01')

    # ---- Formaciones Adaptativas: 12-drone frustum ("shield") mesh ----
    # Webots/real spawn is a compact, safe take-off grid -- NOT the target
    # mesh shape -- so the experiment actually demonstrates convergence
    # into the triangulated formation under the control law, rather than
    # starting already at the target (see Herding_*/topics.yaml pattern:
    # target_pose commands drive the two leaders to their reference mesh
    # position after take-off; the followers converge under eq 55).
    grid_x = [-0.75, -0.25, 0.25, 0.75]
    grid_y = [-0.5, 0.0, 0.5]
    grid = [(x, y) for y in grid_y for x in grid_x]  # 12 slots, 0.5 m spacing
    robots = [
        {
            'name': f'dron{i:02d}', 'kind': 'Crazyflie',
            'x': grid[i - 1][0], 'y': grid[i - 1][1], 'z': 0.3,
            'supervisor': (i == 1),
        }
        for i in range(1, 13)
    ]
    generate_wbt(
        WORLDS_DIR / 'AdaptiveFormation_N12.wbt',
        'Triangulated 3D affine formation control (adaptive/scale-flexible). N: 12.',
        robots, floor=(-1, 10, 8))
    generate_rviz(
        RVIZ_DIR / 'AdaptiveFormation_N12.rviz',
        [f'dron{i:02d}' for i in range(1, 13)], sheep_name=None)
