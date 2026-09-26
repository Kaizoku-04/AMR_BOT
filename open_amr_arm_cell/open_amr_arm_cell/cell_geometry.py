"""Geometry of a UR10e palletizing cell: arm on a pedestal, two EUR pallets beside/behind it, the AMR's conveyor deck
in front. Pure Python (no ROS), the single source for the pallet pattern: the cell controller, the reach study, the
world builder and the simulator's box drawing all use it, so a box is always where the arm reaches for it.

Frames
  world  warehouse frame (sim/configs/warehouse_layout.yaml), z up from the floor.
  cell   origin on the floor under the arm's base, x pointing from the arm to the AMR bay, z up. The robot description
         (urdf/arm_cell.urdf.xacro) uses this as its root link `world`: the UR base_link sits at z = pedestal_height.

Layout (chosen by a reach study, 2026-09-26, see OpenAMR notes/Isaac Sim Setup.md "Arm cells"):
  - the AMR docks nose-in on the cell's x axis; its deck centre is `deck_distance` from the arm's axis;
  - pallet 0 at +pallet_angle_deg, pallet 1 at -pallet_angle_deg around the arm (angle from the x axis), centres at
    `pallet_radius`, long side (1.2 m) tangential;
  - pattern 3 x 2 boxes per layer (along the pallet's long / short side), `layers` layers; slot k = layer k // 6,
    in-layer index k % 6 = long index * 2 + short index. A pallet holding n boxes has slots 0 .. n-1 filled
    (depalletizing takes slot n-1, palletizing fills slot n).
"""
import math
from dataclasses import dataclass, field


@dataclass
class CellParams:
    pedestal_height: float = 1.1         # UR base mounting face above the floor
    pedestal_size: float = 0.5           # square pedestal footprint
    pallet_radius: float = 0.8           # arm axis -> pallet centre
    pallet_angle_deg: float = 120.0      # pallets at +/- this angle from the cell x axis (towards the AMR)
    deck_distance: float = 0.85          # arm axis -> centre of the docked AMR's conveyor deck
    deck_top: float = 0.334              # conveyor deck surface above the floor (Mission Design §2)
    pallet_size: tuple = (1.2, 0.8, 0.144)   # EUR pallet: long, short, height
    box: tuple = (0.35, 0.35, 0.25)      # carton: along pallet long side, along short side, height
    pattern: tuple = (3, 2, 4)           # boxes along long side, along short side, layers
    gap: float = 0.005                   # between boxes on a layer
    tool_length: float = 0.20            # UR tool flange -> vacuum cup face (the TCP)
    approach: float = 0.10               # TCP above a box top before descending / after releasing
    lift_clearance: float = 0.05         # a lifted box's bottom clears neighbouring box tops by this

    @classmethod
    def from_layout(cls, layout):
        """CellParams from the `arm_cell` block of warehouse_layout.yaml (+ `box.size`)."""
        c = dict(layout.get('arm_cell', {}))
        if 'box' in layout and 'size' in layout['box']:
            c['box'] = tuple(layout['box']['size'])
        for k in ('pallet_size', 'box', 'pattern'):
            if k in c:
                c[k] = tuple(c[k])
        return cls(**c)

    @property
    def per_layer(self):
        return self.pattern[0] * self.pattern[1]

    @property
    def capacity(self):
        return self.per_layer * self.pattern[2]


@dataclass
class Cell:
    """One cell placed in the warehouse. arm_pose = (x, y, yaw_deg): the arm's axis on the floor and the direction
    from the arm to its AMR bay."""
    arm_pose: tuple
    p: CellParams = field(default_factory=CellParams)

    # ---------------------------------------------------------------- frames
    def to_world(self, x, y, z=0.0):
        ax, ay, ayaw = self.arm_pose
        c, s = math.cos(math.radians(ayaw)), math.sin(math.radians(ayaw))
        return (ax + c * x - s * y, ay + s * x + c * y, z)

    def yaw_world(self, yaw_cell_deg):
        return (yaw_cell_deg + self.arm_pose[2] + 180.0) % 360.0 - 180.0

    # ---------------------------------------------------------------- cell frame
    def pallet_cell(self, i):
        """Pallet i centre (x, y) and yaw (deg, direction of its long side) in the cell frame."""
        a = math.radians(self.p.pallet_angle_deg if i == 0 else -self.p.pallet_angle_deg)
        r = self.p.pallet_radius
        return (r * math.cos(a), r * math.sin(a)), math.degrees(a) + 90.0

    def slot_cell(self, i, k):
        """Top centre (x, y, z) of the box in slot k of pallet i, and the box's yaw (deg), cell frame."""
        nl, ns, nz = self.p.pattern
        if not 0 <= k < self.p.capacity:
            raise ValueError(f'slot {k} outside 0..{self.p.capacity - 1}')
        layer, j = divmod(k, self.p.per_layer)
        a, b = divmod(j, ns)
        pl, ps = self.p.box[0] + self.p.gap, self.p.box[1] + self.p.gap
        u, v = (a - (nl - 1) / 2) * pl, (b - (ns - 1) / 2) * ps
        (cx, cy), yaw = self.pallet_cell(i)
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        z = self.p.pallet_size[2] + (layer + 1) * self.p.box[2]
        return (cx + c * u - s * v, cy + s * u + c * v, z), yaw

    def deck_cell(self):
        """Top centre of a box standing on the docked AMR's deck, and its yaw (the AMR faces the arm), cell frame."""
        return (self.p.deck_distance, 0.0, self.p.deck_top + self.p.box[2]), 180.0

    def lift_height(self, box_top_z):
        """TCP height that lifts a box whose top is at box_top_z clear of neighbouring boxes of the same layer."""
        return box_top_z + self.p.box[2] + self.p.lift_clearance

    def transit_height(self):
        """TCP height at which a carried box clears a full pallet (the taught via height for transfers that can't
        swing across directly)."""
        return self.lift_height(self.p.pallet_size[2] + self.p.pattern[2] * self.p.box[2])

    # ---------------------------------------------------------------- world frame
    def pallet_world(self, i):
        (x, y), yaw = self.pallet_cell(i)
        wx, wy, _ = self.to_world(x, y)
        return (wx, wy, self.yaw_world(yaw))

    def slot_world(self, i, k):
        (x, y, z), yaw = self.slot_cell(i, k)
        return self.to_world(x, y, z), self.yaw_world(yaw)

    def bay_pose(self):
        """Where the AMR's base_footprint stops (deck centred on it): (x, y, yaw_deg), facing the arm."""
        x, y, _ = self.to_world(self.p.deck_distance, 0.0)
        return (x, y, self.yaw_world(180.0))

    def footprint_world(self):
        """Floor rectangles occupied by the cell (pedestal, pallets) as (cx, cy, size_x, size_y, yaw_deg)."""
        out = [(*self.to_world(0.0, 0.0)[:2], self.p.pedestal_size, self.p.pedestal_size, self.arm_pose[2])]
        for i in (0, 1):
            x, y, yaw = self.pallet_world(i)
            out.append((x, y, self.p.pallet_size[0], self.p.pallet_size[1], yaw))
        return out
