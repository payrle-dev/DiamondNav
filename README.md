# DiamondNav


<img width="4096" height="4096" alt="d2f3d761-d9cb-439d-92f1-ed72a506a713" src="https://github.com/user-attachments/assets/95a88f22-a7e4-43c9-81b8-4fd04f2d135c" />

[![NPC police cars chasing a player through Los Santos](docs/preview.gif)](https://www.youtube.com/watch?v=mnA-6iFgfE4)

**[Watch the preview on YouTube](https://www.youtube.com/watch?v=mnA-6iFgfE4)**: driving, a police chase,
and walking on foot up the escalators.

Walking paths and driving routes for [open.mp](https://open.mp) NPCs.

- **Walking:** NPCs find their way on foot round walls, up stairs and across
  bridges, on a navmesh baked from the game world's collision (Recast/Detour).
- **Driving:** NPC drivers follow the roads and keep to their lane, on a road graph
  built from GTA's own traffic paths. They drive like their car: GTA's own vehicle
  physics are run offline on each model's handling data (gears, engine, drag,
  tyres, brakes, steering, turn mass, off-road ability) and the size of its model,
  so each pulls away, tops out, brakes, corners and turns in as it does in the
  game, on a road and on dirt. Each road has a speed limit by its class (highway, main road,
  street, minor road, service road), they brake in time for corners and junctions,
  and a driving style (cautious, normal, fast, pursuit) sets how hard they push.
- **Chasing:** an NPC driver chases a player along the roads, and off them across a
  car-sized navmesh when the player leaves the road; close in, it drives straight
  at them when nothing is in the way, keeps up alongside a car that is getting
  away, and follows a player off a bridge or a ledge, through a gap in its railing.
  With the world's collision loaded they see every wall and object (yours too) and
  never drive or fall through one, and a pursuit takes the shortest way, turning
  round or going the wrong way down a one-way street. Several chasers come up either
  side of their target. Drivers steer like cars: they curve into corners, brake for
  them, and back up to turn round in tight spots.
- **Planes:** NPC pilots fly planes from gate to gate: pushed back, along the
  taxiways, up, along a route and down a proper approach to a landing and a gate,
  each plane kept to the runways, taxiways and gates its wings fit. Air traffic
  control keeps them apart on runways, taxiways and in the air, with holds and
  go-arounds. San Andreas' four airports are built in; add your own or reshape them
  in a text file.
- **Traffic sense:** every driver swerves round, or brakes behind, the vehicles in
  its way (other NPCs' cars, players' cars, parked cars), by each model's own
  collision box and where it is heading.
- **Your maps too:** custom SA-MP maps are baked in: objects your scripts create,
  buildings they remove, and custom models with their own collision. Build the
  world from a GTA San Andreas install or from ColAndreas's database.
- **Never holds up the server:** searches run on their own threads; results come
  back on the game thread through a callback.
- **Optional extra:** a build of open.mp's NPC component in which NPCs can fire from
  cars (drive-bys) and cars look right while NPCs drive them.

```pawn
#include <open.mp>
#include <DiamondNav>

Nav_WalkTo(npcid, x, y, z);                // walk there along the navmesh
Nav_DriveTo(npcid, x, y, z);               // drive the NPC's car there along the roads
Nav_Chase(npcid, playerid);                // chase a player, on or off the road
Nav_FlyTo(npcid, x, y, z);                 // fly the NPC's helicopter there
Nav_Land(npcid, "LSIA");                   // land the NPC's plane at LS airport, taxi to a gate

public OnNavMoveEnd(npcid, reason)
{
    if (reason == NAV_END_ARRIVED) { /* ... */ }
    return 1;
}
```

Licensed under the [Mozilla Public License 2.0](LICENSE). Works with open.mp
v1.6.7.3218 (the 32-bit server, Linux or Windows).

## How it works

DiamondNav works in two halves: an offline bake on your PC, and a component that
uses what was baked while the server runs.

**Offline (`navtools/`).**
- `dnav.py world` reads the map's collision from your GTA install (each model's
  collision, placed where the map puts it), adds your own maps, and writes it out as
  triangles.
- `navbake` turns those triangles into navmeshes with Recast: a walking one, sized
  for a ped, and a car one, sized for a car. It keeps only the floor GTA's own
  pedestrian paths can reach, so roofs and counter tops don't count as walkable.
- `dnav.py roads` turns GTA's traffic paths into a road graph. Each road has its
  class, lanes and direction.
- `dnav.py vehicles` runs GTA's vehicle physics on every model's handling data and
  saves how each one accelerates, brakes and turns.
- The car triangles are also kept as `world.col`, so drivers can see walls at run time.

**On the server (the component).**
- It loads `navmesh/` at start.
- A script's request (walk, drive, chase) goes into a queue. Worker threads, each
  with its own Detour query, run the search.
  - Walking: A* across the navmesh, then the straightest line along the result, with
    points added where the ground bends.
  - Driving: A* over the road graph, offset into the right-hand lane, with a speed
    for every point.
- Results come back on the game thread, so no Pawn code ever runs off it. The
  component then moves the NPC every tick:
  - Walkers step from point to point.
  - Cars are driven like cars: speed, braking and steering follow that model's
    numbers, with rays against `world.col` for walls, ledges and the ground ahead.
  - Helicopters fly their own flight model over the tops of whatever is ahead.
  - Planes follow a pilot through each phase of a flight (src/pilot.cpp): on the
    ground along paths planned for their turning circle and speed, in the air a
    fixed-wing flight model, with air traffic control (src/atc.cpp) between them.
- A chase plans again as its target moves, and keeps driving the old plan until
  the new one is ready.
- Two kinds of extra links are added on top of the meshes at start:
  - drops off ledges, which only chases use;
  - escalators from `navmesh/links.txt`, since their moving steps are not in GTA's
    collision.

## Install

Download the archive for your server from
[Releases](https://github.com/payrle-dev/DiamondNav/releases) and unpack it into
the server folder:

| From the archive | What it is |
| --- | --- |
| `components/DiamondNav.so` / `.dll` | the component; open.mp loads it at start |
| `qawno/include/DiamondNav.inc` | the include for your scripts |
| `navmesh/` | where the baked files go |
| `navtools/` | `navbake`, `navquery` and `dnav.py`, to bake on your PC |
| `extras/NPCs.so` / `.dll` | optional: the improved NPC component ([below](#the-improved-npc-component)) |

Then bake your world into `navmesh/` (next section). The server log says what it
loaded:

```
[nav] loaded roads.dnr: <nodes> road nodes, <links> links
[nav] loaded world.nav (walking)
[nav] loaded vehicles.dnv: <models> vehicle models
[nav] loaded world.col: <faces> faces
[nav] 4 airports, 0 flight routes (0 airport files)
[nav] DiamondNav 1.4.0: <tiles> walking tiles, <tiles> car tiles, <nodes> road nodes, 3 search threads
```

## Baking your world

Baking happens once, on your PC, whenever your maps change. The tools need
Python 3.9+ with numpy (`pip install numpy`, plus `pillow` for previews).

### 1. The world's collision

From a GTA San Andreas install (the 0.3.DL client folder players use is best: it
has SA-MP's own objects in `SAMP/samp.img`):

```sh
python navtools/dnav.py world bake/world --gta "C:/Games/GTA San Andreas"
```

Or from [ColAndreas](https://github.com/Pottus/ColAndreas)'s database, if you
already have one (`scriptfiles/colandreas/ColAndreas.cadb`):

```sh
python navtools/dnav.py world bake/world --colandreas scriptfiles/colandreas/ColAndreas.cadb
```

Give both to get the database's objects and the install's path nodes and water
(the path nodes let the bake drop roofs and closed-off spaces nobody walks on).

### 2. Your maps

Add `--map` for each map file or folder, and `--models` for your custom models:

```sh
python navtools/dnav.py world bake/world --gta "C:/Games/GTA San Andreas" \
    --map gamemodes/maps --map filterscripts/mapping.pwn --map maps/island.map \
    --models models
```

- **Map files** are read for literal `CreateObject`, `CreateDynamicObject`,
  `CreateDynamicObjectEx`, `CreatePlayerObject` and `RemoveBuildingForPlayer` calls
  (`.pwn`, `.inc`, `.txt`), or MTA `.map` files (`<object>` and
  `<removeWorldObject>`), which most map editors export. A call whose arguments
  are variables can't be read; the build counts them.
- **Custom models** (`AddSimpleModel`, in your scripts or in
  `models/artconfig.txt`) get their collision from their DFF, as in the game
  (SA-MP's collision plugin, which 0.3.DL models carry, or Rockstar's), or from a
  `.col` file in the models folder.

The build prints what it placed and what had no collision.

### 3. Bake the navmesh

```sh
navtools/navbake bake/world.tris navmesh/world.nav --seeds bake/world.seeds.txt --water bake/world.water.txt
```

For cars that leave the road (`Nav_Chase`, `NAV_DRIVE_OFFROAD`), bake the same world
again for a car:

```sh
navtools/navbake bake/world.car.tris navmesh/world_car.nav --agent car --water bake/world.water.txt
```

For chases, copy the world's collision itself into `navmesh/` too. Drivers cast
rays against it to see walls and your objects, and to find the ground under a car
that falls; without it they only know what the car navmesh says:

```sh
cp bake/world.car.tris navmesh/world.col
```

`world.car.tris` is `world.tris` without the stock objects a car breaks or knocks
over in the game (data/object.dat: fences, railings, lamp posts, signs, bins,
barriers), so cars go straight through those, on the car navmesh and in a chase,
as a player's car would. Objects your scripts create never break in SA-MP, so they
stay solid.

(`dnav.py world` prints these lines with the files it wrote.) `--area "X0 Y0 X1 Y1"` on `dnav.py world` bakes part of it, which is
quicker for trying things out. A `--routes` file makes navbake check that paths
exist (each line `name<TAB>x y z<TAB>x y z`) and fail if one doesn't.

### 4. The roads

```sh
python navtools/dnav.py roads navmesh/roads.dnr --gta "C:/Games/GTA San Andreas"
```

For a custom map with its own streets, add a roads file with `--roads my_roads.txt`
(`dnav.py roads --help-format` explains it):

```
# a car park off the main road
node gate   1520.0 -1650.0 13.5
node park   1560.0 -1650.0 13.5
road 1500.0,-1650.0,13.5 gate 1 1      # join the nearest stock road node
road gate park 1 0                     # one-way into the car park
remove 1600 -1700 1700 -1600           # this map builds over the stock road here
```

### 5. How each vehicle drives

```sh
python navtools/dnav.py vehicles navmesh/vehicles.dnv --gta "C:/Games/GTA San Andreas"
```

This runs GTA's own vehicle physics for every model, once, on your PC: the gearbox
(gears, the first- and second-gear boosts, the speed-limit flag), the engine's
acceleration and inertia, air drag, the tyres (traction, traction loss, the
front/rear bias, wheelspin), the brakes and their bias, from `data/handling.cfg`
(and its bike lines: how far a bike leans); how well tyres grip a road and dirt
from `data/surface.dat` (with the off-road ability flags); and each model's
wheelbase, track and size from its DFF in `models/gta3.img`. It writes each model's
acceleration at every speed on a road and on dirt, top speed, braking, cornering
grip (the weaker axle, a bike's lean, the roll a tall vehicle takes), turning
circle, how fast it can start to turn (its turn mass), reverse speed and size. A
car with the Infernus's line tops out at 221 km/h, as measured in the game (222),
not handling.cfg's 240.

Without it every car drives as the same average car. To check a model against the
real game, drive it yourself with a script that times your top speed, 0-100 km/h,
braking and cornering next to `Nav_GetVehicleInfo`'s numbers, and if one is off, set
it from your script with `Nav_SetVehicleHandling`.

### 6. Escalators (links.txt)

An escalator's moving steps are not in GTA's collision, so the navmesh has nothing
joining its foot to its top, and whatever only an escalator leads to (the plaza over
the Los Santos bank's) is baked as unreachable, like a roof. List each one in
`navmesh/links.txt` and walkers use it, both ways, with no rebake:

```
# x y z of each point: the ground at its foot, its first step, its top step, the ground at the top
1435.90 -1498.37 12.55   1435.03 -1497.87 12.90   1426.54 -1492.98 19.42   1424.80 -1491.98 19.42
```

A link is two or more points; its ends must be within 1.5 m of the walking mesh, and a
walker follows the points between (so it climbs the steps' slope, not a line from
landing to landing). Anything only a link reaches is opened at start. The server's log
says how many it loaded, and an error names a link whose ends find no mesh or that
does not join them. A walker sent somewhere from partway up, or after someone standing
on the steps, goes along the escalator instead of off its side.

### Checking a bake

```sh
navtools/navquery navmesh info
navtools/navquery navmesh path  1700 -2490 13.5  1650 -2300 13.5
navtools/navquery navmesh route 1700 -2490 13.5  1480 -1730 13.5
navtools/navquery navmesh path  2350 -1819 12.5  2350 -1852 0.7 --car --ledges
navtools/navquery navmesh route 1480 -1730 13.5  1700 -1440 13.5 --heading 90 --speed 20
navtools/navquery navmesh road  1480 -1730
python navtools/dnav.py preview check.png --bake bake/world --roads navmesh/roads.dnr
```

## Scripting

See [`include/DiamondNav.inc`](include/DiamondNav.inc) for every native, and
[`examples/`](examples) for whole scripts (NPCs wandering the city, a taxi that
drives to you).

**Let DiamondNav move the NPC.** `Nav_WalkTo` and `Nav_DriveTo` search, then move
the NPC along the result point by point with open.mp's `NPC_Move`, and call
`OnNavMoveEnd(npcid, reason)` at the end: `NAV_END_ARRIVED`, `NAV_END_NO_PATH`,
`NAV_END_STOPPED` (`Nav_Stop`, or a new move), or `NAV_END_INTERRUPTED` (another
script moved it, it left the driver's seat, died or was destroyed). A driver must
be in seat 0; `carZ` is how high the car's centre sits over the road, and the
default (0.0) takes the model's own: `vehicles.dnv`'s, worked out from its wheels
and springs as the game places it, else a guess from open.mp's model table
(`NAV_VINFO_RIDE`). Off a road route (a straight run at a chase target, or a car
left with no path), the car keeps to the ground under it: the car navmesh, else the
walking navmesh, else the road surface nearby; it never climbs onto a level it has
no ground for.

**How cars drive.** A driver steers like a car: it looks ahead along its path,
turns no tighter than its turning circle (nor, at speed, than its grip holds),
eases into and out of turns as fast as its turn mass lets it, pulls away as its
own gears and engine allow at each speed (less on dirt) and
brakes in time to take each corner, junction or lower limit at the speed it
allows, then to stop at the end. What sets the speed:

- **The road's class**, read from GTA's path data, each with a limit you can change
  with `Nav_SetRoadSpeed` (km/h): `NAV_CLASS_HIGHWAY` 130, `NAV_CLASS_MAIN` 80 (two
  or more lanes each way, or one-way), `NAV_CLASS_STREET` 60, `NAV_CLASS_MINOR` 45
  (GTA's low-traffic roads), `NAV_CLASS_SERVICE` 30 (emergency-only roads) and
  `NAV_CLASS_OFFROAD` 60 (the car navmesh). A custom roads file marks its own with
  `main` / `minor` on a node.
- **The car**, from `vehicles.dnv` or `Nav_SetVehicleHandling`; read any of it with
  `Nav_GetVehicleInfo` (`NAV_VINFO_*`), `Nav_GetVehicleAccel` and
  `Nav_GetVehicleAccelTime`.
- **The style**, `Nav_SetDriveStyle(npcid, style)`: `NAV_STYLE_CAUTIOUS` (under the
  limits, gentle), `NAV_STYLE_NORMAL` (the limits; `Nav_DriveTo`'s default),
  `NAV_STYLE_FAST` (a quarter over, brisk corners, late braking) and
  `NAV_STYLE_PURSUIT` (no limits at all: flat out, cornering on the limit of the
  car's grip and braking as hard as it can; `Nav_Chase`'s default).
- **A cap**, if you pass a `speed` above 0 to `Nav_DriveTo` or `Nav_Chase`
  (in `NPC_Move`'s units: 1.0 is 10 m/s).

`Nav_GetDriveInfo` tells you how a driver is doing: its speed, the speed it is going
for, the road it is on and that road's limit. With the improved NPC component a
car also backs up to turn round in a tight spot.

**Other vehicles.** Every driver sweeps the course it is about to take against
every vehicle in its world, by each model's collision box (`vehicles.dnv`, from its
DFF: its size and where the box sits; else open.mp's model table) turned the way
that vehicle faces, and moved on at its own speed. If it would touch one, it
swerves onto the smallest change of heading that touches nothing, keeping to one
side until it is past; with no way past it brakes to stop short of it, or follows
it at its speed, and after a moment stopped it backs up to make room (with the
improved NPC component). Parked cars, players' cars and other NPCs' all count; a
chaser with `NAV_CHASE_RAM` goes into its target's car on purpose.

**Chase.** `Nav_Chase(npcid, targetid, speed, carZ, flags)` keeps a driver after a
player (or another NPC) until `Nav_Stop`. It follows the roads while the target is
on one, the car navmesh once it leaves them, and drives straight at where the
target is going (as far ahead as the car takes to get there) when it is close and
nothing is in the way: `Nav_IsLineClear` on the car navmesh, or, without one, a
line that runs over road all the way (up to 150 m) or a very short one. With
`world.col` loaded that line must also clear every wall at bumper height and keep
to the ground, and a wall that blocks a straight run sends it to the roads for a
moment. Its road routes are a pursuit's: from wherever the car can get on the road,
turning round if that is shorter (it costs about 35 m), and the wrong way down a
one-way road at a small cost. It aims at where the target is heading on average,
not where it points this instant, so a target swerving about does not make it
brake. With `world.col`, a target below it is driven at straight down a bank or off
a ledge, and a chaser that goes over an edge falls. With the car navmesh as well,
the server finds every ledge a car could drive off at boot (a bridge's open side, a
storm drain's edge: a run-up clear of walls and posts, a drop of 2 to 15 m onto more
of the car navmesh), and a chase's routes use them as shortcuts, so a chaser drives
round to the nearest way down instead of the long way by road. It turns round in a
narrow street with a three-point turn, and backs out and tries the other side when
it is wedged on a post. A chaser left far behind a
moving target gets extra speed (up to 35% at 160 m), as the game's police do. It keeps
up with a target that is moving, pulling up alongside on its left, stops beside
one that stands still, and re-plans as the target moves without pausing. A target
that drops off a bridge or a ledge (up to 30 m down, within 60 m) is followed off
the edge (without the car navmesh's ledges): the car drives to it on its own level and falls, keeping its speed, to
the road below. With `world.col` it finds where the edge is, lines up square to it
and goes through a gap in the railing (the nearest within 12 m), stops falling
forward when it meets a wall, and lands on whatever is under it (`NAV_CHASE_NO_DROP` takes the roads round instead). Several NPCs
after one target come up either side of it instead of in a line;
`NAV_CHASE_RAM` drives into it without slowing. `Nav_GetChaseInfo(npcid, distance,
mode)` says how far the target is and what the chaser is doing (beside it,
straight at it, on a route, running at an edge, in the air). It ends with
`NAV_END_LOST` if the target leaves or changes world. What the NPCs do during the
chase (shoot, give up) is your script's. `NAV_STYLE_PURSUIT` drives flat out:
cornering on the limit of the car's grip and braking as hard as it can.

**Helicopters.** `Nav_FlyTo(npcid, x, y, z, speed, flags)` flies an NPC pilot's
helicopter to a point. There is no path: it goes straight there, high enough to keep
20 m over everything under and ahead of it (the tops of buildings, trees and hills
in `world.col`, and the sea), climbing first when a tower is in the way and stopping
short of anything solid ahead. It slows in time to stop and hovers at `z`, or, with
`NAV_FLY_LAND`, comes down a glide slope and sets down on the ground or roof there.
It flies like a helicopter: nose down to speed up, nose up to slow, banked into
turns, at each model's own speed (a Maverick about 180 km/h, a Hunter 215, a
Leviathan 135). `Nav_Chase` with an NPC flying a helicopter keeps over the target instead of
driving after it (`NAV_CHASE_OVERHEAD`): 15 m behind and 28 m up while it moves,
circling over it when it stops, looking at it. It stays below 45 m over the target
and plans its way round anything taller (a grid search over the tops in
`world.col`, so it flies down the streets between towers and keeps its rotor 10 m
off walls), climbing over only what it cannot get round. The flight is DiamondNav's
own: no NPC_Move, the helicopter's position, rotation and velocity are set every tick
and go out in its driver sync. Without `world.col` it keeps only to the height it is
given.

**Planes.** An NPC put in a plane as its driver flies it the way a pilot would:

```pawn
Nav_ParkPlane(npcid, "LSIA", "E1");       // at a gate, parked
Nav_Takeoff(npcid, "", "LS_TO_LV");       // pushed back, taxi, take off, fly the route
// ... OnNavMoveEnd(npcid, NAV_END_ARRIVED) once it has flown it
Nav_Land(npcid, "LVA");                   // approach, land, taxi to a free gate that fits
```

- **On the ground** it taxis along the airport's taxiways on a path rounded to its
  turning circle, slowing for turns, and a tug pushes it back off a gate. It keeps to
  the taxiways and gates its wings fit, so an AT-400 never takes a taxiway that only
  a Dodo fits down.
- **Taking off,** it holds short of the runway until cleared, lines up, rolls and
  rotates at its own speed, and climbs out (along the runway's departure fixes, if
  it has any) before turning onto its route.
- **Landing,** it picks the runway (long and wide enough for it, the airport's
  preferred way), flies a pattern onto a straight final sized to the plane, comes
  down the glide path, flares and touches down, brakes and turns off at the first
  exit it can make, then taxis to the gate.
- **Air traffic control** keeps one plane on a runway at a time, holds arrivals
  when the runway will be busy, sends a plane round when the runway is not clear at
  the decision point (or it is not lined up), and on the ground stops planes short
  of each other and of vehicles in their way and has them give way at a taxiway
  someone is coming along the other way.
- **In the air** it keeps clear of the ground and buildings ahead in `world.col`,
  climbing or turning away, and of other planes (the lower one turns right, the
  higher climbs).
- `Nav_FlyRoute` flies a route and then circles, `Nav_FlyTo` flies to a point, and
  `Nav_Chase` goes after a player (keeping `carZ` metres over them).
- `OnNavPlanePhase(npcid, phase)` reports each phase (pushback, taxi out, hold
  short, line up, take-off, climb, cruise, holding, approach, final, go-around,
  landing, taxi in, at gate), for camera shots, announcements or tickets.

The airports are text files: San Andreas' four (LSIA, SFIA, LVA, Verdant Meadows)
are built in, and `navmesh/airports/*.txt` add your own airports or reshape the
stock ones (a new taxiway, a gate moved, a runway closed). See
[docs/AIRPORTS.md](docs/AIRPORTS.md) for the format, the stock airports and how much
runway each plane needs.

**Lines.** `Nav_IsLineClear(x1, y1, z1, x2, y2, z2, kind)` says whether a ped
(`NAV_MESH_PED`) or a car (`NAV_MESH_CAR`) can go straight between two points on the
ground: no wall, drop or obstacle. It follows the ground, so it is not an eye-level
line of sight; drivers use the world's collision (`world.col`) for that.

**Or take the points.** `Nav_RequestPath` (walking) and `Nav_RequestRoute`
(driving) return a request id; `OnNavPathResult(requestid, status, points)` follows
on a later tick. Read the points with `Nav_GetPathPoint` (and, for routes, each
point's speed factor with `Nav_GetPathPointSpeed` and road class with
`Nav_GetPathPointClass`) inside that callback.

**Awaiting a search with PawnPlus (optional, recommended).** DiamondNav needs
nothing but open.mp. Every search already runs on the component's own threads and
never holds up the server. With [PawnPlus](https://github.com/IllidanS4/PawnPlus)
you can also wait for a search in the middle of a function, without splitting your
code into a request here and a callback there. The function pauses at
`task_await`, the server keeps running, and the function carries on with the points
once the search is done:

```pawn
#include <PawnPlus>
#include <DiamondNav>

static Map:s_NavSearches;   // request id -> the Task waiting on it

// Starts a walking path search and waits for it without blocking the server: the
// calling function pauses here and carries on when the path is found. Returns a
// List of points (each a Float[3]: x, y, z), or INVALID_LIST when there is no way.
// Delete the list with list_delete when done with it.
stock List:Nav_FindPathAsync(Float:fromX, Float:fromY, Float:fromZ, Float:toX, Float:toY, Float:toZ)
{
    new id = Nav_RequestPath(fromX, fromY, fromZ, toX, toY, toZ);
    if (!id) return INVALID_LIST;
    if (!map_valid(s_NavSearches)) s_NavSearches = map_new();
    new Task:t = task_new();
    map_add(s_NavSearches, id, t);
    return List:task_await(t);
}

public OnNavPathResult(requestid, status, points)
{
    new Task:t;
    if (!map_valid(s_NavSearches) || !map_get_safe(s_NavSearches, requestid, t)) return 1;
    map_remove(s_NavSearches, requestid);
    new List:path = INVALID_LIST;
    if (status == NAV_OK || status == NAV_PARTIAL) {
        path = list_new();
        new Float:p[3];
        for (new i = 0; i < points; i++) {
            Nav_GetPathPoint(requestid, i, p[0], p[1], p[2]);
            list_add_arr(path, p);
        }
    }
    task_set_result(t, path);
    return 1;
}

// For example, how far a walk is:
forward ShowWalk(playerid, Float:x, Float:y, Float:z);
public ShowWalk(playerid, Float:x, Float:y, Float:z)
{
    new Float:px, Float:py, Float:pz;
    GetPlayerPos(playerid, px, py, pz);
    new List:path = Nav_FindPathAsync(px, py, pz - 1.0, x, y, z);
    if (path == INVALID_LIST) return SendClientMessage(playerid, -1, "No way there on foot.");
    new msg[64];
    format(msg, sizeof msg, "The way there has %d points.", list_size(path));
    SendClientMessage(playerid, -1, msg);
    list_delete(path);
    return 1;
}
```

`task_await` pauses only the public it was called from (a command, a callback), and
the rest of that public runs on a later tick. `Nav_RequestRoute` works the same way.
Without PawnPlus, read the points in `OnNavPathResult` as above.

Road flags: `NAV_ROAD_EMERGENCY` lets a route use emergency-only roads;
`NAV_ROAD_BOATS` searches the water routes instead (for boats).

## The improved NPC component

`extras/NPCs.so` (`NPCs.dll`) is open.mp's own NPC component, built from the
v1.6.7.3218 source with [our patch](npcs/patches) applied. Copy it over
`components/NPCs.so`. Everything else in open.mp stays stock, and DiamondNav works
without it. What it changes:

- **Drive-bys.** `NPC_AimAt` / `NPC_AimAtPlayer` with shooting on now fire from a
  car. Passengers lean out and aim (any bullet weapon but the sniper rifle and
  minigun); drivers fire the Uzi, MP5 and Tec-9 sideways, as in GTA.
- **Passengers ride along**: their position follows the car (stock left them where
  they got in).
- **Driving looks right**: the front wheels steer into turns, straighten when the
  car stops, and the car never rolls. (Sending the car's speed in the game's units
  and pitching it along slopes were ours until open.mp 1.6.7 did both itself.)
- **Cars back up**: an `NPC_Move` drive with a negative speed reverses away from its
  target. DiamondNav uses it for three-point turns, and for a target just behind the
  car; with the stock component it notices, says so once in the log, and turns
  round going forward instead.

It only fits the open.mp release it was built from. To build it yourself or for
another release, see [npcs/README.md](npcs/README.md).

## Building from source

You only need this if you want to change DiamondNav or build it yourself. The
[Releases](https://github.com/payrle-dev/DiamondNav/releases) archives are built
the same way by CI.

The open.mp server is 32-bit, so everything is built 32-bit. CMake (3.19 or newer)
downloads what it needs while it configures: the open.mp SDK, pawn-natives, the Pawn
headers and Recast/Detour. You need git, and an internet connection the first time.

### Linux

1. Install the tools. On Debian or Ubuntu:

   ```sh
   sudo dpkg --add-architecture i386 && sudo apt-get update
   sudo apt-get install -y git g++-multilib cmake ninja-build python3 python3-numpy python3-pil libstdc++6:i386
   ```

2. Get the code and build it:

   ```sh
   git clone https://github.com/payrle-dev/DiamondNav
   cd DiamondNav
   cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS=-m32 -DCMAKE_CXX_FLAGS=-m32
   cmake --build build
   ```

   You get `build/DiamondNav.so` (the component), `build/navbake` and `build/navquery`.

3. Run the tests (optional):

   ```sh
   ctest --test-dir build                    # the C++ tests
   python3 tools/tests/test_tools.py build   # the Python tools, with a small bake
   ```

### Windows

1. Install [Visual Studio 2022](https://visualstudio.microsoft.com/) (Community is
   fine) with the **Desktop development with C++** workload and its **C++ Clang tools
   for Windows** option, plus [Git](https://git-scm.com/) and
   [Python 3.9+](https://www.python.org/) (then `python -m pip install numpy pillow`
   for the tools).

2. In a Developer PowerShell for VS 2022:

   ```sh
   git clone https://github.com/payrle-dev/DiamondNav
   cd DiamondNav
   cmake -S . -B build -A Win32 -T ClangCL
   cmake --build build --config Release
   ctest --test-dir build -C Release         # optional: the tests
   ```

   You get `build\Release\DiamondNav.dll`, `navbake.exe` and `navquery.exe`.

### Putting it on your server

Copy the component into your server's `components/` folder and
`include/DiamondNav.inc` into `qawno/include/`. Put `navbake`, `navquery` and the
`tools/` folder wherever you bake (see [Baking your world](#baking-your-world)).
Or pack an archive laid out like a release:

```sh
python3 scripts/package.py build build/npcs dist linux    # or windows
```

`package.py` also packs the improved NPC component, which is built on its own:
build it first ([npcs/README.md](npcs/README.md) explains how).

`tests/server/run.py` boots a real open.mp server with the component on a small
made-up map. It checks that an NPC walks round a wall and that another drives a
route. CI runs every test on every push.

## Performance

Measured on a 4-core cloud VPS (Intel Xeon, 2.8 GHz) with all of San Andreas loaded, N NPCs working
for a minute each. A server tick at
open.mp's 200 ticks a second has 5 ms; DiamondNav's searches run on their own threads, and what it does
on the main thread is steering the NPCs.

| Example | Main-thread time | Slowest ticks |
|---|---|---|
| 50 NPCs, mostly on foot | 2% of a core | under 1 ms |
| 100 cars driving | 7% of a core | about 4 ms |
| 500 NPCs only walking | 1.3% of a core | under 0.2 ms |
| 500 cars driving | 43% of a core | 20-38 ms (a stutter) |

![DiamondNav's main-thread time by NPC count](docs/bench/cpu.svg)

![Slowest ticks by NPC count](docs/bench/ticks.svg)

Memory depends on what you load, not on how many NPCs there are: about 360 MB with everything,
flat over time.

![Server memory as each data file is loaded](docs/bench/memory.svg)

![How long one path search takes](docs/bench/search.svg)

## Limitations

- **No physics:** there is no physics on the server. DiamondNav drives the car as a
  driver would (speed, turning, braking), but it does not crash, skid or get pushed.
  Drivers don't stop for traffic or players yet; that's up to your script.
- **Ledges and drops are incomplete:** walkers and chasers take some ledges and
  drops, but not all of them yet.
- **Ramp chasing:** chasing a player up or down a ramp is not finished.
- **No boats:** NPCs walk, drive cars and fly helicopters and planes. Boats are not
  driven yet.
- **Planes on the ground take no physics either:** a plane keeps to its taxi path; a
  player's car parked on a taxiway stops it rather than being pushed aside.

## Good to know

- How GTA's path files mark lane direction and median width is read from the files
  as documented by the community, and has not been checked against every road.
  If a car drives on the wrong side somewhere, please open an issue with the place.
- Nothing from GTA is in this repository or the releases: you bake from your own
  copy, and the navmesh holds only walkable floor polygons.

## Credits

Built on [Recast & Detour](https://github.com/recastnavigation/recastnavigation)
(zlib license) and the [open.mp](https://github.com/openmultiplayer/open.mp) SDK
(MPL-2.0). The ColAndreas database reader was written from the file format; no
ColAndreas code is used. See [THIRD_PARTY.md](THIRD_PARTY.md).
