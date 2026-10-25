# DiamondNav roadmap

DiamondNav gives NPCs the ability to move well. It never decides which NPCs exist:
spawning traffic, pedestrians or gangs is the gamemode's job. Each item below is a
native (or a few) that a script calls on an NPC it already made.

## 1.0 (done)

- `Nav_WalkTo`: walking paths on a navmesh (a map, or the whole world from GTA).
- `Nav_DriveTo`: road routes on GTA's traffic paths (`roads.dnr`), in the right-hand
  lane, slowing for corners.
- `Nav_RequestPath` / `Nav_RequestRoute`: the same searches, with the points handed
  to the script.

## 1.1: chasing (done)

- **`Nav_Chase(npcid, targetid, speed, flags)`**: a driver follows a player (or
  another NPC) until `Nav_Stop`. The component re-plans as the target moves, and
  keeps driving the old plan until the new one is ready, so the car never pauses.
  - Far away, with the target near a road: a road route, as `Nav_DriveTo`.
  - The target off the roads (a field, a car park, a beach): a path on the **car
    navmesh**, so the car leaves the road where it has to.
  - Close and with a clear straight line: it drives straight at where the target
    will be in half a second.
  - It ends with `OnNavMoveEnd(NAV_END_LOST)` if the target leaves or changes world.
- **Car navmesh**: a second navmesh baked for a car (2 m tall, 1.4 m round, climbs
  0.6 m, slopes up to 35°), from the same world geometry as the walking one.
  `dnav.py world` bakes both; `.nav` files say which kind they are.
- **`Nav_IsLineClear(x1, y1, z1, x2, y2, z2, kind)`**: whether a ped (or a car) can go
  straight from one point to the other, with no wall, drop or obstacle in between.
  Scripts use it to steer, or as a cheap check before shooting. (A true eye-level
  line of sight through windows and fences needs the full collision mesh; that stays
  ColAndreas' job, which a gamemode can use alongside DiamondNav.)

## 1.1.1-1.1.2 (done)

- Drivers steer like cars: a lookahead point, a turning circle, smooth speed, the
  car navmesh's ground off the road, and backing up to turn round.

## 1.2: driving like a car (done)

- Road classes from GTA's own path data (highway, main, street, minor, service), each
  with a speed limit (`Nav_SetRoadSpeed`).
- Each vehicle's handling from the game's `handling.cfg` (`dnav.py vehicles`):
  top speed, acceleration, braking, grip, turning circle (`Nav_SetVehicleHandling`).
- A speed plan along every path: limits, corners by the car's grip, junctions, and
  braking in time for each.
- Driving styles (`Nav_SetDriveStyle`): cautious, normal, fast, pursuit.
- `Nav_GetDriveInfo`, `Nav_GetPathPointClass`.

## 1.3: the game's own handling (done)

- `dnav.py vehicles` runs GTA's vehicle physics offline on every handling.cfg
  column: the gearbox, engine inertia, drag, tyres and wheelspin, brakes, steering,
  turn mass, off-road ability, bike lean; surface.dat for road and dirt; each
  model's DFF for its wheelbase, track and size. vehicles.dnv version 2 carries an
  acceleration table per model (road and dirt) and the rest.
- Drivers pull away by that table, brake and corner on dirt by the dirt numbers, and
  ease into turns by the turn mass.
- `Nav_GetVehicleInfo`, `Nav_GetVehicleAccel`, `Nav_GetVehicleAccelTime`.
- 1.3.1: built for open.mp 1.6.7 (its SDK, and the NPC patch rebased onto it; open.mp
  now sends a driving NPC's speed in the game's units and pitches the car itself).
- 1.3.2: each car sits at its own height over the road (vehicles.dnv version 3, from
  its wheels and springs; `carZ` 0.0 by default), and cars off a road route keep to
  the ground (car mesh, walking mesh or the road surface) instead of floating.
- 1.3.3: chases. A chaser follows its target off a bridge or a ledge (falling to the
  road below), keeps up alongside a moving target instead of stopping, leads it by
  the time it takes to get there, drives straight along a road up to 150 m without
  a car navmesh, and spreads out with other chasers; `NAV_CHASE_RAM`,
  `NAV_CHASE_NO_DROP`, `Nav_GetChaseInfo`; pursuit brakes flat out. Every driver
  swerves round or stops for vehicles in its way by their collision boxes
  (vehicles.dnv version 4 adds where each box sits). Road routes start and end
  along long stretches of road, not only near a node.
- 1.3.4: chases know the map. `navmesh/world.col` (the bake's world.tris) is loaded
  for rays: chasers check walls and the ground before a straight run, swerve round
  walls and objects, find a railing's gap and line up square to it, and fall without
  passing through anything. Breakable stock objects (object.dat: fences, railings,
  lamp posts, bins) are left out of the car bake and world.col, so cars smash
  through them as in the game. Pursuit routes start wherever the car can join the road
  (turning round if that is shorter) and may go against one-way traffic, and the
  chaser aims at the target's smoothed heading so it stops braking for every swerve.
- 1.3.5: chases down banks and ledges (from a test in the Los Santos storm drain): with
  world.col a chaser drives straight at a target below it, down any slope or off a
  ledge (climbing no more than 27 degrees), ground counts up to 63 degrees, a chaser
  that goes over an edge falls instead of floating on, and a chaser far behind a
  moving target gets up to 35% more speed and acceleration to catch up.
- 1.3.6: chases on a real map (from a test on a full Los Santos bake). With the car navmesh and world.col
  the server finds every ledge a car can drive off at boot (about 50,000 across San
  Andreas, in about 2 s) and chase routes use them, so a chaser takes the nearest way
  down to a target in a storm drain. Cars no longer float after driving off a bridge,
  turn round in narrow streets with a three-point turn, back out of a post the other
  way when wedged twice, stop swerving off their road at speed for a building past
  the next corner, and turn round when the target goes behind them instead of driving
  on. `navquery path ... --car --ledges` shows a chase's route.
- 1.3.7: chases that keep going (from a 12-scenario bench on the Los Santos bake, with a
  recorded getaway car so every build chases the same target). A moving car weighs a
  U-turn's real cost (stopping, turning, getting back up to speed) against carrying on,
  swings round forward where there is room, keeps its speed when it is given a new
  destination or starts a chase, and a replan no longer doubles back round a post it
  just passed. Road routes no longer start on a road the car cannot reach (below a
  terrace or behind a wall): the way onto the first road crosses the car mesh and is
  checked against world.col. Three-point turns use the whole street, a car scraping a
  kerb or post can still steer away from it, a car boxed in by a post far ahead takes
  the course that gets furthest, a car braking from speed to turn round still avoids
  the wall it slides at, and cars no longer rock forward and back at a pillar. Slopes:
  the car's height follows the ground just ahead (no more sinking into hills or
  floating over dips), steps and steep ramps no longer read as walls, and a car past an
  edge falls instead of driving on level. `navquery route ... --heading h --speed m/s`
  shows a moving car's route; `navquery road x y` the nearest road node.
- 1.3.8: chasers that don't brake at every corner or end up in the sea. Steering at a
  near point round a corner of a car-mesh path no longer brakes the car to a crawl (the
  speed plan already slows it for the corner), a chase across the car mesh is weighed
  against the roads (a metre off the road or up a steep grade costs more, so the shortcut
  across a hillside loses to the road round it), and the sea is a wall: below sea level
  where the car mesh has no ground, a car no longer drives off a quay, and a straight run
  never ends in the water. On the bench against 1.3.7: Ocean Docks 169 to 115 m behind
  with no car stuck or in the water, the freeway 173 to 138, downtown zig-zag 140 to 104,
  the beach 50 to 41.
- 1.3.9: a walker given a new place while on its way walks on while the new way is
  found, then turns onto it from where it has got to, instead of stopping dead each
  time (a follower sent after someone every half second used to stutter). On a bench of
  a follower after a target changing pace round the Idlewood alleys: 8.9 m behind on
  average (was 10.4), at most 24 m (was 31).
- 1.3.10: helicopters. `Nav_FlyTo` flies an NPC pilot's helicopter to a point (hovering
  there, or landing with `NAV_FLY_LAND`), keeping 20 m over the tops of everything
  under and ahead of it in world.col; `Nav_Chase` in a helicopter keeps over the
  target (`NAV_CHASE_OVERHEAD`), as a police helicopter does, flying round towers
  taller than 45 m over the target instead of over them. Its own flight model
  (src/flying.cpp, tested in tests/test_flying.cpp): each model's speed, climb and turn
  rate, leaning into speed changes and banking into turns. Planes came in 1.4; boats are later.
- 1.3.11: escalators. `navmesh/links.txt` lists ways the walking mesh lacks (an
  escalator's steps are not in GTA's collision): each is walked both ways along its own
  points, and opens what only it reaches (the bake took the Los Santos bank's plaza for a
  roof). A walker or target partway along one goes along it. No rebake. Later:
  reading every escalator from GTA's own files (2dfx) at bake time.
- 1.3.12: chasers that don't overshoot your corners. A chaser goes no faster than lets
  it brake, in the gap it keeps, to the target's own speed along its way, so it no
  longer comes up behind at full speed; a target that turns off at a junction has none
  of its speed along the chaser's way, so the chaser brakes for the corner too, down to
  what its grip takes round it. A route is re-planned when where the target will be has
  moved off it (it was re-planned 4 times a second), and a car backing up to turn round
  goes on until the way forward is clear as far as its avoidance looks (it rocked forward
  and back at a building's corner for up to 20 s). On the bench against 1.3.11, 8 runs
  each: a getaway with hard 90-degree turns round the downtown grid 80 to 64 m behind on
  average, with half the reversing and stuck 1.9 to 0.3 s; downtown zig-zag 116 to 109,
  the hills 102 to 96, the freeway 138 to 145 (within its spread).

## 1.4: planes (done)

- NPC pilots fly planes gate to gate: `Nav_ParkPlane`, `Nav_TaxiTo`, `Nav_Takeoff`,
  `Nav_Land`, `Nav_FlyRoute`, and `Nav_FlyTo` / `Nav_Chase` for planes;
  `OnNavPlanePhase` for each phase of a flight. Pushback, taxiing on paths planned for
  the plane's turning circle and speed, line-up, the take-off roll and rotation, a
  climb-out, routes, a pattern onto a final sized to the plane, the glide path, flare,
  touchdown, braking to the first exit it can make and taxiing to a gate. Each model's
  size and speeds are its own (src/aircraft.cpp), scaled to San Andreas' runways.
- Airports as text files (docs/AIRPORTS.md): runways, taxiways, gates, approach and
  departure fixes, routes. San Andreas' four airports are built in, measured from the
  game's collision; `navmesh/airports/*.txt` add airports or reshape the stock ones.
  Planes only use the runways, taxiways and gates their wings fit.
- Air traffic control (src/atc.cpp): one plane on a runway at a time, holds and
  go-arounds, departures waiting for arrivals, planes stopping short of each other and
  of vehicles on the ground and giving way at a taxiway someone is coming along the
  other way; `Nav_SetRunwayOpen`, `Nav_GetRunwayUser`. In the air they keep clear of
  each other and of `world.col`.
- tests/test_aircraft.cpp flies them without a server: departures, arrivals, traffic,
  ground conflicts, head-on taxiing, terrain, a chase, and every stock gate both ways.

## 1.5: traffic

- **`Nav_Cruise(npcid, speed, flags)`**: a driver drives around the road network on
  its own, picking turns at random (or towards an area), in its lane.
  - Keeps a gap to the vehicle in front: other DiamondNav cars and players' cars
    (1.3.3's avoidance already swerves round or stops for them; traffic adds lanes).
  - Slows into junctions and gives way to a car already in the junction.
  - Ends only on `Nav_Stop`, or when the NPC leaves the car.
- `Nav_DriveTo` gets the same spacing and junction behaviour.

## 1.6: pedestrians

- **`Nav_Wander(npcid, radius, flags)`**: a ped walks to random points around it,
  preferring sidewalks; the walking navmesh marks road surface as costly (from
  `roads.dnr` at bake time), so peds cross roads instead of walking along them.
- Peds walking with DiamondNav keep apart from each other instead of walking
  through each other.
- `Nav_Flee(npcid, x, y, z)`: run away from a point (a shooting, a crash).
