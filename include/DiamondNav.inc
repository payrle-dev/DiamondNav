// DiamondNav: walking paths and driving routes for open.mp NPCs.
// https://github.com/payrle-dev/DiamondNav  (MPL-2.0)
//
// The DiamondNav component (components/DiamondNav.so or .dll) loads, from the
// server's navmesh/ folder:
//   *.nav       where peds can walk, and where a car fits off the road (navbake
//               --agent ped or car; dnav.py world bakes both)
//   roads.dnr   where cars can drive (built with tools/dnav.py roads)
//   vehicles.dnv how each vehicle model drives (tools/dnav.py vehicles, from the
//               game's handling.cfg; without it every car drives alike)
//   airports/*.txt runways, taxiways, gates and flight routes for planes, on top of
//               the built-in San Andreas airports (LSIA, SFIA, LVA, VM)
// and searches paths on its own threads, so a search never holds up the server.
//
// Two ways to use it:
//   - Let it move the NPC: Nav_WalkTo / Nav_DriveTo / Nav_FlyTo / Nav_Chase, then
//     OnNavMoveEnd says how it ended. It follows the path point by point with
//     open.mp's NPC_Move (a helicopter flies on its own, with no path).
//   - Ask for the points and move the NPC yourself: Nav_RequestPath /
//     Nav_RequestRoute, then OnNavPathResult hands over the points.
//
// Coordinates are GTA's. Path points are on the ground (a ped's feet, the road
// surface); Nav_WalkTo and Nav_DriveTo take care of the height of a ped or a car.

#if defined _DiamondNav_included
    #endinput
#endif
#define _DiamondNav_included

#define DIAMONDNAV_INCLUDE_VERSION      (10400)     // 1.4.0

// ── Results ──────────────────────────────────────────────────────────────────

// OnNavPathResult's status.
#define NAV_OK                  (0)     // the whole way
#define NAV_PARTIAL             (1)     // walking: as near to the end as the navmesh gets
#define NAV_NO_START            (-1)    // nothing walkable (or no road) near the start
#define NAV_NO_END              (-2)    // nothing walkable (or no road) near the end
#define NAV_NO_PATH             (-3)    // start and end are not connected
#define NAV_NOT_LOADED          (-4)    // no navmesh (or no road graph) was loaded

// OnNavMoveEnd's reason.
#define NAV_END_ARRIVED         (0)     // it got there
#define NAV_END_NO_PATH         (1)     // there was no way there
#define NAV_END_STOPPED         (2)     // Nav_Stop, or a new Nav_WalkTo / Nav_DriveTo
#define NAV_END_INTERRUPTED     (3)     // something else moved it, it left its car (or the
                                        // driver's seat), died or was destroyed
#define NAV_END_LOST            (4)     // Nav_Chase: the target left, or went to another world

// Road flags for Nav_RequestRoute, Nav_DriveTo and Nav_GetNearestRoadPoint.
#define NAV_ROAD_EMERGENCY      (1)     // may use roads only emergency vehicles drive on
#define NAV_ROAD_BOATS          (2)     // a boat: water routes only
// Nav_DriveTo only: across the car navmesh (anywhere a car fits) instead of the roads.
#define NAV_DRIVE_OFFROAD       (4)

// Nav_Chase's flags.
#define NAV_CHASE_RAM           (1)     // drive into the target: no slowing to its speed, no spreading out
#define NAV_CHASE_NO_DROP       (2)     // never off a bridge or ledge after a target below: the roads round

// What a chaser is doing (Nav_GetChaseInfo).
#define NAV_CHASE_BESIDE        (0)     // stopped beside a target that stands still
#define NAV_CHASE_STRAIGHT      (1)     // close, straight at where the target is going
#define NAV_CHASE_ROUTE         (2)     // along a road route (or across the car navmesh)
#define NAV_CHASE_PLANNING      (3)     // waiting on its first route
#define NAV_CHASE_EDGE          (4)     // the target is below: driving at the edge to go off it
#define NAV_CHASE_AIRBORNE      (5)     // off the edge, falling
#define NAV_CHASE_OVERHEAD      (6)     // a helicopter, circling over the target

// Nav_FlyTo's flags.
#define NAV_FLY_LAND            (1)     // set down on the ground (or a roof) there, instead of hovering

// What a plane is doing (Nav_GetPlanePhase, OnNavPlanePhase).
#define NAV_PLANE_NONE          (0)
#define NAV_PLANE_PARKED        (1)     // standing on the ground, nothing to do
#define NAV_PLANE_PUSHBACK      (2)     // backed off its gate by a tug
#define NAV_PLANE_TAXI_OUT      (3)     // to the runway
#define NAV_PLANE_HOLD_SHORT    (4)     // waiting to line up (the runway is busy)
#define NAV_PLANE_LINE_UP       (5)     // onto the runway
#define NAV_PLANE_TAKEOFF       (6)     // the take-off roll
#define NAV_PLANE_CLIMB         (7)     // climbing away
#define NAV_PLANE_CRUISE        (8)     // flying a route, or to an airport
#define NAV_PLANE_HOLDING       (9)     // circling until the runway it lands on will be free
#define NAV_PLANE_APPROACH      (10)    // the pattern down to the final
#define NAV_PLANE_FINAL         (11)    // lined up, on the glide path
#define NAV_PLANE_GO_AROUND     (12)    // climbing away from a missed landing
#define NAV_PLANE_LANDING       (13)    // the landing roll, to a turn-off
#define NAV_PLANE_TAXI_IN       (14)    // to a gate (or a taxi node)
#define NAV_PLANE_AT_GATE       (15)    // parked at a gate
#define NAV_PLANE_CHASE         (16)    // after someone (Nav_Chase)
#define NAV_PLANE_LOITER        (17)    // circling, nothing left to do in the air

// Nav_Land's gate for "no gate": it stops on a taxiway once off the runway.
#define NAV_NO_GATE             "-"

// Which navmesh: where peds walk, or where a car fits.
#define NAV_MESH_PED            (0)
#define NAV_MESH_CAR            (1)

// Road classes, each with its own speed limit (Nav_SetRoadSpeed). Read from GTA's own
// path data: its highway flag, the lanes each way and its traffic level.
#define NAV_CLASS_HIGHWAY       (0)     // freeways (default limit 130 km/h)
#define NAV_CLASS_MAIN          (1)     // two or more lanes each way, or one-way (80)
#define NAV_CLASS_STREET        (2)     // one lane each way (60)
#define NAV_CLASS_MINOR         (3)     // GTA's low-traffic roads: back streets, country lanes (45)
#define NAV_CLASS_SERVICE       (4)     // emergency-only roads: aprons, yards, car parks (30)
#define NAV_CLASS_OFFROAD       (5)     // off the roads, on the car navmesh (60)

// How a driver drives (Nav_SetDriveStyle).
#define NAV_STYLE_CAUTIOUS      (0)     // under the limits, gentle corners, early braking
#define NAV_STYLE_NORMAL        (1)     // the limits, comfortable corners (Nav_DriveTo's default)
#define NAV_STYLE_FAST          (2)     // a quarter over the limits, brisk corners, late braking
#define NAV_STYLE_PURSUIT       (3)     // no limits: flat out, cornering and braking on the limit of the car
                                        // (Nav_Chase's default)
#define NAV_STYLE_DEFAULT       (-1)    // Nav_SetDriveStyle: back to the default

// Nav_GetVehicleInfo: what to read about a model (vehicles.dnv, from handling.cfg).
#define NAV_VINFO_TOP_KMH       (0)     // top speed on the flat
#define NAV_VINFO_ACCEL         (1)     // pulling away, m/s^2
#define NAV_VINFO_BRAKE         (2)     // full brakes on a road, m/s^2
#define NAV_VINFO_BRAKE_OFFROAD (3)     // ... on dirt
#define NAV_VINFO_COAST         (4)     // rolling with no throttle, m/s^2
#define NAV_VINFO_DRAG          (5)     // the air's deceleration per (m/s)^2
#define NAV_VINFO_GRIP          (6)     // sideways in a corner on a road, m/s^2
#define NAV_VINFO_GRIP_OFFROAD  (7)     // ... on dirt
#define NAV_VINFO_RADIUS        (8)     // the tightest circle its centre turns, m
#define NAV_VINFO_YAW           (9)     // how fast it can start turning, rad/s^2
#define NAV_VINFO_REVERSE_KMH   (10)    // top speed backing up
#define NAV_VINFO_LENGTH        (11)    // m, from its collision
#define NAV_VINFO_WIDTH         (12)
#define NAV_VINFO_HEIGHT        (13)
#define NAV_VINFO_WHEELBASE     (14)    // m, from where its wheels are
#define NAV_VINFO_ZERO_100      (15)    // seconds from 0 to 100 km/h (0: it never gets there)
#define NAV_VINFO_RIDE          (16)    // m its centre sits over the road (what carZ 0.0 uses)

// ── Information ──────────────────────────────────────────────────────────────

// Navmesh tiles loaded (25.6 m squares walking, 64 m for cars); 0 without that navmesh.
native Nav_GetTileCount(kind = NAV_MESH_PED);

// Road nodes loaded; 0 without a road graph.
native Nav_GetRoadNodeCount();

// Faces of the world's collision (world.col) loaded; 0 without it.
native Nav_GetCollisionFaceCount();

// Searches queued or running.
native Nav_GetPendingCount();

// ── Moving NPCs ──────────────────────────────────────────────────────────────

// Walks (or runs, or sprints) an NPC to a point along the navmesh. moveType is
// open.mp's NPC_MOVE_TYPE_WALK, _JOG or _SPRINT; speed as NPC_Move takes it.
// Returns false when the NPC does not exist or the search queue is full. Called again
// while it walks (following someone), it walks on until the new way is found.
native bool:Nav_WalkTo(npcid, Float:x, Float:y, Float:z, moveType = 1, Float:speed = -1.0);

// Drives an NPC's car to a point along the roads, in its lane. The NPC must be the
// driver. It drives like its car (vehicles.dnv) in its style (Nav_SetDriveStyle):
// up to each road's speed limit, slowing for corners and junctions in time, and
// braking to a stop at the end. speed, if above 0, caps it in NPC_MOVE_TYPE_DRIVE's
// units (1.0 is 10 m/s); carZ is how far the car's centre sits above the road, and
// 0.0 takes the model's own (vehicles.dnv's, from its wheels and springs, else a
// guess from open.mp's model table; NAV_VINFO_RIDE). Like every DiamondNav driver,
// it swerves round or stops for vehicles in its way (parked, players', other NPCs'),
// by each model's collision box.
native bool:Nav_DriveTo(npcid, Float:x, Float:y, Float:z, Float:speed = 0.0, Float:carZ = 0.0, flags = 0);

// Chases a player (or another NPC) with an NPC's car until Nav_Stop. Far off, it
// follows the roads while the target is on them, and the car navmesh once the target
// leaves them; close and with nothing in the way, it drives straight at where the
// target is going (as far ahead as it takes the car to get there), keeps up with a
// moving target and stops beside a still one. A target that drops off a bridge or a
// ledge (up to 30 m below, within 60 m) is followed off the edge: the car drives to
// it on its own level and falls, unless NAV_CHASE_NO_DROP. Several NPCs chasing one
// target come up either side of it instead of in a line; NAV_CHASE_RAM drives
// straight into it. It re-plans as the target moves, without pausing. The NPC must
// be the driver. Ends with NAV_END_LOST when the target leaves or changes world.
// An NPC flying a helicopter keeps over the target instead (NAV_CHASE_OVERHEAD;
// carZ and flags do not apply): 15 m behind and 28 m up while it moves, circling
// over it when it stops, always looking at it. It stays below 45 m over the target,
// flying round anything taller (down the streets between towers), and climbs over
// only what it cannot get round.
// What the NPCs do in the chase (shoot, give up) is the script's: Nav_GetChaseInfo
// says how far the target is. It drives in NAV_STYLE_PURSUIT unless the NPC has a
// style of its own; speed, if above 0, caps it (as Nav_DriveTo's).
native bool:Nav_Chase(npcid, targetid, Float:speed = 0.0, Float:carZ = 0.0, flags = 0);

// A chaser: how far its target is (m) and what it is doing (NAV_CHASE_BESIDE ..
// NAV_CHASE_OVERHEAD). False when the NPC is not chasing.
native bool:Nav_GetChaseInfo(npcid, &Float:distance, &mode);

// Flies an NPC's helicopter to a point: up from wherever it is, straight there high
// enough to clear everything under and ahead of it (20 m over the tops of buildings,
// trees and hills, and over the sea; world.col says where they are, and without it
// it only keeps to z), slowing to stop there and hovering at z, or, with
// NAV_FLY_LAND, setting down on the ground or roof at (x, y). It leans as it flies:
// nose down to speed up, nose up to slow, banked into turns. The NPC must be the
// pilot of a helicopter (not an RC one). speed, if above 0, caps it in
// NPC_MOVE_TYPE_DRIVE's units (1.0 is 10 m/s; a Maverick tops out at 5.0). Called
// again while it flies, it turns for the new point without stopping. OnNavMoveEnd
// says NAV_END_ARRIVED once it hovers there or is down; stopped, it hangs where it is.
native bool:Nav_FlyTo(npcid, Float:x, Float:y, Float:z, Float:speed = 0.0, flags = 0);

// Stops a Nav_WalkTo, Nav_DriveTo, Nav_FlyTo or Nav_Chase where the NPC is
// (OnNavMoveEnd: NAV_END_STOPPED). A plane circles where it is (or stops on the ground).
native bool:Nav_Stop(npcid);

// Whether a Nav_WalkTo, Nav_DriveTo, Nav_FlyTo or Nav_Chase is under way (or searching).
// A plane is moving unless it is parked.
native bool:Nav_IsMoving(npcid);

// ── Planes ───────────────────────────────────────────────────────────────────
//
// An NPC piloting a plane (AT-400, Andromada, Nevada, Shamal, Hydra, Rustler,
// Beagle, Cropduster, Stuntplane, Dodo, Skimmer) flies it from the gate to the
// sky and back: pushed back off its gate, along the taxiways to the runway, up,
// along a route, down a proper approach (a pattern onto a straight final and a
// glide path), the landing roll, off at the first turn-off it can make and on to a
// gate. Each plane's size and speeds are its own (the AT-400 needs a runway and
// taxiways wide enough for its 62 m wings, and gates that take them).
//
// Air traffic control keeps them apart: one plane on a runway at a time (landing,
// taking off or crossing; runways that cross count as one), departures wait for
// arrivals, an arrival holds when its runway will be busy and goes around when it
// is not clear by the decision point (or it is not lined up). On the ground every
// plane stops short of every other (by their wings and fuselages, and any vehicle
// standing on a taxiway: a car on a runway closes it), and waits before a stretch of
// taxiway another plane is about to taxi the other way. In the air they keep clear
// of each other and of the ground (world.col), climbing over or turning away from
// what is ahead.
//
// Airports come from the built-in San Andreas file and navmesh/airports/*.txt (a
// script can load more with Nav_LoadAirports): runways, taxiway nodes and edges,
// gates, approach and departure fixes and flight routes. docs/AIRPORTS.md in the
// repository says how they are written. Names here (airports, gates, runways,
// routes) are those files'.
//
// Put the NPC in the plane as its driver first (NPC_PutInVehicle). A plane standing
// on a gate is parked at it. Orders replace each other (the one under way ends with
// NAV_END_STOPPED); OnNavMoveEnd says how each ended, OnNavPlanePhase what it is
// doing on the way. Nav_Stop circles it where it is in the air, or stops it on the
// ground. Nav_FlyTo (fly to a point, then circle it) and Nav_Chase (after a player,
// carZ is the height over them, 40 m with 0.0) work for planes too.

// Puts the NPC's plane at a gate, parked (NAV_END_ARRIVED once it is).
native bool:Nav_ParkPlane(npcid, const airport[], const gate[]);

// Taxis it to a gate or a taxi node at the airport it is on.
native bool:Nav_TaxiTo(npcid, const where[]);

// Out to a runway ("" picks one: long and wide enough for it, the airport's
// preferred way, towards where it is going) and up, then along a route ("": none),
// then circling. NAV_END_ARRIVED once it has climbed away (or flown the route).
native bool:Nav_Takeoff(npcid, const runway[] = "", const route[] = "");

// Lands at an airport and taxis to a gate: "" any free one it fits, NAV_NO_GATE
// none (it stops once off the runway). From the ground at another airport it takes
// off first. runway ("" picks one) and a route to fly first ("" straight there) are
// up to you. NAV_END_ARRIVED once at the gate; NAV_END_NO_PATH when it cannot land
// there (no open runway long and wide enough) or cannot get to a gate.
native bool:Nav_Land(npcid, const airport[], const gate[] = "", const runway[] = "", const via[] = "");

// Flies a route (in the air), then circles its last point; loop: round and round.
// speed, if above 0, in NPC_MOVE_TYPE_DRIVE's units (1.0 is 10 m/s).
native bool:Nav_FlyRoute(npcid, const route[], Float:speed = 0.0, bool:loop = false);

// What a plane is doing (NAV_PLANE_*), -1 if DiamondNav does not fly it.
native Nav_GetPlanePhase(npcid);
// ...and more: its speed, how far it still flies to touch down (approach and final,
// else -1), the airport (an index) it is at or landing at and its gate there (-1).
native bool:Nav_GetPlaneInfo(npcid, &phase, &Float:speedKmh, &Float:toTouchdown, &airport, &gate);
native bool:Nav_GetPlanePhaseName(phase, name[], len = sizeof name);

// Airports: how many, by code ("LSIA", case does not matter), at a point (-1: none).
native Nav_GetAirportCount();
native Nav_FindAirport(const code[]);
native Nav_GetAirportAt(Float:x, Float:y, Float:z);
native bool:Nav_GetAirportCode(index, code[], len = sizeof code);

// A gate: where a plane parked there stands (the ground under its centre) and faces.
native bool:Nav_GetGateSpot(const airport[], const gate[], &Float:x, &Float:y, &Float:z, &Float:heading);
native bool:Nav_IsGateFree(const airport[], const gate[]);
// The smallest free gate a model fits.
native bool:Nav_FindFreeGate(const airport[], modelid, gate[], len = sizeof gate);

// A runway end ("27"): its landing threshold and the heading planes land at there.
native bool:Nav_GetRunwayEnd(const airport[], const end[], &Float:x, &Float:y, &Float:z, &Float:heading);
// Closes a runway ("09/27", or either end) to planes, or opens it again.
native bool:Nav_SetRunwayOpen(const airport[], const runway[], bool:open);
// The NPC landing on, taking off from or crossing a runway, or -1.
native Nav_GetRunwayUser(const airport[], const runway[]);

// Reads another airports file (on top of what is loaded; nothing changes if a line
// is wrong, and the log says which).
native bool:Nav_LoadAirports(const path[]);
// Builds a flight route point by point (Nav_FlyRoute, Nav_Takeoff, Nav_Land use it).
native bool:Nav_AddRoutePoint(const route[], Float:x, Float:y, Float:z);
native bool:Nav_ClearRoute(const route[]);

// ── How cars drive ───────────────────────────────────────────────────────────

// An NPC's driving style (NAV_STYLE_*), kept until the NPC is destroyed; it applies
// at once to a drive or chase under way. NAV_STYLE_DEFAULT clears it, and
// Nav_GetDriveStyle is NAV_STYLE_DEFAULT for an NPC that has none (Nav_DriveTo then
// drives normally, Nav_Chase in pursuit).
native bool:Nav_SetDriveStyle(npcid, style);
native Nav_GetDriveStyle(npcid);

// A road class's speed limit (NAV_CLASS_*), in km/h, for every driver.
native bool:Nav_SetRoadSpeed(roadClass, Float:kmh);
native Float:Nav_GetRoadSpeed(roadClass);

// How a vehicle model drives: top speed (km/h), acceleration and hardest braking
// (m/s^2), the most sideways acceleration it holds in a corner (m/s^2) and its
// tightest turning circle (m). Setting it overrides vehicles.dnv for that model with
// these five numbers (its acceleration then fades towards its top speed; dirt takes
// 30% of its grip and brakes). Without either, a model drives as an average car.
native bool:Nav_SetVehicleHandling(modelid, Float:topKmh, Float:accel, Float:brake, Float:grip, Float:radius);
native bool:Nav_GetVehicleHandling(modelid, &Float:topKmh, &Float:accel, &Float:brake, &Float:grip, &Float:radius);

// Everything else vehicles.dnv knows about a model, one number at a time (NAV_VINFO_*).
native Float:Nav_GetVehicleInfo(modelid, info);

// A model's full-throttle acceleration (m/s^2) at a speed, on a road or on dirt:
// its gears, engine, the air and what its tyres put down.
native Float:Nav_GetVehicleAccel(modelid, Float:speedKmh, bool:offRoad = false);

// Seconds a model takes from one speed to another at full throttle on a road (0 when
// it never gets there).
native Float:Nav_GetVehicleAccelTime(modelid, Float:fromKmh, Float:toKmh);

// Vehicle models vehicles.dnv gave handling for; 0 without it.
native Nav_GetVehicleModelCount();

// A driver under way: its speed, the speed it is going for (the road, the next
// corner, the end), the class of road it is on and that road's limit. False when the
// NPC is not driving with DiamondNav. For a helicopter: its speed, the speed it is
// going for and its top speed as the limit.
native bool:Nav_GetDriveInfo(npcid, &Float:speedKmh, &Float:wantKmh, &roadClass, &Float:limitKmh, &bool:reversing);

// ── Searching ────────────────────────────────────────────────────────────────

// Queues a walking path search. Returns the request id OnNavPathResult will carry,
// or 0 when the queue is full.
native Nav_RequestPath(Float:fromX, Float:fromY, Float:fromZ, Float:toX, Float:toY, Float:toZ);

// Queues a driving route search on the road graph (flags: NAV_ROAD_*).
native Nav_RequestRoute(Float:fromX, Float:fromY, Float:fromZ, Float:toX, Float:toY, Float:toZ, flags = 0);

// Point `index` (0 .. points - 1) of the result being handed to OnNavPathResult.
// Only valid during that callback.
native bool:Nav_GetPathPoint(requestid, index, &Float:x, &Float:y, &Float:z);

// A route point's speed, 0.0 .. 1.0 of cruising speed (lower in corners and at
// the end). Walking paths are always 1.0.
native Float:Nav_GetPathPointSpeed(requestid, index);

// A route point's road class (NAV_CLASS_*); -1 for a walking path.
native Nav_GetPathPointClass(requestid, index);

// ── Points ───────────────────────────────────────────────────────────────────

// The nearest point a ped can walk on (or a car fit), within `reach` metres.
native bool:Nav_GetNearestPoint(Float:x, Float:y, Float:z, &Float:outX, &Float:outY, &Float:outZ, Float:reach = 3.0,
    kind = NAV_MESH_PED);

// A random point a ped can walk on (or a car fit), anywhere on that navmesh.
native bool:Nav_GetRandomPoint(&Float:x, &Float:y, &Float:z, kind = NAV_MESH_PED);

// Whether a ped (or a car) can go in a straight line from one point to the other:
// no wall, drop or obstacle in between. Points are on the ground. Use it to steer,
// or as a cheap check before an NPC shoots; it follows the ground, so it does not
// see over fences or down from a roof (an eye's line of sight is
// Nav_HasLineOfSight). False when either point is off the navmesh.
native bool:Nav_IsLineClear(Float:x1, Float:y1, Float:z1, Float:x2, Float:y2, Float:z2, kind = NAV_MESH_PED);

// Whether nothing solid in the world's collision (world.col: buildings, walls, fences,
// the ground) stands between two points: an eye's line of sight, through the air, unlike
// Nav_IsLineClear. Give the points at eye or chest height (a ped's position is about a
// metre above its feet). NAV_LOS_UNKNOWN without world.col: the script falls back on
// Nav_IsLineClear or distance. Small props cars smash through (lamp posts, bins) and
// vehicles are not in world.col, so they never block it.
#define NAV_LOS_UNKNOWN         (-1)
#define NAV_LOS_BLOCKED         (0)
#define NAV_LOS_CLEAR           (1)
native Nav_HasLineOfSight(Float:x1, Float:y1, Float:z1, Float:x2, Float:y2, Float:z2);

// The nearest road node, within `reach` metres (flags: NAV_ROAD_*).
native bool:Nav_GetNearestRoadPoint(Float:x, Float:y, Float:z, &Float:outX, &Float:outY, &Float:outZ,
    Float:reach = 50.0, flags = 0);

// ── Callbacks ────────────────────────────────────────────────────────────────

// A search finished. Read the points with Nav_GetPathPoint (and
// Nav_GetPathPointSpeed) here; they are gone after the callback returns.
forward OnNavPathResult(requestid, status, points);

// A Nav_WalkTo, Nav_DriveTo, Nav_FlyTo or Nav_Chase ended (reason: NAV_END_*), or a
// plane's order (Nav_Takeoff, Nav_Land, Nav_TaxiTo, Nav_FlyRoute, Nav_ParkPlane).
forward OnNavMoveEnd(npcid, reason);

// A plane went into a new phase (NAV_PLANE_*): touchdown is NAV_PLANE_LANDING,
// lift-off NAV_PLANE_CLIMB.
forward OnNavPlanePhase(npcid, phase);
