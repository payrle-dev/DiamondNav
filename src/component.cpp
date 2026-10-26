// DiamondNav: the open.mp component (README.md).
//
// Loads navmesh/*.nav (where peds walk, and where cars fit off the road),
// navmesh/roads.dnr (where cars drive) and navmesh/world.col (what is solid) at start, searches paths on worker threads,
// and either hands the points to the script that asked (Nav_RequestPath /
// Nav_RequestRoute -> OnNavPathResult) or moves an NPC along them itself
// (Nav_WalkTo / Nav_DriveTo / Nav_Chase -> OnNavMoveEnd). Every callback runs on
// the game thread.

#include <sdk.hpp>

#include <Server/Components/NPCs/npcs.hpp>
#include <Server/Components/Pawn/pawn.hpp>
#include <Server/Components/Pawn/Impl/pawn_natives.hpp>
#include <Server/Components/Pawn/Impl/pawn_impl.hpp>
#include <Server/Components/Vehicles/vehicles.hpp>
#include <Server/Components/Vehicles/vehicle_models.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <thread>
#include <unordered_map>

#include "collision.hpp"
#include "driving.hpp"
#include "airports.hpp"
#include "atc.hpp"
#include "flying.hpp"
#include "pathfinder.hpp"
#include "pilot.hpp"

#include <filesystem>

#ifndef DNAV_VERSION_MAJOR
    #define DNAV_VERSION_MAJOR 1
    #define DNAV_VERSION_MINOR 0
    #define DNAV_VERSION_PATCH 0
#endif

namespace {

constexpr char NAV_FOLDER[] = "navmesh";
constexpr size_t MAX_RESULTS_PER_TICK = 256;    // the rest wait for the next tick
constexpr float FOOT_Z = 1.0f;                  // a ped's position stands this far over the ground
constexpr float WALK_STOP = 0.4f;               // how near a walking point counts as reached
constexpr float DRIVE_STOP = 1.5f;
constexpr float DRIVE_MIN_SPEED = 0.25f;        // NPC_MOVE_TYPE_DRIVE speed floor in corners

// Driving: the component steers the car itself, like a driver, instead of sliding it
// from point to point. Each time it looks a little way down the path, turns toward
// that point no faster than the car could, speeds up or brakes as the car and the
// road allow (driving.hpp: handling, speed limits, styles, the speed plan), and hands
// NPC_Move a target just ahead, on the ground.
// NPC_Move restarts the NPC's clock, so a move sent between two NPC updates loses the
// time since the last one: drive() runs right after each NPC update instead (the
// NPC's position changed), or after DRIVE_IDLE when it stands still.
constexpr auto DRIVE_IDLE = std::chrono::milliseconds(120);
constexpr float SPEED_UNIT = 10.0f;     // m/s per NPC_MOVE_TYPE_DRIVE speed 1.0
constexpr float TURN_MIN_SPEED = 3.0f;  // never slower than this while it still has to turn
constexpr float LOOK_MIN = 4.0f;        // how far down the path it aims: 4 m at a crawl...
constexpr float LOOK_TIME = 0.6f;       // ...plus 0.6 s of travel
constexpr float LOOK_MAX = 24.0f;
constexpr float PASSED_CORNER = 3.0f;   // still this near the point it passed, it keeps that point's speed
constexpr float SCAN_EXTRA = 20.0f;     // looks for slower points this far past its braking distance
constexpr float AHEAD_TIME = 0.5f;      // NPC_Move's target: this far ahead of the car
constexpr float AHEAD_MIN = 3.0f;
constexpr float NEAR_TIME = 0.1f;       // its height follows the ground this far ahead...
constexpr float NEAR_MIN = 1.0f;        // ...and no nearer
constexpr float ARRIVE = 2.5f;          // a drive ends this near its last point
constexpr float GROUND_REACH = 3.0f;    // how far up or down the car mesh's ground is looked for
constexpr float ROAD_GROUND_REACH = 25.0f; // with no mesh there, the road surface this near stands for the ground
constexpr float RIDE_DEFAULT = 1.0f;    // a car's centre over the road when nothing says otherwise
constexpr float REVERSE_SPEED = 4.0f;   // m/s backing up (or less: a third of the car's top speed)
constexpr float REVERSE_BEHIND = 100.0f; // backs up when the point it aims at is further round than this...
constexpr float REVERSE_NEAR = 2.5f;    // ...and nearer than this many turning radii (a U-turn is two across)
constexpr float REVERSE_INSIDE = 60.0f; // or this far round and inside its turning circle
constexpr float REVERSE_DONE = 40.0f;   // drives forward again once it points this near the aim
constexpr float REVERSE_MAX = 12.0f;    // backs up at most this far at a time
constexpr float REVERSE_CHECK = 4.0f;   // the car mesh must be clear this far behind it
constexpr float SEG_LEVEL = 3.0f;       // m: a stretch of path this far above or below the car is on another level
constexpr float LEDGE_LANE = 2.5f;      // m: as near a ledge's run-up as counts as on it...
constexpr float LEDGE_FACING = 60.0f;   // ...facing within this of its way
constexpr float STUCK_SECS = 3.0f;      // s wanting to move without getting STUCK_MOVE anywhere: it backs out
constexpr float STUCK_MOVE = 1.0f;
constexpr float STUCK_SAME = 4.0f;      // m: stuck again this near where it last was, it tries the other side
constexpr float NUDGE_SECS = 1.5f;      // s it edges forward when there is no room behind either
constexpr float TURN_GAP = 0.35f;       // m: turning round, it changes direction this far short of a wall...
constexpr float TURN_CHECK = 0.5f;      // s: ...checked in two steps (WALL_STEP)
constexpr float TURN_END = 0.1f;        // m: ...give or take this
constexpr float TURN_LOOK = 4.0f;       // m: a leg looks this far along its arc for the wall it ends at
constexpr float TURN_CREEP = 1.0f;      // m/s: a leg's least speed, so it still gets there
constexpr float CIRCLING_TURN = 60.0f;  // turning this hard...
constexpr float CIRCLING_SECS = 5.0f;   // ...this long means it is going round in circles: back up
constexpr float LOOP_LEVEL = 1.0f;      // m: a forward U-turn needs ground within this of the car's level all round

// Nav_Chase.
constexpr auto CHASE_THINK = std::chrono::milliseconds(250);   // how often a chaser looks at its target
constexpr auto CHASE_REPLAN_AFTER = std::chrono::seconds(6);   // a plan this old is redone anyway
constexpr auto CHASE_NO_OFFROAD = std::chrono::seconds(4);     // after the car mesh found no way
constexpr float CHASE_CLOSE = 6.0f;         // this near, the car stops beside its target
constexpr float CHASE_DIRECT = 60.0f;       // within this, a clear straight line is driven straight
constexpr float CHASE_KEEP = 15.0f;         // ...and, once it is (or once it is going for an edge), this much further
constexpr float CHASE_BLIND = 25.0f;        // ...or, with no car mesh to check the line, within this
constexpr float CHASE_DECKS = 150.0f;       // ...or within this, where the line runs over road all the way
constexpr float CHASE_DECK_SLACK = 8.0f;    // ...but for this last bit (the target stands beside the road)
constexpr float CHASE_DECK_KEEP = 3.0f;     // m: already driving straight, the road may be this much wider
constexpr float CHASE_DECK_RISE = 1.0f;     // m a line over road may climb or fall per sample
constexpr float CHASE_DIRECT_DZ = 4.0f;     // no straight line up or down a level
constexpr float CHASE_REPLAN = 10.0f;       // re-plan once the target is this far from the plan's end
constexpr float CHASE_REPLAN_SHARE = 0.2f;  // ...or a fifth of the way to it, if more
constexpr float CHASE_ON_ROAD = 15.0f;      // a target this near a road node is on the road
constexpr float VELOCITY_UNIT = 50.0f;      // m/s per unit of a player's velocity
constexpr float CHASE_LEAD_MIN = 0.3f;      // s: aims where the target will be by the time it gets there,
constexpr float CHASE_LEAD_MAX = 1.5f;      // ...looking this far ahead at least and at most
constexpr float CHASE_STILL = 2.0f;         // m/s: a target slower than this is waited beside, not followed
constexpr float CHASE_ALONGSIDE = 3.0f;     // m: a lone chaser keeps up with a moving target this far to its left
constexpr float CHASE_SPREAD = 4.0f;        // m: several chasers aim this far apart, either side of the target
constexpr float CHASE_OFFROAD_RANGE = 200.0f; // with a car mesh, nearer than this the chase goes straight across it
constexpr float CHASE_GROUND_STEP = 4.0f;   // a straight run samples the ground this often
constexpr float CHASE_DROP = 60.0f;         // a target below and this near, past an edge: off it after them
constexpr float CHASE_DROP_DZ = 30.0f;      // ...no further down than this
constexpr float CHASE_EDGE_STEP = 2.0f;     // how often the way to an edge is sampled
constexpr float GRAVITY = 9.81f;            // m/s^2, a car off an edge
constexpr float FALL_MAX = 50.0f;           // m/s, the fastest it falls
constexpr float AVOID_EVERY = 0.05f;        // s: a driver looks for what is in its way this often
constexpr float AVOID_STEP = 0.1f;          // s: a driver's course is checked against every vehicle this often
constexpr float AVOID_HORIZON_MIN = 1.5f;   // s: ...this far ahead at least,
constexpr float AVOID_HORIZON_MAX = 4.0f;   // ...at most, and as long as it takes to stop plus a second
constexpr float AVOID_CREEP = 3.0f;         // m/s: standing still, it checks the course it would pull away on
constexpr float AVOID_SIDE = 0.25f;         // m: room it keeps either side of its own box
constexpr float AVOID_FRONT = 0.6f;         // m: ...and ahead of it
constexpr float AVOID_GAP = 1.5f;           // m: where it stops behind one it cannot get past
constexpr float AVOID_LEVEL = 4.0f;         // m: a vehicle further up or down is on another level
constexpr float AVOID_STICK = 2.0f;         // s: it keeps swerving to the same side this long after
constexpr float AVOID_LATER = 0.3f;         // s: boxed in, a course that meets it this much later...
constexpr float AVOID_RUN = 3.0f;           // m: ...and gets this far first is taken for now
constexpr float AVOID_STUCK = 1.0f;         // s: stopped this long with no way past, it backs up
constexpr float AVOID_BACK = 5.0f;          // m: ...this far, the nose swinging to the side it will pass on
constexpr float AVOID_SWERVES[] = { 0.0f, 8.0f, 16.0f, 28.0f, 45.0f, 65.0f };   // degrees off its heading it tries
constexpr float LANDED = 0.3f;              // m over the ground it falls to: down
constexpr float NOSE = 2.0f;                // the car is off the edge once this far ahead of its centre is
constexpr float NOSE_COL = 0.5f;            // ...or, with the world's collision, this far
constexpr float CHASE_AIM_CONE = 35.0f;     // degrees: the lead never aims further off the way to the target
constexpr float CHASE_VEL_SMOOTH = 0.4f;    // each think's share of the target's velocity it goes by (the rest: before)
constexpr float CHASE_PLAN_LEAD = 2.5f;     // s: a route is planned to where the target will be, at most this far ahead
constexpr float FOLLOW_GAP = 6.0f;          // m: behind a moving target, it keeps this far back from it...
constexpr float FOLLOW_REACT = 0.3f;        // s: ...and this long at its own speed (it sees the target 4 times a second)
constexpr float FOLLOW_LANE = 2.5f;         // m: a target this near the line ahead of it is in its way
constexpr float FOLLOW_TURNED = 45.0f;      // degrees: a target going this far off its own way has turned off it...
constexpr float FOLLOW_NEAR = 50.0f;        // m: ...and, this near, is followed round the corner at...
constexpr float FOLLOW_CORNER = 8.0f;       // m: ...the speed its grip holds on a turn this tight
constexpr float CHASE_GAP_STEP = 1.0f;      // m: a railing in the way of a drop: a gap is looked for this far apart...
constexpr int CHASE_GAP_TRIES = 12;         // ...this many times either side
constexpr float CHASE_DROP_RUNUP = 8.0f;    // m: a gap is driven through square on from this far back
constexpr float CHASE_WALL_HOLD = 2.0f;     // s: a straight run a wall stopped is not tried again for this long
// The world's collision (world.col): what a car would hit.
constexpr float BUMPER = 0.7f;              // m over the ground: a ray at this height meets what a car hits
constexpr float WALL_OUT = 0.02f;            // m: nosing into a wall, a course must take it this much out
constexpr float WALL_STEP = 0.25f;          // s: a course is checked against walls this often
constexpr float WALL_GAP = 1.0f;            // m: where it stops short of a wall it cannot get round
constexpr float WALL_SIDE = 0.15f;          // m: room it keeps either side of its box from a wall
constexpr float WALL_CLIMB = 0.4f;          // m: the ground the wall rays follow rises or falls this much a step...
constexpr float WALL_SLOPE = 0.6f;          // ...or this much a metre (a steep hill road)
constexpr float WALL_REACH = 35.0f;          // m: walls further down the straight line than this are not looked for...
constexpr float WALL_HORIZON_MIN = 1.0f;     // s: ...unless that is under this long at its speed
constexpr float COL_UP = 2.0f;              // m: the ground under a point is looked for from this far above it...
constexpr float COL_DOWN = 6.0f;            // ...to this far below
constexpr float FALL_REACH = 120.0f;        // m: how far down a falling car looks for what it lands on
constexpr float STEP_UP = 0.5f;             // m: the most a car's wheels climb at once (a kerb, not a railing)
constexpr float GROUND_RISE = 1.2f;         // m: a run-up's ground climbs or falls no more than this per sample
constexpr float CLIMB_RISE = 2.0f;          // m: a straight run climbs no more than this per sample (27 degrees);
                                            // going down, any slope or drop, as far as CHASE_DROP_DZ in all
constexpr float SEA_FLOOR = -3.0f;          // m: ground deeper than this, with no car mesh on it, is under the sea
constexpr float FALL_STEP = 1.5f;           // m: the ground this much lower just past its middle: the car is off
                                            // an edge, and falls
constexpr float BANK_SLOPE = 1.2f;          // m down per m along: the steepest a chasing car follows the ground down
constexpr float CATCHUP_FROM = 40.0f;       // m: a chaser further than this behind a moving target...
constexpr float CATCHUP_FULL = 160.0f;      // ...gets more and more extra speed up to this far,
constexpr float CATCHUP_MAX = 0.35f;        // ...this share of its top speed and acceleration at most

// OnNavMoveEnd's reasons (DiamondNav.inc).
enum MoveEnd : int {
    END_ARRIVED = 0,
    END_NO_PATH = 1,        // no way there (or no navmesh / road graph)
    END_STOPPED = 2,        // Nav_Stop, or a new Nav_WalkTo / Nav_DriveTo
    END_INTERRUPTED = 3,    // something else moved it, it left the car, died or was destroyed
    END_LOST = 4,           // Nav_Chase: the target left, or is in another world
};

// Nav_DriveTo's flags beyond the road ones (DiamondNav.inc).
constexpr int DRIVE_OFFROAD = 4;
// Nav_Chase's flags.
constexpr int CHASE_RAM = 1;            // drive into the target: no slowing at the end, no spreading out
constexpr int CHASE_NO_DROP = 2;        // never off an edge after it: the roads round instead
// Nav_GetChaseInfo's modes.
enum ChaseMode : int { CHASE_BESIDE = 0, CHASE_STRAIGHT = 1, CHASE_ROUTE = 2, CHASE_PLANNING = 3, CHASE_EDGE = 4,
    CHASE_AIRBORNE = 5, CHASE_OVERHEAD = 6 };
// Nav_FlyTo's flags.
constexpr int FLY_LAND = 1;             // down onto the ground there, instead of hovering
// A helicopter chasing someone follows over their trail while they move, and circles
// over them when they stop, looking down at them.
constexpr float HELI_TRAIL = 15.0f;         // m behind them, on the move
constexpr float HELI_TRAIL_FROM = 5.0f;     // m/s: faster than this, they are on the move...
constexpr float HELI_TRAIL_TO = 3.0f;       // ...and slower than this, stopped
constexpr float HELI_CHASE_RADIUS = 22.0f;  // m out from them
constexpr float HELI_CHASE_HEIGHT = 28.0f;  // m over them
constexpr float HELI_CHASE_RATE = 9.0f;     // degrees a second round them
constexpr float HELI_CHASE_CEILING = 45.0f; // m over them: it flies round anything that would take it higher
constexpr float HELI_DT_MAX = 0.1f;         // s: a longer gap between ticks flies as this
constexpr float HELI_MOVED = 3.0f;          // m: something else moved the helicopter this far: it flies on from there
constexpr float HELI_SEA = 0.0f;            // the sea's surface (no face in the collision)

bool driving(dnav::PathKind k)
{
    return k != dnav::PathKind::Walk;
}

struct Mover {
    int npc = -1;
    void* owner = nullptr;              // the script told when it ends
    int request = 0;                    // the search it waits on (0: moving)
    dnav::PathKind kind = dnav::PathKind::Walk;
    NPCMoveType moveType = NPCMoveType_Walk;
    float speed = NPC_MOVE_SPEED_AUTO;
    float carZ = 1.0f;                  // Drive: the car's centre over the road
    std::vector<Vector3> points;
    std::vector<float> speeds;
    // Driving: each point's road class, whether it is in a junction, the fastest the
    // car may go there (m/s, the speed plan) and the distance along the path to it.
    std::vector<dnav::RoadClass> classes;
    std::vector<uint8_t> junctions;
    std::vector<uint8_t> ledges;        // 1 where a drop off a ledge starts (OffRoad, a chase)
    Vector3 rescueAt {};                // where it last backed out of being stuck...
    float rescueSide = 0.0f;            // ...and to which side
    bool rearHit = false;               // backing up, the wall behind came near: forward next
    bool onLedge = false;               // on one's run-up: its way off was found clear, no wall checks
    std::vector<float> limits, cum;
    dnav::Handling car;                 // the car's handling, read when the path is set
    float boost = 1.0f;                 // a chaser's catch-up: its top speed and acceleration times this
    dnav::DriveStyle style = dnav::DriveStyle::Normal;
    float wantSpeed = 0.0f;             // the speed it is going for (Nav_GetDriveInfo)
    size_t next = 0;
    Vector3 target {};

    // Driving (drive()): the path segment the car is on, its speed and heading.
    size_t seg = 0;
    float v = 0.0f;                     // m/s
    float heading = 0.0f;               // degrees, GTA's
    float yawRate = 0.0f;               // degrees a second it is turning at
    bool rolling = false;               // speed and heading are the car's own
    bool walkOn = false;                // a walker given a new place while on its way: it walks on meanwhile
    bool ground = false;                // read the height off the car mesh as it goes
    TimePoint lastDrive {};
    Vector3 lastPos {};                 // the NPC's position at the last drive()
    bool reversing = false;             // backing up (a three-point turn, or out of a corner)
    float backed = 0.0f;                // metres backed up this time
    bool turning = false;               // turning round in a tight spot: forward and back, on full lock
    float circling = 0.0f;              // seconds spent turning hard
    int wrongWay = 0;                   // drive()s in a row that a back-up went forward

    // Nav_Chase: the car follows a player until Nav_Stop. It re-plans as the target
    // moves and keeps driving the old plan until the new one comes back.
    bool chase = false;
    int quarry = -1;                    // the player chased
    int plan = 0;                       // a re-plan being searched
    Vector3 planGoal {};                // where the path being driven ends up
    Vector3 askedGoal {};               // where the re-plan being searched ends up
    TimePoint nextThink {}, planned {}, noOffRoadUntil {};
    bool direct = false;                // driving straight at the target
    bool idle = false;                  // stopped beside it
    // A straight run off an edge after a target below (a bridge, a ledge): on the
    // deck's level to its edge, then off it, falling to the first deck under it or
    // the level the target is on.
    bool drop = false;
    float dropZ = 0.0f;                 // the target's ground
    bool airborne = false;              // off the edge, falling
    float vz = 0.0f;                    // m/s, up
    int chaseFlags = 0;                 // Nav_Chase's flags (CHASE_RAM, CHASE_NO_DROP)
    float match = 0.0f;                 // m/s: the target's speed, which a run ends at (a moving target)
    float quarryDist = 0.0f;            // m: how far the target was at the last think (Nav_GetChaseInfo)
    int quarryCar = -1;                 // the target's vehicle (a ram goes into it)
    float avoidSide = 0.0f;             // the way it is swerving round a vehicle (1 left, -1 right)
    float avoidFor = 0.0f;              // s it keeps to that side after the way is clear
    float blocked = 0.0f;               // s stopped with no way past a vehicle
    bool backOff = false;               // backing up to get round one
    float backSteer = 0.0f;             // ...the nose swinging this way (degrees, left positive)
    float stuckFor = 0.0f;              // s it has wanted to move and has not got anywhere
    Vector3 stuckAt {};                 // ...from here
    float nudge = 0.0f;                 // s it edges forward through what is in its way (boxed in)
    Vector3 tvel {};                    // the target's velocity, smoothed over the last few thinks
    Vector3 quarryPos {};               // where the target was at the last think...
    Vector2 quarryVel {};               // ...and its velocity then, m/s (not smoothed)
    TimePoint quarryAt {};
    bool seen = false;                  // tvel has a value
    bool walled = false;                // drive(): a wall is in the way and there is no way round it
    float wallHold = 0.0f;              // s left before a straight run is tried again
    float avoidDt = 1.0f;               // s since avoid() last looked
    bool avoidHasTurn = false;          // ...and what it decided: a swerve,
    float avoidTurn = 0.0f;
    float avoidCap = 1e9f;              // ...and a speed to keep under (m/s)
};

// A vehicle's footprint: its collision box from above.
struct Box {
    Vector2 c, f, r;                    // its middle, and its forward and right axes
    float hw, hl;                       // half its width (along r) and length (along f)
};

// Whether two boxes overlap (separating axes: theirs).
bool overlaps(const Box& a, const Box& b)
{
    const Vector2 d = b.c - a.c;
    for (const Vector2& ax : { a.f, a.r, b.f, b.r }) {
        const float ra = a.hw * std::fabs(glm::dot(a.r, ax)) + a.hl * std::fabs(glm::dot(a.f, ax));
        const float rb = b.hw * std::fabs(glm::dot(b.r, ax)) + b.hl * std::fabs(glm::dot(b.f, ax));
        if (std::fabs(glm::dot(d, ax)) > ra + rb) return false;
    }
    return true;
}

// A model's collision box: half its width and length, and where its middle sits from
// the car's centre (right, front).
struct Shape {
    float hw, hl, ox, oy;
};

// GTA's heading (degrees, 0 north, turning left) as forward and right axes.
Vector2 forwardOf(float heading)
{
    const float h = glm::radians(heading);
    return Vector2(-std::sin(h), std::cos(h));
}

Box boxAt(const Shape& s, const Vector2& centre, float heading, float wider = 0.0f, float longer = 0.0f)
{
    const Vector2 f = forwardOf(heading), r(f.y, -f.x);
    return Box { centre + r * s.ox + f * s.oy, f, r, s.hw + wider, s.hl + longer };
}

// A vehicle as drivers see it this tick (avoid()).
struct Traffic {
    int id;
    int world;
    Vector3 pos;
    Vector2 vel;                        // m/s
    float heading;                      // GTA's, degrees
    Shape shape;
};

// Planes.
constexpr float PLANE_DT_MAX = 0.1f;        // s: a longer gap between ticks flies as this
constexpr float PLANE_MOVED = 5.0f;         // m: something else moved the plane this far: it goes on from there
constexpr float PLANE_GROUND = 2.5f;        // m over its wheels' height: still on the ground
constexpr float PLANE_SEE_EVERY = 0.25f;    // s between looking at what stands on the airports
constexpr float PLANE_TOP_CELL = 10.0f;     // m: the ground's height is kept for cells this big
constexpr size_t PLANE_TOP_KEEP = 400000;   // cells kept before they are all forgotten
constexpr char AIRPORTS_FOLDER[] = "airports";
// Nav_Land's gate for "none, stop once off the runway".
constexpr char NO_GATE[] = "-";

// An NPC flying a plane: its pilot (pilot.hpp) does everything from the gate on.
struct Plane {
    void* owner = nullptr;              // the script told how its orders end
    dnav::Pilot pilot;
    Vector3 placed {};                  // where it was put last
    int gear = -1;                      // the gear state set last
    bool busy = false;                  // an order is under way (a new one ends it STOPPED)
    explicit Plane(int id, int model)
        : pilot(id, model)
    {
    }
};

// An NPC flying a helicopter (Nav_FlyTo, or Nav_Chase in one).
struct Flyer {
    void* owner = nullptr;              // the script told when it ends
    int quarry = -1;                    // Nav_Chase: who it flies over (-1: Nav_FlyTo)
    dnav::FlyGoal goal;
    dnav::Flight flight;
    dnav::HeliSpec spec;
    float angle = 0.0f;                 // round its quarry, degrees
    float quarryDist = 0.0f;
    bool trailing = false;              // following its quarry's trail (else circling it)
    Vector3 placed {};                  // where it was put last
    TimePoint last {};
};

class DiamondNav final : public IComponent, public PawnEventHandler, public CoreEventHandler, public NPCEventHandler {
public:
    PROVIDE_UID(0xc7e6abc6cb842c2c);

    static DiamondNav* instance;

    dnav::Pathfinder finder;
    dnav::Driving drivingModel;         // handling, speed limits, the speed plan
    std::unordered_map<int, dnav::DriveStyle> styles;   // Nav_SetDriveStyle, by NPC
    bool canReverse_ = true;            // the NPC component backs cars up (our patched build)
    // The result being handed to a script right now (Nav_GetPathPoint reads it).
    const dnav::PathResult* current = nullptr;

    ~DiamondNav()
    {
        if (pawn_) pawn_->getEventDispatcher().removeEventHandler(this);
        if (npcs_) npcs_->getEventDispatcher().removeEventHandler(this);
        if (core_) core_->getEventDispatcher().removeEventHandler(this);
        finder.stop();
        instance = nullptr;
    }

    StringView componentName() const override { return "DiamondNav"; }
    SemanticVersion componentVersion() const override
    {
        return SemanticVersion(DNAV_VERSION_MAJOR, DNAV_VERSION_MINOR, DNAV_VERSION_PATCH, 0);
    }

    void onLoad(ICore* c) override
    {
        core_ = c;
        instance = this;
        setAmxLookups(core_);
        std::vector<std::string> log;
        finder.load(NAV_FOLDER, log);
        for (const auto& line : log) {
            if (line.rfind("ERROR: ", 0) == 0) core_->logLn(LogLevel::Error, "[nav] %s", line.c_str() + 7);
            else core_->printLn("[nav] %s", line.c_str());
        }
        loadVehicles();
        loadCollision();
        loadAirports();
        if (world_.loaded() && finder.meshLoaded(dnav::MeshKind::Car)) {
            float secs = 0.0f;
            const int n = finder.addLedges(world_, secs);
            core_->printLn("[nav] %d ledges a chase may drive off (found in %.1f s)", n, secs);
        }
        const unsigned hw = std::thread::hardware_concurrency();
        const int threads = static_cast<int>(std::clamp(hw > 1 ? hw - 1 : 1u, 1u, 4u));
        if (finder.loaded()) finder.start(threads);
        core_->printLn("[nav] DiamondNav %d.%d.%d: %d walking tiles, %d car tiles, %d road nodes, %d search threads",
            DNAV_VERSION_MAJOR, DNAV_VERSION_MINOR, DNAV_VERSION_PATCH, finder.tileCount(dnav::MeshKind::Ped),
            finder.tileCount(dnav::MeshKind::Car), static_cast<int>(finder.roads().nodeCount()),
            finder.loaded() ? threads : 0);
        core_->getEventDispatcher().addEventHandler(this);
    }

    void loadVehicles()
    {
        const std::string path = std::string(NAV_FOLDER) + "/vehicles.dnv";
        if (!std::ifstream(path)) {
            core_->printLn("[nav] no vehicles.dnv: every car drives alike (tools/dnav.py vehicles makes it from GTA)");
            return;
        }
        std::string error;
        if (drivingModel.loadVehicles(path, error)) {
            core_->printLn("[nav] loaded vehicles.dnv: %d vehicle models", static_cast<int>(drivingModel.vehicleCount()));
        } else {
            core_->logLn(LogLevel::Error, "[nav] %s", error.c_str());
        }
    }

    // The built-in airports (airports_gta.cpp: San Andreas' own, unless navmesh/airports
    // has a gta_sa.txt of its own), then navmesh/airports/*.txt in name order.
    void loadAirports()
    {
        namespace fs = std::filesystem;
        const fs::path dir = fs::path(NAV_FOLDER) / AIRPORTS_FOLDER;
        std::vector<fs::path> files;
        std::error_code ec;
        if (fs::is_directory(dir, ec)) {
            for (const auto& e : fs::directory_iterator(dir, ec))
                if (e.is_regular_file() && e.path().extension() == ".txt") files.push_back(e.path());
        }
        std::sort(files.begin(), files.end());
        std::string error;
        bool ownGta = false;
        for (const auto& f : files)
            if (f.filename() == "gta_sa.txt") ownGta = true;
        if (!ownGta && !airports_.loadText(dnav::AIRPORTS_GTA_SA, "built-in gta_sa.txt", error))
            core_->logLn(LogLevel::Error, "[nav] %s", error.c_str());
        for (const auto& f : files) {
            if (!airports_.loadFile(f.string(), error)) core_->logLn(LogLevel::Error, "[nav] %s", error.c_str());
        }
        atc_.setDb(&airports_);
        core_->printLn("[nav] %d airports, %d flight routes (%d airport files)", airports_.liveCount(),
            static_cast<int>(airports_.routeCount()), static_cast<int>(files.size()));
    }

    void loadCollision()
    {
        const std::string path = std::string(NAV_FOLDER) + "/world.col";
        if (!std::ifstream(path)) {
            core_->printLn("[nav] no world.col: cars only know walls from the car navmesh (copy the world.tris tools/dnav.py world writes here as world.col)");
            return;
        }
        std::string error;
        if (world_.load(path, error))
            core_->printLn("[nav] loaded world.col: %d faces", static_cast<int>(world_.triangleCount()));
        else
            core_->logLn(LogLevel::Error, "[nav] %s", error.c_str());
    }

    // The style npc drives in: its own (Nav_SetDriveStyle), else pursuit for a chase
    // and normal for anything else.
    dnav::DriveStyle styleOf(int npcid, bool chase) const
    {
        auto it = styles.find(npcid);
        if (it != styles.end()) return it->second;
        return chase ? dnav::DriveStyle::Pursuit : dnav::DriveStyle::Normal;
    }

    void onInit(IComponentList* components) override
    {
        pawn_ = components->queryComponent<IPawnComponent>();
        if (pawn_) {
            setAmxFunctions(pawn_->getAmxFunctions());
            setAmxLookups(components);
            pawn_->getEventDispatcher().addEventHandler(this);
        }
        vehicles_ = components->queryComponent<IVehiclesComponent>();
        npcs_ = components->queryComponent<INPCComponent>();
        if (npcs_) npcs_->getEventDispatcher().addEventHandler(this);
        else core_->printLn("[nav] no NPC component: Nav_WalkTo / Nav_DriveTo are off, path searches still work");
    }

    void onAmxLoad(IPawnScript& script) override { pawn_natives::AmxLoad(script.GetAMX()); }

    void onAmxUnload(IPawnScript& script) override
    {
        void* amx = script.GetAMX();
        finder.forgetOwner(amx);
        for (auto& r : results_)
            if (r.owner == amx) r.owner = nullptr;
        for (auto& [id, m] : movers_)
            if (m.owner == amx) m.owner = nullptr;
        for (auto& [id, f] : flyers_)
            if (f.owner == amx) f.owner = nullptr;
        for (auto& [id, p] : planes_)
            if (p.owner == amx) p.owner = nullptr;
    }

    void onTick(Microseconds, TimePoint now) override
    {
        if (!planes_.empty()) planesStep(now);
        // Helicopters fly (ids first: arriving or losing the target ends a flight).
        thinking_.clear();
        for (const auto& [id, f] : flyers_) thinking_.push_back(id);
        for (int id : thinking_) {
            auto it = flyers_.find(id);
            if (it != flyers_.end()) flyStep(id, it->second, now);
        }
        // Chasers look at their targets (ids first: thinking can end a chase).
        thinking_.clear();
        for (const auto& [id, m] : movers_)
            if (m.chase && m.nextThink <= now) thinking_.push_back(id);
        for (int id : thinking_) {
            auto it = movers_.find(id);
            if (it != movers_.end()) think(id, it->second, now);
        }
        // Drivers steer (ids first: arriving ends a drive).
        thinking_.clear();
        for (const auto& [id, m] : movers_) {
            if (!driving(m.kind) || m.request || (!m.chase && m.points.empty())) continue;
            INPC* npc = npcs_ ? npcs_->get(id) : nullptr;
            if (!npc || npc->getPosition() != m.lastPos || now - m.lastDrive >= DRIVE_IDLE) thinking_.push_back(id);
        }
        if (!thinking_.empty()) seeTraffic();
        for (int id : thinking_) {
            auto it = movers_.find(id);
            if (it != movers_.end()) drive(id, it->second, now);
        }

        fresh_.clear();
        finder.takeResults(fresh_);
        for (auto& r : fresh_) results_.push_back(std::move(r));
        // Oldest first, at most MAX_RESULTS_PER_TICK; the rest wait for the next tick.
        size_t handed = 0;
        for (; handed < results_.size() && handed < MAX_RESULTS_PER_TICK; handed++) {
            const auto& r = results_[handed];
            auto mv = byRequest_.find(r.id);
            if (mv != byRequest_.end()) {
                const int npc = mv->second;
                byRequest_.erase(mv);
                auto m = movers_.find(npc);
                if (m != movers_.end() && m->second.chase) planDone(npc, m->second, r, now);
                else startMoving(npc, r);
                continue;
            }
            if (!r.owner || !pawn_) continue;           // its script has gone
            IPawnScript* script = pawn_->getScript(static_cast<AMX*>(r.owner));
            if (!script) continue;
            current = &r;
            script->Call("OnNavPathResult", DefaultReturnValue_False, r.id, static_cast<int>(r.status),
                static_cast<int>(r.points.size()));
            current = nullptr;
        }
        results_.erase(results_.begin(), results_.begin() + handed);
    }

    // ── NPC movement ─────────────────────────────────────────────────────────

    // How high a model's centre sits over the road: vehicles.dnv's (the game's own,
    // from its wheels and springs), else a guess from open.mp's model table (its front
    // wheels, with wheels and springs that grow with the vehicle), else 1 m.
    float rideHeight(int model) const
    {
        const float ride = drivingModel.handling(model).ride;
        if (ride > 0.0f) return ride;
        Vector3 size, wheels;
        if (Impl::getVehicleModelInfo(model, VehicleModelInfo_Size, size)
            && Impl::getVehicleModelInfo(model, VehicleModelInfo_WheelsFront, wheels) && wheels.z < 0.0f) {
            const float grow = std::clamp((size.z - 1.5f) / 2.7f, 0.0f, 1.0f);
            return -wheels.z + 0.5f + 0.35f * grow;
        }
        return RIDE_DEFAULT;
    }

    // The ground at (x, y) near height z: the world's collision, else the car mesh,
    // else the walking mesh, else the road surface nearby. False when none of them
    // knows the place.
    bool groundUnder(float x, float y, float z, float& out) const
    {
        if (world_.loaded() && world_.ground(x, y, z, COL_UP, COL_DOWN, out)) return true;
        if (finder.meshLoaded(dnav::MeshKind::Car) && finder.groundAt(x, y, z, GROUND_REACH, out, dnav::MeshKind::Car))
            return true;
        if (finder.meshLoaded(dnav::MeshKind::Ped) && finder.groundAt(x, y, z, GROUND_REACH, out, dnav::MeshKind::Ped))
            return true;
        return finder.roads().surfaceAt(x, y, z, ROAD_GROUND_REACH, out);
    }

    // A model's collision box: vehicles.dnv's (from its DFF), else open.mp's model table
    // (centred), else a car's.
    Shape shapeOf(int model)
    {
        auto it = shapes_.find(model);
        if (it != shapes_.end()) return it->second;
        Shape sh { 0.95f, 2.25f, 0.0f, 0.0f };
        Vector3 size;
        if (const dnav::Handling* h = drivingModel.fromFile(model))
            sh = Shape { h->width * 0.5f, h->length * 0.5f, h->boxX, h->boxY };
        else if (Impl::getVehicleModelInfo(model, VehicleModelInfo_Size, size) && size.x > 0.1f && size.y > 0.1f)
            sh = Shape { size.x * 0.5f, size.y * 0.5f, 0.0f, 0.0f };
        shapes_[model] = sh;
        return sh;
    }

    // Where every vehicle is, which way it faces and how fast it goes, once a tick for
    // every driver: our own drivers by what they are doing, the rest by what the
    // server last heard.
    void seeTraffic()
    {
        traffic_.clear();
        if (!vehicles_) return;
        std::unordered_map<int, const Mover*> driven;
        for (const auto& [id, m] : movers_) {
            INPC* npc = npcs_ && driving(m.kind) ? npcs_->get(id) : nullptr;
            if (npc && npc->getVehicle()) driven[npc->getVehicle()->getID()] = &m;
        }
        for (IVehicle* v : *vehicles_) {
            Traffic t;
            t.id = v->getID();
            t.world = v->getVirtualWorld();
            t.pos = v->getPosition();
            t.heading = v->getRotation().ToEuler().z;
            auto d = driven.find(t.id);
            if (d != driven.end() && d->second->rolling) {
                t.heading = d->second->heading;
                t.vel = forwardOf(t.heading) * (d->second->reversing ? -d->second->v : d->second->v);
            } else {
                t.vel = Vector2(v->getVelocity()) * VELOCITY_UNIT;
            }
            t.shape = shapeOf(v->getModel());
            traffic_.push_back(t);
        }
    }

    // When a car on a course first touches another vehicle (seconds from now, or -1):
    // its own box (with room round it) sweeps along, turning at `rate` degrees a second
    // until it is `turn` degrees off `heading`, and every other one moves at its own
    // velocity. One it already touches beside or behind it is no matter.
    float firstHit(const std::vector<const Traffic*>& nearby, const Shape& me, const Vector2& from, float heading,
        float turn, float rate, float speed, float horizon, const Traffic** who) const
    {
        Vector2 at = from;
        float h = heading, turned = 0.0f;
        const Box now = boxAt(me, from, heading, AVOID_SIDE, AVOID_FRONT);
        for (float t = AVOID_STEP; t <= horizon + 1e-4f; t += AVOID_STEP) {
            const float d = std::min(std::fabs(rate) * AVOID_STEP, std::fabs(turn) - turned);
            if (d > 0.0f) {
                h += std::copysign(d, turn);
                turned += d;
            }
            at += forwardOf(h) * (speed * AVOID_STEP);
            const Box mine = boxAt(me, at, h, AVOID_SIDE, AVOID_FRONT);
            for (const Traffic* o : nearby) {
                const Vector2 start(o->pos);
                const Box was = boxAt(o->shape, start, o->heading);
                if (overlaps(now, was) && glm::dot(start - from, now.f) <= 0.0f) continue;
                if (overlaps(mine, boxAt(o->shape, start + o->vel * t, o->heading))) {
                    if (who) *who = o;
                    return t;
                }
            }
        }
        return -1.0f;
    }

    // Vehicles in the way, other DiamondNav cars and everyone else's, moving or parked,
    // by their real collision boxes, and, with the world's collision, walls, posts and
    // railings: the course it is about to take (toward its aim at its turn rate) is
    // swept forward against every one. If it touches one, it swerves onto the smallest
    // change of heading that touches nothing (the side it swerved to last first), no
    // faster than it is going; if every swerve touches something, it brakes to stop
    // short of it, or to follow a vehicle at its speed. A chaser that rams ignores its
    // target's car.
    void avoid(Mover& m, IVehicle& car, const Vector3& pos, bool offRoad, float tick, float& error, float& target)
    {
        // Looked at AVOID_EVERY at most (it costs rays and boxes); in between, the last
        // look's swerve and speed hold.
        m.avoidDt += tick;
        if (m.avoidDt < AVOID_EVERY) {
            if (m.avoidHasTurn) error = m.avoidTurn;
            target = std::min(target, m.avoidCap);
            return;
        }
        const float dt = m.avoidDt;
        m.avoidDt = 0.0f;
        m.avoidHasTurn = false;
        m.avoidCap = 1e9f;
        const float wantError = error;
        const float wantTarget = target;
        look(m, car, pos, offRoad, dt, error, target);
        if (error != wantError) {
            m.avoidHasTurn = true;
            m.avoidTurn = error;
        }
        if (target < wantTarget) m.avoidCap = target;
    }

    void look(Mover& m, IVehicle& car, const Vector3& pos, bool offRoad, float dt, float& error, float& target)
    {
        m.avoidFor = std::max(0.0f, m.avoidFor - dt);
        if (m.avoidFor <= 0.0f) m.avoidSide = 0.0f;
        m.walled = false;
        // (On a ledge's run-up the way off was found clear, a car's width and more:
        // its posts and railing ends are not taken for a wall it must stop at.)
        const bool walls = world_.loaded() && !m.onLedge;
        if (traffic_.empty() && !walls) return;
        const Shape me = shapeOf(car.getModel());
        const int world = car.getVirtualWorld();
        const int ignore = m.chase && (m.chaseFlags & CHASE_RAM) ? m.quarryCar : -1;
        const float brake = offRoad ? m.car.brakeOff : m.car.brake;
        const float grip = (offRoad ? m.car.gripOff : m.car.grip) * 0.95f;
        const float speed = std::max(m.v, AVOID_CREEP);
        const float horizon = std::clamp(speed / brake + 1.0f, AVOID_HORIZON_MIN, AVOID_HORIZON_MAX);
        const float range = speed * horizon + me.hl + 15.0f;
        // Walls only so far ahead: its way bends long before a wall far down the
        // straight line (a building past the next corner), and its speed plan slows
        // it for the corner.
        const float wallHorizon = std::min(horizon, std::max(WALL_REACH / speed, WALL_HORIZON_MIN));
        near_.clear();
        for (const Traffic& o : traffic_) {
            if (o.id == car.getID() || o.id == ignore || o.world != world || std::fabs(o.pos.z - pos.z) > AVOID_LEVEL)
                continue;
            if (glm::distance(Vector2(o.pos), Vector2(pos)) < range + o.shape.hl + glm::length(o.vel) * horizon)
                near_.push_back(&o);
        }
        if (near_.empty() && !walls) return;
        const float groundZ = pos.z - m.carZ;
        // As fast as it can turn at this speed: its turning circle, its grip.
        const float rate = glm::degrees(speed / std::max(m.car.radius, speed * speed / grip));
        // When a course first meets a vehicle or a wall, and which vehicle.
        auto blockedAt = [&](float turn, const Traffic** who) {
            const Traffic* o = nullptr;
            float t = near_.empty() ? -1.0f : firstHit(near_, me, Vector2(pos), m.heading, turn, rate, speed, horizon, &o);
            const float w = walls ? wallHit(me, Vector2(pos), m.heading, turn, rate, speed, wallHorizon, groundZ) : -1.0f;
            if (w >= 0.0f && (t < 0.0f || w < t)) {
                t = w;
                o = nullptr;
            }
            if (who) *who = o;
            return t;
        };
        const Traffic* hit = nullptr;
        const float t = blockedAt(error, &hit);
        if (t < 0.0f) {
            m.blocked = 0.0f;
            return;
        }
        // The side to pass on: the one it is already swerving to, else for a vehicle the
        // one its box leaves more room on (where its corners reach across the way), for
        // a wall the one that meets it later, else the way it is turning anyway. It
        // tries every swerve on that side before the other.
        float first = m.avoidSide;
        if (first == 0.0f && hit) {
            const Box b = boxAt(hit->shape, Vector2(hit->pos), hit->heading);
            const Vector2 left = -boxAt(me, Vector2(pos), m.heading).r;
            float lo = 1e9f, hi = -1e9f;
            for (float i : { -1.0f, 1.0f })
                for (float j : { -1.0f, 1.0f }) {
                    const float lat = glm::dot(b.c + b.r * (b.hw * i) + b.f * (b.hl * j) - Vector2(pos), left);
                    lo = std::min(lo, lat);
                    hi = std::max(hi, lat);
                }
            first = std::fabs(hi + lo) > 0.5f ? (hi + lo < 0.0f ? 1.0f : -1.0f) : (error >= 0.0f ? 1.0f : -1.0f);
        } else if (first == 0.0f) {
            // (Both ways meeting it as soon, a wall it already noses into: away from the
            // side of the car it touches.)
            const Vector2 touch(lastWallHit_.x, lastWallHit_.y);
            const float h = glm::radians(m.heading);
            const bool onRight = glm::dot(touch - Vector2(pos), Vector2(std::cos(h), std::sin(h))) > 0.0f;
            const float l = wallHit(me, Vector2(pos), m.heading, 28.0f, rate, speed, wallHorizon, groundZ);
            const float r = wallHit(me, Vector2(pos), m.heading, -28.0f, rate, speed, wallHorizon, groundZ);
            first = l < 0.0f && r >= 0.0f ? 1.0f : r < 0.0f && l >= 0.0f ? -1.0f
                : l != r ? (l > r ? 1.0f : -1.0f) : (onRight ? 1.0f : -1.0f);
        }
        float latest = t, latestTurn = error, latestSide = 0.0f;
        for (float side : { first, -first }) {
            for (float swerve : AVOID_SWERVES) {
                if (swerve == 0.0f && side != first) continue;     // straight on: tried once
                const float turn = side * swerve;
                const float when = blockedAt(turn, nullptr);
                if (when >= 0.0f) {
                    if (when > latest + AVOID_LATER) {
                        latest = when;
                        latestTurn = turn;
                        latestSide = side;
                    }
                    continue;
                }
                m.avoidSide = side;
                m.avoidFor = AVOID_STICK;
                m.blocked = 0.0f;
                error = turn;
                target = std::min(target, std::max(m.v, AVOID_CREEP));
                return;
            }
        }
        // No course clear all the way, but one that gets well along before it meets
        // anything (along the road to a lamp post far ahead, out from a corner it
        // noses into): it takes that one, slow enough to stop short, and looks again
        // on the way, instead of standing there boxed in by what is ten metres off.
        if (latestSide != 0.0f && speed * latest - WALL_GAP > AVOID_RUN) {
            m.avoidSide = latestSide;
            m.avoidFor = AVOID_STICK;
            m.blocked = 0.0f;
            error = latestTurn;
            target = std::min(target, std::max(AVOID_CREEP * 0.5f,
                std::min(std::max(m.v, AVOID_CREEP), std::sqrt(2.0f * brake * (speed * latest - WALL_GAP)))));
            return;
        }
        // Boxed in: stop short of it, or follow a vehicle at its speed; stopped there a
        // while, back up to make room to swing round it.
        m.walled = !hit;
        const float follow = hit ? std::max(0.0f, glm::dot(hit->vel, forwardOf(m.heading))) : 0.0f;
        const float room = speed * t - (hit ? AVOID_GAP : WALL_GAP);
        target = std::min(target, room > 0.0f ? std::sqrt(follow * follow + 2.0f * brake * room) : follow);
        if (m.v < 0.5f && follow < 0.5f) m.blocked += dt;
        // A chaser held up near a target that stands still has got as near as it will.
        const bool waiting = m.chase && m.quarryDist < CHASE_CLOSE * 2.0f;
        if (m.blocked > AVOID_STUCK && canReverse_ && !waiting) {
            m.blocked = 0.0f;
            m.avoidSide = first;
            m.avoidFor = AVOID_STICK + 5.0f;
            m.backOff = true;
            m.backSteer = first * 45.0f;
            backOffClear(m, car, pos);
            // (No room behind either: it edges forward past it now, a scrape at worst,
            // rather than sit there till the stuck rescue does the same.)
            if (!m.backOff) m.nudge = NUDGE_SECS;
        }
    }

    mutable dnav::Vec3 lastWallHit_ {};     // where wallHit last met a wall
    // Backing up to get round a vehicle or a wall: not into another one behind it.
    void backOffClear(Mover& m, IVehicle& car, const Vector3& pos)
    {
        Shape back = shapeOf(car.getModel());
        back.ox = -back.ox;
        back.oy = -back.oy;
        const float speed = std::min(REVERSE_SPEED, m.car.reverse);
        // The way it is backing, else straight back, else the other way.
        for (float steer : { m.backSteer, 0.0f, -m.backSteer }) {
            if ((!near_.empty() && firstHit(near_, back, Vector2(pos), m.heading + 180.0f, -steer, 30.0f,
                    speed, AVOID_HORIZON_MIN, nullptr) >= 0.0f)
                || wallHit(back, Vector2(pos), m.heading + 180.0f, -steer, 30.0f, speed, AVOID_HORIZON_MIN,
                    pos.z - m.carZ) >= 0.0f)
                continue;
            m.backSteer = steer;
            return;
        }
        m.backOff = false;
    }

    // Ground actually under (x, y) near height z: the meshes, else a road's own deck
    // (not just a road nearby, as groundUnder takes): false past a bridge's side.
    bool deckUnder(float x, float y, float z, float& out, float wider = 0.0f) const
    {
        // The collision knows every surface it was built with (where it has none, the
        // meshes and the roads still say).
        if (world_.loaded() && world_.ground(x, y, z, COL_UP, GROUND_REACH + 1.0f, out)) return true;
        if (finder.meshLoaded(dnav::MeshKind::Car) && finder.groundAt(x, y, z, GROUND_REACH, out, dnav::MeshKind::Car))
            return true;
        if (finder.meshLoaded(dnav::MeshKind::Ped) && finder.groundAt(x, y, z, GROUND_REACH, out, dnav::MeshKind::Ped))
            return true;
        return finder.roads().deckAt(x, y, z, out, wider);
    }

    static dnav::Vec3 v3(const Vector3& v) { return dnav::Vec3 { v.x, v.y, v.z }; }

    // Ground at (x, y, gz) under the sea: below SEA_FLOOR, where the car mesh (baked
    // with the water) has nothing. The collision has the sea bed but no water over it:
    // a car that drove off a quay at it drove round down there for good.
    bool underSea(float x, float y, float gz) const
    {
        if (gz >= SEA_FLOOR) return false;
        float out;
        return !(finder.meshLoaded(dnav::MeshKind::Car) && finder.groundAt(x, y, gz, GROUND_REACH, out, dnav::MeshKind::Car));
    }

    // With the world's collision: whether the ground just past a car's middle is a
    // fall below the ground under it (it is going over an edge), not a slope it drives.
    bool offEdge(const Mover& m, const Vector3& pos, float headingRad) const
    {
        float here, there;
        if (!world_.ground(pos.x, pos.y, pos.z - m.carZ, COL_UP, COL_DOWN, here)) {
            // Already out over a drop (an edge it passed between two looks): it falls,
            // unless there is nothing under it at all.
            return world_.ground(pos.x, pos.y, pos.z - m.carZ, 0.5f, FALL_REACH, there);
        }
        // Out over a drop already, with ground not far under it (a bank's foot, the road
        // below a bridge's end): it drove on level past the edge, and falls from there.
        if (pos.z - m.carZ - here > FALL_STEP) return true;
        // Two probes a metre apart, so a seam between two faces is not taken for a drop.
        const Vector2 fwd(-std::sin(headingRad), std::cos(headingRad));
        for (float d : { NOSE_COL, NOSE_COL + 1.0f }) {
            const Vector2 p = Vector2(pos) + fwd * d;
            // (Nothing under it at all, the sea or the map's end, is no edge to fall off.)
            if (!world_.ground(p.x, p.y, here, 0.5f, FALL_REACH, there) || here - there <= FALL_STEP) return false;
        }
        return true;
    }

    // With the world's collision: when a car on a course first meets a wall (seconds
    // from now, or -1). Its box's front corners and middle sweep along at bumper
    // height over the ground it stands on, turning as firstHit's do.
    float wallHit(const Shape& me, const Vector2& from, float heading, float turn, float rate, float speed,
        float horizon, float groundZ) const
    {
        if (!world_.loaded()) return -1.0f;
        const float z = groundZ + BUMPER;
        auto corners = [&](const Vector2& c, float h, Vector2 out[3]) {
            const Box b = boxAt(me, c, h, WALL_SIDE, 0.0f);
            const Vector2 front = b.c + b.f * b.hl;
            out[0] = front - b.r * b.hw;
            out[1] = front;
            out[2] = front + b.r * b.hw;
        };
        Vector2 at = from, was[3], now[3];
        float h = heading, turned = 0.0f;
        corners(at, h, was);
        // The first step also reaches from the middle of the car to its front, so a
        // wall it already noses into counts: its own box, without the room it keeps
        // (inside that room already, a course away from the wall is still open).
        auto noseRays = [&](const Vector2& c, float h, Vector2 mid[3], Vector2 nose[3]) {
            const Box b = boxAt(me, c, h, 0.0f, 0.0f);
            const Vector2 front = b.c + b.f * b.hl;
            for (int k = 0; k < 3; k++) {
                mid[k] = b.c + b.r * (b.hw * static_cast<float>(k - 1));
                nose[k] = front + b.r * (b.hw * static_cast<float>(k - 1));
            }
        };
        Vector2 mid[3], nose[3];
        noseRays(at, h, mid, nose);
        // The rays follow the ground from step to step, as the car would: up a slope or a
        // flight of steps, a level ray would meet the slope's own face (a step's riser)
        // ahead and call it a wall. Where the ground falls away (a ledge), or climbs more
        // than a car does (a wall's top), they run on level.
        float level = groundZ, gz;
        if (world_.ground(was[1].x, was[1].y, level, std::max(WALL_CLIMB, me.hl * WALL_SLOPE),
                std::max(WALL_CLIMB, me.hl * WALL_SLOPE), gz))
            level = gz;
        float wasZ = level + BUMPER;
        dnav::RayHit hit;
        // How far into a wall its nose is now (m, or -1): a course that takes it no
        // deeper on its first step (backing its corner out of a kerb it scraped, turning
        // away from a post it touches) is open; any other is blocked from the start.
        float into[3];
        bool touching = false;
        for (int k = 0; k < 3; k++) {
            into[k] = -1.0f;
            if (world_.wall({ mid[k].x, mid[k].y, z }, { nose[k].x, nose[k].y, wasZ }, hit)) {
                lastWallHit_ = hit.at;
                into[k] = glm::distance(mid[k], nose[k]) * (1.0f - hit.t);
                touching = true;
            }
        }
        for (float t = WALL_STEP; t <= horizon + 1e-4f; t += WALL_STEP) {
            const float d = std::min(std::fabs(rate) * WALL_STEP, std::fabs(turn) - turned);
            if (d > 0.0f) {
                h += std::copysign(d, turn);
                turned += d;
            }
            const float step = speed * WALL_STEP;
            at += forwardOf(h) * step;
            corners(at, h, now);
            const float rise = std::max(WALL_CLIMB, step * WALL_SLOPE);
            if (world_.ground(now[1].x, now[1].y, level, rise, rise, gz)) level = gz;
            // (Off a quay into the sea is a wall too.)
            else if (level > SEA_FLOOR + 1.0f && world_.ground(now[1].x, now[1].y, level, 0.5f, FALL_REACH, gz)
                && underSea(now[1].x, now[1].y, gz))
                return std::max(0.0f, t - WALL_STEP);
            const float nowZ = level + BUMPER;
            if (touching) {
                touching = false;
                noseRays(at, h, mid, nose);
                for (int k = 0; k < 3; k++)
                    if (world_.wall({ mid[k].x, mid[k].y, z }, { nose[k].x, nose[k].y, nowZ }, hit)
                        && glm::distance(mid[k], nose[k]) * (1.0f - hit.t) > into[k] - WALL_OUT) {
                        lastWallHit_ = hit.at;
                        return 0.0f;
                    }
                // (Its corners start inside the room it keeps: the sweep goes on from here.)
                std::copy(now, now + 3, was);
                wasZ = nowZ;
                continue;
            }
            for (int k = 0; k < 3; k++)
                if (world_.wall({ was[k].x, was[k].y, wasZ }, { now[k].x, now[k].y, nowZ }, hit)) {
                    lastWallHit_ = hit.at;
                    return std::max(0.0f, t - WALL_STEP * (1.0f - hit.t));
                }
            std::copy(now, now + 3, was);
            wasZ = nowZ;
        }
        return -1.0f;
    }

    // With the world's collision: whether a car can drive straight from one point on
    // the ground to another: nothing in the way of its sides or middle at bumper
    // height, and ground all along that never climbs more than a car can. Going down,
    // any slope (a storm drain's bank) or drop (a ledge) will do: the car drives or
    // falls down it, as far as CHASE_DROP_DZ in all. The rays follow the ground from
    // sample to sample; where it falls away, they run on level at the top (a railing
    // there stops it) and the car falls past the rest.
    bool runClear(const Shape& me, const Vector3& fromGround, const Vector3& toGround) const
    {
        const Vector2 a(fromGround), b(toGround);
        const float len = glm::distance(a, b);
        if (len < 0.5f) return true;
        const Vector2 f = (b - a) / len, r(f.y, -f.x);
        dnav::RayHit hit;
        auto segment = [&](const Vector2& p, float pz, const Vector2& q, float qz) {
            for (float side : { -1.0f, 0.0f, 1.0f }) {
                const Vector2 o = r * ((me.hw + WALL_SIDE) * side);
                if (world_.wall({ p.x + o.x, p.y + o.y, pz + BUMPER }, { q.x + o.x, q.y + o.y, qz + BUMPER }, hit))
                    return false;
            }
            return true;
        };
        Vector2 last = a;
        float level = fromGround.z;
        for (float d = CHASE_GROUND_STEP;; d += CHASE_GROUND_STEP) {
            const bool end = d >= len;
            const Vector2 p = end ? b : a + f * d;
            float gz;
            if (!world_.ground(p.x, p.y, level, COL_UP, CHASE_DROP_DZ, gz) && !deckUnder(p.x, p.y, level, gz)) {
                if (!end) return false;
                gz = toGround.z;
            }
            // At the end, the first ground under the run is the target's (not a deck
            // over it).
            if (end && std::fabs(gz - toGround.z) >= CHASE_DIRECT_DZ) return false;
            if (underSea(p.x, p.y, gz) && fromGround.z > SEA_FLOOR + 1.0f) return false;
            if (gz - level > CLIMB_RISE * std::max(1.0f, glm::distance(last, p) / CHASE_GROUND_STEP)) return false;
            if (fromGround.z - gz > CHASE_DROP_DZ) return false;
            // Falling away: level at the top, then down.
            if (!segment(last, level, p, level - gz > GROUND_RISE ? level : gz)) return false;
            if (end) break;
            last = p;
            level = gz;
        }
        return true;
    }

    // Driving straight from car toward target on the car's own level: how far it goes
    // before the deck under it ends. False if it is on no deck now, or it gets to the
    // target first.
    bool edgeToward(const dnav::Point& car, const dnav::Point& target, float& at) const
    {
        float level;
        if (!deckUnder(car.x, car.y, car.z, level)) return false;
        const Vector2 from(car.x, car.y), to(target.x, target.y);
        const float len = glm::distance(from, to);
        for (float d = CHASE_EDGE_STEP; d < len; d += CHASE_EDGE_STEP) {
            const Vector2 p = from + (to - from) * (d / len);
            float gz;
            // (A step up a car cannot drive, a railing's top, a wall's, is no deck
            // either: the edge is there.)
            if (!deckUnder(p.x, p.y, level, gz) || (world_.loaded() && gz - level > STEP_UP)) {
                // Where between this sample and the last one, to a few centimetres.
                float lo = d - CHASE_EDGE_STEP, hi = d;
                for (int k = 0; k < 5; k++) {
                    const float mid = (lo + hi) * 0.5f;
                    const Vector2 q = from + (to - from) * (mid / len);
                    float qz;
                    if (deckUnder(q.x, q.y, level, qz) && !(world_.loaded() && qz - level > STEP_UP)) lo = mid;
                    else hi = mid;
                }
                at = hi;
                return true;
            }
            level = gz;
        }
        return false;
    }

    // A way off the edge of the deck the car is on to a target below: straight at it,
    // or, with the world's collision, through a gap in the railing either side of
    // where that line meets the edge (a car does not drive through a railing). A gap
    // is driven at square on: first to a point on the deck in front of it, then
    // through it. way: those points on the ground, after the car's own (the last is
    // where it aims to land); false if there is no way off.
    bool dropWay(const Shape& me, const dnav::Point& car, const Vector3& target, std::vector<Vector3>& way) const
    {
        way.clear();
        const dnav::Point tg { target.x, target.y, target.z - FOOT_Z };
        const Vector2 from(car.x, car.y), to(target.x, target.y);
        const float dist = glm::distance(from, to);
        if (dist < 1.0f) return false;
        const Vector2 dir = (to - from) / dist, side(dir.y, -dir.x);
        float lip0;
        if (!edgeToward(car, tg, lip0)) return false;
        const Vector2 l0 = from + dir * lip0;
        if (!world_.loaded()) {
            way.push_back(Vector3(l0.x, l0.y, car.z));
            way.push_back(Vector3(target.x, target.y, tg.z));
            return true;
        }
        // Which way the edge runs there: where it is a few degrees either side.
        Vector2 along = side;
        {
            Vector2 at[2];
            int found = 0;
            for (float a : { 0.12f, -0.12f }) {
                const Vector2 d(dir.x * std::cos(a) - dir.y * std::sin(a), dir.x * std::sin(a) + dir.y * std::cos(a));
                const Vector2 q = from + d * dist;
                float l;
                if (edgeToward(car, dnav::Point { q.x, q.y, tg.z }, l)) at[found++] = from + d * l;
                else break;
            }
            if (found == 2 && glm::distance(at[0], at[1]) > 0.1f) along = glm::normalize(at[0] - at[1]);
        }
        // Out over the edge: across it, the side the target is on.
        Vector2 out(along.y, -along.x);
        if (glm::dot(out, to - l0) < 0.0f) out = -out;
        dnav::RayHit hit;
        for (int i = 0; i <= CHASE_GAP_TRIES * 2; i++) {
            // 0, then +1, -1, +2, -2 ... metres along the edge.
            const float off = i == 0 ? 0.0f : (i % 2 ? 1.0f : -1.0f) * CHASE_GAP_STEP * static_cast<float>((i + 1) / 2);
            const Vector2 c = l0 + along * off;
            // Where the edge really is there, square on to it.
            const Vector2 back = c - out * CHASE_DROP_RUNUP;
            float bz;
            if (!deckUnder(back.x, back.y, car.z, bz) || std::fabs(bz - car.z) > GROUND_RISE) continue;
            float lip;
            const Vector2 beyond = c + out * (CHASE_DROP_RUNUP + 10.0f);
            if (!edgeToward(dnav::Point { back.x, back.y, bz }, dnav::Point { beyond.x, beyond.y, tg.z }, lip)) continue;
            const Vector2 edge = back + out * lip;
            // Nothing over the edge there, a car's width across.
            const Vector2 before = edge - out * CHASE_EDGE_STEP, past = edge + out * 3.0f;
            bool railed = false;
            for (float sd : { -1.0f, 0.0f, 1.0f }) {
                const Vector2 o = along * ((me.hw + WALL_SIDE) * sd);
                if (world_.wall({ before.x + o.x, before.y + o.y, bz + BUMPER }, { past.x + o.x, past.y + o.y, bz + BUMPER }, hit)) {
                    railed = true;
                    break;
                }
            }
            if (railed) continue;
            // Clear on the deck to the run-up (or to the edge, when that is where the car
            // is going anyway), and from the run-up to the edge.
            const Vector3 runup(back.x, back.y, bz);
            const bool direct = i == 0 && glm::dot(dir, out) > 0.85f;
            if (direct ? !runClear(me, Vector3(car.x, car.y, car.z), Vector3(before.x, before.y, bz))
                       : !runClear(me, Vector3(car.x, car.y, car.z), runup)
                           || !runClear(me, runup, Vector3(before.x, before.y, bz)))
                continue;
            if (!direct && glm::distance(from, back) > 2.0f) way.push_back(runup);
            way.push_back(Vector3(edge.x, edge.y, bz));
            // Where it comes down: on, the way it leaves, as far as the target is.
            const Vector2 land = direct ? to : edge + out * std::max(4.0f, glm::distance(edge, to));
            way.push_back(Vector3(land.x, land.y, tg.z));
            return true;
        }
        return false;
    }

    // With no car mesh to check a line: whether the way straight to the target runs
    // over road all along, on one level (nothing stands on a road), but for the last
    // few metres.
    bool alongDecks(const dnav::Point& car, const dnav::Point& target, float wider) const
    {
        // The car itself may be on the kerb, the target beside the road: the first and
        // last few metres need no road under them, the rest must be, never climbing or
        // falling more than a road does.
        float level = car.z, known = 0.0f;
        const Vector2 from(car.x, car.y), to(target.x, target.y);
        const float len = glm::distance(from, to);
        for (float d = CHASE_EDGE_STEP; d < len - CHASE_DECK_SLACK; d += CHASE_EDGE_STEP) {
            const Vector2 p = from + (to - from) * (d / len);
            float gz;
            const bool on = deckUnder(p.x, p.y, level, gz, wider)
                && std::fabs(gz - level) <= CHASE_DECK_RISE * std::max(1.0f, (d - known) / CHASE_EDGE_STEP);
            if (!on) {
                if (d > CHASE_DECK_SLACK) return false;
                continue;
            }
            level = gz;
            known = d;
        }
        return std::fabs(level - target.z) < CHASE_DIRECT_DZ;
    }

    // Off an edge: no grip, no steering, no engine. The car keeps its speed and heading
    // and falls until it meets a deck on the way down, or the level its target is on.
    // With the world's collision it lands on whatever is under it, and a wall it flies
    // into stops it going forward: it drops down the wall's face, never through it.
    void fly(int npcid, Mover& m, INPC& npc, const Vector3& pos, float dt)
    {
        m.vz = std::max(m.vz - GRAVITY * dt, -FALL_MAX);
        m.yawRate = 0.0f;
        const float h = glm::radians(m.heading);
        const Vector2 fwd(-std::sin(h), std::cos(h));
        float floor = m.dropZ + m.carZ, gz;
        if (world_.loaded()) {
            // What it will hit going forward this step (and its nose's length on).
            if (m.v > 0.0f && shapeHalfLength(npc) > 0.0f) {
                const float reachFwd = m.v * std::max(dt, 0.05f) * 2.0f + shapeHalfLength(npc);
                dnav::RayHit hit;
                for (float up : { -0.4f, 0.4f }) {
                    const Vector3 from(pos.x, pos.y, pos.z + up);
                    const Vector3 to(pos.x + fwd.x * reachFwd, pos.y + fwd.y * reachFwd, pos.z + up + m.vz * dt);
                    if (world_.wall(v3(from), v3(to), hit)) {
                        m.v = 0.0f;
                        break;
                    }
                }
            }
            // What it comes down on: under its middle, and, still moving on, a little
            // ahead of it (offEdge saw the drop there; under its middle can still be
            // the edge it is tipping off).
            const float on = m.v > 0.5f ? NOSE_COL : 0.0f;
            if (world_.ground(pos.x + fwd.x * on, pos.y + fwd.y * on, pos.z - m.carZ, 0.5f, FALL_REACH, gz))
                floor = gz + m.carZ;
        } else if (deckUnder(pos.x, pos.y, pos.z - m.carZ, gz) && gz + m.carZ < pos.z) {
            floor = std::max(floor, gz + m.carZ);
        }
        const float reach = std::max(AHEAD_MIN, m.v * AHEAD_TIME);
        // NPC_Move's target: as far ahead as reach takes at its speed, as far down as it
        // falls in that time; straight down once a wall has stopped it.
        const float t = m.v > 0.5f ? reach / m.v : 0.25f;
        const float fwdReach = m.v > 0.5f ? reach : 0.0f;
        Vector3 ahead(pos.x + fwd.x * fwdReach, pos.y + fwd.y * fwdReach, pos.z + m.vz * t);
        if (!world_.loaded() && deckUnder(ahead.x, ahead.y, ahead.z - m.carZ, gz) && gz + m.carZ >= ahead.z
            && gz + m.carZ < pos.z)
            floor = std::max(floor, gz + m.carZ);
        if (pos.z <= floor + LANDED) {
            // Down: it drives on from here, and its next think looks at the target again.
            ahead.z = floor;
            m.airborne = false;
            m.drop = false;
            m.direct = false;
            m.vz = 0.0f;
            m.points.clear();
            m.nextThink = TimePoint {};
        } else {
            ahead.z = std::max(ahead.z, floor);
        }
        const float speed = std::max(std::sqrt(m.v * m.v + m.vz * m.vz), 0.5f) / SPEED_UNIT;
        if (!npc.move(ahead, NPCMoveType_Drive, speed, 0.05f)) {
            end(npcid, END_INTERRUPTED, false);
            return;
        }
        uint16_t ud, lr, keys;
        npc.getKeys(ud, lr, keys);
        npc.setKeys(ud, 0, keys);
    }

    float shapeHalfLength(INPC& npc)
    {
        IVehicle* car = npc.getVehicle();
        return car ? shapeOf(car->getModel()).hl : 0.0f;
    }

    // Starts a search for npc's walk or drive; true if it was queued.
    bool goTo(AMX* amx, int npcid, dnav::PathKind kind, const Vector3& to, NPCMoveType moveType, float speed,
        float carZ, const dnav::RouteOptions& road)
    {
        INPC* npc = npcs_ ? npcs_->get(npcid) : nullptr;
        if (!npc) return false;
        Vector3 from = npc->getPosition();
        if (driving(kind)) {
            IVehicle* car = npc->getVehicle();
            if (!car || npc->getVehicleSeat() != 0) return false;   // only a driver drives
            if (dnav::isHelicopter(car->getModel())) return false;  // Nav_FlyTo
            if (carZ <= 0.0f) carZ = rideHeight(car->getModel());
            from = car->getPosition() - Vector3(0.0f, 0.0f, carZ);
        } else {
            from.z -= FOOT_Z;
        }
        // A car already on its way to somewhere else drives on to the new place without
        // stopping first: it keeps its speed and heading, and the search starts it on
        // the road ahead (weighing a turn round at that speed) instead of from a standstill.
        Mover m;
        dnav::RouteOptions how = road;
        auto was = movers_.find(npcid);
        const bool keep = driving(kind) && was != movers_.end() && driving(was->second.kind) && was->second.rolling
            && !was->second.airborne;
        if (keep) {
            const Mover& o = was->second;
            m.rolling = true;
            m.v = o.reversing ? 0.0f : o.v;
            m.heading = o.heading;
            m.yawRate = o.reversing ? 0.0f : o.yawRate;
            m.lastDrive = o.lastDrive;
            m.lastPos = o.lastPos;
            if (!how.hasHeading) {
                how.hasHeading = true;
                how.heading = o.heading;
                how.speed = m.v;
            }
        }
        // A walker already on its way walks on while the new way is found (a follower sent
        // after someone every second would otherwise stop dead each time), then turns
        // onto it from where it has got to.
        const bool walkOn = !driving(kind) && was != movers_.end() && !driving(was->second.kind)
            && !was->second.request && was->second.next < was->second.points.size();
        end(npcid, END_STOPPED, !keep && !walkOn);
        m.walkOn = walkOn;
        const Vector3 goal = driving(kind) ? to - Vector3(0.0f, 0.0f, carZ) : to - Vector3(0.0f, 0.0f, FOOT_Z);
        const int id = finder.request(amx, dnav::Point { from.x, from.y, from.z }, dnav::Point { goal.x, goal.y, goal.z },
            kind, how);
        if (!id) return false;
        m.npc = npcid;
        m.owner = amx;
        m.request = id;
        m.kind = kind;
        m.moveType = driving(kind) ? NPCMoveType_Drive : moveType;
        m.speed = speed;
        m.carZ = carZ;
        m.style = styleOf(npcid, false);
        movers_[npcid] = std::move(m);
        byRequest_[id] = npcid;
        return true;
    }

    // Ends npc's walk or drive, if it has one, and tells its script why.
    void end(int npcid, MoveEnd why, bool stopNpc)
    {
        auto pl = planes_.find(npcid);
        if (pl != planes_.end()) {
            void* owner = pl->second.owner;
            if (why == END_STOPPED && stopNpc) {
                // Nav_Stop: it circles where it is, or stops on the ground, still flown.
                dnav::PilotEnv env = planeEnv();
                pl->second.pilot.stop(env);
                pl->second.pilot.ended = -1;
                pl->second.busy = false;
            } else {
                dnav::PilotEnv env = planeEnv();
                pl->second.pilot.forget(env);
                planes_.erase(pl);
            }
            tell(owner, npcid, why);
            return;
        }
        auto fl = flyers_.find(npcid);
        if (fl != flyers_.end()) {
            void* owner = fl->second.owner;
            flyers_.erase(fl);
            // Stopped, it hangs where it is (or sits where it landed).
            if (stopNpc && npcs_) {
                if (INPC* npc = npcs_->get(npcid)) npc->setVelocity(Vector3(0.0f), false);
            }
            tell(owner, npcid, why);
            return;
        }
        auto it = movers_.find(npcid);
        if (it == movers_.end()) return;
        Mover m = std::move(it->second);
        movers_.erase(it);
        if (m.request) byRequest_.erase(m.request);
        if (m.plan) byRequest_.erase(m.plan);
        if (stopNpc && !m.request && npcs_) {
            if (INPC* npc = npcs_->get(npcid)) npc->stopMove();
        }
        if (m.owner && pawn_) {
            if (IPawnScript* script = pawn_->getScript(static_cast<AMX*>(m.owner)))
                script->Call("OnNavMoveEnd", DefaultReturnValue_False, npcid, static_cast<int>(why));
        }
    }

    bool isMoving(int npcid) const
    {
        auto pl = planes_.find(npcid);
        if (pl != planes_.end()) {
            const dnav::PlanePhase ph = pl->second.pilot.phase();
            return ph != dnav::PlanePhase::Parked && ph != dnav::PlanePhase::AtGate && ph != dnav::PlanePhase::None;
        }
        return movers_.count(npcid) != 0 || flyers_.count(npcid) != 0;
    }

    // Nav_HasLineOfSight: whether any face of world.col lies between a and b (an eye's
    // line, unlike the navmesh's lines along the ground). -1 without world.col.
    int lineOfSight(const Vector3& a, const Vector3& b) const
    {
        if (!world_.loaded()) return -1;
        dnav::RayHit hit;
        return world_.ray({ a.x, a.y, a.z }, { b.x, b.y, b.z }, hit) ? 0 : 1;
    }

    int collisionFaces() const { return static_cast<int>(world_.triangleCount()); }

    void tell(void* owner, int npcid, MoveEnd why)
    {
        if (!owner || !pawn_) return;
        if (IPawnScript* script = pawn_->getScript(static_cast<AMX*>(owner)))
            script->Call("OnNavMoveEnd", DefaultReturnValue_False, npcid, static_cast<int>(why));
    }

    // Nav_SetDriveStyle: from now on, and for the drive it is on. -1 goes back to the
    // default (normal, or pursuit in a chase).
    void setStyle(int npcid, int style)
    {
        if (style < 0) styles.erase(npcid);
        else styles[npcid] = static_cast<dnav::DriveStyle>(style);
        auto it = movers_.find(npcid);
        if (it == movers_.end() || !driving(it->second.kind)) return;
        Mover& m = it->second;
        m.style = styleOf(npcid, m.chase);
        INPC* npc = npcs_ ? npcs_->get(npcid) : nullptr;
        if (npc && npc->getVehicle() && !m.points.empty()) planSpeeds(m, *npc->getVehicle());
    }

    // A rescan of every driver's plan: speed limits or a model's handling changed.
    void replanAll()
    {
        for (auto& [id, m] : movers_) {
            if (!driving(m.kind) || m.points.empty()) continue;
            INPC* npc = npcs_ ? npcs_->get(id) : nullptr;
            if (npc && npc->getVehicle()) planSpeeds(m, *npc->getVehicle());
        }
    }

    // Nav_GetDriveInfo: how a driver is doing right now.
    bool driveInfo(int npcid, float& speed, float& want, dnav::RoadClass& cls, float& limit, bool& reversing) const
    {
        auto pl = planes_.find(npcid);
        if (pl != planes_.end()) {
            const dnav::Pilot& p = pl->second.pilot;
            speed = p.st.speed;
            want = p.st.speed;
            cls = dnav::RoadClass::Street;
            limit = p.spec.cruise;
            reversing = p.phase() == dnav::PlanePhase::Pushback;
            return true;
        }
        auto fl = flyers_.find(npcid);
        if (fl != flyers_.end()) {
            const dnav::Flight& f = fl->second.flight;
            speed = std::sqrt(f.vel.x * f.vel.x + f.vel.y * f.vel.y + f.vel.z * f.vel.z);
            want = f.want;
            cls = dnav::RoadClass::Street;
            limit = fl->second.spec.top;
            reversing = false;
            return true;
        }
        auto it = movers_.find(npcid);
        if (it == movers_.end() || !driving(it->second.kind)) return false;
        const Mover& m = it->second;
        speed = m.v;
        want = m.wantSpeed;
        cls = m.seg + 1 < m.classes.size() ? m.classes[m.seg + 1] : m.classes.empty() ? dnav::RoadClass::Street : m.classes.back();
        limit = drivingModel.limit(cls);
        reversing = m.reversing;
        return true;
    }

    void startMoving(int npcid, const dnav::PathResult& r)
    {
        auto it = movers_.find(npcid);
        if (it == movers_.end() || it->second.request != r.id) return;
        Mover& m = it->second;
        m.request = 0;
        if (r.status < dnav::PathStatus::Ok || r.points.empty()) {
            end(npcid, END_NO_PATH, false);
            return;
        }
        if (driving(m.kind)) {
            setDrivePath(npcid, m, r);
            return;
        }
        for (const auto& p : r.points) m.points.push_back(Vector3(p.x, p.y, p.z + FOOT_Z));
        // (Walking on, the first point is where it was when the way was asked for: behind it.)
        m.next = m.walkOn && m.points.size() > 1 ? 1 : 0;
        step(npcid, m);
    }

    // A driver's new path: from where the car is now along the search's points. The
    // car keeps its speed and heading.
    void setDrivePath(int npcid, Mover& m, const dnav::PathResult& r)
    {
        INPC* npc = npcs_ ? npcs_->get(npcid) : nullptr;
        IVehicle* car = npc ? npc->getVehicle() : nullptr;
        if (!car) {
            end(npcid, END_INTERRUPTED, false);
            return;
        }
        m.points.clear();
        m.speeds.clear();
        m.classes.clear();
        m.junctions.clear();
        const dnav::RoadClass first = r.classes.empty() ? dnav::RoadClass::Street : r.classes.front();
        m.points.push_back(car->getPosition());
        m.speeds.push_back(1.0f);
        m.classes.push_back(first);
        m.junctions.push_back(0);
        m.ledges.assign(1, 0);
        for (size_t i = 0; i < r.points.size(); i++) {
            const auto& p = r.points[i];
            m.points.push_back(Vector3(p.x, p.y, p.z + m.carZ));
            m.speeds.push_back(i < r.speeds.size() ? r.speeds[i] : 1.0f);
            m.classes.push_back(i < r.classes.size() ? r.classes[i] : first);
            m.junctions.push_back(i < r.junctions.size() ? r.junctions[i] : 0);
            m.ledges.push_back(i < r.ledges.size() ? r.ledges[i] : 0);
        }
        m.seg = 0;
        m.ground = r.kind == dnav::PathKind::OffRoad && finder.meshLoaded(dnav::MeshKind::Car);
        planSpeeds(m, *car);
    }

    // The speed plan for m's path in this car (driving.hpp), and the distance along it.
    void planSpeeds(Mover& m, IVehicle& car)
    {
        m.car = drivingModel.handling(car.getModel());
        if (m.boost > 1.0f) {
            // Catching up (as the game's police do): a faster engine, the same grip.
            m.car.top *= m.boost;
            for (float& a : m.car.accel) a *= m.boost;
            for (float& a : m.car.accelOff) a *= m.boost;
        }
        std::vector<dnav::Point> pts;
        pts.reserve(m.points.size());
        for (const auto& p : m.points) pts.push_back(dnav::Point { p.x, p.y, p.z });
        m.classes.resize(m.points.size(), m.classes.empty() ? dnav::RoadClass::Street : m.classes.back());
        m.junctions.resize(m.points.size(), 0);
        m.limits = drivingModel.plan(pts, m.classes, m.junctions, m.car, m.style, m.speed > 0.0f ? m.speed * SPEED_UNIT : 0.0f);
        m.cum.assign(m.points.size(), 0.0f);
        for (size_t i = 1; i < m.points.size(); i++)
            m.cum[i] = m.cum[i - 1] + glm::distance(Vector2(m.points[i - 1]), Vector2(m.points[i]));
    }

    // Chasing a moving target: no faster than lets the car brake, in the gap it keeps,
    // to the target's own speed along its way (the target may brake as hard). A target
    // ahead that turns off at a corner has none of its speed along the car's way: the
    // car brakes for the corner too, down to what its grip takes round it. (Driving at
    // the point ahead of it, it came up behind at full speed, and a target braking and
    // turning off at a junction was overshot by tens of metres.)
    float followSpeed(const Mover& m, const Vector3& pos, TimePoint now, float brake) const
    {
        if (m.quarryAt == TimePoint {}) return 1e9f;
        const float since = std::clamp(std::chrono::duration<float>(now - m.quarryAt).count(), 0.0f, 0.5f);
        const float tspeed = glm::length(m.quarryVel);
        if (tspeed < CHASE_STILL) return 1e9f;
        // (On another level, over a bridge or under it, it is not in the way.)
        if (std::fabs(m.quarryPos.z - pos.z) > CHASE_DIRECT_DZ) return 1e9f;
        const Vector2 rel = Vector2(m.quarryPos) + m.quarryVel * since - Vector2(pos);
        const Vector2 fwd = forwardOf(m.heading);
        const float ahead = glm::dot(rel, fwd), side = std::fabs(fwd.x * rel.y - fwd.y * rel.x);
        const float along = std::max(0.0f, glm::dot(m.quarryVel, fwd));
        const float react = m.v * FOLLOW_REACT;
        if (ahead > 0.0f && side < FOLLOW_LANE)
            return std::sqrt(along * along + 2.0f * brake * std::max(0.0f, ahead - FOLLOW_GAP - react));
        const bool turned = along < tspeed * std::cos(glm::radians(FOLLOW_TURNED));
        // (Off to one side of it, as chasers spread out: it may come up beside it, no
        // further.)
        if (!turned && ahead > 0.0f && glm::length(rel) < FOLLOW_NEAR)
            return std::sqrt(along * along + 2.0f * brake * ahead);
        if (turned && ahead > -FOLLOW_LANE && glm::length(rel) < FOLLOW_NEAR) {
            const float corner = std::max(along, std::sqrt(m.car.grip * std::max(FOLLOW_CORNER, m.car.radius)));
            return std::sqrt(corner * corner + 2.0f * brake * std::max(0.0f, ahead - react));
        }
        return 1e9f;
    }

    static float wrap180(float a)
    {
        while (a > 180.0f) a -= 360.0f;
        while (a < -180.0f) a += 360.0f;
        return a;
    }

    // The point `d` metres along the path from segment `seg` at fraction `t`.
    static Vector3 along(const std::vector<Vector3>& pts, size_t seg, float t, float d)
    {
        Vector3 at = pts[seg] + (pts[seg + 1] - pts[seg]) * t;
        for (size_t i = seg; i + 1 < pts.size(); i++) {
            const Vector3 to = pts[i + 1];
            const float len = glm::distance(Vector2(at), Vector2(to));
            if (d <= len) return len > 1e-4f ? at + (to - at) * (d / len) : to;
            d -= len;
            at = to;
        }
        return pts.back();
    }

    // One step of driving (DRIVE_TICK): steer toward a point down the path, no faster
    // than the car turns; ease the speed toward what the path allows; move.
    void drive(int npcid, Mover& m, TimePoint now)
    {
        INPC* npc = npcs_ ? npcs_->get(npcid) : nullptr;
        IVehicle* car = npc ? npc->getVehicle() : nullptr;
        if (!npc || !car || npc->getVehicleSeat() != 0) {
            end(npcid, END_INTERRUPTED, false);
            return;
        }
        float dt = std::chrono::duration<float>(now - m.lastDrive).count();
        m.lastDrive = now;
        const Vector3 pos = npc->getPosition();
        const Vector3 moved = pos - m.lastPos;
        m.lastPos = pos;
        if (!m.rolling) {
            m.heading = car->getRotation().ToEuler().z;
            m.v = 0.0f;
            m.yawRate = 0.0f;
            m.rolling = true;
            dt = 0.05f;
        }
        dt = std::clamp(dt, 0.0f, 0.25f);
        if (m.airborne) {
            fly(npcid, m, *npc, pos, dt);
            return;
        }

        float target = 0.0f, error = 0.0f, remaining = 0.0f;
        bool offRoad = false;
        Vector3 ahead2;
        const size_t n = m.points.size();
        // (A chaser already moving holds its speed while its first plan is worked out.)
        if (n < 2 && m.chase && m.plan) target = m.v;
        if (n >= 2) {
            // Where along the path the car is: the nearest point of the next few segments.
            size_t best = m.seg;
            float bestD = 1e30f, bestT = 0.0f;
            for (size_t i = m.seg; i + 1 < n && i < m.seg + 8; i++) {
                const Vector2 a(m.points[i]), b(m.points[i + 1]), p(pos);
                const Vector2 ab = b - a;
                const float len2 = glm::dot(ab, ab);
                const float t = len2 > 1e-6f ? std::clamp(glm::dot(p - a, ab) / len2, 0.0f, 1.0f) : 1.0f;
                // A stretch on another level (the way on, under the bridge it is on)
                // is not where it is, however near from above.
                const float dz = std::fabs(m.points[i].z + (m.points[i + 1].z - m.points[i].z) * t - pos.z);
                float d = glm::distance(p, a + ab * t) + (dz > SEG_LEVEL ? dz * 2.0f : 0.0f);
                // Already past a ledge's run-up start, in line with it and facing its
                // way: it is on the run-up, not on the way to its start.
                if (i < m.ledges.size() && m.ledges[i] && m.ledges.size() == n && t > 0.02f && d < LEDGE_LANE
                    && len2 > 1e-6f && std::fabs(wrap180(glm::degrees(std::atan2(-ab.x, ab.y)) - m.heading)) < LEDGE_FACING)
                    d = -1.0f;
                if (d < bestD) {
                    bestD = d;
                    best = i;
                    bestT = t;
                }
            }
            m.seg = best;
            m.onLedge = best < m.ledges.size() && m.ledges[best] && m.points.size() == m.ledges.size();
            const Vector3 at = m.points[best] + (m.points[best + 1] - m.points[best]) * bestT;
            const float here = m.cum[best] + (m.cum[best + 1] - m.cum[best]) * bestT;   // along the path
            remaining = m.cum[n - 1] - here;

            if (!m.chase && remaining < ARRIVE) {
                npc->stopMove();
                end(npcid, END_ARRIVED, false);
                return;
            }

            float look = std::clamp(LOOK_MIN + m.v * LOOK_TIME, LOOK_MIN, LOOK_MAX);
            Vector3 aim = along(m.points, best, bestT, look);
            // On a ledge's run-up, straight at its landing: the way on from there (often
            // back under where it went off) is for once it is down.
            if (m.onLedge) aim = m.points[best + 1];
            const float fullLook = look;
            const float fullError = wrap180(glm::degrees(std::atan2(-(aim.x - pos.x), aim.y - pos.y)) - m.heading);
            // Off the road, never aim across a corner the car would clip: shorten the
            // look until the car mesh says the straight line to it is clear.
            // (A ledge's run-up leaves the mesh at its lip: up to its start is what the mesh
            // can say, and on it, nothing.)
            // (A road route's way onto its first road, across the car mesh, too.)
            if ((m.ground || (m.classes[best + 1] == dnav::RoadClass::OffRoad && finder.meshLoaded(dnav::MeshKind::Car)))
                && !m.onLedge) {
                const dnav::Point from { pos.x, pos.y, pos.z - m.carZ };
                size_t ledge = n;
                if (m.ledges.size() == n)
                    for (size_t k = best + 1; k + 1 < n && m.cum[k] - here < look; k++)
                        if (m.ledges[k]) {
                            ledge = k;
                            break;
                        }
                auto upTo = [&]() {
                    const Vector3 q = ledge < n && m.cum[ledge] - here < look ? m.points[ledge] : aim;
                    return dnav::Point { q.x, q.y, q.z - m.carZ };
                };
                // (No nearer than its turning circle's radius: an aim a couple of metres
                // off its nose swings from side to side as it turns, and backing up for
                // it and driving on past it rocked it on the spot.)
                const float least = std::max(LOOK_MIN, m.car.radius);
                while (look > least && !finder.lineClear(from, upTo(), dnav::MeshKind::Car)) {
                    look = std::max(least, look * 0.5f);
                    aim = along(m.points, best, bestT, look);
                }
            }
            const float want = glm::degrees(std::atan2(-(aim.x - pos.x), aim.y - pos.y));
            error = wrap180(want - m.heading);

            // How fast: no faster than the speed plan allows here, nor than lets it brake
            // down to each slower point ahead (a corner, a junction, a lower limit) by the
            // time it gets there, with the share of its brakes its style plans with.
            const dnav::StyleUse& use = dnav::styleUse(m.style);
            offRoad = m.classes[best + 1] == dnav::RoadClass::OffRoad;
            const float brakePlan = (offRoad ? m.car.brakeOff : m.car.brake) * use.brake;
            // The road it is on (a point carries the class of the stretch leading to it).
            target = drivingModel.roadSpeed(m.classes[best + 1], m.car, m.style, m.speed > 0.0f ? m.speed * SPEED_UNIT : 0.0f);
            if (glm::distance(Vector2(at), Vector2(m.points[best])) < PASSED_CORNER) target = std::min(target, m.limits[best]);
            const float scan = m.v * m.v / (2.0f * brakePlan) + SCAN_EXTRA;
            for (size_t i = best + 1; i < n && m.cum[i] - here <= scan; i++)
                target = std::min(target, std::sqrt(m.limits[i] * m.limits[i] + 2.0f * brakePlan * (m.cum[i] - here)));
            // Turning toward the aim follows an arc through it: no faster than the car's
            // grip holds on that arc, but fast enough to get round.
            const float abs = std::fabs(error);
            // (The arc to the aim before a corner shortened the look, when that is the
            // wider one: steering for a near aim at a corner of the path swung the error
            // up and braked the car to a crawl at every corner across the car mesh,
            // which the speed plan already slows it for.)
            if (abs > 3.0f) {
                auto arcTo = [](float l, float e) {
                    return std::fabs(e) > 0.5f ? l / (2.0f * std::sin(glm::radians(std::min(std::fabs(e), 90.0f)))) : 1e9f;
                };
                const float arc = std::max(arcTo(look, abs), arcTo(fullLook, fullError));
                target = std::min(target, std::sqrt((offRoad ? m.car.gripOff : m.car.grip) * use.grip
                    * std::max(arc, m.car.radius)));
            }
            if (abs > 30.0f) target = std::max(target, TURN_MIN_SPEED);
            // Brake to stop at the end; chasing, down to the target's own speed (still,
            // a stop beside it; moving, keeping up with it), and ramming it, not at all.
            if (!m.chase || !(m.chaseFlags & CHASE_RAM))
                target = std::min(target, std::sqrt(m.match * m.match
                    + 2.0f * brakePlan * std::max(0.0f, remaining - ARRIVE * 0.5f)));
            if (m.chase && !(m.chaseFlags & CHASE_RAM)) target = std::min(target, followSpeed(m, pos, now, brakePlan));
            ahead2 = Vector3(aim);

            // Backing up. A car can't turn on the spot: a point behind it and near, or
            // inside its turning circle, is reached by backing up with the wheels turned
            // so the nose swings round to it (a three-point turn), not by a wide loop.
            const float abs2 = std::fabs(error);
            const Vector2 to = Vector2(aim) - Vector2(pos);
            const float d = glm::length(to);
            const float hr = glm::radians(m.heading);
            const Vector2 fwd(-std::sin(hr), std::cos(hr));
            const Vector2 left(-fwd.y, fwd.x);
            const float turnR = m.car.radius;
            const Vector2 centre = Vector2(pos) + left * (glm::dot(to, left) >= 0.0f ? turnR : -turnR);
            const bool inside = glm::distance(Vector2(aim), centre) < turnR * 0.95f;
            // open.mp's own NPC component only drives forward (our patched build backs up):
            // a car told to back up that moves forward instead means it never will.
            if (m.reversing && m.v > 1.0f) {
                if (glm::dot(Vector2(moved), fwd) > 0.02f) {
                    if (++m.wrongWay >= 3 && canReverse_) {
                        canReverse_ = false;
                        core_->printLn("[nav] this NPC component can't drive backwards, so cars turn round going forward (the patched NPCs build in npcs/ can)");
                    }
                } else {
                    m.wrongWay = 0;
                }
            }
            bool back = m.reversing;
            // Turning round where there is no room to (a narrow street, a car facing a
            // wall): with the world's collision it backs up on full lock till the wall
            // behind is near, then goes forward on full lock till the one ahead is, and
            // so on until it points near enough the way it wants.
            const Shape me = shapeOf(car->getModel());
            const float groundZ = pos.z - m.carZ;
            // (Checked over TURN_GAP and the distance it takes to stop from its speed,
            // along the arc full lock gives.)
            const float room = TURN_GAP + m.v * m.v / (2.0f * (offRoad ? m.car.brakeOff : m.car.brake));
            const float roomSpeed = room / TURN_CHECK, turnRate = glm::degrees(roomSpeed / turnR);
            auto wallNear = [&](const Shape& shape, float heading) {
                return wallHit(shape, Vector2(pos), heading, std::copysign(90.0f, error), turnRate, roomSpeed,
                           TURN_CHECK, groundZ) >= 0.0f;
            };
            auto rearBlocked = [&]() {
                Shape rear = me;
                rear.ox = -rear.ox;
                rear.oy = -rear.oy;
                return wallNear(rear, m.heading + 180.0f);
            };
            // How far it can still go on full lock, ahead or backing up, before a wall
            // (up to TURN_LOOK): a leg of turning round uses all of it, slowing to stop
            // TURN_GAP short, instead of running up to speed and stopping a metre in.
            auto freeOnLock = [&](bool rear) {
                if (!world_.loaded()) return TURN_LOOK;
                Shape s = me;
                float h = m.heading;
                if (rear) {
                    s.ox = -s.ox;
                    s.oy = -s.oy;
                    h += 180.0f;
                }
                const float t = wallHit(s, Vector2(pos), h, std::copysign(90.0f, error), glm::degrees(1.0f / turnR),
                    1.0f, TURN_LOOK, groundZ);
                return t < 0.0f ? TURN_LOOK : t;
            };
            auto legSpeed = [&](float free) {
                const float brake = (offRoad ? m.car.brakeOff : m.car.brake) * 0.7f;
                return std::sqrt(2.0f * brake * std::max(0.0f, free - TURN_GAP));
            };
            auto loopClear = [&]() {
                const float v = std::max(TURN_MIN_SPEED, std::min(m.v, std::sqrt(m.car.grip * 0.8f * turnR)));
                const float r = std::max(turnR, v * v / (m.car.grip * 0.95f));
                const float side = std::copysign(1.0f, error);
                // (Only as far round as it still has to go.)
                const float round = std::min(180.0f, abs2 + 20.0f);
                if (wallHit(me, Vector2(pos), m.heading, side * round, glm::degrees(v / r), v,
                        glm::radians(round) * r / v + WALL_STEP, groundZ) >= 0.0f)
                    return false;
                // (Ground all the way round, near its own level: not off a ledge or down a bank.)
                const float h = glm::radians(m.heading);
                const Vector2 fwd(-std::sin(h), std::cos(h)), left(-fwd.y, fwd.x);
                const Vector2 centre = Vector2(pos) + left * (side * r);
                for (int k = 1; k <= 4; k++) {
                    const float a = glm::radians(round * k / 4.0f) * side;
                    const Vector2 off = Vector2(pos) - centre;
                    const Vector2 p = centre + Vector2(off.x * std::cos(a) - off.y * std::sin(a), off.x * std::sin(a) + off.y * std::cos(a));
                    float gz;
                    if (!world_.ground(p.x, p.y, groundZ, LOOP_LEVEL, LOOP_LEVEL, gz)) return false;
                }
                return true;
            };
            // Backed up far enough: from here it can swing round to the aim going forward
            // without meeting a wall (a pillar by its nose). Backing up only till the aim
            // was in front sent it forward into the same post, and back, and forward.
            // (Looked at as far as avoid() looks pulling away: swung round and on toward
            // the aim. Only as far as facing it, a building's corner just past a turn in
            // the way was out of sight, and pulling away avoid() swerved off from it the
            // other way, undoing the turn, back and forth for as long as it was let.)
            auto forwardClear = [&]() {
                if (!world_.loaded()) return true;
                const float v = TURN_MIN_SPEED;
                return wallHit(me, Vector2(pos), m.heading, error, glm::degrees(v / turnR), v,
                           std::max(glm::radians(abs2) * turnR / v + WALL_STEP, AVOID_HORIZON_MIN), groundZ) < 0.0f;
            };
            if (abs2 < REVERSE_DONE) m.turning = false;
            if (m.backOff) {
                back = m.backed < AVOID_BACK;
                if (!back) m.backOff = false;
                else error = m.backSteer;           // the nose swings to the side it passes on
            } else if (!m.reversing) {
                if (abs2 > CIRCLING_TURN && m.v > 1.0f) m.circling += dt;
                else if (abs2 < CIRCLING_TURN * 0.5f) m.circling = 0.0f;
                back = (abs2 > REVERSE_BEHIND && d < REVERSE_NEAR * turnR) || (abs2 > REVERSE_INSIDE && inside)
                    || m.circling > CIRCLING_SECS;
                // With room for it (no wall, ground all round), it swings round forward
                // in one go on full lock, as a driver in a hurry would, keeping some
                // of its speed, instead of stopping to back up.
                if (back && !m.turning && m.circling <= CIRCLING_SECS && world_.loaded() && loopClear())
                    back = false;
                // Its forward leg of turning round goes on till the wall ahead is near.
                if (m.turning)
                    back = freeOnLock(false) < TURN_GAP + TURN_END;
            } else if ((abs2 < REVERSE_DONE && forwardClear()) || m.backed > REVERSE_MAX) {
                back = false;
            } else if (m.rearHit || (world_.loaded() && m.v > 0.5f && freeOnLock(true) < TURN_GAP + TURN_END)) {
                // (Until it has stopped and changed direction: stopped, nothing is behind
                // it at its speed, and it would only back on into the wall.)
                back = false;
                m.turning = true;
                m.rearHit = true;
            }
            if (!canReverse_) back = false;
            // Never back into a wall: the world's collision, else the car mesh.
            if (back && !m.reversing && !m.backOff) {
                if (world_.loaded()) {
                    if (rearBlocked()) {
                        back = false;
                        m.turning = abs2 > REVERSE_DONE;
                    }
                } else {
                    float gz;
                    const dnav::Point from { pos.x, pos.y, pos.z - m.carZ };
                    const Vector2 rear = Vector2(pos) - fwd * REVERSE_CHECK;
                    if (finder.groundAt(from.x, from.y, from.z, GROUND_REACH, gz, dnav::MeshKind::Car)
                        && !finder.lineClear(from, dnav::Point { rear.x, rear.y, from.z }, dnav::MeshKind::Car))
                        back = false;
                }
            }
            if (back != m.reversing) {
                // Stop before changing direction.
                target = 0.0f;
                if (m.v < 0.5f) {
                    m.reversing = back;
                    m.rearHit = false;
                    m.backed = 0.0f;
                    m.circling = 0.0f;
                }
            } else if (m.reversing) {
                target = std::min(std::min(REVERSE_SPEED, m.car.reverse), std::max(target, TURN_MIN_SPEED));
                if (!m.backOff) target = std::min(target, std::max(TURN_CREEP, legSpeed(freeOnLock(true))));
            }
            if (m.turning && !m.reversing) {
                // On full lock toward the way it wants, slowly: the wall ahead sends it back.
                error = std::copysign(90.0f, error);
                target = std::min(target, std::max(TURN_CREEP, std::min(TURN_MIN_SPEED, legSpeed(freeOnLock(false)))));
                // Still braking down from speed (a replan put the way behind it), it
                // slides on much wider than full lock: it still looks out for the wall
                // it is sliding at, as it does driving on, instead of going through it.
                if (m.v > TURN_MIN_SPEED * 1.5f) avoid(m, *car, pos, offRoad, dt, error, target);
            } else if (m.nudge > 0.0f && !m.reversing) {
                // Boxed in front and behind: it edges forward through it (a post, a
                // railing's end), as a car would shove past, rather than stay there.
                m.nudge -= dt;
                target = std::min(target, TURN_MIN_SPEED);
            } else if (!m.reversing) avoid(m, *car, pos, offRoad, dt, error, target);
            else if (m.backOff) backOffClear(m, *car, pos);
            // Not getting anywhere (wedged against a post, nose in a corner, going
            // back and forth): it backs out, straight or on either lock, and looks for
            // its way again; with no room behind either, it edges forward.
            const bool wants = !m.idle && !(m.chase && m.quarryDist < CHASE_CLOSE * 2.0f);
            if (wants && glm::distance(Vector2(pos), Vector2(m.stuckAt)) < STUCK_MOVE) {
                m.stuckFor += dt;
            } else {
                m.stuckFor = 0.0f;
                m.stuckAt = pos;
            }
            if (m.stuckFor > STUCK_SECS && canReverse_) {
                m.stuckFor = 0.0f;
                m.stuckAt = pos;
                m.turning = false;
                m.reversing = false;
                m.rearHit = false;
                m.backed = 0.0f;
                // The side it was passing on, else the way it wants; stuck again in the
                // same place, the other side.
                float side = m.avoidSide != 0.0f ? m.avoidSide : (error >= 0.0f ? 1.0f : -1.0f);
                if (m.rescueSide != 0.0f && glm::distance(Vector2(pos), Vector2(m.rescueAt)) < STUCK_SAME)
                    side = -m.rescueSide;
                m.rescueAt = pos;
                m.rescueSide = side;
                m.avoidSide = side;
                m.avoidFor = AVOID_STICK + 5.0f;
                m.backSteer = m.avoidSide * 45.0f;
                m.backOff = true;
                backOffClear(m, *car, pos);
                if (!m.backOff) m.nudge = NUDGE_SECS;
                if (m.chase) m.nextThink = TimePoint {};
            }
            // A straight run at the target with a wall in the way and no way round it:
            // the next think finds a way round instead, and keeps to it a while.
            if (m.chase && m.direct && m.walled && !m.drop) {
                m.wallHold = CHASE_WALL_HOLD;
                m.nextThink = TimePoint {};
            }
        }
        m.wallHold = std::max(0.0f, m.wallHold - dt);

        // Pulling away follows the car's own acceleration at this speed (its gears, its
        // engine, the air, what its tyres put down on this ground), the style's share of
        // it; braking is as hard as the car can when it must (the plan itself only
        // counts on its style's share).
        m.wantSpeed = target;
        if (target > m.v) {
            const float pull = m.car.accelAt(m.v, offRoad) * dnav::styleUse(m.style).accel;
            m.v = std::min(target, m.v + pull * dt);
        } else {
            m.v = std::max(target, m.v - (offRoad ? m.car.brakeOff : m.car.brake) * dt);
        }
        m.v = std::max(0.0f, m.v);
        if (m.v < 0.3f && target < 0.3f) {
            m.v = 0.0f;
            if (npc->isMoving()) npc->stopMove();
            return;
        }
        // It turns no tighter than its turning circle, nor, at speed, than its grip holds,
        // and it gets into a turn (and out of it) only as fast as its front tyres can
        // swing its weight round (its turn mass): the turn rate builds up and dies down,
        // easing off in time to stop on the heading it wants. Backing up, the nose still
        // swings toward the aim: the wheels turn the other way.
        const float turning = std::max(m.car.radius, m.v * m.v / ((offRoad ? m.car.gripOff : m.car.grip) * 0.95f));
        const float maxYaw = glm::degrees(std::max(m.v, TURN_MIN_SPEED * 0.5f) / turning);
        const float yawAccel = glm::degrees(m.car.yaw);
        float wantRate = std::copysign(std::min(maxYaw, std::sqrt(2.0f * yawAccel * std::fabs(error))), error);
        if (dt > 0.0f) wantRate = std::clamp(wantRate, -std::fabs(error) / dt, std::fabs(error) / dt);
        m.yawRate += std::clamp(wantRate - m.yawRate, -yawAccel * dt, yawAccel * dt);
        m.yawRate = std::clamp(m.yawRate, -maxYaw, maxYaw);
        m.heading = wrap180(m.heading + m.yawRate * dt);
        if (m.reversing) m.backed += m.v * dt;

        // NPC_Move's target: straight ahead on the new heading. Its height carries on the
        // slope of the ground a short way ahead (NEAR_TIME), not the line to the ground at
        // the target: the car climbs from the point it is at toward the target's height, so
        // aimed at the target's own ground it cut under every crest and over every dip by
        // up to a metre on the hills.
        const float reach = std::max(AHEAD_MIN, m.v * AHEAD_TIME);
        const float nearBy = std::min(reach, std::max(NEAR_MIN, m.v * NEAR_TIME));
        const float h = glm::radians(m.heading);
        const Vector2 fwd(-std::sin(h), std::cos(h));
        Vector3 ahead(pos.x + fwd.x * reach, pos.y + fwd.y * reach, pos.z);
        if (m.drop && m.direct && !m.reversing) {
            // Running at an edge: on the deck's own level until the nose is past it,
            // then off it.
            // (With the world's collision, once its middle is: it tips over the edge
            // there, and lands on what is under its middle.)
            float gz;
            const Vector2 nose = Vector2(pos) + fwd * (world_.loaded() ? NOSE_COL : NOSE);
            if (!deckUnder(nose.x, nose.y, pos.z - m.carZ, gz)) {
                m.airborne = true;
                m.vz = 0.0f;
                fly(npcid, m, *npc, pos, dt);
                return;
            }
        } else if (m.chase && world_.loaded() && !m.reversing && m.v > 2.0f && offEdge(m, pos, h)) {
            // Chasing, it went over an edge (a ledge, a bank's lip at speed) on its way:
            // it falls, as a car would, instead of floating on at the height it was.
            m.dropZ = pos.z - m.carZ - FALL_REACH;
            m.airborne = true;
            m.vz = 0.0f;
            fly(npcid, m, *npc, pos, dt);
            return;
        }
        // The car's height over a point `d` along the heading (behind it backing up): the
        // path's height there, or off the roads' own heights (a straight run at a target, or
        // no path left to follow) the ground under it: a car never climbs onto something it
        // has no ground for, nor floats on at the height it was.
        auto heightAt = [&](float d) -> float {
            const Vector2 at = Vector2(pos) + fwd * (m.reversing ? -d : d);
            float z = pos.z, gz;
            if (n >= 2 && !m.reversing)
                z = along(m.points, m.seg, 0.0f, glm::distance(Vector2(m.points[m.seg]), Vector2(pos)) + d).z;
            if (m.reversing) {
                if (world_.loaded() && world_.ground(at.x, at.y, pos.z - m.carZ, COL_UP, COL_DOWN, gz)) return gz + m.carZ;
                if (finder.groundAt(at.x, at.y, pos.z - m.carZ, GROUND_REACH, gz, dnav::MeshKind::Car)) return gz + m.carZ;
                return pos.z;
            }
            if (m.drop && m.direct) return deckUnder(at.x, at.y, pos.z - m.carZ, gz) ? gz + m.carZ : pos.z;
            if (m.chase && world_.loaded()) {
                // Chasing, the ground ahead from no lower than the car's own level, and no
                // steeper down than a bank: an edge on the way (a drop off a ledge the car
                // mesh has, a kerb's lip) is driven at level, and offEdge tips it over once
                // its middle is past, instead of it sinking into the deck short of it.
                const float level = pos.z - m.carZ, from = std::max(z - m.carZ, level);
                return world_.ground(at.x, at.y, from, COL_UP, from - level + d * BANK_SLOPE + 0.5f, gz) ? gz + m.carZ : pos.z;
            }
            if (world_.loaded() && world_.ground(at.x, at.y, z - m.carZ, COL_UP, COL_DOWN, gz))
                return gz + m.carZ;     // the ground itself, wherever the car is going
            if (m.ground && finder.groundAt(at.x, at.y, z - m.carZ, GROUND_REACH, gz, dnav::MeshKind::Car))
                return gz + m.carZ;
            if ((m.direct || n < 2) && groundUnder(at.x, at.y, z - m.carZ, gz)) return gz + m.carZ;
            return z;
        };
        ahead.z = pos.z + (heightAt(nearBy) - pos.z) * reach / nearBy;
        float moveSpeed = std::max(m.v, 0.5f) / SPEED_UNIT;
        if (m.reversing) {
            // NPC_Move faces its target and a negative speed moves away from it, so
            // backing toward a point behind is a move to its mirror image ahead.
            const Vector3 behind(pos.x - fwd.x * reach, pos.y - fwd.y * reach, ahead.z);
            ahead = pos * 2.0f - behind;
            moveSpeed = -moveSpeed;
        }
        if (!npc->move(ahead, NPCMoveType_Drive, moveSpeed, 0.05f)) {
            end(npcid, END_INTERRUPTED, false);
            return;
        }
        // The front wheels show the turn (GTA: left is negative); backing up, they
        // turn the other way, and the car holds the brake/reverse key.
        uint16_t ud, lr, keys;
        npc->getKeys(ud, lr, keys);
        int steer = std::clamp(static_cast<int>(-error * 3.0f), -128, 128);
        if (m.reversing) {
            steer = -steer;
            keys = (keys & ~static_cast<uint16_t>(Key::SPRINT)) | static_cast<uint16_t>(Key::JUMP);
        }
        npc->setKeys(ud, static_cast<uint16_t>(static_cast<int16_t>(steer)), keys);
    }

    // Sends npc to its next point, or ends the move there.
    void step(int npcid, Mover& m)
    {
        INPC* npc = npcs_ ? npcs_->get(npcid) : nullptr;
        if (!npc) {
            end(npcid, END_INTERRUPTED, false);
            return;
        }
        if (driving(m.kind) && (!npc->getVehicle() || npc->getVehicleSeat() != 0)) {
            end(npcid, END_INTERRUPTED, false);
            return;
        }
        if (m.next >= m.points.size()) {
            // A chase never arrives: the next think plans again or drives straight on.
            if (m.chase) {
                m.points.clear();
                m.nextThink = TimePoint {};
                return;
            }
            end(npcid, END_ARRIVED, false);
            return;
        }
        m.target = m.points[m.next];
        float speed = m.speed;
        float stop = WALK_STOP;
        if (driving(m.kind)) {
            const float factor = m.next < m.speeds.size() ? m.speeds[m.next] : 1.0f;
            speed = std::max(DRIVE_MIN_SPEED, m.speed * factor);
            stop = DRIVE_STOP;
        }
        if (!npc->move(m.target, m.moveType, speed, stop)) end(npcid, END_INTERRUPTED, false);
    }

    void onNPCFinishMove(INPC& npc) override
    {
        const int id = npc.getID();
        auto it = movers_.find(id);
        if (it == movers_.end() || it->second.request) return;
        Mover& m = it->second;
        // A driver is steered by drive(), which gives it its next move.
        if (driving(m.kind)) return;
        // Finished a move of ours, or one some other script started?
        const Vector3 at = driving(m.kind) && npc.getVehicle() ? npc.getVehicle()->getPosition() : npc.getPosition();
        const float slack = driving(m.kind) ? DRIVE_STOP + 3.0f : WALK_STOP + 1.5f;
        if (!m.chase && glm::distance(at, m.target) > slack) {
            end(id, END_INTERRUPTED, false);
            return;
        }
        m.next++;
        step(id, m);
    }

    // ── Nav_Chase ────────────────────────────────────────────────────────────

    bool chase(AMX* amx, int npcid, int quarry, float speed, float carZ, int flags)
    {
        if (isPlaneNpc(npcid)) {
            if (!core_->getPlayers().get(quarry)) return false;
            return planeOrder(amx, npcid, [&](Plane& p, dnav::PilotEnv& env) {
                return p.pilot.chase(env, quarry, speed > 0.0f ? speed * SPEED_UNIT : 0.0f, carZ > 0.0f ? carZ : 0.0f);
            });
        }
        INPC* npc = npcs_ ? npcs_->get(npcid) : nullptr;
        if (!npc || !npc->getVehicle() || npc->getVehicleSeat() != 0) return false;
        if (carZ <= 0.0f) carZ = rideHeight(npc->getVehicle()->getModel());
        IPlayer* target = core_->getPlayers().get(quarry);
        if (!target || target->getID() == npc->getID()) return false;
        if (dnav::isHelicopter(npc->getVehicle()->getModel())) {
            Vector3 tpos, tvel;
            whereIs(*target, tpos, tvel);
            Flyer& f = startFlying(amx, *npc, speed);
            f.quarry = quarry;
            f.angle = dnav::angleFrom(dv(tpos), f.flight.pos);
            return true;
        }
        // A car already driving somewhere gives chase at the speed it has: it does not
        // stop dead first (its first plan weighs turning round against carrying on).
        Mover m;
        auto was = movers_.find(npcid);
        const bool keep = was != movers_.end() && driving(was->second.kind) && was->second.rolling
            && !was->second.airborne;
        if (keep) {
            const Mover& o = was->second;
            m.rolling = true;
            m.v = o.reversing ? 0.0f : o.v;
            m.heading = o.heading;
            m.yawRate = o.reversing ? 0.0f : o.yawRate;
            m.lastDrive = o.lastDrive;
            m.lastPos = o.lastPos;
        }
        end(npcid, END_STOPPED, !keep);
        m.npc = npcid;
        m.owner = amx;
        m.kind = dnav::PathKind::Drive;
        m.moveType = NPCMoveType_Drive;
        m.speed = speed;
        m.carZ = carZ;
        m.chase = true;
        m.quarry = quarry;
        m.chaseFlags = flags;
        m.style = styleOf(npcid, true);
        movers_[npcid] = std::move(m);
        return true;
    }

    bool isPlaneNpc(int npcid) const
    {
        INPC* npc = npcs_ ? npcs_->get(npcid) : nullptr;
        return npc && npc->getVehicle() && npc->getVehicleSeat() == 0 && dnav::isPlane(npc->getVehicle()->getModel());
    }

    // ── Helicopters ──────────────────────────────────────────────────────────

    static dnav::Vec3 dv(const Vector3& v) { return { v.x, v.y, v.z }; }

    // What a flight knows of the world: the highest face at (x, y) (never under the
    // sea's surface), and rays. With no world.col it only keeps over its goal's height.
    dnav::FlyWorld flyWorld() const
    {
        dnav::FlyWorld w;
        if (!world_.loaded()) return w;
        w.top = [this](float x, float y) {
            dnav::RayHit hit;
            if (world_.ray({ x, y, 1500.0f }, { x, y, -200.0f }, hit)) return std::max(hit.at.z, HELI_SEA);
            return HELI_SEA;
        };
        w.ray = [this](const dnav::Vec3& a, const dnav::Vec3& b, float& t) {
            dnav::RayHit hit;
            if (!world_.ray(a, b, hit)) return false;
            t = hit.t;
            return true;
        };
        return w;
    }

    // An NPC in a helicopter takes off (or flies on) under DiamondNav: whatever it was
    // doing ends, and its flight starts from where the helicopter is, as it is moving.
    Flyer& startFlying(AMX* amx, INPC& npc, float speed)
    {
        IVehicle& heli = *npc.getVehicle();
        const int id = npc.getID();
        Flyer f;
        auto was = flyers_.find(id);
        const bool flying = was != flyers_.end();
        if (flying) f.flight = was->second.flight;
        end(id, END_STOPPED, false);
        if (npc.isMoving()) npc.stopMove();
        f.owner = amx;
        f.spec = dnav::HeliSpec::of(heli.getModel());
        f.placed = heli.getPosition();
        if (!flying || glm::distance(f.placed, Vector3(f.flight.pos.x, f.flight.pos.y, f.flight.pos.z)) > HELI_MOVED) {
            f.flight = dnav::Flight {};
            f.flight.pos = dv(f.placed);
            f.flight.vel = dv(heli.getVelocity() * VELOCITY_UNIT);
            f.flight.heading = heli.getRotation().ToEuler().z;
        }
        f.flight.planIn = 0.0f;
        f.goal.speed = speed > 0.0f ? speed * SPEED_UNIT : 0.0f;
        f.last = TimePoint {};
        return flyers_[id] = f;
    }

    bool flyTo(AMX* amx, int npcid, const Vector3& to, float speed, int flags)
    {
        if (isPlaneNpc(npcid))
            return planeOrder(amx, npcid, [&](Plane& p, dnav::PilotEnv& env) {
                return p.pilot.fly(env, { dv(to) }, speed > 0.0f ? speed * SPEED_UNIT : 0.0f, false);
            });
        INPC* npc = npcs_ ? npcs_->get(npcid) : nullptr;
        if (!npc || !npc->getVehicle() || npc->getVehicleSeat() != 0) return false;
        if (!dnav::isHelicopter(npc->getVehicle()->getModel())) return false;
        Flyer& f = startFlying(amx, *npc, speed);
        f.goal.at = dv(to);
        f.goal.land = (flags & FLY_LAND) != 0;
        return true;
    }

    void flyStep(int npcid, Flyer& f, TimePoint now)
    {
        INPC* npc = npcs_ ? npcs_->get(npcid) : nullptr;
        IVehicle* heli = npc ? npc->getVehicle() : nullptr;
        if (!npc || !heli || npc->getVehicleSeat() != 0) {
            end(npcid, END_INTERRUPTED, false);
            return;
        }
        const float dt = f.last == TimePoint {} ? 0.0f
            : std::min(HELI_DT_MAX, duration_cast<Microseconds>(now - f.last).count() / 1e6f);
        f.last = now;
        // Put somewhere else (SetVehiclePos): it flies on from there.
        const Vector3 at = heli->getPosition();
        if (glm::distance(at, f.placed) > HELI_MOVED) {
            f.flight.pos = dv(at);
            f.flight.vel = {};
            f.flight.planIn = 0.0f;
        }
        if (f.quarry >= 0) {
            IPlayer* target = core_->getPlayers().get(f.quarry);
            if (!target || target->getVirtualWorld() != npc->getVirtualWorld()) {
                end(npcid, END_LOST, true);
                return;
            }
            Vector3 tpos, tvel;
            whereIs(*target, tpos, tvel);
            f.quarryDist = glm::distance(tpos, at);
            const float speed = f.goal.speed;
            const Vector3 v = tvel * VELOCITY_UNIT;
            const float tspeed = glm::length(Vector2(v));
            f.trailing = tspeed > (f.trailing ? HELI_TRAIL_TO : HELI_TRAIL_FROM);
            if (f.trailing) {
                f.goal = dnav::trail(dv(tpos), dv(v), HELI_TRAIL, HELI_CHASE_HEIGHT);
                f.angle = dnav::angleFrom(dv(tpos), f.flight.pos);    // circles on from where it is
            } else {
                f.goal = dnav::orbit(dv(tpos), dv(v), HELI_CHASE_RADIUS, HELI_CHASE_HEIGHT, HELI_CHASE_RATE,
                    f.angle, true, dt);
            }
            f.goal.speed = speed;
            f.goal.ceiling = tpos.z + HELI_CHASE_CEILING;
        }
        if (dt <= 0.0f) return;
        const bool there = dnav::fly(f.flight, f.goal, f.spec, flyWorld(), dt);
        const dnav::Flight& fl = f.flight;
        f.placed = Vector3(fl.pos.x, fl.pos.y, fl.pos.z);
        // Not an NPC move: its position, rotation and velocity are set as they are, and
        // its driver sync carries them (the velocity lets everyone's game carry it on
        // smoothly between syncs).
        npc->setVelocity(Vector3(fl.vel.x, fl.vel.y, fl.vel.z) / VELOCITY_UNIT, false);
        npc->setVehiclePosition(f.placed, false);
        npc->setVehicleRotation(GTAQuat(Vector3(fl.pitch, fl.roll, fl.heading)), false);
        if (there && f.quarry < 0) end(npcid, END_ARRIVED, true);
    }

    // ── Planes ───────────────────────────────────────────────────────────────

    // What a plane knows of the world: the highest face at (x, y) (never under the
    // sea), kept by cell, since planes look a long way ahead every half second.
    dnav::FlyWorld planeWorld()
    {
        dnav::FlyWorld w;
        if (!world_.loaded()) return w;
        w.top = [this](float x, float y) {
            const long long kx = static_cast<long long>(std::floor(x / PLANE_TOP_CELL));
            const long long ky = static_cast<long long>(std::floor(y / PLANE_TOP_CELL));
            const long long key = (kx << 32) ^ (ky & 0xffffffffLL);
            auto it = tops_.find(key);
            if (it != tops_.end()) return it->second;
            if (tops_.size() > PLANE_TOP_KEEP) tops_.clear();
            const float cx = (static_cast<float>(kx) + 0.5f) * PLANE_TOP_CELL;
            const float cy = (static_cast<float>(ky) + 0.5f) * PLANE_TOP_CELL;
            dnav::RayHit hit;
            float t = HELI_SEA;
            if (world_.ray({ cx, cy, 1500.0f }, { cx, cy, -200.0f }, hit)) t = std::max(hit.at.z, HELI_SEA);
            tops_[key] = t;
            return t;
        };
        w.ray = [this](const dnav::Vec3& a, const dnav::Vec3& b, float& t) {
            dnav::RayHit hit;
            if (!world_.ray(a, b, hit)) return false;
            t = hit.t;
            return true;
        };
        return w;
    }

    dnav::PilotEnv planeEnv()
    {
        dnav::PilotEnv env;
        env.db = &airports_;
        env.atc = &atc_;
        env.world = planeWorld();
        env.now = clock_;
        for (const auto& [id, p] : planes_) env.others.push_back(&p.pilot);
        env.target = [this](int who, dnav::Vec3& pos, dnav::Vec3& vel) {
            IPlayer* t = core_->getPlayers().get(who);
            if (!t) return false;
            Vector3 tp, tv;
            whereIs(*t, tp, tv);
            pos = dv(tp);
            vel = dv(tv * VELOCITY_UNIT);
            return true;
        };
        return env;
    }

    // A model's size from vehicles.dnv (its collision, wings and all), where it has one
    // that looks like the plane's; its ride height from its wheels.
    void sizePlane(dnav::PlaneSpec& spec, int model) const
    {
        if (const dnav::Handling* h = drivingModel.fromFile(model)) {
            if (h->width > spec.span * 0.6f && h->width < spec.span * 1.6f) spec.span = h->width;
            if (h->length > spec.length * 0.6f && h->length < spec.length * 1.6f) spec.length = h->length;
            if (h->ride > 0.3f) spec.ride = h->ride;
        }
    }

    // A plane put somewhere (or taken over where it is): on the ground if its wheels
    // are, parked at a gate if it stands on one, else flying on as it was going.
    void placePlane(Plane& p, IVehicle& v)
    {
        dnav::PilotEnv env = planeEnv();
        const Vector3 pos = v.getPosition();
        const float heading = v.getRotation().ToEuler().z;
        float ground = 0.0f;
        const bool onGround = groundUnder(pos.x, pos.y, pos.z, ground) && pos.z - ground < p.pilot.spec.ride + PLANE_GROUND;
        const float speed = glm::length(v.getVelocity()) * VELOCITY_UNIT;
        p.pilot.place(env, dv(pos), heading, onGround, speed);
        p.placed = pos;
        if (!onGround) return;
        const int ap = airports_.at(dv(pos), 50.0f);
        const dnav::Airport* a = airports_.get(ap);
        for (size_t g = 0; a && g < a->gates.size(); g++) {
            const dnav::Gate& gt = a->gates[g];
            if (gt.removed || glm::distance(Vector2(pos), Vector2(gt.p.x, gt.p.y)) > 6.0f) continue;
            if (std::fabs(dnav::wrapDeg(gt.heading - heading)) > 30.0f) continue;
            if (p.pilot.park(env, ap, static_cast<int>(g))) {
                const dnav::Vec3& at = p.pilot.st.pos;
                p.placed = Vector3(at.x, at.y, at.z);
            }
            break;
        }
    }

    // The pilot of the plane npc flies (made the first time it is given an order).
    Plane* planeOf(AMX* amx, int npcid)
    {
        INPC* npc = npcs_ ? npcs_->get(npcid) : nullptr;
        IVehicle* v = npc ? npc->getVehicle() : nullptr;
        if (!npc || !v || npc->getVehicleSeat() != 0 || !dnav::isPlane(v->getModel())) return nullptr;
        auto it = planes_.find(npcid);
        if (it == planes_.end()) {
            end(npcid, END_STOPPED, false);
            if (npc->isMoving()) npc->stopMove();
            it = planes_.emplace(npcid, Plane(npcid, v->getModel())).first;
            sizePlane(it->second.pilot.spec, v->getModel());
            placePlane(it->second, *v);
        } else if (glm::distance(v->getPosition(), it->second.placed) > PLANE_MOVED) {
            placePlane(it->second, *v);
        }
        return &it->second;
    }

    // Gives the plane npc an order (`order` says false, with the pilot's why, if it
    // cannot). An order it was on ends STOPPED.
    template <typename F>
    bool planeOrder(AMX* amx, int npcid, F order)
    {
        Plane* p = planeOf(amx, npcid);
        if (!p) return false;
        dnav::PilotEnv env = planeEnv();
        const bool was = p->busy;
        void* wasOwner = p->owner;
        p->pilot.why.clear();
        if (!order(*p, env)) {
            core_->printLn("[nav] plane %d: %s", npcid, p->pilot.why.c_str());
            return false;
        }
        p->owner = amx;
        p->busy = true;
        if (was) tell(wasOwner, npcid, END_STOPPED);
        return true;
    }

    // What stands on and round the airports that no pilot flies: a car on a runway
    // closes it, anything on a taxiway stops planes short of it.
    void seeAirports()
    {
        obstacles_.clear();
        if (!vehicles_ || airports_.liveCount() == 0) return;
        std::unordered_map<int, bool> flown;
        for (const auto& [id, p] : planes_) {
            INPC* npc = npcs_ ? npcs_->get(id) : nullptr;
            if (npc && npc->getVehicle()) flown[npc->getVehicle()->getID()] = true;
        }
        for (IVehicle* v : *vehicles_) {
            if (flown.count(v->getID())) continue;
            const Vector3 pos = v->getPosition();
            const int ap = airports_.at(dv(pos), 100.0f);
            const dnav::Airport* a = airports_.get(ap);
            if (!a || pos.z > a->hi.z + 15.0f) continue;
            dnav::Obstacle o;
            o.pos = dv(pos);
            o.vel = dv(v->getVelocity() * VELOCITY_UNIT);
            o.vehicle = v->getID();
            const int model = v->getModel();
            o.radius = dnav::isPlane(model) ? dnav::PlaneSpec::of(model).span * 0.5f : std::max(shapeOf(model).hl, shapeOf(model).hw);
            // Its centre over the ground, as the pilots' poses are on the ground.
            o.pos.z -= rideHeight(model);
            obstacles_.push_back(o);
        }
    }

    void callPhase(void* owner, int npcid, dnav::PlanePhase phase)
    {
        if (!owner || !pawn_) return;
        if (IPawnScript* script = pawn_->getScript(static_cast<AMX*>(owner)))
            script->Call("OnNavPlanePhase", DefaultReturnValue_False, npcid, static_cast<int>(phase));
    }

    // Every plane one step on, in id order (ATC answers whoever asks first).
    void planesStep(TimePoint now)
    {
        const float dt = planeLast_ == TimePoint {} ? 0.0f
            : std::min(PLANE_DT_MAX, duration_cast<Microseconds>(now - planeLast_).count() / 1e6f);
        planeLast_ = now;
        if (dt <= 0.0f) return;
        clock_ += dt;
        seeIn_ -= dt;
        if (seeIn_ <= 0.0f) {
            seeIn_ = PLANE_SEE_EVERY;
            seeAirports();
        }
        atc_.update(airports_, obstacles_, clock_);
        dnav::PilotEnv env = planeEnv();
        planeIds_.clear();
        for (const auto& [id, p] : planes_) planeIds_.push_back(id);
        for (int id : planeIds_) {
            auto it = planes_.find(id);
            if (it == planes_.end()) continue;
            Plane& p = it->second;
            INPC* npc = npcs_ ? npcs_->get(id) : nullptr;
            IVehicle* v = npc ? npc->getVehicle() : nullptr;
            if (!npc || !v || npc->getVehicleSeat() != 0) {
                end(id, END_INTERRUPTED, false);
                continue;
            }
            if (glm::distance(v->getPosition(), p.placed) > PLANE_MOVED) placePlane(p, *v);
            p.pilot.step(env, dt);
            const dnav::PlaneState& s = p.pilot.st;
            const dnav::Vec3 vel = p.pilot.velocity();
            p.placed = Vector3(s.pos.x, s.pos.y, s.pos.z);
            // Not an NPC move: its position, rotation and velocity are set as they are
            // (as a helicopter's), and its driver sync carries them.
            npc->setVelocity(Vector3(vel.x, vel.y, vel.z) / VELOCITY_UNIT, false);
            npc->setVehiclePosition(p.placed, false);
            npc->setVehicleRotation(GTAQuat(Vector3(s.pitch, s.bank, s.heading)), false);
            const int gear = s.gearDown ? 0 : 1;
            if (gear != p.gear) {
                npc->setVehicleGearState(gear);
                p.gear = gear;
            }
            // What happened, told last: a callback may give it a new order, or destroy it.
            std::vector<dnav::PlanePhase> phases;
            phases.swap(p.pilot.phases);
            const int ended = p.pilot.ended;
            p.pilot.ended = -1;
            if (ended >= 0) p.busy = false;
            void* owner = p.owner;
            for (dnav::PlanePhase ph : phases) callPhase(owner, id, ph);
            if (ended >= 0) tell(owner, id, static_cast<MoveEnd>(ended));
        }
    }

    // Where a player is and how fast they go (in their car, if they are in one).
    static void whereIs(IPlayer& p, Vector3& pos, Vector3& vel)
    {
        IPlayerVehicleData* data = queryExtension<IPlayerVehicleData>(p);
        IVehicle* car = data ? data->getVehicle() : nullptr;
        if (car) {
            pos = car->getPosition();
            vel = car->getVelocity();
        } else {
            pos = p.getPosition();
            vel = p.getVelocity();
        }
    }

    // A chaser decides what to do next: stop beside its target, drive straight at
    // it, or follow a path to it (a road route, or the car mesh off the roads).
    void think(int npcid, Mover& m, TimePoint now)
    {
        m.nextThink = now + CHASE_THINK;
        INPC* npc = npcs_ ? npcs_->get(npcid) : nullptr;
        IVehicle* car = npc ? npc->getVehicle() : nullptr;
        if (!npc || !car || npc->getVehicleSeat() != 0) {
            end(npcid, END_INTERRUPTED, false);
            return;
        }
        IPlayer* target = core_->getPlayers().get(m.quarry);
        if (!target || target->getVirtualWorld() != npc->getVirtualWorld()) {
            end(npcid, END_LOST, true);
            return;
        }
        // Off an edge, it falls where it is going; it looks again once it is down.
        if (m.airborne) return;
        Vector3 tpos, tvel;
        whereIs(*target, tpos, tvel);
        IPlayerVehicleData* tdata = queryExtension<IPlayerVehicleData>(*target);
        m.quarryCar = tdata && tdata->getVehicle() ? tdata->getVehicle()->getID() : -1;
        const Vector3 cpos = car->getPosition();
        const float dist = glm::distance(Vector2(tpos.x, tpos.y), Vector2(cpos.x, cpos.y));
        // Both on the ground, as the meshes and roads have them.
        const dnav::Point carGround { cpos.x, cpos.y, cpos.z - m.carZ };
        const dnav::Point targetGround { tpos.x, tpos.y, tpos.z - FOOT_Z };

        const bool ram = m.chaseFlags & CHASE_RAM;
        m.quarryPos = tpos;
        m.quarryVel = Vector2(tvel) * VELOCITY_UNIT;
        m.quarryAt = now;
        // Its velocity, smoothed: one sharp turn of the wheel does not swing every
        // chaser after it at once.
        m.tvel = m.seen ? m.tvel + (tvel - m.tvel) * CHASE_VEL_SMOOTH : tvel;
        m.seen = true;
        tvel = m.tvel;
        const float tspeed = glm::length(Vector2(tvel)) * VELOCITY_UNIT;
        m.quarryDist = glm::distance(tpos, cpos);
        m.match = tspeed;
        // Fallen behind a target that is getting away: extra speed to catch up, more the
        // further behind, as the game gives its police. Its speed plan follows.
        {
            const float boost = tspeed > CHASE_STILL
                ? 1.0f + CATCHUP_MAX * std::clamp((dist - CATCHUP_FROM) / (CATCHUP_FULL - CATCHUP_FROM), 0.0f, 1.0f)
                : 1.0f;
            if (std::fabs(boost - m.boost) > 0.02f) {
                m.boost = boost;
                if (!m.points.empty()) planSpeeds(m, *car);
            }
        }
        // Held up by a vehicle in the way (another chaser beside it) near it is as good.
        const float close = m.blocked > 0.3f ? CHASE_CLOSE * 2.0f : CHASE_CLOSE;
        if (dist < close && std::fabs(tpos.z - cpos.z) < CHASE_DIRECT_DZ && tspeed < CHASE_STILL && !ram) {
            // Beside a target that is standing still: drive() brakes to a stop with no
            // path left. One that is moving is kept up with instead.
            dropPlan(m);
            m.points.clear();
            m.idle = true;
            m.direct = false;
            return;
        }

        const bool carMesh = finder.meshLoaded(dnav::MeshKind::Car);
        const bool roads = finder.roads().loaded();
        const bool walls = world_.loaded();
        const Shape me = shapeOf(car->getModel());
        // Where the target stands, on the ground the collision has under it.
        Vector3 tground(targetGround.x, targetGround.y, targetGround.z);
        {
            float gz;
            if (walls && world_.ground(tground.x, tground.y, tground.z, COL_UP, COL_DOWN, gz)) tground.z = gz;
        }
        const Vector3 cground(carGround.x, carGround.y, carGround.z);
        // Straight at it when nothing is in the way: the world's collision says so best,
        // else the car mesh, else the roads' decks.
        // (With the collision, a target below is driven at straight too, down a bank or
        // off a ledge, when nothing is in the way: runClear checks the way down.)
        const float below = cpos.z - tpos.z;
        // (Already driving straight at it, or for an edge, it keeps to that a little
        // further out than it would start: no flicking between that and a route.)
        const float direct = m.direct && !m.drop ? CHASE_DIRECT + CHASE_KEEP : CHASE_DIRECT;
        const float dropFrom = m.drop ? CHASE_DROP + CHASE_KEEP : CHASE_DROP;
        bool straight = (std::fabs(below) < CHASE_DIRECT_DZ || (walls && below > 0.0f && below <= CHASE_DROP_DZ))
            && m.wallHold <= 0.0f
            && (walls ? dist < direct && runClear(me, cground, tground)
                : carMesh ? dist < direct && finder.lineClear(carGround, targetGround, dnav::MeshKind::Car)
                        : dist < CHASE_BLIND || (dist < CHASE_DECKS
                            && alongDecks(carGround, targetGround, m.direct ? CHASE_DECK_KEEP : 0.0f)));
        // With nothing to plan on, straight at it is all there is.
        if (!carMesh && !roads && !walls) straight = true;
        // A target below, past the edge of the deck the car is on (it dropped off a
        // bridge, a ledge): straight at it and off the edge, not the roads round.
        std::vector<Vector3> way;
        // (With ledges in the car mesh its routes go off them where a car can, run up
        // square between the posts; this is for a world with none.)
        const bool drop = !straight && !(m.chaseFlags & CHASE_NO_DROP) && !(carMesh && finder.ledgeCount() > 0)
            && dist < dropFrom && below >= CHASE_DIRECT_DZ && below <= CHASE_DROP_DZ
            && dropWay(me, carGround, tpos, way);
        m.drop = drop;
        if (drop) {
            dropPlan(m);
            // The landing: leading the target when the way is straight at it.
            Vector3 aim = way.back();
            if (way.size() == 2) aim = chaseAim(npcid, m, cpos, tpos, tvel, dist);
            float gz;
            if (!groundUnder(aim.x, aim.y, targetGround.z, gz)) gz = targetGround.z;
            aim.z = gz + m.carZ;
            m.dropZ = std::min(gz, targetGround.z);
            // On the deck's level to the edge (drive() finds the deck as it goes), then
            // down to the target's.
            m.points.clear();
            m.speeds.clear();
            m.points.push_back(cpos);
            m.speeds.push_back(1.0f);
            for (size_t i = 0; i + 1 < way.size(); i++) {
                m.points.push_back(Vector3(way[i].x, way[i].y, way[i].z + m.carZ));
                m.speeds.push_back(1.0f);
            }
            m.points.push_back(aim);
            m.speeds.push_back(1.0f);
            m.seg = 0;
            m.ground = false;
            m.kind = dnav::PathKind::Drive;
            m.classes.assign(m.points.size(), dnav::RoadClass::Street);
            m.junctions.assign(m.points.size(), 0);
            planSpeeds(m, *car);
            m.direct = true;
            m.idle = false;
            return;
        }
        if (straight) {
            dropPlan(m);
            // Lead the target, unless that point is round a corner.
            Vector3 aim = chaseAim(npcid, m, cpos, tpos, tvel, dist);
            float lz;
            if (walls) {
                Vector3 ag(aim.x, aim.y, tground.z);
                if (world_.ground(ag.x, ag.y, ag.z, COL_UP, COL_DOWN, lz)) ag.z = lz;
                if (!runClear(me, cground, ag)) aim = tpos;
            } else if (carMesh
                && !finder.lineClear(carGround, dnav::Point { aim.x, aim.y, aim.z - FOOT_Z }, dnav::MeshKind::Car))
                aim = tpos;
            // Its ground: the target's own height only where something says there is
            // ground there; never up onto a level the car has no ground for.
            float gz;
            if (groundUnder(aim.x, aim.y, targetGround.z, gz) || groundUnder(aim.x, aim.y, carGround.z, gz))
                aim.z = gz + m.carZ;
            else
                aim.z = std::min(targetGround.z, carGround.z) + m.carZ;
            // A straight run, with the ground under it every few metres.
            m.points.clear();
            m.speeds.clear();
            m.points.push_back(cpos);
            m.speeds.push_back(1.0f);
            const float len = glm::distance(Vector2(aim), Vector2(cpos));
            const int steps = static_cast<int>(len / CHASE_GROUND_STEP);
            for (int i = 1; i <= steps; i++) {
                Vector3 p = cpos + (aim - cpos) * (static_cast<float>(i) / (steps + 1));
                if (groundUnder(p.x, p.y, p.z - m.carZ, gz))
                    p.z = gz + m.carZ;
                m.points.push_back(p);
                m.speeds.push_back(1.0f);
            }
            m.points.push_back(aim);
            m.speeds.push_back(1.0f);
            m.seg = 0;
            m.ground = carMesh;
            m.kind = carMesh ? dnav::PathKind::OffRoad : dnav::PathKind::Drive;
            m.classes.assign(m.points.size(), carMesh ? dnav::RoadClass::OffRoad : dnav::RoadClass::Street);
            m.junctions.assign(m.points.size(), 0);
            planSpeeds(m, *car);
            m.direct = true;
            m.idle = false;
            return;
        }

        // Follow a path; plan one when there is none or the target has moved off it: when
        // where it will be is no longer near where the plan was aimed. (Against where it
        // is, a plan aimed ahead of a fast target was stale the moment it came back, and
        // was re-planned 4 times a second.)
        const float lead = std::clamp(dist / std::max(m.v, 15.0f), 0.0f, CHASE_PLAN_LEAD);
        Vector3 goal = tpos + tvel * (VELOCITY_UNIT * lead);
        const float moved = std::min(glm::distance(goal, m.planGoal), glm::distance(tpos, m.planGoal));
        const bool stale = m.direct || m.idle || m.points.empty()
            || moved > std::max(CHASE_REPLAN, dist * CHASE_REPLAN_SHARE) || now - m.planned > CHASE_REPLAN_AFTER;
        // (Not on a ledge's run-up: a plan from part way along it would turn back to its start.)
        if (!stale || m.plan || (m.onLedge && !m.points.empty())) return;
        // Near, straight across the car mesh (on or off the road); far, the roads while
        // the target is on one.
        const bool onRoad = roads && finder.roads().nearest(targetGround.x, targetGround.y, targetGround.z,
            CHASE_ON_ROAD, {}) >= 0;
        const bool offRoad = carMesh && now >= m.noOffRoadUntil && (!roads || !onRoad || dist < CHASE_OFFROAD_RANGE);
        const dnav::PathKind kind = offRoad ? dnav::PathKind::OffRoad : dnav::PathKind::Drive;
        // A pursuit: any road either way, starting on the road ahead of the car, to
        // where the target will be by the time it gets near (cutting it off), not where
        // it was.
        dnav::RouteOptions how;
        how.anyWay = true;
        how.hasHeading = true;
        how.heading = m.rolling ? m.heading : car->getRotation().ToEuler().z;
        how.ledges = !(m.chaseFlags & CHASE_NO_DROP);
        dnav::Point goalGround { goal.x, goal.y, targetGround.z };
        if (!roads || finder.roads().nearest(goal.x, goal.y, targetGround.z, CHASE_ON_ROAD, how) < 0) {
            goal = tpos;
            goalGround = targetGround;
        }
        // (Moving, the search weighs turning round against carrying on: turnRoundCost.)
        if (m.rolling && !m.reversing) how.speed = m.v;
        const dnav::Point from = carGround;
        const int id = finder.request(m.owner, from, goalGround, kind, how);
        if (!id) return;
        m.plan = id;
        m.askedGoal = goal;
        byRequest_[id] = npcid;
    }

    // Where a chaser aims: where the target will be by the time the car gets there and,
    // with others after the same target, to one side of it, so they come up either
    // side of it instead of in a line (not when ramming it).
    Vector3 chaseAim(int npcid, const Mover& m, const Vector3& cpos, const Vector3& tpos, const Vector3& tvel,
        float dist) const
    {
        const float lead = std::clamp(dist / std::max(m.v, 10.0f), CHASE_LEAD_MIN, CHASE_LEAD_MAX);
        Vector3 aim = tpos + tvel * (VELOCITY_UNIT * lead);
        if (m.chaseFlags & CHASE_RAM) return coneAim(cpos, tpos, aim);
        int rank = 0, others = 0;
        for (const auto& [id, o] : movers_) {
            if (id == npcid || !o.chase || o.quarry != m.quarry || (o.chaseFlags & CHASE_RAM)) continue;
            others++;
            if (id < npcid) rank++;
        }
        // Across the way the target is going, or, standing still, across the way to it.
        Vector2 fwd = Vector2(tvel) * VELOCITY_UNIT;
        const bool moving = glm::length(fwd) >= CHASE_STILL;
        if (!others && !moving) return coneAim(cpos, tpos, aim);
        if (!moving) fwd = Vector2(tpos) - Vector2(cpos);
        if (glm::length(fwd) < 1e-3f) return coneAim(cpos, tpos, aim);
        fwd = glm::normalize(fwd);
        // Alone after a moving target, it pulls up alongside on the target's left (its
        // own right, the passenger's side, faces the target), not into its back.
        const float off = others ? (static_cast<float>(rank) - others * 0.5f) * CHASE_SPREAD : -CHASE_ALONGSIDE;
        aim.x += fwd.y * off;
        aim.y -= fwd.x * off;
        return coneAim(cpos, tpos, aim);
    }

    // The aim, turned back to no more than CHASE_AIM_CONE off the way to the target:
    // leading it never sends the car off sideways when it swerves.
    static Vector3 coneAim(const Vector3& cpos, const Vector3& tpos, Vector3 aim)
    {
        const Vector2 to = Vector2(tpos) - Vector2(cpos), want = Vector2(aim) - Vector2(cpos);
        const float lt = glm::length(to), lw = glm::length(want);
        if (lt < 1.0f || lw < 1.0f) return aim;
        const float a = std::atan2(to.x * want.y - to.y * want.x, glm::dot(to, want));
        const float cone = glm::radians(CHASE_AIM_CONE);
        if (std::fabs(a) <= cone) return aim;
        const float r = std::copysign(cone, a), c = std::cos(r), sn = std::sin(r);
        const Vector2 dir = to / lt;
        const Vector2 turned(dir.x * c - dir.y * sn, dir.x * sn + dir.y * c);
        aim.x = cpos.x + turned.x * lw;
        aim.y = cpos.y + turned.y * lw;
        return aim;
    }

    // Nav_GetChaseInfo: how far the target is and what the chaser is doing about it.
    bool chaseInfo(int npcid, float& distance, int& mode) const
    {
        auto fl = flyers_.find(npcid);
        if (fl != flyers_.end()) {
            if (fl->second.quarry < 0) return false;
            distance = fl->second.quarryDist;
            mode = CHASE_OVERHEAD;
            return true;
        }
        auto it = movers_.find(npcid);
        if (it == movers_.end() || !it->second.chase) return false;
        const Mover& m = it->second;
        distance = m.quarryDist;
        if (m.airborne) mode = CHASE_AIRBORNE;
        else if (m.idle) mode = CHASE_BESIDE;
        else if (m.direct) mode = m.drop ? CHASE_EDGE : CHASE_STRAIGHT;
        else if (m.points.empty()) mode = CHASE_PLANNING;
        else mode = CHASE_ROUTE;
        return true;
    }

    void dropPlan(Mover& m)
    {
        if (m.plan) byRequest_.erase(m.plan);
        m.plan = 0;
    }

    void planDone(int npcid, Mover& m, const dnav::PathResult& r, TimePoint now)
    {
        if (m.plan != r.id) return;
        m.plan = 0;
        if (r.status < dnav::PathStatus::Ok || r.points.empty()) {
            // The car mesh found no way: roads for a while. The roads found none:
            // the next think tries again (or drives straight once close).
            if (r.kind == dnav::PathKind::OffRoad) {
                m.noOffRoadUntil = now + CHASE_NO_OFFROAD;
                core_->printLn("[nav] chase: npc %d found no way across the car navmesh (status %d)", npcid,
                    static_cast<int>(r.status));
            }
            m.nextThink = TimePoint {};
            return;
        }
        m.kind = r.kind;
        setDrivePath(npcid, m, r);
        m.planGoal = m.askedGoal;
        m.planned = now;
        m.direct = false;
        m.idle = false;
    }

    void onNPCDeath(INPC& npc, IPlayer*, int) override { end(npc.getID(), END_INTERRUPTED, false); }

    void onNPCDestroy(INPC& npc) override
    {
        end(npc.getID(), END_INTERRUPTED, false);
        styles.erase(npc.getID());
    }

    void onFree(IComponent* component) override
    {
        if (component == pawn_) {
            pawn_ = nullptr;
            setAmxFunctions();
            setAmxLookups();
        }
        if (component == vehicles_) {
            vehicles_ = nullptr;
            traffic_.clear();
        }
        if (component == npcs_) {
            npcs_ = nullptr;
            movers_.clear();
            flyers_.clear();
            planes_.clear();
            atc_ = dnav::Atc {};
            atc_.setDb(&airports_);
            byRequest_.clear();
        }
    }

    void reset() override
    {
        movers_.clear();
        flyers_.clear();
        planes_.clear();
        atc_ = dnav::Atc {};
        atc_.setDb(&airports_);
        byRequest_.clear();
    }

    void free() override { delete this; }

private:
    ICore* core_ = nullptr;
    IPawnComponent* pawn_ = nullptr;
    INPCComponent* npcs_ = nullptr;
    IVehiclesComponent* vehicles_ = nullptr;
    dnav::Collision world_;                     // world.col: what is solid
    std::vector<Traffic> traffic_;              // every vehicle, this tick
    std::unordered_map<int, Shape> shapes_;     // model -> its collision box
    std::vector<const Traffic*> near_;          // avoid(): the vehicles near one driver
    std::vector<dnav::PathResult> results_;     // waiting to be handed over
    std::vector<dnav::PathResult> fresh_;
    std::vector<int> thinking_;
    std::unordered_map<int, Mover> movers_;     // npc id -> its walk or drive
    std::unordered_map<int, Flyer> flyers_;     // npc id -> its flight
    std::map<int, Plane> planes_;               // npc id -> its pilot (in id order: ATC asks in turn)
    std::vector<int> planeIds_;
    std::unordered_map<long long, float> tops_; // the ground's height, by cell (planes)
    float clock_ = 0.0f;                        // s the planes have flown (ATC's clock)
    float seeIn_ = 0.0f;
    TimePoint planeLast_ {};
    std::vector<dnav::Obstacle> obstacles_;

public:
    dnav::Airports airports_;
    dnav::Atc atc_;
    const std::map<int, Plane>& planes() const { return planes_; }
    void logError(const std::string& e) { core_->logLn(LogLevel::Error, "[nav] %s", e.c_str()); }
    std::unordered_map<int, int> byRequest_;    // request id -> npc id
};

DiamondNav* DiamondNav::instance = nullptr;

dnav::RouteOptions roadOptions(int flags)
{
    dnav::RouteOptions o;
    o.emergency = (flags & 1) != 0;
    o.boats = (flags & 2) != 0;
    return o;
}

} // namespace

COMPONENT_ENTRY_POINT()
{
    return new DiamondNav();
}

// ── Information ──────────────────────────────────────────────────────────────

dnav::MeshKind meshKind(int kind)
{
    return kind == 1 ? dnav::MeshKind::Car : dnav::MeshKind::Ped;
}

SCRIPT_API(Nav_GetTileCount, int(int kind))
{
    return DiamondNav::instance ? DiamondNav::instance->finder.tileCount(meshKind(kind)) : 0;
}

SCRIPT_API(Nav_GetRoadNodeCount, int())
{
    return DiamondNav::instance ? static_cast<int>(DiamondNav::instance->finder.roads().nodeCount()) : 0;
}

SCRIPT_API(Nav_GetCollisionFaceCount, int())
{
    return DiamondNav::instance ? DiamondNav::instance->collisionFaces() : 0;
}

SCRIPT_API(Nav_GetPendingCount, int())
{
    return DiamondNav::instance ? static_cast<int>(DiamondNav::instance->finder.pending()) : 0;
}

// ── Searches the script reads itself ─────────────────────────────────────────

SCRIPT_API(Nav_RequestPath, int(float fromX, float fromY, float fromZ, float toX, float toY, float toZ))
{
    if (!DiamondNav::instance) return 0;
    return DiamondNav::instance->finder.request(GetAMX(), dnav::Point { fromX, fromY, fromZ },
        dnav::Point { toX, toY, toZ });
}

SCRIPT_API(Nav_RequestRoute, int(float fromX, float fromY, float fromZ, float toX, float toY, float toZ, int flags))
{
    if (!DiamondNav::instance) return 0;
    return DiamondNav::instance->finder.request(GetAMX(), dnav::Point { fromX, fromY, fromZ },
        dnav::Point { toX, toY, toZ }, dnav::PathKind::Drive, roadOptions(flags));
}

SCRIPT_API(Nav_GetPathPoint, bool(int requestid, int index, float& x, float& y, float& z))
{
    const dnav::PathResult* r = DiamondNav::instance ? DiamondNav::instance->current : nullptr;
    if (!r || r->id != requestid || index < 0 || index >= static_cast<int>(r->points.size())) return false;
    x = r->points[index].x;
    y = r->points[index].y;
    z = r->points[index].z;
    return true;
}

SCRIPT_API(Nav_GetPathPointSpeed, float(int requestid, int index))
{
    const dnav::PathResult* r = DiamondNav::instance ? DiamondNav::instance->current : nullptr;
    if (!r || r->id != requestid || index < 0 || index >= static_cast<int>(r->speeds.size())) return 1.0f;
    return r->speeds[index];
}

SCRIPT_API(Nav_GetNearestPoint, bool(float x, float y, float z, float& outX, float& outY, float& outZ, float reach, int kind))
{
    dnav::Point out {};
    if (!DiamondNav::instance
        || !DiamondNav::instance->finder.nearest(dnav::Point { x, y, z }, reach, out, meshKind(kind)))
        return false;
    outX = out.x;
    outY = out.y;
    outZ = out.z;
    return true;
}

SCRIPT_API(Nav_GetRandomPoint, bool(float& x, float& y, float& z, int kind))
{
    dnav::Point out {};
    if (!DiamondNav::instance || !DiamondNav::instance->finder.random(out, meshKind(kind))) return false;
    x = out.x;
    y = out.y;
    z = out.z;
    return true;
}

SCRIPT_API(Nav_IsLineClear, bool(float x1, float y1, float z1, float x2, float y2, float z2, int kind))
{
    return DiamondNav::instance
        && DiamondNav::instance->finder.lineClear(dnav::Point { x1, y1, z1 }, dnav::Point { x2, y2, z2 }, meshKind(kind));
}

SCRIPT_API(Nav_HasLineOfSight, int(float x1, float y1, float z1, float x2, float y2, float z2))
{
    return DiamondNav::instance ? DiamondNav::instance->lineOfSight(Vector3(x1, y1, z1), Vector3(x2, y2, z2)) : -1;
}

SCRIPT_API(Nav_GetNearestRoadPoint, bool(float x, float y, float z, float& outX, float& outY, float& outZ, float reach, int flags))
{
    if (!DiamondNav::instance) return false;
    const auto& roads = DiamondNav::instance->finder.roads();
    const int i = roads.nearest(x, y, z, reach, roadOptions(flags));
    if (i < 0) return false;
    const auto& n = roads.nodes()[i];
    outX = n.x;
    outY = n.y;
    outZ = n.z;
    return true;
}

// ── NPCs moved by the component ──────────────────────────────────────────────

SCRIPT_API(Nav_WalkTo, bool(int npcid, float x, float y, float z, int moveType, float speed))
{
    if (!DiamondNav::instance) return false;
    NPCMoveType type = static_cast<NPCMoveType>(moveType);
    if (type != NPCMoveType_Walk && type != NPCMoveType_Jog && type != NPCMoveType_Sprint) type = NPCMoveType_Walk;
    return DiamondNav::instance->goTo(GetAMX(), npcid, dnav::PathKind::Walk, Vector3(x, y, z), type, speed, 0.0f, {});
}

SCRIPT_API(Nav_DriveTo, bool(int npcid, float x, float y, float z, float speed, float carZ, int flags))
{
    if (!DiamondNav::instance) return false;
    const dnav::PathKind kind = (flags & DRIVE_OFFROAD) ? dnav::PathKind::OffRoad : dnav::PathKind::Drive;
    return DiamondNav::instance->goTo(GetAMX(), npcid, kind, Vector3(x, y, z), NPCMoveType_Drive, speed, carZ,
        roadOptions(flags));
}

SCRIPT_API(Nav_Chase, bool(int npcid, int targetid, float speed, float carZ, int flags))
{
    return DiamondNav::instance && DiamondNav::instance->chase(GetAMX(), npcid, targetid, speed, carZ, flags);
}

SCRIPT_API(Nav_FlyTo, bool(int npcid, float x, float y, float z, float speed, int flags))
{
    return DiamondNav::instance && DiamondNav::instance->flyTo(GetAMX(), npcid, Vector3(x, y, z), speed, flags);
}

SCRIPT_API(Nav_Stop, bool(int npcid))
{
    if (!DiamondNav::instance || !DiamondNav::instance->isMoving(npcid)) return false;
    DiamondNav::instance->end(npcid, END_STOPPED, true);
    return true;
}

SCRIPT_API(Nav_IsMoving, bool(int npcid))
{
    return DiamondNav::instance && DiamondNav::instance->isMoving(npcid);
}

// ── How cars drive ───────────────────────────────────────────────────────────

namespace {

constexpr float KMH = 3.6f;

bool validClass(int c)
{
    return c >= 0 && c < static_cast<int>(dnav::RoadClass::Count);
}

} // namespace

SCRIPT_API(Nav_SetDriveStyle, bool(int npcid, int style))
{
    if (!DiamondNav::instance || style < -1 || style >= static_cast<int>(dnav::DriveStyle::Count)) return false;
    DiamondNav::instance->setStyle(npcid, style);
    return true;
}

SCRIPT_API(Nav_GetDriveStyle, int(int npcid))
{
    if (!DiamondNav::instance) return -1;
    auto it = DiamondNav::instance->styles.find(npcid);
    return it == DiamondNav::instance->styles.end() ? -1 : static_cast<int>(it->second);
}

SCRIPT_API(Nav_SetRoadSpeed, bool(int roadClass, float kmh))
{
    if (!DiamondNav::instance || !validClass(roadClass) || !(kmh > 0.0f)) return false;
    DiamondNav::instance->drivingModel.setLimit(static_cast<dnav::RoadClass>(roadClass), kmh / KMH);
    DiamondNav::instance->replanAll();
    return true;
}

SCRIPT_API(Nav_GetRoadSpeed, float(int roadClass))
{
    if (!DiamondNav::instance || !validClass(roadClass)) return 0.0f;
    return DiamondNav::instance->drivingModel.limit(static_cast<dnav::RoadClass>(roadClass)) * KMH;
}

SCRIPT_API(Nav_SetVehicleHandling, bool(int model, float topKmh, float accel, float brake, float grip, float radius))
{
    if (!DiamondNav::instance || !(topKmh > 0.0f && accel > 0.0f && brake > 0.0f && grip > 0.0f && radius > 0.0f))
        return false;
    DiamondNav::instance->drivingModel.setHandling(model, dnav::Handling::simple(topKmh / KMH, accel, brake, grip, radius));
    DiamondNav::instance->replanAll();
    return true;
}

SCRIPT_API(Nav_GetVehicleHandling, bool(int model, float& topKmh, float& accel, float& brake, float& grip, float& radius))
{
    const dnav::Handling h = DiamondNav::instance ? DiamondNav::instance->drivingModel.handling(model) : dnav::Handling {};
    topKmh = h.top * KMH;
    accel = h.accel[0];
    brake = h.brake;
    grip = h.grip;
    radius = h.radius;
    return DiamondNav::instance != nullptr;
}

// One number about how a model drives (NAV_VINFO_* in DiamondNav.inc).
SCRIPT_API(Nav_GetVehicleInfo, float(int model, int info))
{
    const dnav::Handling h = DiamondNav::instance ? DiamondNav::instance->drivingModel.handling(model) : dnav::Handling {};
    switch (info) {
    case 0: return h.top * KMH;
    case 1: return h.accel[0];
    case 2: return h.brake;
    case 3: return h.brakeOff;
    case 4: return h.coast;
    case 5: return h.drag;
    case 6: return h.grip;
    case 7: return h.gripOff;
    case 8: return h.radius;
    case 9: return h.yaw;
    case 10: return h.reverse * KMH;
    case 11: return h.length;
    case 12: return h.width;
    case 13: return h.height;
    case 14: return h.wheelbase;
    case 15: return h.accelTime(0.0f, 100.0f / KMH);
    case 16: return DiamondNav::instance ? DiamondNav::instance->rideHeight(model) : RIDE_DEFAULT;
    default: return 0.0f;
    }
}

SCRIPT_API(Nav_GetVehicleAccel, float(int model, float speedKmh, bool offRoad))
{
    const dnav::Handling h = DiamondNav::instance ? DiamondNav::instance->drivingModel.handling(model) : dnav::Handling {};
    return h.accelAt(speedKmh / KMH, offRoad);
}

SCRIPT_API(Nav_GetVehicleAccelTime, float(int model, float fromKmh, float toKmh))
{
    const dnav::Handling h = DiamondNav::instance ? DiamondNav::instance->drivingModel.handling(model) : dnav::Handling {};
    return h.accelTime(fromKmh / KMH, toKmh / KMH);
}

SCRIPT_API(Nav_GetVehicleModelCount, int())
{
    return DiamondNav::instance ? static_cast<int>(DiamondNav::instance->drivingModel.vehicleCount()) : 0;
}

SCRIPT_API(Nav_GetPathPointClass, int(int requestid, int index))
{
    const dnav::PathResult* r = DiamondNav::instance ? DiamondNav::instance->current : nullptr;
    if (!r || r->id != requestid || index < 0 || index >= static_cast<int>(r->classes.size())) return -1;
    return static_cast<int>(r->classes[index]);
}

SCRIPT_API(Nav_GetChaseInfo, bool(int npcid, float& distance, int& mode))
{
    float d = 0.0f;
    int how = 0;
    if (!DiamondNav::instance || !DiamondNav::instance->chaseInfo(npcid, d, how)) return false;
    distance = d;
    mode = how;
    return true;
}

SCRIPT_API(Nav_GetDriveInfo, bool(int npcid, float& speedKmh, float& wantKmh, int& roadClass, float& limitKmh, bool& reversing))
{
    float speed = 0.0f, want = 0.0f, limit = 0.0f;
    dnav::RoadClass cls = dnav::RoadClass::Street;
    bool back = false;
    if (!DiamondNav::instance || !DiamondNav::instance->driveInfo(npcid, speed, want, cls, limit, back)) return false;
    speedKmh = speed * KMH;
    wantKmh = want * KMH;
    roadClass = static_cast<int>(cls);
    limitKmh = limit * KMH;
    reversing = back;
    return true;
}

// ── Planes ───────────────────────────────────────────────────────────────────

namespace {

DiamondNav* nav()
{
    return DiamondNav::instance;
}

const dnav::Airport* airportNamed(const std::string& code, int* index = nullptr)
{
    if (!nav()) return nullptr;
    const int i = nav()->airports_.find(code);
    if (index) *index = i;
    return nav()->airports_.get(i);
}

// A route by name, or none for "" (false: there is no such route).
bool routeNamed(const std::string& name, std::vector<dnav::Vec3>& out)
{
    out.clear();
    if (name.empty()) return true;
    const dnav::FlightRoute* r = nav()->airports_.route(name);
    if (!r) return false;
    out = r->points;
    return true;
}

} // namespace

SCRIPT_API(Nav_ParkPlane, bool(int npcid, std::string const& airport, std::string const& gate))
{
    int ap = -1;
    const dnav::Airport* a = airportNamed(airport, &ap);
    if (!a) return false;
    const int g = a->gate(gate);
    if (g < 0) return false;
    return nav()->planeOrder(GetAMX(), npcid, [&](Plane& p, dnav::PilotEnv& env) {
        if (!p.pilot.park(env, ap, g)) return false;
        p.pilot.ended = static_cast<int>(dnav::PilotEnd::Arrived);
        return true;
    });
}

SCRIPT_API(Nav_TaxiTo, bool(int npcid, std::string const& where))
{
    if (!nav()) return false;
    return nav()->planeOrder(GetAMX(), npcid, [&](Plane& p, dnav::PilotEnv& env) {
        const dnav::Airport* a = env.db->get(p.pilot.airport());
        if (!a) {
            p.pilot.why = "it is not at an airport";
            return false;
        }
        const int g = a->gate(where), n = g < 0 ? a->node(where) : -1;
        if (g < 0 && n < 0) {
            p.pilot.why = "no gate or taxi node '" + where + "' at " + a->code;
            return false;
        }
        return p.pilot.taxiTo(env, g, n);
    });
}

SCRIPT_API(Nav_Takeoff, bool(int npcid, std::string const& runway, std::string const& route))
{
    if (!nav()) return false;
    std::vector<dnav::Vec3> after;
    if (!routeNamed(route, after)) return false;
    return nav()->planeOrder(GetAMX(), npcid, [&](Plane& p, dnav::PilotEnv& env) { return p.pilot.takeoff(env, runway, after); });
}

SCRIPT_API(Nav_Land, bool(int npcid, std::string const& airport, std::string const& gate, std::string const& runway, std::string const& via))
{
    int ap = -1;
    const dnav::Airport* a = airportNamed(airport, &ap);
    if (!a) return false;
    int g = -1;
    if (gate == NO_GATE) g = -2;
    else if (!gate.empty() && (g = a->gate(gate)) < 0) return false;
    std::vector<dnav::Vec3> way;
    if (!routeNamed(via, way)) return false;
    return nav()->planeOrder(GetAMX(), npcid, [&](Plane& p, dnav::PilotEnv& env) { return p.pilot.land(env, ap, runway, g, way); });
}

SCRIPT_API(Nav_FlyRoute, bool(int npcid, std::string const& route, float speed, bool loop))
{
    if (!nav()) return false;
    std::vector<dnav::Vec3> pts;
    if (route.empty() || !routeNamed(route, pts)) return false;
    return nav()->planeOrder(GetAMX(), npcid, [&](Plane& p, dnav::PilotEnv& env) {
        return p.pilot.fly(env, pts, speed > 0.0f ? speed * SPEED_UNIT : 0.0f, loop);
    });
}

SCRIPT_API(Nav_GetPlanePhase, int(int npcid))
{
    if (!nav()) return -1;
    auto it = nav()->planes().find(npcid);
    return it == nav()->planes().end() ? -1 : static_cast<int>(it->second.pilot.phase());
}

SCRIPT_API(Nav_GetPlaneInfo, bool(int npcid, int& phase, float& speedKmh, float& toTouchdown, int& airport, int& gate))
{
    if (!nav()) return false;
    auto it = nav()->planes().find(npcid);
    if (it == nav()->planes().end()) return false;
    const dnav::Pilot& p = it->second.pilot;
    phase = static_cast<int>(p.phase());
    speedKmh = p.st.speed * KMH;
    toTouchdown = p.toTouchdown();
    airport = p.airport();
    gate = p.gate();
    return true;
}

SCRIPT_API(Nav_GetPlanePhaseName, bool(int phase, OutputOnlyString& name))
{
    if (phase < 0 || phase > static_cast<int>(dnav::PlanePhase::Loiter)) return false;
    name = StringView(dnav::phaseName(static_cast<dnav::PlanePhase>(phase)));
    return true;
}

SCRIPT_API(Nav_GetAirportCount, int())
{
    return nav() ? static_cast<int>(nav()->airports_.count()) : 0;
}

SCRIPT_API(Nav_FindAirport, int(std::string const& code))
{
    return nav() ? nav()->airports_.find(code) : -1;
}

SCRIPT_API(Nav_GetAirportAt, int(float x, float y, float z))
{
    return nav() ? nav()->airports_.at({ x, y, z }, 50.0f) : -1;
}

SCRIPT_API(Nav_GetAirportCode, bool(int index, OutputOnlyString& code))
{
    const dnav::Airport* a = nav() ? nav()->airports_.get(index) : nullptr;
    if (!a) return false;
    code = StringView(a->code);
    return true;
}

SCRIPT_API(Nav_GetGateSpot, bool(std::string const& airport, std::string const& gate, float& x, float& y, float& z, float& heading))
{
    const dnav::Airport* a = airportNamed(airport);
    const int g = a ? a->gate(gate) : -1;
    if (g < 0) return false;
    const dnav::Gate& gt = a->gates[g];
    x = gt.p.x;
    y = gt.p.y;
    z = gt.p.z;
    heading = gt.heading;
    return true;
}

SCRIPT_API(Nav_IsGateFree, bool(std::string const& airport, std::string const& gate))
{
    int ap = -1;
    const dnav::Airport* a = airportNamed(airport, &ap);
    const int g = a ? a->gate(gate) : -1;
    return g >= 0 && nav()->atc_.gateFree(ap, g);
}

// A free gate a model fits at an airport (the smallest that fits).
SCRIPT_API(Nav_FindFreeGate, bool(std::string const& airport, int model, OutputOnlyString& gate))
{
    int ap = -1;
    const dnav::Airport* a = airportNamed(airport, &ap);
    if (!a) return false;
    const float span = dnav::PlaneSpec::of(model).span;
    int best = -1;
    for (size_t g = 0; g < a->gates.size(); g++) {
        const dnav::Gate& gt = a->gates[g];
        if (gt.removed || gt.span < span || !nav()->atc_.gateFree(ap, static_cast<int>(g))) continue;
        if (best < 0 || gt.span < a->gates[best].span) best = static_cast<int>(g);
    }
    if (best < 0) return false;
    gate = StringView(a->gates[best].name);
    return true;
}

SCRIPT_API(Nav_GetRunwayEnd, bool(std::string const& airport, std::string const& end, float& x, float& y, float& z, float& heading))
{
    const dnav::Airport* a = airportNamed(airport);
    if (!a) return false;
    for (const auto& r : a->runways) {
        if (r.removed) continue;
        for (int e = 0; e < 2; e++) {
            if (r.endName[e] != end) continue;
            const dnav::Vec3 p = r.pointAt(r.thresholdAlong(e));
            x = p.x;
            y = p.y;
            z = p.z;
            heading = r.headingFrom(e);
            return true;
        }
    }
    return false;
}

SCRIPT_API(Nav_SetRunwayOpen, bool(std::string const& airport, std::string const& runway, bool open))
{
    int ap = -1;
    const dnav::Airport* a = airportNamed(airport, &ap);
    const int r = a ? a->runway(runway) : -1;
    if (r < 0) return false;
    nav()->atc_.setOpen(ap, r, open);
    return true;
}

// Who has a runway (an NPC id), or -1.
SCRIPT_API(Nav_GetRunwayUser, int(std::string const& airport, std::string const& runway))
{
    int ap = -1;
    const dnav::Airport* a = airportNamed(airport, &ap);
    const int r = a ? a->runway(runway) : -1;
    return r < 0 ? -1 : nav()->atc_.user(ap, r);
}

SCRIPT_API(Nav_LoadAirports, bool(std::string const& path))
{
    if (!nav()) return false;
    std::string error;
    if (!nav()->airports_.loadFile(path, error)) {
        nav()->logError(error);
        return false;
    }
    return true;
}

SCRIPT_API(Nav_AddRoutePoint, bool(std::string const& route, float x, float y, float z))
{
    if (!nav() || route.empty()) return false;
    nav()->airports_.addRoutePoint(route, { x, y, z });
    return true;
}

SCRIPT_API(Nav_ClearRoute, bool(std::string const& route))
{
    if (!nav() || !nav()->airports_.route(route)) return false;
    nav()->airports_.setRoute(route, {});
    return true;
}
