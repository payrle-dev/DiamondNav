// How NPC planes move: each plane model's size and performance, the paths they
// follow on the ground (taxiing, pushing back, the take-off and landing rolls) and
// how they fly (a fixed-wing flight model: banked turns, climbs and descents within
// what the plane can do, never slower than it flies).
//
// On the ground a plane follows a path worked out beforehand: the taxiway's centre
// line with its corners rounded to a circle the plane can turn, and a speed for
// every metre of it (slow in the turns, braking in time for each stop). In the air
// it is steered: a heading and a height to make for (the pilot, pilot.hpp, says
// which), turned into a bank, a climb and a speed.
//
// Nothing here touches open.mp: tests/test_aircraft.cpp flies planes against a
// made-up world.

#pragma once

#include <string>
#include <vector>

#include "collision.hpp"
#include "flying.hpp"

namespace dnav {

// Whether a model is a plane a pilot flies (not the RC ones, not helicopters).
bool isPlane(int model);

// How big a plane is and how it flies, in metres, seconds and degrees.
struct PlaneSpec {
    int model = 0;
    float span = 15.0f;         // wingspan
    float length = 12.0f;
    float ride = 1.2f;          // its centre over the ground, standing on its wheels
    // Speeds, m/s.
    float rotate = 32.0f;       // lifts off at this speed
    float approach = 36.0f;     // on final
    float touchdown = 32.0f;
    float cruise = 50.0f;
    float minimum = 30.0f;      // the slowest it flies away from a runway
    float climb = 10.0f;        // m/s up, at most
    float sink = 8.0f;          // m/s down, at most (an approach is shallower)
    float bank = 35.0f;         // degrees, the steepest turn it flies
    float rollRate = 30.0f;     // degrees a second it rolls into or out of a turn
    float accel = 3.0f;         // m/s^2 on the take-off roll
    float airAccel = 1.5f;      // m/s^2 picking up speed in the air
    float airDecel = 1.5f;      // m/s^2 slowing in the air
    float brake = 4.0f;         // m/s^2 on the landing roll (wheel brakes, reversers)
    // On the ground.
    float taxi = 10.0f;         // m/s on a straight taxiway
    float taxiCorner = 1.2f;    // m/s^2 sideways in a taxi turn
    float taxiRadius = 12.0f;   // m, the tightest turn its nose wheel makes
    float taxiAccel = 0.8f;
    float taxiBrake = 1.5f;
    float push = 1.5f;          // m/s backing off a stand behind a tug
    // The approach.
    float glide = 3.2f;         // degrees, the glide path
    float finalLength = 1200.0f;// m of straight final before the runway
    float flare = 12.0f;        // m over the runway its flare starts
    float attitude = 3.0f;      // degrees nose up over its flight path on approach
    float touchPitch = 6.0f;    // degrees nose up as its wheels touch
    float rotatePitch = 10.0f;  // degrees nose up as it lifts off and climbs out
    std::string name = "plane";

    static PlaneSpec of(int model);

    // A rough length of runway each needs, with a margin (m).
    float takeoffRun() const { return rotate * rotate / (2.0f * accel) * 1.15f; }
    float landingRun() const { return touchdown * touchdown / (2.0f * brake) + 2.0f * touchdown; }
    // The tightest circle it flies at speed v (m).
    float turnRadius(float v) const;
};

// GTA's heading (degrees, 0 north, turning left) of a direction, and back.
Vec3 forwardOf(float heading);
float wrapDeg(float a);

// A path on the ground with a speed at every point: what a taxiing plane follows.
struct GroundPoint {
    Vec3 p;                     // the ground (not the plane's centre)
    float heading;              // which way the plane points (GTA's)
    float s;                    // metres along the path
    float vmax;                 // the fastest it may be going here (m/s)
};

struct GroundPath {
    std::vector<GroundPoint> pts;
    bool reverse = false;       // pushed backwards (heading is opposite to the way it goes)

    bool empty() const { return pts.size() < 2; }
    float length() const { return pts.empty() ? 0.0f : pts.back().s; }
    // The point s metres along (between samples), and the heading there.
    GroundPoint at(float s) const;
    // The highest speed still allowed at s for something that must keep under the
    // plan's speeds ahead, braking at `brake` m/s^2 (it already does: vmax is planned).
    float speedAt(float s) const { return at(s).vmax; }
};

// Rounds a polyline's corners off to circles of at least radius r (less only where
// the legs are too short), samples it every `step` metres and gives every point a
// speed: `top` on the straight, what `corner` m/s^2 sideways allows round a turn, and
// braking at `brake` (accelerating at `accel`) in time for each slower point; it
// starts at `v0` and ends at `vEnd`. A leg that doubles back (a U-turn) is swung
// round a loop of radius r beside it.
GroundPath groundPath(const std::vector<Vec3>& points, float r, float top, float corner, float accel,
    float brake, float v0, float vEnd, float step = 1.0f);

// The same for a path backwards (a pushback): a curve from the stand back to `to`,
// leaving it so the plane ends up pointing `endHeading`.
GroundPath pushPath(const Vec3& from, float heading, const Vec3& to, float endHeading, float speed, float step = 0.5f);

// A plane in the air (or on its wheels on the ground).
struct PlaneState {
    Vec3 pos {};                // its centre
    float heading = 0.0f;       // GTA's, degrees
    float speed = 0.0f;         // m/s along its way (airspeed; no wind)
    float vz = 0.0f;            // m/s up
    float bank = 0.0f;          // degrees, left wing down positive (GTA's roll)
    float pitch = 0.0f;         // degrees, nose up
    float yawRate = 0.0f;       // degrees a second
    bool gearDown = true;
    bool ground = true;         // on its wheels
    Vec3 velocity() const;
};

// What the pilot wants of the plane in the air this step.
struct AirCommand {
    float heading = 0.0f;       // to turn to (GTA's)
    float z = 0.0f;             // to climb or descend to
    float vz = 0.0f;            // m/s up it is told to hold instead, when `holdVz`
    bool holdVz = false;
    float speed = 0.0f;         // to make for (m/s)
    float maxBank = 0.0f;       // degrees (0: the plane's own)
    float pitchExtra = 0.0f;    // degrees nose up on top of its flight path and attitude (the flare)
};

// One step of dt seconds in the air.
void flyPlane(PlaneState& s, const AirCommand& c, const PlaneSpec& spec, float dt);

// How high it must be at (x, y) heading `heading` to clear everything `look`
// metres ahead and its wings' width either side, climbing as it can: the top of
// whatever is there plus `clear`. Very low with nothing there.
float terrainFloor(const FlyWorld& w, const Vec3& pos, float heading, float speed, const PlaneSpec& spec,
    float clear, float look);

} // namespace dnav
