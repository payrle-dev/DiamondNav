// A plane's pilot: what an NPC plane does from the stand to the sky and back.
//
// The script gives orders (taxi to a gate, take off, fly a route, land at an airport,
// chase someone) and the pilot works them through, phase by phase:
//
//   parked -> pushback -> taxi out -> hold short -> line up -> take-off -> climb ->
//   cruise (a route) -> approach (a pattern onto the final) -> final -> landing roll ->
//   taxi in -> at the gate
//
// with holding (the runway will be busy when it gets there) and going around (the
// runway is not clear by the decision point, or the plane is not lined up) on the
// way down. ATC (atc.hpp) says who may use which runway and stretch of taxiway.
// Away from a runway it keeps clear of the ground and of other planes in the air.
//
// Nothing here touches open.mp: the component moves the vehicle where the pilot says
// it is every tick, and tests/test_aircraft.cpp flies pilots against a made-up world.

#pragma once

#include <functional>
#include <string>
#include <vector>

#include "aircraft.hpp"
#include "airports.hpp"
#include "atc.hpp"

namespace dnav {

// What a plane is doing (Nav_GetPlanePhase, OnNavPlanePhase).
enum class PlanePhase : int {
    None = 0,
    Parked,         // standing still on the ground, nothing to do
    Pushback,       // backed off its gate by a tug
    TaxiOut,        // to the runway
    HoldShort,      // waiting to line up (the runway is busy)
    LineUp,         // onto the runway
    Takeoff,        // the take-off roll
    Climb,          // climbing away from the runway
    Cruise,         // flying a route, or to an airport
    Holding,        // circling until the runway it lands on will be free
    Approach,       // the pattern down to the final
    Final,          // lined up, on the glide path
    GoAround,       // climbing away from a missed landing
    Landing,        // the landing roll, to a turn-off
    TaxiIn,         // to a gate (or a taxiway node)
    AtGate,         // parked at a gate
    Chase,          // after someone
    Loiter,         // circling, nothing left to do in the air
};

const char* phaseName(PlanePhase p);

// Why an order ended (OnNavMoveEnd's reason, as the component's).
enum class PilotEnd : int { Arrived = 0, NoPath = 1, Stopped = 2, Lost = 4 };

class Pilot;

// What a pilot sees of the world while it steps.
struct PilotEnv {
    Airports* db = nullptr;
    Atc* atc = nullptr;
    FlyWorld world;
    float now = 0.0f;                   // seconds, any clock that goes on
    std::vector<const Pilot*> others;   // every other pilot
    // Where someone a pilot chases is (false: gone).
    std::function<bool(int target, Vec3& pos, Vec3& vel)> target;
};

class Pilot {
public:
    Pilot(int id, int model);

    int id;
    PlaneSpec spec;
    PlaneState st;

    PlanePhase phase() const { return phase_; }
    // How it moves (backwards behind a tug).
    Vec3 velocity() const;
    bool airborne() const { return !st.ground; }
    // The airport it is at or going to (-1: none) and its gate there (-1: none).
    int airport() const { return ap_; }
    int gate() const { return gate_; }
    // Metres to go to where it lands (approach and final), else -1.
    float toTouchdown() const;
    std::string why;                    // why the last order was refused

    // ── Orders (false, with `why`, if it cannot) ─────────────────────────────
    // Standing where it is (a plane just put somewhere): on the ground at an airport
    // with ground true, else in the air at `speed` (0: cruising).
    void place(PilotEnv& env, const Vec3& pos, float heading, bool ground, float speed);
    // Put at a gate, parked.
    bool park(PilotEnv& env, int ap, int gate);
    // On the ground: taxi to a gate (or a taxi node, gate -1).
    bool taxiTo(PilotEnv& env, int gate, int node);
    // On the ground: out to a runway (`runway`: a runway or end name, "" any) and away,
    // then the route `after` (then circling over its end).
    bool takeoff(PilotEnv& env, const std::string& runway, const std::vector<Vec3>& after);
    // Anywhere: (off the ground first, then) the route `via`, then land at airport ap
    // on `runway` ("" the best) and taxi to `gate` (-1 any free one that fits, -2 none:
    // it stops once off the runway).
    bool land(PilotEnv& env, int ap, const std::string& runway, int gate, const std::vector<Vec3>& via);
    // In the air: through these points (loop: round and round), then circling the last.
    bool fly(PilotEnv& env, const std::vector<Vec3>& points, float speed, bool loop);
    // In the air: after someone (env.target), keeping `height` over them.
    bool chase(PilotEnv& env, int target, float speed, float height);
    // Whatever it was doing ends: in the air it circles where it is, on the ground it
    // stops.
    void stop(PilotEnv& env);

    // One step of dt seconds.
    void step(PilotEnv& env, float dt);

    // What happened since the last look: phases it went into, in order, and how its
    // order ended (-1: it has not).
    std::vector<PlanePhase> phases;
    int ended = -1;

    // Everything it holds with ATC, gone (its NPC is going away).
    void forget(PilotEnv& env);

private:
    struct Range {                      // a runway the ground path goes onto
        int rw = -1;
        float enter = 0.0f, exit = 0.0f;
        bool cleared = false;
        bool own = false;               // the one it lands on or takes off from
    };
    enum class After { Stop, Hold, Gate, Node, TaxiIn, Push, Roll };

    void setPhase(PlanePhase p);
    void finish(PilotEnd why);

    // The ground.
    bool startGround(PilotEnv& env, const GroundPath& path, After after, float brake);
    void findRanges(PilotEnv& env);
    void groundStep(PilotEnv& env, float dt);
    void groundDone(PilotEnv& env);
    bool planTaxi(PilotEnv& env, int goalGate, int goalNode, bool fromGateAllowed);
    bool planDeparture(PilotEnv& env);
    GroundPath pushOff(const Airport& a, const Gate& g, const Vec3& next, std::vector<Vec3>& pts) const;
    std::vector<Vec3> taxiPoints(PilotEnv& env, const TaxiRoute& r, const Airport& a) const;
    bool pickDeparture(PilotEnv& env, int& rw, int& end) const;
    void claimHere(PilotEnv& env);
    void setLegs(PilotEnv& env);
    void lineUp(PilotEnv& env);
    void takeoffStep(PilotEnv& env, float dt);

    // The air.
    void airStep(PilotEnv& env, float dt);
    void afterDeparture(PilotEnv& env);
    void startRoute(PilotEnv& env);
    bool startApproach(PilotEnv& env);
    bool pickLanding(PilotEnv& env, int& rw, int& end) const;
    void planPattern(PilotEnv& env);
    void goAround(PilotEnv& env);
    void touchDown(PilotEnv& env);
    void loiterHere(const Vec3& c);
    float cruiseZ(PilotEnv& env) const;
    float lookAhead() const;
    float floorAhead(PilotEnv& env, float heading, float clear);
    void separate(PilotEnv& env, AirCommand& c);
    void steerClear(PilotEnv& env, AirCommand& c);
    void steerTo(const Vec3& to, AirCommand& c) const;
    void steerOrbit(const Vec3& c, float r, AirCommand& cmd) const;
    float surfaceZ(const Runway& r) const;

    PlanePhase phase_ = PlanePhase::None;

    // Where it is, as ATC knows it.
    int ap_ = -1;
    int gate_ = -1;

    // Its order.
    std::vector<Vec3> route_;           // to fly through
    size_t leg_ = 0;
    bool loop_ = false;
    float speed_ = 0.0f;                // m/s it flies at (0: its own)
    bool landing_ = false;              // landing at the end of it all
    int landAp_ = -1, landGate_ = -1;
    std::string landRunway_, departRunway_;
    int chase_ = -1;
    float chaseHeight_ = 40.0f;
    int taxiGate_ = -1, taxiNode_ = -1;
    Vec3 gateLead_ {};                  // where its straight line into the gate starts
    float gateLineS_ = -1.0f;           // ... that far along path_ (-1: not going to a gate)

    // On the ground.
    GroundPath path_;
    float s_ = 0.0f;
    float brake_ = 1.5f;
    After after_ = After::Stop;
    std::vector<Range> ranges_;
    struct Leg {                        // a taxi edge on its ground path
        int edge = -1, from = -1;
        float s0 = 0.0f, s1 = 0.0f;
    };
    std::vector<Leg> legs_;
    TaxiRoute route_t_;                 // the taxi route its next taxi path follows
    GroundPath next_;                   // a taxi path waiting behind a pushback
    After nextAfter_ = After::Stop;
    PlanePhase nextPhase_ = PlanePhase::TaxiOut;
    GroundPath lineUp_;                 // from the hold short to where its roll starts
    float waited_ = 0.0f;               // s held up by someone's claim
    float holdAsk_ = 0.0f;              // when it last asked ATC
    int blocker_ = -1;
    float stopAt_ = 1e9f;               // Nav_Stop on the ground: it stops by here

    // Its runway (taking off from or landing on).
    int rw_ = -1, end_ = 0;
    float roll_ = 0.0f;                 // m along it from where the roll started
    Vec3 rollFrom_ {};
    float rwZ_ = 0.0f;
    float since_ = 0.0f;                // s in this phase

    // The approach.
    std::vector<Vec3> fixes_;
    size_t fix_ = 0;
    bool established_ = false;
    bool cleared_ = false;
    float askAt_ = 0.0f;
    float patternZ_ = 0.0f;
    float aimAlong_ = 0.0f;             // m from the threshold it touches down
    Vec3 holdAt_ {};
    int goArounds_ = 0;
    float toGo_ = -1.0f;                // m to its touchdown point
    Vec3 aimP_ {}, fwd_ {};             // the touchdown point, and the way it lands

    // In the air.
    Vec3 loiter_ {};
    float loiterR_ = 400.0f;
    float floorIn_ = 0.0f, floor_ = -1e9f, floorHeading_ = 0.0f;
    float escape_ = 0.0f;               // s left turning away from high ground
    float escapeHeading_ = 0.0f;
    float sepIn_ = 0.0f;
    float sepHeading_ = 0.0f, sepZ_ = 0.0f;
    bool sep_ = false;
};

} // namespace dnav
