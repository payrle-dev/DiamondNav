// Air traffic control for NPC planes: who may use a runway, who may taxi where, and
// which gates are taken.
//
//   - A runway has one user at a time: a plane cleared to land (from short final until
//     it has turned off), one cleared to take off (from lining up until it has climbed
//     away) or one crossing it. Runways whose centre lines cross count as one. A
//     vehicle standing on a runway (a player's car, a plane parked on it) closes it
//     until it moves: landings go around, departures hold short.
//   - Planes coming in say when they will land (inbound): a departure only lines up
//     with time to be gone before the next arrival, and an arrival that would land on
//     a busy runway holds or goes around.
//   - On the ground, every plane claims the stretch of its way ahead that it needs to
//     stop in, as wide as its wings. A claim stops short of anyone else's (planes,
//     and any vehicle standing there), and short of a runway the plane is not cleared
//     onto: the plane brakes to a stop at its claim's end and waits. Taxi routes are
//     searched round the stretches other planes are about to taxi the other way.
//
// Nothing here touches open.mp; pilot.cpp asks, the component tells it what stands
// where every tick.

#pragma once

#include <map>
#include <unordered_map>
#include <vector>

#include "airports.hpp"

namespace dnav {

// Where a plane stands (or will): its centre on the ground and its heading.
struct Pose {
    Vec3 p {};
    float heading = 0.0f;
};

// How far apart two planes standing like this are from above, wings and fuselage
// (0: touching).
float outlineGap(const Pose& a, float spanA, float lengthA, const Pose& b, float spanB, float lengthB);

// Something on the ground or in the air this tick that is not a plane under ATC.
struct Obstacle {
    Vec3 pos {};
    Vec3 vel {};
    float radius = 2.5f;        // half its biggest size
    int vehicle = -1;
};

class Atc {
public:
    enum class Use { None, Landing, Takeoff, Crossing };

    // ── Runways ──────────────────────────────────────────────────────────────
    // Whether pilot `who` may land on runway `rw` of airport `ap`, landing in `eta`
    // seconds and off it `occupy` seconds after that. Granted, the runway is its.
    bool requestLanding(int who, int ap, int rw, float eta, float now);
    // ...line up and take off (gone in `occupy` seconds).
    bool requestTakeoff(int who, int ap, int rw, float occupy, float now);
    // ...cross it (or taxi along it to a turn-off).
    bool requestCrossing(int who, int ap, int rw, float occupy, float now);
    // Done with it (or no longer coming).
    void release(int who, int ap, int rw);
    void releaseAll(int who);
    // Whether `who` holds runway rw.
    bool holds(int who, int ap, int rw) const;
    // An arrival says when it will land (seconds from now), until it lands or gives up.
    void inbound(int who, int ap, int rw, float eta, float now);
    void notInbound(int who);
    // Seconds until runway rw is expected to be free for a plane landing in `eta`
    // seconds: 0 if it should be.
    float busyFor(int who, int ap, int rw, float eta, float now) const;
    // Opened or closed by the script.
    bool open(const Airports& db, int ap, int rw) const;
    // Something standing on runway rw (worked out by update()).
    bool blocked(int ap, int rw) const;
    int user(int ap, int rw, Use* use = nullptr) const;

    // ── The ground ───────────────────────────────────────────────────────────
    // A plane's claim: where it stands now (poses[0]) and where its centre will pass
    // as far as it may go, as a plane `span` wide and `length` long (its wings and
    // its fuselage, a cross); and whose claim it is waiting at (-1: none).
    void setClaim(int who, int ap, const std::vector<Pose>& poses, float span, float length, int waitingFor);
    void dropClaim(int who);
    // How far along `ahead` (poses its centre will pass, in order) pilot `who` may go
    // before it meets someone else's claim or an obstacle: the index of the first pose
    // it may not reach (ahead.size(): all), and whose it is in `by` (-1: an obstacle).
    // Two planes waiting at each other's claims: the lower id goes, past the other's
    // way ahead (never its body). `ignore` 1 leaves out everyone's way ahead, 2 their
    // claims altogether (a plane stuck for long).
    size_t clearAhead(int who, int ap, const std::vector<Pose>& ahead, float span, float length, float gap,
        int& by, int ignore = 0) const;
    // The taxi edges a plane still means to use, and which way (edge, the node it
    // enters it from), so others route round head-on meetings.
    void setIntent(int who, int ap, const std::vector<std::pair<int, int>>& edges);
    void dropIntent(int who);
    // Whether someone else means to taxi edge `e` the other way (entering it from
    // the other end) than from node `from`.
    bool opposed(int who, int ap, int e, int from, const TaxiEdge& edge) const;
    // Whether pilot `who` should wait before taxiing the stretch `run` (edges and the
    // nodes it enters them from, in order) because someone means to taxi it the other
    // way: they are on it already, or neither is and theirs is the lower id.
    bool mustYield(int who, int ap, const std::vector<std::pair<int, int>>& run) const;
    // How many others mean to taxi edge e at all.
    int using_(int who, int ap, int e) const;

    // ── Gates ────────────────────────────────────────────────────────────────
    bool gateFree(int ap, int gate, int except = -1) const;
    void takeGate(int who, int ap, int gate);
    void leaveGate(int who);
    int gateOf(int who) const;

    // ── Every tick ───────────────────────────────────────────────────────────
    // What stands where (vehicles not flown by ATC), and which runways that blocks.
    void update(const Airports& db, const std::vector<Obstacle>& obstacles, float now);
    const std::vector<Obstacle>& obstacles() const { return obstacles_; }
    // Opening or closing a runway (Nav_SetRunwayOpen).
    void setOpen(int ap, int rw, bool open) { closed_[key(ap, rw)] = !open; }
    // Everything a pilot holds, gone (its NPC was destroyed).
    void forget(int who);
    // Runways that count as one with rw (rw itself, and every one it crosses).
    std::vector<int> group(const Airports& db, int ap, int rw) const;

    void setDb(const Airports* db) { db_ = db; }

private:
    static long long key(int ap, int rw) { return (static_cast<long long>(ap) << 16) | static_cast<unsigned>(rw); }
    struct RunwayUse {
        int who = -1;
        Use use = Use::None;
        float until = 0.0f;     // expected free then (seconds, `now` clock)
    };
    struct Inbound {
        int ap = -1, rw = -1;
        float at = 0.0f;        // landing time (now + eta when it said so)
    };
    struct Claim {
        int ap = -1;
        std::vector<Pose> poses;
        float span = 0.0f, length = 0.0f;
        int waitingFor = -1;
    };
    struct Intent {
        int ap = -1;
        std::vector<std::pair<int, int>> edges;
    };
    bool freeFor(int who, int ap, int rw, float now) const;

    const Airports* db_ = nullptr;
    std::map<long long, RunwayUse> runways_;
    std::map<long long, bool> closed_, blocked_;
    std::unordered_map<int, Inbound> inbound_;
    std::unordered_map<int, Claim> claims_;
    std::unordered_map<int, Intent> intents_;
    std::map<long long, int> gates_;            // (airport, gate) -> pilot
    std::vector<Obstacle> obstacles_;
};

} // namespace dnav
