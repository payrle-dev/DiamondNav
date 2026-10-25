// A plane's pilot (pilot.hpp).

#include "pilot.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace dnav {

namespace {

constexpr float PI = 3.14159265f;
constexpr float DEG = PI / 180.0f;

constexpr float ASK_EVERY = 1.0f;           // s between asking ATC again
constexpr float CLAIM_STEP = 3.0f;          // m between the poses of a claim
constexpr float CLAIM_GAP = 2.0f;           // m kept between two planes on the ground
constexpr float ASK_CROSSING = 300.0f;      // m before a runway it asks to cross
constexpr float WAIT_IGNORE_AHEAD = 45.0f;  // s held up before it goes past others' way ahead
constexpr float WAIT_IGNORE_ALL = 120.0f;   // ...and past everyone (something is stuck)
constexpr float ON_RUNWAY_PAD = 10.0f;      // m round a runway that counts as on it
constexpr float GEAR_UP = 20.0f;            // m over the runway the gear comes up
constexpr float RELEASE_AT = 100.0f;        // m up a departure lets the runway go
constexpr float LOW_LEVEL = 150.0f;         // m up before it turns off the runway's heading
constexpr float APPROACH_FROM = 8000.0f;    // m past its final it starts the approach
constexpr float HOLD_IF_BUSY = 20.0f;       // s the runway will still be busy: it holds
constexpr float FLOOR_EVERY = 0.5f;         // s between looking at the ground ahead
constexpr float SEP_EVERY = 0.5f;           // s between looking at other planes
constexpr float SEP_RANGE = 1500.0f;        // m it looks for other planes in
constexpr float SEP_MISS = 150.0f;          // m past each other's wings it keeps
constexpr float SEP_HEIGHT = 120.0f;        // m one climbs over the other
constexpr float SEP_TIME = 30.0f;           // s ahead it looks

float clampf(float v, float lo, float hi)
{
    return std::max(lo, std::min(hi, v));
}

float dist2(const Vec3& a, const Vec3& b)
{
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

float headingTo(const Vec3& from, const Vec3& to)
{
    float h = std::atan2(-(to.x - from.x), to.y - from.y) / DEG;
    if (h < 0.0f) h += 360.0f;
    return h;
}

Vec3 add(const Vec3& a, const Vec3& b, float k)
{
    return { a.x + b.x * k, a.y + b.y * k, a.z + b.z * k };
}

Vec3 ground(const Vec3& p, float ride)
{
    return { p.x, p.y, p.z - ride };
}

} // namespace

const char* phaseName(PlanePhase p)
{
    switch (p) {
    case PlanePhase::None: return "none";
    case PlanePhase::Parked: return "parked";
    case PlanePhase::Pushback: return "pushback";
    case PlanePhase::TaxiOut: return "taxi out";
    case PlanePhase::HoldShort: return "hold short";
    case PlanePhase::LineUp: return "line up";
    case PlanePhase::Takeoff: return "take-off";
    case PlanePhase::Climb: return "climb";
    case PlanePhase::Cruise: return "cruise";
    case PlanePhase::Holding: return "holding";
    case PlanePhase::Approach: return "approach";
    case PlanePhase::Final: return "final";
    case PlanePhase::GoAround: return "go-around";
    case PlanePhase::Landing: return "landing";
    case PlanePhase::TaxiIn: return "taxi in";
    case PlanePhase::AtGate: return "at gate";
    case PlanePhase::Chase: return "chase";
    case PlanePhase::Loiter: return "loiter";
    }
    return "?";
}

Pilot::Pilot(int id_, int model)
    : id(id_)
    , spec(PlaneSpec::of(model))
{
}

void Pilot::setPhase(PlanePhase p)
{
    if (p == phase_) return;
    phase_ = p;
    since_ = 0.0f;
    phases.push_back(p);
}

void Pilot::finish(PilotEnd why)
{
    ended = static_cast<int>(why);
}

Vec3 Pilot::velocity() const
{
    Vec3 v = st.velocity();
    if (st.ground && path_.reverse && !path_.empty()) v = { -v.x, -v.y, -v.z };
    return v;
}

float Pilot::toTouchdown() const
{
    return phase_ == PlanePhase::Approach || phase_ == PlanePhase::Final || phase_ == PlanePhase::Holding ? toGo_ : -1.0f;
}

void Pilot::forget(PilotEnv& env)
{
    if (env.atc) env.atc->forget(id);
    gate_ = -1;
}

// ── Orders ───────────────────────────────────────────────────────────────────

void Pilot::place(PilotEnv& env, const Vec3& pos, float heading, bool onGround, float speed)
{
    forget(env);
    st = PlaneState {};
    st.pos = pos;
    st.heading = heading;
    st.ground = onGround;
    st.gearDown = onGround;
    st.speed = onGround ? 0.0f : (speed > 0.0f ? speed : spec.cruise);
    ap_ = env.db ? env.db->at(pos) : -1;
    route_.clear();
    landing_ = false;
    chase_ = -1;
    path_ = GroundPath {};
    if (onGround) {
        setPhase(PlanePhase::Parked);
        claimHere(env);
    } else {
        loiterHere(add(pos, forwardOf(heading), spec.turnRadius(st.speed) * 1.5f));
        setPhase(PlanePhase::Loiter);
    }
}

bool Pilot::park(PilotEnv& env, int ap, int g)
{
    const Airport* a = env.db->get(ap);
    if (!a || g < 0 || g >= static_cast<int>(a->gates.size()) || a->gates[g].removed) {
        why = "no such gate";
        return false;
    }
    if (!env.atc->gateFree(ap, g, id)) {
        why = "the gate is taken";
        return false;
    }
    const Gate& gt = a->gates[g];
    forget(env);
    st = PlaneState {};
    st.pos = { gt.p.x, gt.p.y, gt.p.z + spec.ride };
    st.heading = gt.heading;
    ap_ = ap;
    gate_ = g;
    env.atc->takeGate(id, ap, g);
    route_.clear();
    landing_ = false;
    chase_ = -1;
    path_ = GroundPath {};
    setPhase(PlanePhase::AtGate);
    claimHere(env);
    return true;
}

bool Pilot::taxiTo(PilotEnv& env, int g, int node)
{
    if (!st.ground || ap_ < 0) {
        why = "it is not on the ground at an airport";
        return false;
    }
    if (st.speed > spec.taxi * 1.5f) {
        why = "it is going too fast";
        return false;
    }
    route_.clear();
    landing_ = false;
    chase_ = -1;
    if (!planTaxi(env, g, node, true)) return false;
    ended = -1;
    return true;
}

bool Pilot::takeoff(PilotEnv& env, const std::string& runway, const std::vector<Vec3>& after)
{
    if (!st.ground || ap_ < 0) {
        why = "it is not on the ground at an airport";
        return false;
    }
    if (phase_ == PlanePhase::Takeoff || phase_ == PlanePhase::Landing) {
        why = "it is on a runway, rolling";
        return false;
    }
    departRunway_ = runway;
    route_ = after;
    leg_ = 0;
    loop_ = false;
    landing_ = false;
    chase_ = -1;
    if (!planDeparture(env)) return false;
    ended = -1;
    return true;
}

bool Pilot::land(PilotEnv& env, int ap, const std::string& runway, int g, const std::vector<Vec3>& via)
{
    if (!env.db->get(ap)) {
        why = "no such airport";
        return false;
    }
    chase_ = -1;
    if (st.ground) {
        if (ap_ == ap) {
            // Already there: it only taxis.
            route_.clear();
            landing_ = false;
            if (!planTaxi(env, g == -2 ? -3 : g, -1, true)) return false;
            ended = -1;
            return true;
        }
        departRunway_.clear();
        route_ = via;
        leg_ = 0;
        loop_ = false;
        landing_ = true;
        landAp_ = ap;
        landGate_ = g;
        landRunway_ = runway;
        if (!planDeparture(env)) {
            landing_ = false;
            return false;
        }
        ended = -1;
        return true;
    }
    route_ = via;
    leg_ = 0;
    loop_ = false;
    landing_ = true;
    landAp_ = ap;
    landGate_ = g;
    landRunway_ = runway;
    env.atc->notInbound(id);
    env.atc->releaseAll(id);
    cleared_ = false;
    ended = -1;
    startRoute(env);
    return true;
}

bool Pilot::fly(PilotEnv& env, const std::vector<Vec3>& points, float speed, bool loop)
{
    if (st.ground) {
        why = "it is on the ground (Nav_Takeoff first)";
        return false;
    }
    if (points.empty()) {
        why = "no points";
        return false;
    }
    env.atc->notInbound(id);
    env.atc->releaseAll(id);
    route_ = points;
    leg_ = 0;
    loop_ = loop;
    speed_ = speed;
    landing_ = false;
    chase_ = -1;
    ended = -1;
    startRoute(env);
    return true;
}

bool Pilot::chase(PilotEnv& env, int target, float speed, float height)
{
    if (st.ground) {
        why = "it is on the ground (Nav_Takeoff first)";
        return false;
    }
    env.atc->notInbound(id);
    env.atc->releaseAll(id);
    chase_ = target;
    speed_ = speed;
    chaseHeight_ = height > 0.0f ? height : 40.0f;
    landing_ = false;
    route_.clear();
    ended = -1;
    setPhase(PlanePhase::Chase);
    return true;
}

void Pilot::stop(PilotEnv& env)
{
    route_.clear();
    landing_ = false;
    chase_ = -1;
    if (!st.ground) {
        env.atc->notInbound(id);
        env.atc->releaseAll(id);
        loiterHere(add(st.pos, forwardOf(st.heading), spec.turnRadius(st.speed) * 1.5f));
        setPhase(PlanePhase::Loiter);
        return;
    }
    if (path_.empty() || phase_ == PlanePhase::AtGate || phase_ == PlanePhase::Parked || phase_ == PlanePhase::HoldShort) {
        if (phase_ == PlanePhase::HoldShort) setPhase(PlanePhase::Parked);
        return;
    }
    stopAt_ = s_ + st.speed * st.speed / (2.0f * std::max(brake_, spec.taxiBrake)) + 1.0f;
    after_ = After::Stop;
}

// ── The ground ───────────────────────────────────────────────────────────────

void Pilot::claimHere(PilotEnv& env)
{
    if (!env.atc || ap_ < 0) return;
    env.atc->setClaim(id, ap_, { Pose { ground(st.pos, spec.ride), st.heading } }, spec.span, spec.length, -1);
}

void Pilot::setLegs(PilotEnv& env)
{
    legs_.clear();
    const Airport* a = env.db->get(ap_);
    if (!a || path_.empty()) return;
    // Where along the path each node of the route is (the sample nearest it, in order).
    size_t from = 0;
    std::vector<float> at;
    for (int n : route_t_.nodes) {
        const Vec3& p = a->nodes[n].p;
        float best = 1e30f;
        size_t bi = from;
        for (size_t i = from; i < path_.pts.size(); i++) {
            const float d = dist2(path_.pts[i].p, p);
            if (d < best) {
                best = d;
                bi = i;
            }
        }
        from = bi;
        at.push_back(path_.pts[bi].s);
    }
    for (size_t i = 0; i < route_t_.edges.size() && i + 1 < at.size(); i++)
        legs_.push_back({ route_t_.edges[i], route_t_.nodes[i], at[i], at[i + 1] });
}

bool Pilot::startGround(PilotEnv& env, const GroundPath& path, After after, float brake)
{
    if (path.empty()) return false;
    path_ = path;
    s_ = 0.0f;
    after_ = after;
    brake_ = brake;
    waited_ = 0.0f;
    stopAt_ = 1e9f;
    findRanges(env);
    return true;
}

void Pilot::findRanges(PilotEnv& env)
{
    ranges_.clear();
    const Airport* a = env.db->get(ap_);
    if (!a) return;
    for (size_t r = 0; r < a->runways.size(); r++) {
        const Runway& rw = a->runways[r];
        // A closed runway is only a taxiway: nobody lands or takes off there to wait for.
        if (rw.removed || !env.atc->open(*env.db, ap_, static_cast<int>(r))) continue;
        Range cur;
        bool in = false;
        for (const GroundPoint& g : path_.pts) {
            const bool on = rw.contains(g.p, ON_RUNWAY_PAD);
            if (on && !in) {
                cur = Range {};
                cur.rw = static_cast<int>(r);
                cur.enter = g.s;
                in = true;
            }
            if (on) cur.exit = g.s;
            if (!on && in) {
                cur.cleared = env.atc->holds(id, ap_, cur.rw);
                ranges_.push_back(cur);
                in = false;
            }
        }
        if (in) {
            cur.cleared = env.atc->holds(id, ap_, cur.rw);
            ranges_.push_back(cur);
        }
    }
    std::sort(ranges_.begin(), ranges_.end(), [](const Range& x, const Range& y) { return x.enter < y.enter; });
}

std::vector<Vec3> Pilot::taxiPoints(PilotEnv&, const TaxiRoute& r, const Airport& a) const
{
    std::vector<Vec3> pts;
    for (int n : r.nodes) pts.push_back(a.nodes[n].p);
    return pts;
}

GroundPath Pilot::pushOff(const Airport& a, const Gate& g, const Vec3& next, std::vector<Vec3>& pts) const
{
    // The tug backs it out onto the taxiway and swings its tail round, so it ends a
    // little past the push node, facing the way it taxis on (through the node).
    const Vec3 pushTo = a.nodes[g.push].p;
    const Vec3 here = ground(st.pos, spec.ride);
    const float h = headingTo(pushTo, next);
    const float swing = clampf(dist2(here, pushTo) * 0.45f, spec.length * 0.4f, 60.0f);
    const Vec3 end = add(pushTo, forwardOf(h), -swing);
    pts.push_back(end);
    pts.push_back(pushTo);
    return pushPath(here, st.heading, end, h, spec.push);
}

bool Pilot::planTaxi(PilotEnv& env, int goalGate, int goalNode, bool fromGateAllowed)
{
    (void)fromGateAllowed;
    const Airport* a = env.db->get(ap_);
    if (!a) {
        why = "it is not at an airport";
        return false;
    }
    Atc& atc = *env.atc;
    // Where it goes: a gate (any that fits and is free, for -1 with landing), a node,
    // or off the runways.
    std::vector<int> goals;
    int toGate = -1;
    if (goalGate >= 0) {
        if (goalGate >= static_cast<int>(a->gates.size()) || a->gates[goalGate].removed) {
            why = "no such gate";
            return false;
        }
        if (a->gates[goalGate].span < spec.span) {
            why = "the gate is too small for it";
            return false;
        }
        if (!atc.gateFree(ap_, goalGate, id)) {
            why = "the gate is taken";
            return false;
        }
        toGate = goalGate;
    } else if (goalNode >= 0) {
        goals.push_back(goalNode);
    } else if (goalGate == -1) {
        // The smallest free gate it fits, the nearest of those.
        float best = 1e30f;
        for (size_t g = 0; g < a->gates.size(); g++) {
            const Gate& gt = a->gates[g];
            if (gt.removed || gt.span < spec.span || !atc.gateFree(ap_, static_cast<int>(g), id)) continue;
            const float score = (gt.span > 1e8f ? 200.0f : gt.span) * 50.0f + dist2(gt.p, st.pos);
            if (score < best) {
                best = score;
                toGate = static_cast<int>(g);
            }
        }
    }
    if (toGate >= 0) goals.push_back(a->gates[toGate].lead);
    if (goals.empty()) {
        // Nowhere to park: off the runways, clear of the holds.
        for (size_t i = 0; i < a->nodes.size(); i++) {
            const TaxiNode& n = a->nodes[i];
            if (n.removed || n.runway >= 0 || n.holdFor >= 0 || dist2(n.p, st.pos) < spec.length) continue;
            bool onAny = false;
            for (const auto& rw : a->runways)
                if (!rw.removed && rw.contains(n.p, spec.span * 0.5f + ON_RUNWAY_PAD)) onAny = true;
            if (!onAny) goals.push_back(static_cast<int>(i));
        }
    }
    if (goals.empty()) {
        why = "nowhere to taxi to";
        return false;
    }

    // Where it starts: off its gate (pushed back or nose first), or the node nearest.
    const Vec3 here = ground(st.pos, spec.ride);
    int from = -1;
    GroundPath push;
    const Gate* atGate = gate_ >= 0 && gate_ < static_cast<int>(a->gates.size()) ? &a->gates[gate_] : nullptr;
    if (atGate && dist2(atGate->p, here) > 5.0f) atGate = nullptr;
    if (atGate) from = atGate->push >= 0 ? atGate->push : atGate->lead;
    else from = env.db->nearestNode(*a, here, spec.span, 400.0f);
    if (from < 0) {
        why = "it is not near a taxiway it fits";
        return false;
    }
    auto opposed = [&](int e, int n) { return atc.opposed(id, ap_, e, n, a->edges[e]); };
    auto busy = [&](int e, int) { return 80.0f * atc.using_(id, ap_, e); };
    TaxiRoute route = env.db->taxiRoute(*a, from, goals, spec.span, opposed, busy);
    if (!route.ok()) route = env.db->taxiRoute(*a, from, goals, spec.span, {}, busy);
    if (!route.ok()) {
        why = "no taxi route there for its wings";
        return false;
    }
    std::vector<Vec3> pts;
    if (atGate && atGate->push >= 0) {
        const Vec3 next = route.nodes.size() > 1 ? a->nodes[route.nodes[1]].p : add(a->nodes[atGate->push].p, forwardOf(atGate->heading), -20.0f);
        push = pushOff(*a, *atGate, next, pts);
    } else {
        pts.push_back(here);
        // Pointing away from its first node: it starts forwards, and turns.
        if (!route.nodes.empty()) {
            const Vec3 first = a->nodes[route.nodes[0]].p;
            const float off = std::fabs(wrapDeg(headingTo(here, first) - st.heading));
            const float d = dist2(here, first);
            if (off > 30.0f && d > 1.0f) pts.push_back(add(here, forwardOf(st.heading), std::min(spec.taxiRadius, d * 0.5f) + 2.0f));
        }
    }
    for (const Vec3& p : taxiPoints(env, route, *a)) pts.push_back(p);
    if (toGate >= 0) {
        // Nose in, straight for its last stretch.
        const Gate& gt = a->gates[toGate];
        pts.push_back(add(gt.p, forwardOf(gt.heading), -(spec.taxiRadius + spec.length * 0.5f)));
        pts.push_back(gt.p);
    }
    const float v0 = push.empty() ? st.speed : 0.0f;
    GroundPath taxi = groundPath(pts, spec.taxiRadius, spec.taxi, spec.taxiCorner, spec.taxiAccel, spec.taxiBrake, v0, 0.0f);
    if (taxi.empty() && push.empty()) {
        // Already there.
        if (toGate >= 0) {
            atc.takeGate(id, ap_, toGate);
            gate_ = toGate;
            setPhase(PlanePhase::AtGate);
        } else {
            setPhase(PlanePhase::Parked);
        }
        finish(PilotEnd::Arrived);
        return true;
    }
    std::vector<std::pair<int, int>> intent;
    for (size_t i = 0; i < route.edges.size(); i++) intent.push_back({ route.edges[i], route.nodes[i] });
    atc.setIntent(id, ap_, intent);
    if (atGate) atc.leaveGate(id);
    gate_ = -1;
    if (toGate >= 0) {
        atc.takeGate(id, ap_, toGate);
        gate_ = toGate;
    }
    taxiGate_ = toGate;
    taxiNode_ = toGate >= 0 ? -1 : route.nodes.back();
    route_t_ = route;
    legs_.clear();
    const After done = toGate >= 0 ? After::Gate : After::Node;
    if (!push.empty()) {
        next_ = taxi;
        startGround(env, push, After::Push, 0.4f);
        nextAfter_ = done;
        nextPhase_ = PlanePhase::TaxiIn;
        setPhase(PlanePhase::Pushback);
    } else {
        startGround(env, taxi, done, spec.taxiBrake);
        setLegs(env);
        setPhase(PlanePhase::TaxiIn);
    }
    return true;
}

bool Pilot::pickDeparture(PilotEnv& env, int& rw, int& end) const
{
    const Airport* a = env.db->get(ap_);
    if (!a) return false;
    const float need = spec.takeoffRun() * 1.1f;
    // Which way it wants to go once up: its route's first point, or where it lands.
    float wantHeading = -1.0f;
    if (!route_.empty()) wantHeading = headingTo(st.pos, route_[0]);
    else if (landing_ && env.db->get(landAp_)) {
        const Airport* to = env.db->get(landAp_);
        const Vec3 c { (to->lo.x + to->hi.x) * 0.5f, (to->lo.y + to->hi.y) * 0.5f, 0.0f };
        wantHeading = headingTo(st.pos, c);
    }
    float best = 1e30f;
    rw = -1;
    for (size_t r = 0; r < a->runways.size(); r++) {
        const Runway& R = a->runways[r];
        if (R.removed || !R.open || R.span < spec.span || R.length < need || !env.atc->open(*env.db, ap_, static_cast<int>(r)))
            continue;
        if (!departRunway_.empty() && R.name != departRunway_ && R.endName[0] != departRunway_ && R.endName[1] != departRunway_)
            continue;
        for (int e = 0; e < 2; e++) {
            if (!R.depart[e]) continue;
            if (!departRunway_.empty() && departRunway_ != R.name && departRunway_ != R.endName[e]) continue;
            const float h = R.headingFrom(e);
            float score = 0.0f;
            if (a->prefer >= 0.0f) score += std::fabs(wrapDeg(h - a->prefer)) * 10.0f;
            if (wantHeading >= 0.0f) score += std::fabs(wrapDeg(h - wantHeading));
            // Nearer is better, a little.
            score += dist2(R.end[e], st.pos) * 0.02f;
            if (score < best) {
                best = score;
                rw = static_cast<int>(r);
                end = e;
            }
        }
    }
    return rw >= 0;
}

bool Pilot::planDeparture(PilotEnv& env)
{
    const Airport* a = env.db->get(ap_);
    if (!a) {
        why = "it is not at an airport";
        return false;
    }
    int rw = -1, e = 0;
    if (!pickDeparture(env, rw, e)) {
        why = departRunway_.empty() ? "no open runway long and wide enough for it" : "that runway is closed, too short or too narrow";
        return false;
    }
    Atc& atc = *env.atc;
    const Runway& R = a->runways[rw];
    const float need = spec.takeoffRun() * 1.2f;
    // Where it may start its roll: a node on the runway with enough of it ahead.
    std::vector<int> goals;
    for (int n : R.nodes) {
        const float ahead = e == 0 ? R.length - a->nodes[n].along : a->nodes[n].along;
        if (ahead >= need) goals.push_back(n);
    }
    if (goals.empty()) goals.push_back(e == 0 ? R.nodes.front() : R.nodes.back());
    const Vec3 here = ground(st.pos, spec.ride);
    const Gate* atGate = gate_ >= 0 && gate_ < static_cast<int>(a->gates.size()) ? &a->gates[gate_] : nullptr;
    if (atGate && dist2(atGate->p, here) > 5.0f) atGate = nullptr;
    int from = atGate ? (atGate->push >= 0 ? atGate->push : atGate->lead) : env.db->nearestNode(*a, here, spec.span, 400.0f);
    if (from < 0) {
        why = "it is not near a taxiway it fits";
        return false;
    }
    auto opposed = [&](int ei, int n) { return atc.opposed(id, ap_, ei, n, a->edges[ei]); };
    auto busy = [&](int ei, int) { return 80.0f * atc.using_(id, ap_, ei); };
    TaxiRoute route = env.db->taxiRoute(*a, from, goals, spec.span, opposed, busy);
    if (!route.ok()) route = env.db->taxiRoute(*a, from, goals, spec.span, {}, busy);
    if (!route.ok()) {
        why = "no taxi route to the runway for its wings";
        return false;
    }
    // The taxi out ends at the hold short: the node before the first on the runway.
    size_t k = 0;
    while (k < route.nodes.size() && a->nodes[route.nodes[k]].runway != rw) k++;
    std::vector<Vec3> out;
    GroundPath push;
    if (atGate && atGate->push >= 0) {
        const Vec3 next = route.nodes.size() > 1 ? a->nodes[route.nodes[1]].p : R.end[e];
        push = pushOff(*a, *atGate, next, out);
    } else {
        out.push_back(here);
        if (!route.nodes.empty()) {
            const Vec3 first = a->nodes[route.nodes[0]].p;
            const float off = std::fabs(wrapDeg(headingTo(here, first) - st.heading));
            const float d = dist2(here, first);
            if (off > 30.0f && d > 1.0f) out.push_back(add(here, forwardOf(st.heading), std::min(spec.taxiRadius, d * 0.5f) + 2.0f));
        }
    }
    for (size_t i = 0; i < k; i++) out.push_back(a->nodes[route.nodes[i]].p);
    // Lining up: from the hold onto the runway, along it to where the roll starts, and
    // round to face down it.
    const Vec3 dir = e == 0 ? R.dir : Vec3 { -R.dir.x, -R.dir.y, 0.0f };
    const float r = std::min(spec.taxiRadius, R.width * 0.45f);
    std::vector<Vec3> line;
    line.push_back(out.back());
    for (size_t i = k; i < route.nodes.size(); i++) line.push_back(a->nodes[route.nodes[i]].p);
    const Vec3 start = add(a->nodes[route.nodes.back()].p, dir, r + 5.0f);
    line.push_back({ start.x, start.y, R.pointAt(e == 0 ? a->nodes[route.nodes.back()].along + r + 5.0f
                                                    : a->nodes[route.nodes.back()].along - r - 5.0f).z });
    rw_ = rw;
    end_ = e;
    rollFrom_ = line.back();
    next_ = groundPath(line, r, spec.taxi * 0.6f, spec.taxiCorner, spec.taxiAccel, spec.taxiBrake, 0.0f, 0.0f);
    std::vector<std::pair<int, int>> intent;
    for (size_t i = 0; i < route.edges.size(); i++) intent.push_back({ route.edges[i], route.nodes[i] });
    atc.setIntent(id, ap_, intent);
    GroundPath taxi;
    if (out.size() >= 2) {
        const float v0 = push.empty() ? st.speed : 0.0f;
        taxi = groundPath(out, spec.taxiRadius, spec.taxi, spec.taxiCorner, spec.taxiAccel, spec.taxiBrake, v0, 0.0f);
    }
    if (atGate) atc.leaveGate(id);
    gate_ = -1;
    lineUp_ = next_;
    route_t_ = TaxiRoute {};
    for (size_t i = 0; i < k; i++) {
        route_t_.nodes.push_back(route.nodes[i]);
        if (i + 1 < k) route_t_.edges.push_back(route.edges[i]);
    }
    legs_.clear();
    if (!push.empty()) {
        next_ = taxi;
        nextAfter_ = After::Hold;
        nextPhase_ = PlanePhase::TaxiOut;
        startGround(env, push, After::Push, 0.4f);
        setPhase(PlanePhase::Pushback);
    } else if (!taxi.empty()) {
        startGround(env, taxi, After::Hold, spec.taxiBrake);
        setLegs(env);
        setPhase(PlanePhase::TaxiOut);
    } else {
        path_ = GroundPath {};
        setPhase(PlanePhase::HoldShort);
    }
    return true;
}

void Pilot::lineUp(PilotEnv& env)
{
    startGround(env, lineUp_, After::Roll, spec.taxiBrake);
    for (auto& r : ranges_)
        if (r.rw == rw_) {
            r.cleared = true;
            r.own = true;
        }
    setPhase(PlanePhase::LineUp);
}

void Pilot::groundStep(PilotEnv& env, float dt)
{
    Atc& atc = *env.atc;
    if (phase_ == PlanePhase::HoldShort) {
        claimHere(env);
        st.speed = 0.0f;
        if (env.now - holdAsk_ >= ASK_EVERY) {
            holdAsk_ = env.now;
            const float occupy = lineUp_.length() / std::max(2.0f, spec.taxi * 0.5f) + spec.rotate / spec.accel * 1.4f + 20.0f;
            if (atc.requestTakeoff(id, ap_, rw_, occupy, env.now)) lineUp(env);
        }
        return;
    }
    if (path_.empty()) {
        st.speed = 0.0f;
        claimHere(env);
        return;
    }
    float limit = std::min(path_.length(), stopAt_);
    bool runwayWait = false;
    // Runways on the way: across only once cleared, and let go once off.
    for (auto& r : ranges_) {
        if (r.rw < 0) continue;
        if (s_ - spec.length * 0.5f > r.exit + 5.0f) {
            if (r.cleared && !r.own) atc.release(id, ap_, r.rw);
            r.rw = -1;
            continue;
        }
        if (r.cleared) continue;
        const float hold = r.enter - spec.length * 0.5f - 3.0f;
        if (hold - s_ < ASK_CROSSING && env.now - holdAsk_ >= ASK_EVERY) {
            holdAsk_ = env.now;
            const float occupy = (r.exit - s_ + spec.length) / std::max(st.speed, 4.0f) + 10.0f;
            r.cleared = atc.requestCrossing(id, ap_, r.rw, occupy, env.now);
        }
        if (!r.cleared) {
            limit = std::min(limit, std::max(s_, hold));
            runwayWait = true;
            break;
        }
    }
    // Stretches someone means to taxi the other way: it waits before going onto one
    // they are on (or will get to first), and tells ATC what it still means to taxi.
    bool yielding = false;
    if (!legs_.empty() && !runwayWait) {
        bool passed = false;
        while (!legs_.empty() && legs_.front().s1 < s_ - spec.length * 0.5f) {
            legs_.erase(legs_.begin());
            passed = true;
        }
        if (passed) {
            std::vector<std::pair<int, int>> intent;
            for (const Leg& l : legs_) intent.push_back({ l.edge, l.from });
            atc.setIntent(id, ap_, intent);
        }
        const Airport* a = env.db->get(ap_);
        for (size_t i = 0; a && i < legs_.size(); i++) {
            const Leg& l = legs_[i];
            if (l.s0 > s_ + 400.0f) break;
            if (!atc.opposed(id, ap_, l.edge, l.from, a->edges[l.edge])) continue;
            if (s_ + spec.length * 0.5f > l.s0) break;     // on it already: through
            std::vector<std::pair<int, int>> run;
            for (size_t j = i; j < legs_.size() && atc.opposed(id, ap_, legs_[j].edge, legs_[j].from, a->edges[legs_[j].edge]); j++)
                run.push_back({ legs_[j].edge, legs_[j].from });
            if (atc.mustYield(id, ap_, run)) {
                limit = std::min(limit, std::max(s_, l.s0 - std::max(spec.span, spec.length) * 0.5f - 5.0f));
                yielding = true;
            }
            break;
        }
    }
    // Everyone else's claims.
    const float stopDist = st.speed * st.speed / (2.0f * brake_) + 8.0f;
    std::vector<Pose> ahead;
    // A pushback claims all of its way (a tug cannot stop and go round), and only
    // starts once all of it is clear.
    const float reach = path_.reverse ? path_.length() : stopDist + 15.0f;
    for (float d = CLAIM_STEP; s_ + d < limit + CLAIM_STEP && d <= reach; d += CLAIM_STEP) {
        const GroundPoint g = path_.at(std::min(s_ + d, limit));
        ahead.push_back({ g.p, g.heading });
    }
    const int ignore = waited_ > WAIT_IGNORE_ALL ? 2 : waited_ > WAIT_IGNORE_AHEAD ? 1 : 0;
    int by = -1;
    const size_t k = atc.clearAhead(id, ap_, ahead, spec.span, spec.length, CLAIM_GAP, by, ignore);
    blocker_ = -1;
    if (k < ahead.size()) {
        limit = std::min(limit, path_.reverse && s_ < 0.5f ? s_ : s_ + CLAIM_STEP * static_cast<float>(k));
        blocker_ = by;
    }
    std::vector<Pose> claim;
    claim.push_back({ ground(st.pos, spec.ride), st.heading });
    if (!(path_.reverse && s_ < 0.5f && k < ahead.size()))
        for (size_t i = 0; i < std::min(k, ahead.size()); i++) claim.push_back(ahead[i]);
    atc.setClaim(id, ap_, claim, spec.span, spec.length, blocker_ >= 0 ? blocker_ : -1);

    // Speed: the plan's, and stopping in time for the limit.
    const float room = std::max(0.0f, limit - s_);
    float target = std::min(path_.at(s_).vmax, path_.at(std::min(s_ + 2.0f, path_.length())).vmax);
    // (The plan slows to nothing at its end; stopping there is the room's to say.)
    target = std::min(std::max(target, 0.5f), std::sqrt(2.0f * brake_ * room));
    if (room < 0.2f) {
        target = 0.0f;
        s_ = std::max(s_, limit);
    }
    const float accel = path_.reverse ? 0.3f : phase_ == PlanePhase::Landing ? 0.0f : spec.taxiAccel;
    if (st.speed < target) st.speed = std::min(target, st.speed + accel * dt);
    else st.speed = std::max(target, st.speed - brake_ * 1.8f * dt);
    s_ = std::min(s_ + st.speed * dt, std::max(s_, limit));
    if (st.speed < 0.3f && !runwayWait && !yielding && limit < path_.length() - 0.5f && stopAt_ > 1e8f) waited_ += dt;
    else if (st.speed > 1.0f) waited_ = 0.0f;

    const GroundPoint g = path_.at(s_);
    st.pos = { g.p.x, g.p.y, g.p.z + spec.ride };
    st.heading = g.heading;
    st.vz = 0.0f;
    st.bank = 0.0f;
    st.pitch += (0.0f - st.pitch) * std::min(1.0f, dt * 1.5f);
    st.yawRate = 0.0f;

    if (s_ >= limit - 0.05f && stopAt_ < 1e8f && st.speed < 0.1f) {
        // Stopped where it was told to.
        path_ = GroundPath {};
        stopAt_ = 1e9f;
        atc.dropIntent(id);
        setPhase(PlanePhase::Parked);
        finish(PilotEnd::Stopped);
        return;
    }
    if (s_ >= path_.length() - 0.05f) groundDone(env);
}

void Pilot::groundDone(PilotEnv& env)
{
    Atc& atc = *env.atc;
    // Anything it crossed is behind it now.
    for (auto& r : ranges_)
        if (r.rw >= 0 && r.cleared && !r.own) atc.release(id, ap_, r.rw);
    ranges_.clear();
    switch (after_) {
    case After::Push:
        if (next_.empty()) {
            path_ = GroundPath {};
            if (nextAfter_ == After::Hold) setPhase(PlanePhase::HoldShort);
            else {
                setPhase(PlanePhase::Parked);
                finish(PilotEnd::Arrived);
            }
            return;
        }
        startGround(env, next_, nextAfter_, spec.taxiBrake);
        setLegs(env);
        setPhase(nextPhase_);
        return;
    case After::Hold:
        path_ = GroundPath {};
        st.speed = 0.0f;
        setPhase(PlanePhase::HoldShort);
        return;
    case After::Roll: {
        path_ = GroundPath {};
        const Airport* a = env.db->get(ap_);
        const Runway& R = a->runways[rw_];
        rollFrom_ = ground(st.pos, spec.ride);
        fwd_ = end_ == 0 ? R.dir : Vec3 { -R.dir.x, -R.dir.y, 0.0f };
        st.heading = R.headingFrom(end_);
        roll_ = 0.0f;
        atc.dropIntent(id);
        setPhase(PlanePhase::Takeoff);
        return;
    }
    case After::TaxiIn: {
        // Off the landing roll: to its gate.
        path_ = GroundPath {};
        if (!planTaxi(env, landGate_ == -2 ? -3 : landGate_, -1, false)) {
            // Its gate is gone (or there is none): anywhere off the runways.
            if (!planTaxi(env, -3, -1, false)) {
                atc.releaseAll(id);
                setPhase(PlanePhase::Parked);
                finish(PilotEnd::NoPath);
            }
        }
        return;
    }
    case After::Gate:
        path_ = GroundPath {};
        st.speed = 0.0f;
        atc.dropIntent(id);
        atc.releaseAll(id);
        setPhase(PlanePhase::AtGate);
        claimHere(env);
        finish(PilotEnd::Arrived);
        return;
    case After::Node:
    case After::Stop:
        path_ = GroundPath {};
        st.speed = 0.0f;
        atc.dropIntent(id);
        atc.releaseAll(id);
        setPhase(PlanePhase::Parked);
        claimHere(env);
        finish(PilotEnd::Arrived);
        return;
    }
}

void Pilot::takeoffStep(PilotEnv& env, float dt)
{
    const Airport* a = env.db->get(ap_);
    if (!a || rw_ < 0) return;
    const Runway& R = a->runways[rw_];
    const float acc = spec.accel * std::max(0.35f, 1.0f - 0.35f * st.speed / spec.rotate);
    st.speed += acc * dt;
    roll_ += st.speed * dt;
    const Vec3 p = add(rollFrom_, fwd_, roll_);
    float along, off;
    R.place(p, along, off);
    const float surface = R.pointAt(clampf(along, 0.0f, R.length)).z;
    if (st.speed >= spec.rotate) st.pitch = std::min(spec.rotatePitch, st.pitch + 3.0f * dt);
    st.pos = { p.x, p.y, surface + spec.ride };
    st.heading = R.headingFrom(end_);
    env.atc->dropClaim(id);
    const bool overrun = end_ == 0 ? along > R.length - 20.0f : along < 20.0f;
    if (st.pitch >= spec.rotatePitch * 0.7f || overrun) {
        st.ground = false;
        st.vz = 1.5f;
        rwZ_ = surface;
        setPhase(PlanePhase::Climb);
    }
}

// ── The air ──────────────────────────────────────────────────────────────────

void Pilot::loiterHere(const Vec3& c)
{
    loiter_ = c;
    loiterR_ = std::max(400.0f, spec.turnRadius(std::max(st.speed, spec.minimum + 10.0f)) * 1.4f);
}

float Pilot::cruiseZ(PilotEnv& env) const
{
    float z = st.pos.z;
    if (landing_) {
        const Airport* to = env.db->get(landAp_);
        if (to) z = std::max(z, to->hi.z + (spec.span > 40.0f ? 450.0f : 250.0f));
    }
    return z;
}

void Pilot::steerTo(const Vec3& to, AirCommand& c) const
{
    c.heading = headingTo(st.pos, to);
}

void Pilot::steerOrbit(const Vec3& centre, float r, AirCommand& cmd) const
{
    // Round to the left, pulled in (or out) to the circle.
    const float rx = st.pos.x - centre.x, ry = st.pos.y - centre.y;
    const float d = std::max(1.0f, std::sqrt(rx * rx + ry * ry));
    const float tx = -ry / d, ty = rx / d;
    const float k = clampf((d - r) / r, -1.0f, 1.0f) * 1.5f;
    const float dx = tx - rx / d * k, dy = ty - ry / d * k;
    float h = std::atan2(-dx, dy) / DEG;
    if (h < 0.0f) h += 360.0f;
    cmd.heading = h;
}

void Pilot::startRoute(PilotEnv& env)
{
    leg_ = 0;
    if (!route_.empty()) {
        setPhase(PlanePhase::Cruise);
        return;
    }
    if (landing_) {
        setPhase(PlanePhase::Cruise);
        (void)env;
        return;
    }
    loiterHere(add(st.pos, forwardOf(st.heading), spec.turnRadius(st.speed) * 1.5f));
    setPhase(PlanePhase::Loiter);
    finish(PilotEnd::Arrived);
}

void Pilot::afterDeparture(PilotEnv& env)
{
    ap_ = -1;
    if (chase_ >= 0) {
        setPhase(PlanePhase::Chase);
        return;
    }
    startRoute(env);
}

bool Pilot::pickLanding(PilotEnv& env, int& rw, int& end) const
{
    const Airport* a = env.db->get(landAp_);
    if (!a) return false;
    const float need = spec.landingRun();
    float best = 1e30f;
    rw = -1;
    for (size_t r = 0; r < a->runways.size(); r++) {
        const Runway& R = a->runways[r];
        if (R.removed || R.span < spec.span || !env.atc->open(*env.db, landAp_, static_cast<int>(r))) continue;
        if (!landRunway_.empty() && R.name != landRunway_ && R.endName[0] != landRunway_ && R.endName[1] != landRunway_)
            continue;
        for (int e = 0; e < 2; e++) {
            if (!R.land[e]) continue;
            if (!landRunway_.empty() && landRunway_ != R.name && landRunway_ != R.endName[e]) continue;
            const float avail = R.length - R.displaced[e];
            if (avail < need) continue;
            const float h = R.headingFrom(e);
            float score = 0.0f;
            if (a->prefer >= 0.0f) score += std::fabs(wrapDeg(h - a->prefer)) * 10.0f;
            // Coming from behind the threshold is better than flying round to it.
            const Vec3 thr = R.pointAt(R.thresholdAlong(e));
            score += std::fabs(wrapDeg(headingTo(st.pos, thr) - h)) * 2.0f;
            // A runway with room to spare is better.
            score -= std::min(avail - need, 1500.0f) * 0.02f;
            if (score < best) {
                best = score;
                rw = static_cast<int>(r);
                end = e;
            }
        }
    }
    return rw >= 0;
}

float Pilot::surfaceZ(const Runway& R) const
{
    float along, off;
    R.place(st.pos, along, off);
    return R.pointAt(clampf(along, 0.0f, R.length)).z;
}

bool Pilot::startApproach(PilotEnv& env)
{
    int rw = -1, e = 0;
    if (!pickLanding(env, rw, e)) {
        landing_ = false;
        loiterHere(add(st.pos, forwardOf(st.heading), spec.turnRadius(st.speed) * 1.5f));
        setPhase(PlanePhase::Loiter);
        why = "no open runway it can land on";
        finish(PilotEnd::NoPath);
        return false;
    }
    ap_ = landAp_;
    rw_ = rw;
    end_ = e;
    const Runway& R = env.db->get(ap_)->runways[rw];
    fwd_ = e == 0 ? R.dir : Vec3 { -R.dir.x, -R.dir.y, 0.0f };
    const float avail = R.length - R.displaced[e];
    const float aimDist = std::min(spec.span > 40.0f ? 300.0f : 150.0f, avail * 0.15f);
    const float thr = R.thresholdAlong(e);
    aimAlong_ = e == 0 ? thr + aimDist : thr - aimDist;
    aimP_ = R.pointAt(aimAlong_);
    // The pattern is flown at the glide path's height a turn inside the base leg, so the
    // final picks the glide path up from below.
    patternZ_ = aimP_.z + spec.ride + (spec.finalLength + 1.5f * spec.turnRadius(spec.approach + 10.0f) * 1.1f) * std::tan(spec.glide * DEG);
    established_ = false;
    cleared_ = false;
    planPattern(env);
    setPhase(PlanePhase::Approach);
    return true;
}

void Pilot::planPattern(PilotEnv& env)
{
    fixes_.clear();
    fix_ = 0;
    established_ = false;
    const Runway& R = env.db->get(ap_)->runways[rw_];
    if (!R.approach[end_].empty()) {
        fixes_ = R.approach[end_];
        return;
    }
    const float Rt = spec.turnRadius(spec.approach + 10.0f) * 1.1f;
    const Vec3 left { -fwd_.y, fwd_.x, 0.0f };
    const float d = (aimP_.x - st.pos.x) * fwd_.x + (aimP_.y - st.pos.y) * fwd_.y;
    const float x = (st.pos.x - aimP_.x) * left.x + (st.pos.y - aimP_.y) * left.y;
    const float side = x >= 0.0f ? 1.0f : -1.0f;
    const float outer = spec.finalLength + 3.5f * Rt;
    // Coming in from behind the runway: straight onto the final.
    if (d > outer && std::fabs(x) < (d - spec.finalLength) * 1.2f) return;
    const float W = 2.5f * Rt;
    auto at = [&](float along, float across) {
        Vec3 p = add(aimP_, fwd_, along);
        p = add(p, left, across);
        p.z = patternZ_;
        return p;
    };
    // Past the far end: across to the downwind side first.
    if (d < -R.length * 0.5f) fixes_.push_back(at(R.length + Rt, side * W));
    // Downwind, abeam the runway, then the base turn onto the final.
    if (d < outer) fixes_.push_back(at(0.0f, side * W));
    fixes_.push_back(at(-outer, side * W));
}

void Pilot::goAround(PilotEnv& env)
{
    env.atc->release(id, ap_, rw_);
    cleared_ = false;
    established_ = false;
    goArounds_++;
    setPhase(PlanePhase::GoAround);
}

void Pilot::touchDown(PilotEnv& env)
{
    const Airport* a = env.db->get(ap_);
    const Runway& R = a->runways[rw_];
    st.ground = true;
    st.vz = 0.0f;
    st.bank = 0.0f;
    float along, off;
    R.place(st.pos, along, off);
    const float sign = end_ == 0 ? 1.0f : -1.0f;
    // Where it turns off: the first exit it can slow down for.
    const float exitSpeed = std::min(spec.taxi, std::sqrt(spec.taxiCorner * spec.taxiRadius) * 1.5f);
    const float rollBrake = spec.brake * 0.6f;
    const float need = std::max(0.0f, (st.speed * st.speed - exitSpeed * exitSpeed) / (2.0f * rollBrake));
    int exit = -1, last = -1;
    for (size_t i = 0; i < R.nodes.size(); i++) {
        const int n = end_ == 0 ? R.nodes[i] : R.nodes[R.nodes.size() - 1 - i];
        const float ahead = (a->nodes[n].along - along) * sign;
        if (ahead < 0.0f) continue;
        last = n;
        bool off = false;
        for (int ei : a->nodes[n].edges) {
            const TaxiEdge& te = a->edges[ei];
            if (!te.removed && te.span >= spec.span && a->nodes[te.other(n)].runway != rw_) off = true;
        }
        if (off && ahead >= need) {
            exit = n;
            break;
        }
    }
    if (exit < 0) exit = last;
    std::vector<Vec3> pts;
    pts.push_back(ground(st.pos, spec.ride));
    const float settle = std::min(120.0f, std::max(20.0f, (a->nodes[exit >= 0 ? exit : 0].along - along) * sign * 0.3f));
    pts.push_back(R.pointAt(along + sign * settle));
    if (exit >= 0) pts.push_back(a->nodes[exit].p);
    else pts.push_back(R.pointAt(end_ == 0 ? R.length : 0.0f));
    GroundPath roll = groundPath(pts, spec.taxiRadius * 4.0f, st.speed, spec.taxiCorner, spec.taxiAccel, rollBrake, st.speed,
        exitSpeed);
    env.atc->notInbound(id);
    startGround(env, roll, After::TaxiIn, rollBrake);
    for (auto& r : ranges_)
        if (r.rw == rw_) {
            r.cleared = true;
            r.own = true;
        }
    setPhase(PlanePhase::Landing);
}

float Pilot::lookAhead() const
{
    // Far enough to climb over the highest ground there is at what it climbs.
    return std::max(1500.0f, st.speed * 45.0f);
}

float Pilot::floorAhead(PilotEnv& env, float heading, float clear)
{
    return terrainFloor(env.world, st.pos, heading, std::max(st.speed, spec.minimum), spec, clear, lookAhead());
}

void Pilot::steerClear(PilotEnv& env, AirCommand& c)
{
    if (!env.world.top) return;
    const float clear = spec.span > 40.0f ? 80.0f : 50.0f;
    if (floorIn_ <= 0.0f || std::fabs(wrapDeg(c.heading - floorHeading_)) > 20.0f) {
        floorIn_ = FLOOR_EVERY;
        floorHeading_ = c.heading;
        floor_ = floorAhead(env, c.heading, clear);
        // Too high to climb over in time: the heading with the lowest ground, nearest
        // the one it wants.
        const float look = lookAhead();
        const float canClimb = spec.climb * 0.7f * look / std::max(st.speed, spec.minimum);
        if (floor_ - st.pos.z > canClimb && escape_ <= 0.0f) {
            float best = floor_;
            float bestHeading = c.heading;
            for (float off : { 45.0f, -45.0f, 90.0f, -90.0f, 135.0f, -135.0f, 180.0f }) {
                const float h = std::fmod(c.heading + off + 360.0f, 360.0f);
                const float f = floorAhead(env, h, clear) + std::fabs(off) * 0.5f;
                if (f < best) {
                    best = f;
                    bestHeading = h;
                }
            }
            if (bestHeading != c.heading) {
                escapeHeading_ = bestHeading;
                escape_ = 12.0f;
            }
        }
    }
    if (escape_ > 0.0f) c.heading = escapeHeading_;
    if (floor_ > c.z) {
        c.z = floor_;
        c.holdVz = false;
    }
    // Under it: climbing as hard as it can (a height to make for only climbs in
    // proportion, and would lag behind the ground coming up).
    if (floor_ > st.pos.z) {
        c.holdVz = true;
        c.vz = spec.climb;
    }
}

void Pilot::separate(PilotEnv& env, AirCommand& c)
{
    if (sepIn_ <= 0.0f) {
        sepIn_ = SEP_EVERY;
        sep_ = false;
        for (const Pilot* o : env.others) {
            if (!o || o == this || o->st.ground) continue;
            const PlanePhase op = o->phase();
            if (op == PlanePhase::Final && phase_ == PlanePhase::Final) continue;
            const float dx = o->st.pos.x - st.pos.x, dy = o->st.pos.y - st.pos.y, dz = o->st.pos.z - st.pos.z;
            if (std::fabs(dx) > SEP_RANGE || std::fabs(dy) > SEP_RANGE || std::fabs(dz) > SEP_HEIGHT * 2.0f) continue;
            const Vec3 va = st.velocity(), vb = o->st.velocity();
            const float vx = vb.x - va.x, vy = vb.y - va.y, vz = vb.z - va.z;
            const float vv = vx * vx + vy * vy;
            const float t = clampf(vv > 1e-3f ? -(dx * vx + dy * vy) / vv : 0.0f, 0.0f, SEP_TIME);
            const float mx = dx + vx * t, my = dy + vy * t, mz = dz + vz * t;
            const float need = (spec.span + o->spec.span) * 0.5f + SEP_MISS;
            if (mx * mx + my * my > need * need || std::fabs(mz) > SEP_HEIGHT * 0.75f) continue;
            // Both turn right; the higher id goes over the other.
            sep_ = true;
            sepHeading_ = std::fmod(c.heading - 35.0f + 360.0f, 360.0f);
            sepZ_ = id > o->id ? o->st.pos.z + SEP_HEIGHT : c.z;
            break;
        }
    }
    if (!sep_) return;
    // On the final it only climbs over (the runway is its): never off the centre line.
    if (phase_ != PlanePhase::Final) c.heading = sepHeading_;
    if (sepZ_ > c.z) {
        c.z = sepZ_;
        c.holdVz = false;
    }
}

void Pilot::airStep(PilotEnv& env, float dt)
{
    since_ += dt;
    floorIn_ -= dt;
    sepIn_ -= dt;
    escape_ -= dt;
    AirCommand c;
    c.heading = st.heading;
    c.z = st.pos.z;
    c.speed = speed_ > 0.0f ? speed_ : spec.cruise;
    bool clearOfGround = true;
    const Airport* a = env.db ? env.db->get(ap_) : nullptr;
    switch (phase_) {
    case PlanePhase::Climb: {
        const Runway* R = a && rw_ >= 0 ? &a->runways[rw_] : nullptr;
        const float agl = st.pos.z - spec.ride - rwZ_;
        c.heading = R ? R->headingFrom(end_) : st.heading;
        c.z = std::max(rwZ_ + (spec.span > 40.0f ? 450.0f : 250.0f), cruiseZ(env));
        c.speed = agl < 300.0f ? spec.rotate + 15.0f : c.speed;
        c.pitchExtra = agl < 30.0f ? std::max(0.0f, spec.rotatePitch - 6.0f) : 0.0f;
        if (agl > GEAR_UP) st.gearDown = false;
        if (agl > RELEASE_AT && env.atc->holds(id, ap_, rw_)) env.atc->release(id, ap_, rw_);
        clearOfGround = agl > LOW_LEVEL;
        // Its departure fixes, once high enough to turn.
        if (R && agl > LOW_LEVEL && !R->departure[end_].empty()) {
            if (fix_ < R->departure[end_].size()) {
                const Vec3& f = R->departure[end_][fix_];
                steerTo(f, c);
                c.z = std::max(c.z, f.z);
                if (dist2(st.pos, f) < spec.turnRadius(st.speed) * 0.8f + 100.0f) fix_++;
                break;
            }
        }
        if (agl > LOW_LEVEL + 50.0f && (R == nullptr || R->departure[end_].empty() || fix_ >= R->departure[end_].size())) {
            if (env.atc->holds(id, ap_, rw_)) env.atc->release(id, ap_, rw_);
            fix_ = 0;
            afterDeparture(env);
        }
        break;
    }
    case PlanePhase::Cruise: {
        if (leg_ < route_.size()) {
            const Vec3& to = route_[leg_];
            steerTo(to, c);
            c.z = to.z;
            // Turning for the next leg in time.
            float lead = 60.0f;
            if (leg_ + 1 < route_.size() || loop_) {
                const Vec3& nx = route_[leg_ + 1 < route_.size() ? leg_ + 1 : 0];
                const float turn = std::fabs(wrapDeg(headingTo(to, nx) - headingTo(st.pos, to))) * DEG;
                lead += spec.turnRadius(st.speed) * std::tan(std::min(turn, 2.6f) * 0.5f);
            }
            if (dist2(st.pos, to) < lead) {
                leg_++;
                if (leg_ >= route_.size()) {
                    if (loop_) leg_ = 0;
                    else if (!landing_) {
                        loiterHere(to);
                        setPhase(PlanePhase::Loiter);
                        finish(PilotEnd::Arrived);
                    }
                }
            }
        } else if (landing_) {
            const Airport* to = env.db->get(landAp_);
            if (!to) {
                landing_ = false;
                finish(PilotEnd::NoPath);
                loiterHere(st.pos);
                setPhase(PlanePhase::Loiter);
                break;
            }
            const Vec3 centre { (to->lo.x + to->hi.x) * 0.5f, (to->lo.y + to->hi.y) * 0.5f, to->hi.z };
            steerTo(centre, c);
            c.z = cruiseZ(env);
            const float reach = spec.finalLength + APPROACH_FROM + std::max(to->hi.x - to->lo.x, to->hi.y - to->lo.y) * 0.5f;
            if (dist2(st.pos, centre) < reach) startApproach(env);
        } else {
            loiterHere(st.pos);
            setPhase(PlanePhase::Loiter);
        }
        break;
    }
    case PlanePhase::Approach:
    case PlanePhase::Final:
    case PlanePhase::Holding: {
        if (!a || rw_ < 0) {
            setPhase(PlanePhase::Loiter);
            break;
        }
        const Runway& R = a->runways[rw_];
        const Vec3 left { -fwd_.y, fwd_.x, 0.0f };
        const float d = (aimP_.x - st.pos.x) * fwd_.x + (aimP_.y - st.pos.y) * fwd_.y;
        const float x = (st.pos.x - aimP_.x) * left.x + (st.pos.y - aimP_.y) * left.y;
        const float course = R.headingFrom(end_);
        const float tanG = std::tan(spec.glide * DEG);
        const float zGP = aimP_.z + spec.ride + std::max(0.0f, d) * tanG;
        // How far it still flies to touch down.
        float toGo = std::max(0.0f, d);
        if (fix_ < fixes_.size()) {
            toGo = dist2(st.pos, fixes_[fix_]);
            for (size_t i = fix_ + 1; i < fixes_.size(); i++) toGo += dist2(fixes_[i - 1], fixes_[i]);
            toGo += dist2(fixes_.back(), aimP_);
        }
        toGo_ = toGo;
        const float eta = toGo / std::max(st.speed, spec.approach);
        if (env.now - askAt_ >= ASK_EVERY) {
            askAt_ = env.now;
            env.atc->inbound(id, ap_, rw_, eta, env.now);
            if (phase_ == PlanePhase::Approach && !established_ && fix_ < fixes_.size()
                && env.atc->busyFor(id, ap_, rw_, eta, env.now) > HOLD_IF_BUSY) {
                holdAt_ = fixes_[fix_];
                setPhase(PlanePhase::Holding);
            } else if (phase_ == PlanePhase::Holding && env.atc->busyFor(id, ap_, rw_, eta, env.now) < 5.0f) {
                planPattern(env);
                setPhase(PlanePhase::Approach);
            }
            if (phase_ == PlanePhase::Final && !cleared_) cleared_ = env.atc->requestLanding(id, ap_, rw_, eta, env.now);
        }
        const float Rt = spec.turnRadius(spec.approach + 10.0f) * 1.1f;
        if (phase_ == PlanePhase::Holding) {
            steerOrbit(holdAt_, Rt * 1.2f, c);
            c.z = patternZ_ + 150.0f;
            c.speed = spec.approach + 20.0f;
            break;
        }
        if (fix_ < fixes_.size()) {
            const Vec3& f = fixes_[fix_];
            steerTo(f, c);
            c.z = std::max(patternZ_, f.z);
            c.speed = toGo > spec.finalLength + APPROACH_FROM ? c.speed : spec.approach + 20.0f;
            if (dist2(st.pos, f) < Rt * 0.8f + 100.0f) fix_++;
            break;
        }
        // Onto the centre line, and down the glide path once on it.
        const float L = std::max(Rt * (phase_ == PlanePhase::Final ? 0.6f : 1.2f), phase_ == PlanePhase::Final ? 500.0f : 800.0f);
        const Vec3 carrot = d - L > 0.0f ? add(aimP_, fwd_, -(d - L)) : add(aimP_, fwd_, 200.0f);
        steerTo(carrot, c);
        const float herr = std::fabs(wrapDeg(st.heading - course));
        if (!established_ && std::fabs(x) < 60.0f + d * 0.01f && herr < 15.0f) established_ = true;
        const float agl = st.pos.z - spec.ride - surfaceZ(R);
        if (established_ && agl < 60.0f) c.maxBank = 10.0f;
        if (!established_) {
            c.z = std::min(patternZ_, std::max(zGP, aimP_.z + 150.0f));
            c.speed = spec.approach + 15.0f;
            if (d < spec.finalLength * 0.5f) goAround(env);
            break;
        }
        clearOfGround = false;
        if (phase_ == PlanePhase::Approach && d < spec.finalLength + 800.0f) setPhase(PlanePhase::Final);
        c.speed = phase_ == PlanePhase::Final ? spec.approach : spec.approach + 10.0f;
        if (d < spec.finalLength + 3000.0f) st.gearDown = true;
        c.holdVz = true;
        c.vz = clampf(-st.speed * tanG + 0.25f * (zGP - st.pos.z), -spec.sink, spec.climb * 0.5f);
        if (phase_ != PlanePhase::Final) break;
        // The decision point: cleared, lined up and on the glide path, or around again.
        const float decision = std::max(250.0f, st.speed * 7.0f);
        if (d < decision && d > 0.0f) {
            const bool stable = std::fabs(x) < R.width * 0.4f + 10.0f && herr < 12.0f && std::fabs(st.pos.z - zGP) < 30.0f;
            if (!cleared_ || !stable) {
                why = !cleared_ ? "go-around: not cleared to land" : "go-around: not lined up on the glide path";
                goAround(env);
                break;
            }
        }
        // The flare, and down.
        if (agl < spec.flare) {
            const float k = clampf(1.0f - agl / spec.flare, 0.0f, 1.0f);
            c.pitchExtra = (spec.touchPitch - spec.attitude) * k;
            c.vz = -(st.speed * tanG * (1.0f - k) + 0.7f * k);
            c.speed = spec.touchdown;
            c.maxBank = 3.0f;
        }
        // Floated so far there is not the runway left to stop on.
        const Vec3& far = R.end[1 - end_];
        const float runwayLeft = (far.x - st.pos.x) * fwd_.x + (far.y - st.pos.y) * fwd_.y;
        if (runwayLeft < spec.landingRun() + 30.0f && agl > 3.0f) {
            why = "go-around: landing long";
            goAround(env);
            break;
        }
        break;
    }
    case PlanePhase::GoAround: {
        const Runway* R = a && rw_ >= 0 ? &a->runways[rw_] : nullptr;
        c.heading = R ? R->headingFrom(end_) : st.heading;
        c.z = patternZ_ + 50.0f;
        c.speed = spec.approach + 25.0f;
        if (st.vz > 2.0f) st.gearDown = false;
        clearOfGround = st.pos.z > (R ? surfaceZ(*R) : 0.0f) + LOW_LEVEL;
        if (since_ > 30.0f && st.pos.z > patternZ_ - 30.0f) {
            env.atc->notInbound(id);
            startApproach(env);
        }
        break;
    }
    case PlanePhase::Chase: {
        Vec3 tp, tv;
        if (!env.target || !env.target(chase_, tp, tv)) {
            chase_ = -1;
            loiterHere(st.pos);
            setPhase(PlanePhase::Loiter);
            finish(PilotEnd::Lost);
            break;
        }
        const float dist = dist2(st.pos, tp);
        const float lead = std::min(10.0f, dist / std::max(st.speed, 1.0f));
        steerTo(add(tp, tv, lead), c);
        c.z = tp.z + chaseHeight_;
        c.speed = speed_ > 0.0f ? speed_ : spec.cruise;
        break;
    }
    case PlanePhase::Loiter:
    default:
        steerOrbit(loiter_, loiterR_, c);
        c.z = loiter_.z;
        c.speed = speed_ > 0.0f ? speed_ : std::max(spec.minimum + 10.0f, spec.cruise * 0.75f);
        break;
    }
    if (st.ground) return;  // landed (or went somewhere else) this step
    if (clearOfGround) steerClear(env, c);
    if (phase_ != PlanePhase::Climb || st.pos.z - rwZ_ > LOW_LEVEL) separate(env, c);
    flyPlane(st, c, spec, dt);
    // Down on the runway.
    if (phase_ == PlanePhase::Final && a && rw_ >= 0) {
        const Runway& R = a->runways[rw_];
        float along, off;
        R.place(st.pos, along, off);
        const bool over = along >= 0.0f && along <= R.length && std::fabs(off) < R.width * 0.5f;
        if (over && st.pos.z <= surfaceZ(R) + spec.ride + 0.05f) {
            st.pos.z = surfaceZ(R) + spec.ride;
            touchDown(env);
        }
    }
}

void Pilot::step(PilotEnv& env, float dt)
{
    if (dt <= 0.0f) return;
    if (st.ground) {
        if (phase_ == PlanePhase::Takeoff) takeoffStep(env, dt);
        else groundStep(env, dt);
        return;
    }
    env.atc->dropClaim(id);
    airStep(env, dt);
}

} // namespace dnav
