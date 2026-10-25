// Air traffic control for NPC planes (atc.hpp).

#include "atc.hpp"

#include <algorithm>
#include <cmath>

namespace dnav {

namespace {

constexpr float INBOUND_GAP = 20.0f;    // s: a departure is airborne this long before the next arrival lands
constexpr float LANDING_OCCUPY = 45.0f; // s an arrival is on the runway after landing, unless it says
constexpr float CLEAR_EXTRA = 3.0f;     // m between two planes' claims (wingtip to wingtip)

float d2(const Vec3& a, const Vec3& b)
{
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

bool crossLines(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d)
{
    auto orient = [](const Vec3& p, const Vec3& q, const Vec3& r) {
        return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
    };
    const float o1 = orient(a, b, c), o2 = orient(a, b, d), o3 = orient(c, d, a), o4 = orient(c, d, b);
    return ((o1 > 0) != (o2 > 0)) && ((o3 > 0) != (o4 > 0));
}

} // namespace

namespace {

// A plane's outline from above: its wings and its fuselage, two segments.
struct Cross {
    float wx0, wy0, wx1, wy1;
    float fx0, fy0, fx1, fy1;
};

constexpr float THICK = 1.5f;   // m: half a fuselage's (and a wing's) width

Cross crossOf(const Pose& p, float span, float length)
{
    const float h = p.heading * 3.14159265f / 180.0f;
    const float fx = -std::sin(h), fy = std::cos(h);
    const float sx = fy, sy = -fx;
    const float hs = span * 0.5f, hl = length * 0.5f;
    return { p.p.x - sx * hs, p.p.y - sy * hs, p.p.x + sx * hs, p.p.y + sy * hs, p.p.x - fx * hl, p.p.y - fy * hl,
        p.p.x + fx * hl, p.p.y + fy * hl };
}

float pointSeg(float px, float py, float ax, float ay, float bx, float by)
{
    const float dx = bx - ax, dy = by - ay;
    const float l = dx * dx + dy * dy;
    float t = l > 1e-6f ? ((px - ax) * dx + (py - ay) * dy) / l : 0.0f;
    t = std::max(0.0f, std::min(1.0f, t));
    const float qx = ax + dx * t - px, qy = ay + dy * t - py;
    return std::sqrt(qx * qx + qy * qy);
}

float segSeg(float ax, float ay, float bx, float by, float cx, float cy, float dx, float dy)
{
    if (crossLines({ ax, ay, 0 }, { bx, by, 0 }, { cx, cy, 0 }, { dx, dy, 0 })) return 0.0f;
    return std::min({ pointSeg(ax, ay, cx, cy, dx, dy), pointSeg(bx, by, cx, cy, dx, dy), pointSeg(cx, cy, ax, ay, bx, by),
        pointSeg(dx, dy, ax, ay, bx, by) });
}

// How far apart two outlines are (0: touching), edge to edge.
float crossGap(const Cross& a, const Cross& b)
{
    const float d = std::min({ segSeg(a.wx0, a.wy0, a.wx1, a.wy1, b.wx0, b.wy0, b.wx1, b.wy1),
        segSeg(a.wx0, a.wy0, a.wx1, a.wy1, b.fx0, b.fy0, b.fx1, b.fy1),
        segSeg(a.fx0, a.fy0, a.fx1, a.fy1, b.wx0, b.wy0, b.wx1, b.wy1),
        segSeg(a.fx0, a.fy0, a.fx1, a.fy1, b.fx0, b.fy0, b.fx1, b.fy1) });
    return std::max(0.0f, d - 2.0f * THICK);
}

float crossToPoint(const Cross& a, const Vec3& p)
{
    const float d = std::min(pointSeg(p.x, p.y, a.wx0, a.wy0, a.wx1, a.wy1), pointSeg(p.x, p.y, a.fx0, a.fy0, a.fx1, a.fy1));
    return std::max(0.0f, d - THICK);
}

} // namespace

float outlineGap(const Pose& a, float spanA, float lengthA, const Pose& b, float spanB, float lengthB)
{
    return crossGap(crossOf(a, spanA, lengthA), crossOf(b, spanB, lengthB));
}

std::vector<int> Atc::group(const Airports& db, int ap, int rw) const
{
    std::vector<int> out { rw };
    const Airport* a = db.get(ap);
    if (!a || rw < 0 || rw >= static_cast<int>(a->runways.size())) return out;
    const Runway& r = a->runways[rw];
    for (size_t i = 0; i < a->runways.size(); i++) {
        const Runway& o = a->runways[i];
        if (static_cast<int>(i) == rw || o.removed) continue;
        // Crossing centre lines, or ends close enough that one's wings reach the other.
        bool meet = crossLines(r.end[0], r.end[1], o.end[0], o.end[1]);
        if (!meet) {
            for (int k = 0; k < 2 && !meet; k++)
                meet = r.contains(o.end[k], o.width * 0.5f + 20.0f) || o.contains(r.end[k], r.width * 0.5f + 20.0f);
        }
        if (meet) out.push_back(static_cast<int>(i));
    }
    return out;
}

bool Atc::open(const Airports& db, int ap, int rw) const
{
    const Airport* a = db.get(ap);
    if (!a || rw < 0 || rw >= static_cast<int>(a->runways.size()) || a->runways[rw].removed || !a->runways[rw].open)
        return false;
    auto it = closed_.find(key(ap, rw));
    return it == closed_.end() || !it->second;
}

bool Atc::blocked(int ap, int rw) const
{
    auto it = blocked_.find(key(ap, rw));
    return it != blocked_.end() && it->second;
}

int Atc::user(int ap, int rw, Use* use) const
{
    auto it = runways_.find(key(ap, rw));
    if (it == runways_.end() || it->second.who < 0) {
        if (use) *use = Use::None;
        return -1;
    }
    if (use) *use = it->second.use;
    return it->second.who;
}

bool Atc::freeFor(int who, int ap, int rw, float) const
{
    if (!db_) return true;
    if (!open(*db_, ap, rw)) return false;
    for (int r : group(*db_, ap, rw)) {
        if (blocked(ap, r)) return false;
        auto it = runways_.find(key(ap, r));
        if (it != runways_.end() && it->second.who >= 0 && it->second.who != who) return false;
    }
    return true;
}

bool Atc::requestLanding(int who, int ap, int rw, float eta, float now)
{
    if (!freeFor(who, ap, rw, now)) return false;
    // Someone due to land before it, and not far enough before to be off again: theirs.
    if (db_) {
        const auto grp = group(*db_, ap, rw);
        for (const auto& [id, in] : inbound_) {
            if (id == who || in.ap != ap || std::find(grp.begin(), grp.end(), in.rw) == grp.end()) continue;
            const float theirs = in.at - now;
            if (theirs < eta && theirs + LANDING_OCCUPY > eta - 5.0f) return false;
        }
    }
    RunwayUse& u = runways_[key(ap, rw)];
    u.who = who;
    u.use = Use::Landing;
    u.until = now + eta + LANDING_OCCUPY;
    return true;
}

bool Atc::requestTakeoff(int who, int ap, int rw, float occupy, float now)
{
    if (!freeFor(who, ap, rw, now)) return false;
    if (db_) {
        const auto grp = group(*db_, ap, rw);
        for (const auto& [id, in] : inbound_) {
            if (id == who || in.ap != ap || std::find(grp.begin(), grp.end(), in.rw) == grp.end()) continue;
            if (in.at - now < occupy + INBOUND_GAP) return false;
        }
    }
    RunwayUse& u = runways_[key(ap, rw)];
    u.who = who;
    u.use = Use::Takeoff;
    u.until = now + occupy;
    return true;
}

bool Atc::requestCrossing(int who, int ap, int rw, float occupy, float now)
{
    if (holds(who, ap, rw)) return true;
    if (!freeFor(who, ap, rw, now)) return false;
    if (db_) {
        const auto grp = group(*db_, ap, rw);
        for (const auto& [id, in] : inbound_) {
            if (id == who || in.ap != ap || std::find(grp.begin(), grp.end(), in.rw) == grp.end()) continue;
            if (in.at - now < occupy + INBOUND_GAP) return false;
        }
    }
    RunwayUse& u = runways_[key(ap, rw)];
    u.who = who;
    u.use = Use::Crossing;
    u.until = now + occupy;
    return true;
}

void Atc::release(int who, int ap, int rw)
{
    auto it = runways_.find(key(ap, rw));
    if (it != runways_.end() && it->second.who == who) runways_.erase(it);
}

void Atc::releaseAll(int who)
{
    for (auto it = runways_.begin(); it != runways_.end();) {
        if (it->second.who == who) it = runways_.erase(it);
        else ++it;
    }
}

bool Atc::holds(int who, int ap, int rw) const
{
    auto it = runways_.find(key(ap, rw));
    return it != runways_.end() && it->second.who == who;
}

void Atc::inbound(int who, int ap, int rw, float eta, float now)
{
    inbound_[who] = { ap, rw, now + eta };
}

void Atc::notInbound(int who)
{
    inbound_.erase(who);
}

float Atc::busyFor(int who, int ap, int rw, float eta, float now) const
{
    if (!db_) return 0.0f;
    float freeAt = now;
    const auto grp = group(*db_, ap, rw);
    for (int r : grp) {
        auto it = runways_.find(key(ap, r));
        if (it != runways_.end() && it->second.who >= 0 && it->second.who != who) freeAt = std::max(freeAt, it->second.until);
    }
    // Arrivals due before it land first.
    for (const auto& [id, in] : inbound_) {
        if (id == who || in.ap != ap || std::find(grp.begin(), grp.end(), in.rw) == grp.end()) continue;
        if (in.at < now + eta) freeAt = std::max(freeAt, in.at + LANDING_OCCUPY);
    }
    return std::max(0.0f, freeAt - (now + eta));
}

void Atc::setClaim(int who, int ap, const std::vector<Pose>& poses, float span, float length, int waitingFor)
{
    Claim& c = claims_[who];
    c.ap = ap;
    c.poses = poses;
    c.span = span;
    c.length = length;
    c.waitingFor = waitingFor;
}

void Atc::dropClaim(int who)
{
    claims_.erase(who);
}

size_t Atc::clearAhead(int who, int ap, const std::vector<Pose>& ahead, float span, float length, float gap, int& by,
    int ignore) const
{
    by = -1;
    const float reach = std::max(span, length) * 0.5f;
    for (size_t i = 0; i < ahead.size(); i++) {
        const Pose& p = ahead[i];
        const Cross mine = crossOf(p, span, length);
        if (ignore < 2) {
            for (const auto& [id, c] : claims_) {
                if (id == who || c.ap != ap) continue;
                const float theirs = std::max(c.span, c.length) * 0.5f;
                // Waiting at each other: the lower id goes past the other's way ahead.
                auto me = claims_.find(who);
                const bool yield = ignore >= 1
                    || (c.waitingFor == who && who < id && me != claims_.end() && me->second.waitingFor == id);
                const size_t n = yield ? std::min<size_t>(1, c.poses.size()) : c.poses.size();
                for (size_t k = 0; k < n; k++) {
                    const Pose& q = c.poses[k];
                    if (std::fabs(p.p.z - q.p.z) > 8.0f || d2(p.p, q.p) > reach + theirs + gap + CLEAR_EXTRA) continue;
                    if (crossGap(mine, crossOf(q, c.span, c.length)) < gap + CLEAR_EXTRA) {
                        by = id;
                        return i;
                    }
                }
            }
        }
        for (const Obstacle& o : obstacles_) {
            if (std::fabs(p.p.z - o.pos.z) > 8.0f || d2(p.p, o.pos) > reach + o.radius + gap) continue;
            if (crossToPoint(mine, o.pos) < o.radius + gap) return i;
        }
    }
    return ahead.size();
}

void Atc::setIntent(int who, int ap, const std::vector<std::pair<int, int>>& edges)
{
    Intent& in = intents_[who];
    in.ap = ap;
    in.edges = edges;
}

void Atc::dropIntent(int who)
{
    intents_.erase(who);
}

bool Atc::opposed(int who, int ap, int e, int from, const TaxiEdge& edge) const
{
    for (const auto& [id, in] : intents_) {
        if (id == who || in.ap != ap) continue;
        for (const auto& [oe, ofrom] : in.edges)
            if (oe == e && ofrom != from) return true;
    }
    (void)edge;
    return false;
}

bool Atc::mustYield(int who, int ap, const std::vector<std::pair<int, int>>& run) const
{
    for (const auto& [id, in] : intents_) {
        if (id == who || in.ap != ap || in.edges.empty()) continue;
        bool against = false;
        for (const auto& [oe, ofrom] : in.edges)
            for (const auto& [e, from] : run)
                if (oe == e && ofrom != from) against = true;
        if (!against) continue;
        const int theirs = in.edges.front().first;
        bool on = false;
        for (const auto& [e, from] : run)
            if (e == theirs) on = true;
        if (on || id < who) return true;
    }
    return false;
}

int Atc::using_(int who, int ap, int e) const
{
    int n = 0;
    for (const auto& [id, in] : intents_) {
        if (id == who || in.ap != ap) continue;
        for (const auto& [oe, ofrom] : in.edges)
            if (oe == e) {
                n++;
                break;
            }
    }
    return n;
}

bool Atc::gateFree(int ap, int gate, int except) const
{
    auto it = gates_.find(key(ap, gate));
    if (it != gates_.end() && it->second != except) return false;
    // Something else parked on it (a plane nobody flies, a car).
    if (db_) {
        const Airport* a = db_->get(ap);
        if (a && gate >= 0 && gate < static_cast<int>(a->gates.size())) {
            const Gate& g = a->gates[gate];
            for (const Obstacle& o : obstacles_)
                if (d2(o.pos, g.p) < 4.0f + o.radius * 0.5f && std::fabs(o.pos.z - g.p.z) < 8.0f) return false;
        }
    }
    return true;
}

void Atc::takeGate(int who, int ap, int gate)
{
    leaveGate(who);
    gates_[key(ap, gate)] = who;
}

void Atc::leaveGate(int who)
{
    for (auto it = gates_.begin(); it != gates_.end();) {
        if (it->second == who) it = gates_.erase(it);
        else ++it;
    }
}

int Atc::gateOf(int who) const
{
    for (const auto& [k, id] : gates_)
        if (id == who) return static_cast<int>(k & 0xffff);
    return -1;
}

void Atc::update(const Airports& db, const std::vector<Obstacle>& obstacles, float now)
{
    db_ = &db;
    obstacles_ = obstacles;
    blocked_.clear();
    for (int ap = 0; ap < static_cast<int>(db.count()); ap++) {
        const Airport* a = db.get(ap);
        if (!a) continue;
        for (size_t r = 0; r < a->runways.size(); r++) {
            const Runway& rw = a->runways[r];
            if (rw.removed) continue;
            for (const Obstacle& o : obstacles_) {
                // On it and on the ground (within 6 m of its surface), standing or slow.
                float along, off;
                rw.place(o.pos, along, off);
                if (along < 0.0f || along > rw.length || std::fabs(off) > rw.width * 0.5f + o.radius * 0.5f) continue;
                const Vec3 surface = rw.pointAt(along);
                if (o.pos.z - surface.z > 6.0f) continue;
                blocked_[key(ap, static_cast<int>(r))] = true;
                break;
            }
        }
    }
    // A use whose holder never let go (it crashed or was moved away) runs out.
    for (auto it = runways_.begin(); it != runways_.end();) {
        if (now > it->second.until + 120.0f) it = runways_.erase(it);
        else ++it;
    }
    for (auto it = inbound_.begin(); it != inbound_.end();) {
        if (now > it->second.at + 180.0f) it = inbound_.erase(it);
        else ++it;
    }
}

void Atc::forget(int who)
{
    releaseAll(who);
    notInbound(who);
    dropClaim(who);
    dropIntent(who);
    leaveGate(who);
}

} // namespace dnav
