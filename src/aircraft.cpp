// How NPC planes move (aircraft.hpp).

#include "aircraft.hpp"

#include <algorithm>
#include <cmath>

namespace dnav {

namespace {

constexpr float PI = 3.14159265f;
constexpr float DEG = PI / 180.0f;
constexpr float G = 9.81f;
constexpr float REVERSAL = 160.0f;      // degrees: a corner sharper than this is a U-turn
constexpr float VZ_ACCEL = 2.5f;        // m/s^2 it changes its climb or sink by
constexpr float BANK_GAIN = 1.4f;       // degrees of bank per degree off its heading
constexpr float HEIGHT_GAIN = 0.22f;    // m/s up per metre under its height
constexpr float TERRAIN_STEP = 15.0f;   // m between tops looked at ahead (the first 600 m)...
constexpr float TERRAIN_STEP_FAR = 40.0f; // ...and past that
constexpr float WING_SIDE = 10.0f;      // m past each wingtip the tops are looked at too
constexpr float PUSH_EASE = 0.3f;       // m/s^2 the tug starts and stops with

float clampf(float v, float lo, float hi)
{
    return std::max(lo, std::min(hi, v));
}

float headingOfDir(float dx, float dy)
{
    float h = std::atan2(-dx, dy) / DEG;
    if (h < 0.0f) h += 360.0f;
    return h;
}

float dist2(const Vec3& a, const Vec3& b)
{
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

// Builds a spec from its numbers.
PlaneSpec spec(int model, const char* name, float span, float length, float ride, float rotate, float approach,
    float touchdown, float cruise, float minimum, float climb, float bank, float accel, float brake, float taxiRadius,
    float finalLength)
{
    PlaneSpec s;
    s.model = model;
    s.name = name;
    s.span = span;
    s.length = length;
    s.ride = ride;
    s.rotate = rotate;
    s.approach = approach;
    s.touchdown = touchdown;
    s.cruise = cruise;
    s.minimum = minimum;
    s.climb = climb;
    s.sink = std::max(6.0f, climb * 0.8f);
    s.bank = bank;
    s.rollRate = bank * 0.8f;
    s.accel = accel;
    s.brake = brake;
    s.taxiRadius = taxiRadius;
    s.finalLength = finalLength;
    s.flare = span > 40.0f ? 6.0f : span > 20.0f ? 4.5f : 3.0f;
    s.taxi = span > 40.0f ? 10.0f : span > 20.0f ? 9.0f : 8.0f;
    s.airAccel = span > 40.0f ? 1.2f : 2.0f;
    s.airDecel = span > 40.0f ? 1.5f : 2.5f;
    s.push = span > 40.0f ? 2.0f : 2.5f;
    return s;
}

} // namespace

bool isPlane(int model)
{
    switch (model) {
    case 460: case 476: case 511: case 512: case 513: case 519: case 520: case 553: case 577: case 592: case 593:
        return true;
    default:
        return false;
    }
}

PlaneSpec PlaneSpec::of(int model)
{
    // Sizes are the game's (each model's collision, GetVehicleModelInfo; vehicles.dnv's
    // replace them where the component has it). Speeds are the game's scale, not real
    // life's: San Andreas' runways are 700 m to 1.5 km, so an AT-400 lifts off at
    // 50 m/s (180 km/h) and lands in under 400 m.
    switch (model) {
    case 577: return spec(577, "AT-400", 62.5f, 70.0f, 0.92f, 50.0f, 55.0f, 50.0f, 70.0f, 45.0f, 10.0f, 25.0f, 4.0f, 4.5f, 25.0f, 1500.0f);
    case 592: return spec(592, "Andromada", 60.0f, 66.0f, 3.4f, 46.0f, 52.0f, 47.0f, 65.0f, 42.0f, 9.0f, 25.0f, 3.6f, 4.5f, 25.0f, 1400.0f);
    case 553: return spec(553, "Nevada", 27.0f, 22.0f, 2.2f, 40.0f, 45.0f, 40.0f, 60.0f, 36.0f, 9.0f, 30.0f, 3.5f, 4.5f, 12.0f, 1000.0f);
    case 519: return spec(519, "Shamal", 19.0f, 20.0f, 1.6f, 42.0f, 47.0f, 42.0f, 75.0f, 38.0f, 14.0f, 35.0f, 4.5f, 5.0f, 10.0f, 1000.0f);
    case 520: return spec(520, "Hydra", 12.0f, 17.0f, 1.6f, 42.0f, 47.0f, 42.0f, 85.0f, 38.0f, 20.0f, 50.0f, 6.0f, 5.5f, 8.0f, 900.0f);
    case 476: return spec(476, "Rustler", 12.0f, 10.0f, 1.3f, 32.0f, 35.0f, 31.0f, 55.0f, 29.0f, 12.0f, 45.0f, 4.0f, 4.5f, 7.0f, 700.0f);
    case 511: return spec(511, "Beagle", 15.0f, 11.0f, 1.4f, 28.0f, 31.0f, 28.0f, 45.0f, 26.0f, 7.0f, 35.0f, 3.0f, 4.0f, 7.0f, 700.0f);
    case 512: return spec(512, "Cropduster", 14.0f, 9.5f, 1.3f, 26.0f, 29.0f, 26.0f, 40.0f, 24.0f, 7.0f, 40.0f, 3.0f, 4.0f, 6.0f, 600.0f);
    case 513: return spec(513, "Stuntplane", 10.5f, 8.0f, 1.2f, 28.0f, 31.0f, 28.0f, 50.0f, 26.0f, 12.0f, 55.0f, 4.0f, 4.5f, 6.0f, 600.0f);
    case 593: return spec(593, "Dodo", 11.0f, 9.5f, 1.3f, 26.0f, 29.0f, 26.0f, 40.0f, 24.0f, 6.0f, 35.0f, 3.0f, 4.0f, 6.0f, 600.0f);
    case 460: return spec(460, "Skimmer", 11.0f, 9.5f, 1.6f, 26.0f, 29.0f, 26.0f, 40.0f, 24.0f, 6.0f, 35.0f, 3.0f, 4.0f, 6.0f, 600.0f);
    default: return spec(model, "plane", 15.0f, 12.0f, 1.3f, 32.0f, 36.0f, 32.0f, 50.0f, 30.0f, 10.0f, 35.0f, 3.0f, 4.0f, 7.0f, 900.0f);
    }
}

float PlaneSpec::turnRadius(float v) const
{
    return v * v / (G * std::tan(bank * DEG));
}

Vec3 forwardOf(float heading)
{
    const float h = heading * DEG;
    return { -std::sin(h), std::cos(h), 0.0f };
}

float wrapDeg(float a)
{
    a = std::fmod(a, 360.0f);
    if (a > 180.0f) a -= 360.0f;
    if (a < -180.0f) a += 360.0f;
    return a;
}

Vec3 PlaneState::velocity() const
{
    const float h = std::sqrt(std::max(0.0f, speed * speed - vz * vz));
    const Vec3 f = forwardOf(heading);
    return { f.x * h, f.y * h, vz };
}

GroundPoint GroundPath::at(float s) const
{
    if (pts.empty()) return {};
    if (s <= pts.front().s) return pts.front();
    if (s >= pts.back().s) return pts.back();
    auto it = std::upper_bound(pts.begin(), pts.end(), s, [](float v, const GroundPoint& p) { return v < p.s; });
    const GroundPoint& b = *it;
    const GroundPoint& a = *(it - 1);
    const float k = b.s > a.s ? (s - a.s) / (b.s - a.s) : 0.0f;
    GroundPoint o;
    o.p = { a.p.x + (b.p.x - a.p.x) * k, a.p.y + (b.p.y - a.p.y) * k, a.p.z + (b.p.z - a.p.z) * k };
    o.heading = a.heading + wrapDeg(b.heading - a.heading) * k;
    if (o.heading < 0.0f) o.heading += 360.0f;
    if (o.heading >= 360.0f) o.heading -= 360.0f;
    o.s = s;
    o.vmax = a.vmax + (b.vmax - a.vmax) * k;
    return o;
}

namespace {

// A piece of a rounded path: a straight line, or an arc round `c`.
struct Piece {
    bool arc = false;
    Vec3 a {}, b {};            // ends (a line)
    Vec3 c {};                  // the arc's centre
    float r = 0.0f;             // its radius
    float a0 = 0.0f, sweep = 0.0f;  // its start angle and how far it goes (radians, left positive)
    float z0 = 0.0f, z1 = 0.0f; // the arc's height at its ends
    float len() const { return arc ? std::fabs(sweep) * r : dist2(a, b); }
};

} // namespace

GroundPath groundPath(const std::vector<Vec3>& in, float r, float top, float corner, float accel, float brake,
    float v0, float vEnd, float step)
{
    GroundPath out;
    // No repeated points.
    std::vector<Vec3> p;
    for (const Vec3& q : in)
        if (p.empty() || dist2(p.back(), q) > 0.1f) p.push_back(q);
    if (p.size() < 2) return out;
    // A leg that doubles back: out to the left on a loop and back onto the way.
    for (size_t i = 1; i + 1 < p.size(); i++) {
        const float ax = p[i].x - p[i - 1].x, ay = p[i].y - p[i - 1].y;
        const float bx = p[i + 1].x - p[i].x, by = p[i + 1].y - p[i].y;
        const float la = std::sqrt(ax * ax + ay * ay), lb = std::sqrt(bx * bx + by * by);
        if (la < 0.1f || lb < 0.1f) continue;
        const float c = (ax * bx + ay * by) / (la * lb);
        if (c > std::cos(REVERSAL * DEG)) continue;
        const Vec3 q { p[i].x - ay / la * 2.0f * r, p[i].y + ax / la * 2.0f * r, p[i].z };
        p.insert(p.begin() + static_cast<long>(i) + 1, q);
        i++;
    }
    // Round each corner: the legs' ends trimmed back by t, an arc between them.
    std::vector<Piece> pieces;
    Vec3 cur = p[0];
    for (size_t i = 1; i < p.size(); i++) {
        const Vec3& a = p[i - 1];
        const Vec3& b = p[i];
        if (i + 1 == p.size()) {
            Piece l;
            l.a = cur;
            l.b = b;
            pieces.push_back(l);
            break;
        }
        const Vec3& c = p[i + 1];
        const float ax = b.x - a.x, ay = b.y - a.y, la = std::sqrt(ax * ax + ay * ay);
        const float bx = c.x - b.x, by = c.y - b.y, lb = std::sqrt(bx * bx + by * by);
        const float cross = ax * by - ay * bx, dot = ax * bx + ay * by;
        const float turn = std::atan2(cross, dot);      // left positive
        const float th = std::fabs(turn);
        // What is left of the leg in (its start may already be trimmed) and half the next.
        const float inLeft = dist2(cur, b);
        const float outShare = i + 2 == p.size() ? lb : lb * 0.5f;
        float t = th > 1e-3f ? r * std::tan(th * 0.5f) : 0.0f;
        t = std::min({ t, inLeft, outShare });
        if (th < 1e-3f || t < 0.05f) {
            Piece l;
            l.a = cur;
            l.b = b;
            pieces.push_back(l);
            cur = b;
            continue;
        }
        const float re = t / std::tan(th * 0.5f);
        const Vec3 s { b.x - ax / la * t, b.y - ay / la * t, b.z + (a.z - b.z) * t / std::max(la, 0.01f) };
        const Vec3 e { b.x + bx / lb * t, b.y + by / lb * t, b.z + (c.z - b.z) * t / std::max(lb, 0.01f) };
        Piece l;
        l.a = cur;
        l.b = s;
        pieces.push_back(l);
        Piece arc;
        arc.arc = true;
        const float side = turn > 0.0f ? 1.0f : -1.0f;  // centre to the left for a left turn
        arc.c = { s.x - ay / la * re * side, s.y + ax / la * re * side, 0.0f };
        arc.r = re;
        arc.a0 = std::atan2(s.y - arc.c.y, s.x - arc.c.x);
        arc.sweep = turn;
        arc.z0 = s.z;
        arc.z1 = e.z;
        arc.a = s;
        arc.b = e;
        pieces.push_back(arc);
        cur = e;
    }
    // Sample.
    float sAt = 0.0f;
    std::vector<float> radius;
    auto push = [&](const Vec3& q, float heading, float rad) {
        if (!out.pts.empty()) {
            const float d = dist2(out.pts.back().p, q);
            if (d < 0.01f) return;
            sAt += d;
        }
        out.pts.push_back({ q, heading, sAt, top });
        radius.push_back(rad);
    };
    for (const Piece& pc : pieces) {
        const float len = pc.len();
        if (len < 0.01f) continue;
        const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
        for (int k = out.pts.empty() ? 0 : 1; k <= n; k++) {
            const float f = static_cast<float>(k) / n;
            if (!pc.arc) {
                const Vec3 q { pc.a.x + (pc.b.x - pc.a.x) * f, pc.a.y + (pc.b.y - pc.a.y) * f, pc.a.z + (pc.b.z - pc.a.z) * f };
                push(q, headingOfDir(pc.b.x - pc.a.x, pc.b.y - pc.a.y), 1e9f);
            } else {
                const float ang = pc.a0 + pc.sweep * f;
                const Vec3 q { pc.c.x + std::cos(ang) * pc.r, pc.c.y + std::sin(ang) * pc.r, pc.z0 + (pc.z1 - pc.z0) * f };
                // The way round: perpendicular to the radius, the way it sweeps.
                const float dir = pc.sweep > 0.0f ? 1.0f : -1.0f;
                push(q, headingOfDir(-std::sin(ang) * dir, std::cos(ang) * dir), pc.r);
            }
        }
    }
    if (out.pts.size() < 2) {
        out.pts.clear();
        return out;
    }
    // Speeds: the turns, then braking for what is ahead and pulling away from what is behind.
    const size_t n = out.pts.size();
    for (size_t i = 0; i < n; i++) {
        // A turn is taken at its own speed a little before and after it as well.
        float rad = radius[i];
        for (size_t k = i > 3 ? i - 3 : 0; k < std::min(n, i + 4); k++) rad = std::min(rad, radius[k]);
        out.pts[i].vmax = std::min(top, std::sqrt(corner * rad));
    }
    out.pts.back().vmax = std::min(out.pts.back().vmax, vEnd);
    for (size_t i = n - 1; i-- > 0;) {
        const float ds = out.pts[i + 1].s - out.pts[i].s;
        out.pts[i].vmax = std::min(out.pts[i].vmax, std::sqrt(out.pts[i + 1].vmax * out.pts[i + 1].vmax + 2.0f * brake * ds));
    }
    // It starts at v0, even faster than the path allows (a landing roll): it brakes from there.
    out.pts[0].vmax = std::max(out.pts[0].vmax, v0);
    float v = v0;
    for (size_t i = 1; i < n; i++) {
        const float ds = out.pts[i].s - out.pts[i - 1].s;
        const float up = std::sqrt(v * v + 2.0f * accel * ds);
        float down = v * v - 2.0f * brake * ds;
        down = down > 0.0f ? std::sqrt(down) : 0.0f;
        v = std::max(down, std::min(up, out.pts[i].vmax));
        out.pts[i].vmax = v;
    }
    // Never quite stopped before the end (except where the plan says so).
    for (size_t i = 0; i + 1 < n; i++) out.pts[i].vmax = std::max(out.pts[i].vmax, 0.6f);
    return out;
}

GroundPath pushPath(const Vec3& from, float heading, const Vec3& to, float endHeading, float speed, float step)
{
    GroundPath out;
    out.reverse = true;
    const Vec3 fa = forwardOf(heading), fb = forwardOf(endHeading);
    const float d = dist2(from, to);
    if (d < 0.5f) return out;
    const float L = d * 0.45f;
    // Backwards: it leaves the stand tail first and arrives tail first.
    const Vec3 c1 { from.x - fa.x * L, from.y - fa.y * L, from.z };
    const Vec3 c2 { to.x + fb.x * L, to.y + fb.y * L, to.z };
    const int n = std::max(8, static_cast<int>(d * 1.5f / step));
    float s = 0.0f;
    Vec3 last = from;
    for (int i = 0; i <= n; i++) {
        const float t = static_cast<float>(i) / n, u = 1.0f - t;
        const float b0 = u * u * u, b1 = 3 * u * u * t, b2 = 3 * u * t * t, b3 = t * t * t;
        const Vec3 q { b0 * from.x + b1 * c1.x + b2 * c2.x + b3 * to.x, b0 * from.y + b1 * c1.y + b2 * c2.y + b3 * to.y,
            from.z + (to.z - from.z) * t };
        // The way it moves: the curve's tangent; it points the other way.
        const float dx = 3 * u * u * (c1.x - from.x) + 6 * u * t * (c2.x - c1.x) + 3 * t * t * (to.x - c2.x);
        const float dy = 3 * u * u * (c1.y - from.y) + 6 * u * t * (c2.y - c1.y) + 3 * t * t * (to.y - c2.y);
        float h = std::fabs(dx) + std::fabs(dy) > 1e-4f ? headingOfDir(-dx, -dy) : (i == 0 ? heading : endHeading);
        if (i == 0) h = heading;
        if (i == n) h = endHeading;
        if (i > 0) s += dist2(last, q);
        out.pts.push_back({ q, h, s, speed });
        last = q;
    }
    // Eased in and out by the tug.
    const float total = s;
    for (auto& g : out.pts) {
        const float fromStart = std::sqrt(2.0f * PUSH_EASE * g.s) + 0.3f;
        const float toEnd = std::sqrt(2.0f * PUSH_EASE * std::max(0.0f, total - g.s));
        g.vmax = std::min({ speed, fromStart, toEnd });
    }
    out.pts.back().vmax = 0.0f;
    return out;
}

void flyPlane(PlaneState& s, const AirCommand& c, const PlaneSpec& spec, float dt)
{
    if (dt <= 0.0f) return;
    s.ground = false;
    // Speed: towards what it is told, never under its minimum far from a runway (the
    // pilot asks for the approach speeds itself).
    const float want = std::max(c.speed, spec.touchdown * 0.9f);
    const float dv = want - s.speed;
    s.speed += clampf(dv, -spec.airDecel * dt, spec.airAccel * dt);

    // Turning: a bank for how far off its heading it is, rolled into at its roll
    // rate; the bank turns it as fast as its speed allows.
    const float maxBank = c.maxBank > 0.0f ? std::min(c.maxBank, spec.bank) : spec.bank;
    const float err = wrapDeg(c.heading - s.heading);
    const float bankWant = clampf(err * BANK_GAIN, -maxBank, maxBank);
    s.bank += clampf(bankWant - s.bank, -spec.rollRate * dt, spec.rollRate * dt);
    const float v = std::max(s.speed, 5.0f);
    s.yawRate = std::tan(s.bank * DEG) * G / v / DEG;
    // No overshooting the heading it wants within a step.
    float turn = s.yawRate * dt;
    if (std::fabs(err) < std::fabs(turn) && err * turn > 0.0f) turn = err;
    s.heading = std::fmod(s.heading + turn + 360.0f, 360.0f);

    // Up and down: towards its height, as fast as it climbs or sinks; slower than it
    // climbs well at, less.
    float vzWant = c.holdVz ? c.vz : clampf((c.z - s.pos.z) * HEIGHT_GAIN, -spec.sink, spec.climb);
    const float climbShare = clampf((s.speed - spec.touchdown * 0.85f) / std::max(1.0f, spec.rotate * 0.3f), 0.2f, 1.0f);
    vzWant = std::min(vzWant, spec.climb * climbShare);
    vzWant = std::max(vzWant, -std::max(spec.sink, 0.25f * s.speed));
    s.vz += clampf(vzWant - s.vz, -VZ_ACCEL * dt, VZ_ACCEL * dt);

    // Its nose: along its flight path, plus the attitude it flies at (more when slow),
    // plus the flare.
    const float gamma = std::atan2(s.vz, std::sqrt(std::max(1.0f, s.speed * s.speed - s.vz * s.vz))) / DEG;
    const float slow = clampf(spec.approach / std::max(s.speed, 1.0f), 0.6f, 1.6f);
    const float pitchWant = clampf(gamma + spec.attitude * slow * slow + c.pitchExtra, -15.0f, 25.0f);
    s.pitch += (pitchWant - s.pitch) * std::min(1.0f, dt * 2.0f);

    const Vec3 vel = s.velocity();
    s.pos.x += vel.x * dt;
    s.pos.y += vel.y * dt;
    s.pos.z += vel.z * dt;
}

float terrainFloor(const FlyWorld& w, const Vec3& pos, float heading, float speed, const PlaneSpec& spec, float clear,
    float look)
{
    if (!w.top) return -1e9f;
    const Vec3 f = forwardOf(heading);
    const float side = spec.span * 0.5f + WING_SIDE;
    // It climbs as it goes: a top far ahead needs less height now.
    const float slope = 0.7f * spec.climb / std::max(speed, spec.minimum);
    float best = -1e9f;
    for (float s = 0.0f; s <= look; s += s < 600.0f ? TERRAIN_STEP : TERRAIN_STEP_FAR) {
        const float px = pos.x + f.x * s, py = pos.y + f.y * s;
        float t = w.top(px, py);
        t = std::max(t, w.top(px + f.y * side, py - f.x * side));
        t = std::max(t, w.top(px - f.y * side, py + f.x * side));
        best = std::max(best, t + clear - s * slope);
    }
    return best;
}

} // namespace dnav
