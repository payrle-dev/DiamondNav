// How planes fly, without a server: pilots against a made-up airport (one runway, a
// parallel taxiway and two gates on flat ground at z 10, a 600 m hill to the east).
// Run by ctest (or build/dnav_aircraft_tests directly).

#include "pilot.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>

using namespace dnav;

static int failures = 0;

#define CHECK(cond)                                                                 \
    do {                                                                            \
        if (!(cond)) {                                                              \
            std::fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond); \
            failures++;                                                             \
        }                                                                           \
    } while (0)

// Runway 09/27 along y 0 from x 0 to 2500, 45 m wide. Taxiway A along y 150 with
// links to the runway at both ends and two in between; gates G1 and G2 north of it,
// nose north, pushed back onto A.
static const char* AIRPORT = R"(
airport TST Test Field
runway 09/27 0 0 10 2500 0 10 width 45
node R0 0 0 10
node R1 600 0 10
node R3 1900 0 10
node R4 2500 0 10
node A0 0 150 10
node A1 600 150 10
node A2 1150 150 10
node A3 1350 150 10
node A4 1900 150 10
node A5 2500 150 10
taxi A A0 A1 A2 A3 A4 A5
taxi L0 A0 R0
taxi L1 A1 R1
taxi L3 A4 R3
taxi L4 A5 R4
gate G1 1150 300 10 0 lead A2 push A2
gate G2 1350 300 10 0 lead A3 push A3
gate S1 2300 300 10 0 span 20 lead A5
route OUT 3000 -3000 400 6000 -3000 400
)";

// A hill east of the field: 600 m high round (9000, 0), 1500 m across.
static float top(float x, float y)
{
    const float d = std::sqrt((x - 9000.0f) * (x - 9000.0f) + y * y);
    return d < 1500.0f ? 10.0f + 600.0f * (1.0f - d / 1500.0f) : 10.0f;
}

struct Sim {
    Airports db;
    Atc atc;
    std::deque<Pilot> pilots;
    float now = 0.0f;

    Sim()
    {
        std::string error;
        if (!db.loadText(AIRPORT, "test", error)) std::fprintf(stderr, "%s\n", error.c_str());
        atc.setDb(&db);
    }

    PilotEnv env()
    {
        PilotEnv e;
        e.db = &db;
        e.atc = &atc;
        e.world.top = top;
        e.now = now;
        for (const Pilot& p : pilots) e.others.push_back(&p);
        return e;
    }

    Pilot& add(int model)
    {
        pilots.emplace_back(static_cast<int>(pilots.size()), model);
        return pilots.back();
    }

    void step(float dt)
    {
        atc.update(db, {}, now);
        PilotEnv e = env();
        for (Pilot& p : pilots) {
            const size_t was = p.phases.size();
            p.step(e, dt);
            if (trace && (p.phases.size() != was || std::fmod(now, 5.0f) < dt))
                std::printf("  t %6.1f  #%d %-10s (%7.0f %7.0f %5.0f) h %3.0f v %4.1f%s\n", now, p.id, phaseName(p.phase()),
                    p.st.pos.x, p.st.pos.y, p.st.pos.z, p.st.heading, p.st.speed, p.why.empty() ? "" : (" " + p.why).c_str());
        }
        now += dt;
    }
    bool trace = std::getenv("DNAV_TRACE") != nullptr;
};

static bool sane(const PlaneState& s)
{
    return std::isfinite(s.pos.x) && std::isfinite(s.pos.y) && std::isfinite(s.pos.z) && std::isfinite(s.heading)
        && std::isfinite(s.speed);
}

static bool saw(const Pilot& p, PlanePhase ph)
{
    for (PlanePhase x : p.phases)
        if (x == ph) return true;
    return false;
}

// Near the runway or a taxiway (within `pad` of the runway or of a taxi edge).
static bool onPavement(const Airport& a, const Vec3& p, float pad)
{
    for (const auto& r : a.runways)
        if (!r.removed && r.contains(p, pad)) return true;
    for (const auto& e : a.edges) {
        if (e.removed) continue;
        const Vec3& u = a.nodes[e.a].p;
        const Vec3& v = a.nodes[e.b].p;
        const float dx = v.x - u.x, dy = v.y - u.y, l = dx * dx + dy * dy;
        float t = l > 0 ? ((p.x - u.x) * dx + (p.y - u.y) * dy) / l : 0;
        t = std::max(0.0f, std::min(1.0f, t));
        const float qx = u.x + dx * t - p.x, qy = u.y + dy * t - p.y;
        if (qx * qx + qy * qy < pad * pad) return true;
    }
    // The apron round each gate, out to where it is pushed back.
    for (const auto& g : a.gates) {
        if (std::hypot(g.p.x - p.x, g.p.y - p.y) < pad + 10.0f) return true;
        const Vec3& l = a.nodes[g.push >= 0 ? g.push : g.lead].p;
        if (std::fabs(p.x - g.p.x) < 80.0f && p.y <= g.p.y + 5.0f && p.y >= l.y - pad) return true;
    }
    return false;
}

static void testData()
{
    Sim s;
    const int ap = s.db.find("tst");
    CHECK(ap == 0);
    const Airport* a = s.db.get(ap);
    CHECK(a && a->runways.size() == 1);
    CHECK(a->runways[0].nodes.size() == 4);
    // The holds are the taxiway ends of the links.
    CHECK(a->nodes[a->node("A0")].holdFor == 0);
    CHECK(a->nodes[a->node("A2")].holdFor == -1);
    // A route from G1's lead to the far runway end, for an AT-400: along A, not the runway.
    TaxiRoute r = s.db.taxiRoute(*a, a->node("A2"), { a->node("R4") }, 62.5f);
    CHECK(r.ok() && r.nodes.size() == 5);
    // S1's taxiway is fine for anything but its stand only takes span 20.
    CHECK(a->gates[a->gate("S1")].span == 20.0f);
    // A bad file changes nothing.
    std::string error;
    CHECK(!s.db.loadText("airport TST\ntaxi X A0 NOPE\n", "bad", error));
    CHECK(error.find("NOPE") != std::string::npos);
    CHECK(s.db.get(ap)->edges.size() == a->edges.size());
    // Adding to it: a second runway, crossing the first.
    CHECK(s.db.loadText("airport TST\nrunway 18/36 1250 600 10 1250 -600 10 width 30\n", "more", error));
    CHECK(s.db.get(ap)->runways.size() == 2);
    CHECK(s.atc.group(s.db, ap, 0).size() == 2);
}

// An AT-400 off G1, out to the runway, up and along the OUT route.
static void testDeparture()
{
    Sim s;
    Pilot& p = s.add(577);
    PilotEnv e = s.env();
    CHECK(p.park(e, 0, 0));
    const FlightRoute* out = s.db.route("OUT");
    CHECK(out != nullptr);
    CHECK(p.takeoff(e, "", out->points));
    CHECK(p.phase() == PlanePhase::Pushback);
    bool offPavement = false;
    float liftoffX = 0.0f;
    for (int i = 0; i < 50 * 600 && p.ended < 0; i++) {
        s.step(0.02f);
        CHECK(sane(p.st));
        if (p.st.ground && !onPavement(*s.db.get(0), p.st.pos, 25.0f)) offPavement = true;
        if (p.phase() == PlanePhase::Climb && liftoffX == 0.0f) liftoffX = p.st.pos.x;
    }
    CHECK(!offPavement);
    CHECK(saw(p, PlanePhase::TaxiOut));
    CHECK(saw(p, PlanePhase::LineUp));
    CHECK(saw(p, PlanePhase::Takeoff));
    CHECK(saw(p, PlanePhase::Climb));
    CHECK(p.ended == static_cast<int>(PilotEnd::Arrived));
    CHECK(!p.st.gearDown);
    // It lifted off on the runway, and the gate is free again.
    CHECK(liftoffX > 0.0f && liftoffX < 2500.0f);
    CHECK(s.atc.gateFree(0, 0));
    std::printf("departure: lifted off at x %.0f, done at t %.0f s at (%.0f, %.0f, %.0f)\n", liftoffX, s.now, p.st.pos.x,
        p.st.pos.y, p.st.pos.z);
}

// An AT-400 15 km out lands and taxis to a free gate.
static void testArrival()
{
    Sim s;
    Pilot& p = s.add(577);
    PilotEnv e = s.env();
    p.place(e, { -15000.0f, 4000.0f, 600.0f }, 270.0f, false, 0.0f);
    CHECK(p.land(e, 0, "", -1, {}));
    float touchX = 1e9f;
    bool offPavement = false;
    float minAgl = 1e9f;
    for (int i = 0; i < 50 * 1200 && p.ended < 0; i++) {
        s.step(0.02f);
        CHECK(sane(p.st));
        if (!p.st.ground) minAgl = std::min(minAgl, p.st.pos.z - top(p.st.pos.x, p.st.pos.y));
        if (p.phase() == PlanePhase::Landing && touchX > 1e8f) touchX = p.st.pos.x;
        if (p.st.ground && !onPavement(*s.db.get(0), p.st.pos, 25.0f)) offPavement = true;
    }
    CHECK(saw(p, PlanePhase::Approach));
    CHECK(saw(p, PlanePhase::Final));
    CHECK(saw(p, PlanePhase::Landing));
    CHECK(saw(p, PlanePhase::TaxiIn));
    CHECK(p.phase() == PlanePhase::AtGate);
    CHECK(p.ended == static_cast<int>(PilotEnd::Arrived));
    CHECK(!offPavement);
    CHECK(touchX >= 0.0f && touchX <= 1200.0f);
    CHECK(p.gate() == 0 || p.gate() == 1);
    std::printf("arrival: touched down at x %.0f, at gate %d after %.0f s (go-arounds seen: %s)\n", touchX, p.gate(), s.now,
        saw(p, PlanePhase::GoAround) ? "yes" : "no");
}

// Two arrivals a few seconds apart and a departure: one runway user at a time, and
// no two planes ever touch.
static void testTraffic()
{
    Sim s;
    Pilot& dep = s.add(577);
    Pilot& a1 = s.add(519);
    Pilot& a2 = s.add(553);
    PilotEnv e = s.env();
    CHECK(dep.park(e, 0, 0));
    a1.place(e, { -9000.0f, 0.0f, 400.0f }, 270.0f, false, 0.0f);
    a2.place(e, { -9500.0f, 2500.0f, 450.0f }, 270.0f, false, 0.0f);
    CHECK(a1.land(e, 0, "", 1, {}));
    CHECK(a2.land(e, 0, "", -2, {}));
    CHECK(dep.takeoff(e, "", { { 3000, -3000, 400 }, { 6000, -3000, 400 } }));
    float closestGround = 1e9f, closestAir = 1e9f;
    int bothRolling = 0;
    for (int i = 0; i < 50 * 1500; i++) {
        s.step(0.02f);
        const Pilot* ps[3] = { &dep, &a1, &a2 };
        int rolling = 0;
        for (int k = 0; k < 3; k++) {
            CHECK(sane(ps[k]->st));
            const PlanePhase ph = ps[k]->phase();
            if (ph == PlanePhase::Takeoff || ph == PlanePhase::Landing) rolling++;
            for (int j = k + 1; j < 3; j++) {
                const float d = std::hypot(ps[k]->st.pos.x - ps[j]->st.pos.x, ps[k]->st.pos.y - ps[j]->st.pos.y);
                const float dz = std::fabs(ps[k]->st.pos.z - ps[j]->st.pos.z);
                if (ps[k]->st.ground && ps[j]->st.ground) closestGround = std::min(closestGround, d);
                else if (dz < 60.0f) closestAir = std::min(closestAir, d);
            }
        }
        if (rolling > 1) bothRolling++;
        if (a1.ended >= 0 && a2.ended >= 0 && dep.ended >= 0) break;
    }
    CHECK(bothRolling == 0);
    CHECK(a1.phase() == PlanePhase::AtGate && a1.gate() == 1);
    CHECK(a2.phase() == PlanePhase::Parked);
    CHECK(dep.ended == static_cast<int>(PilotEnd::Arrived));
    CHECK(closestGround > 25.0f);
    CHECK(closestAir > 60.0f);
    std::printf("traffic: all done at t %.0f s, closest on the ground %.0f m, in the air %.0f m; a1 %s, a2 %s\n", s.now,
        closestGround, closestAir, saw(a1, PlanePhase::GoAround) || saw(a1, PlanePhase::Holding) ? "held/went around" : "straight in",
        saw(a2, PlanePhase::GoAround) || saw(a2, PlanePhase::Holding) ? "held/went around" : "straight in");
}

// Both gates push back and leave at once while a Dodo taxis in to S1, head-on along A:
// they wait for each other rather than meet, and all get where they are going.
static void testGround()
{
    Sim s;
    Pilot& g1 = s.add(577);
    Pilot& g2 = s.add(592);
    Pilot& dodo = s.add(593);
    PilotEnv e = s.env();
    CHECK(g1.park(e, 0, 0));
    CHECK(g2.park(e, 0, 1));
    dodo.place(e, { 600.0f, 150.0f, 10.0f + dodo.spec.ride }, 270.0f, true, 0.0f);
    CHECK(g1.takeoff(e, "27", {}));
    CHECK(g2.takeoff(e, "27", {}));
    CHECK(dodo.taxiTo(e, s.db.get(0)->gate("S1"), -1));
    float closest = 1e9f;
    for (int i = 0; i < 50 * 1200; i++) {
        s.step(0.02f);
        const Pilot* ps[3] = { &g1, &g2, &dodo };
        for (int k = 0; k < 3; k++)
            for (int j = k + 1; j < 3; j++) {
                if (!ps[k]->st.ground || !ps[j]->st.ground) continue;
                const float gap = outlineGap({ ps[k]->st.pos, ps[k]->st.heading }, ps[k]->spec.span, ps[k]->spec.length,
                    { ps[j]->st.pos, ps[j]->st.heading }, ps[j]->spec.span, ps[j]->spec.length);
                closest = std::min(closest, gap);
            }
        if (g1.ended >= 0 && g2.ended >= 0 && dodo.ended >= 0) break;
    }
    CHECK(g1.ended == static_cast<int>(PilotEnd::Arrived));
    CHECK(g2.ended == static_cast<int>(PilotEnd::Arrived));
    CHECK(dodo.ended == static_cast<int>(PilotEnd::Arrived) && dodo.phase() == PlanePhase::AtGate);
    CHECK(closest > 0.0f);
    std::printf("ground: all done at t %.0f s, %.1f m wingtip to anything at the closest\n", s.now, closest);
}

// Two planes taxiing at each other along A: one waits clear of the other's way until
// it has passed, so they never meet nose to nose.
static void testHeadOn()
{
    Sim s;
    Pilot& east = s.add(519);
    Pilot& west = s.add(553);
    PilotEnv e = s.env();
    const Airport* a = s.db.get(0);
    east.place(e, { 0.0f, 150.0f, 10.0f + east.spec.ride }, 270.0f, true, 0.0f);
    west.place(e, { 2500.0f, 150.0f, 10.0f + west.spec.ride }, 90.0f, true, 0.0f);
    CHECK(east.taxiTo(e, -1, a->node("A4")));
    CHECK(west.taxiTo(e, -1, a->node("A1")));
    float closest = 1e9f;
    for (int i = 0; i < 50 * 900; i++) {
        s.step(0.02f);
        closest = std::min(closest, outlineGap({ east.st.pos, east.st.heading }, east.spec.span, east.spec.length,
                                        { west.st.pos, west.st.heading }, west.spec.span, west.spec.length));
        if (east.ended >= 0 && west.ended >= 0) break;
    }
    CHECK(east.ended == static_cast<int>(PilotEnd::Arrived));
    CHECK(west.ended == static_cast<int>(PilotEnd::Arrived));
    CHECK(closest > 0.0f);
    std::printf("head-on: both through at t %.0f s, %.1f m apart at the closest\n", s.now, closest);
}

// Flying at 300 m into the 600 m hill: it climbs over or turns away, never into it.
static void testTerrain()
{
    Sim s;
    Pilot& p = s.add(519);
    PilotEnv e = s.env();
    p.place(e, { 4000.0f, 0.0f, 300.0f }, 270.0f, false, 0.0f);
    CHECK(p.fly(e, { { 14000.0f, 0.0f, 300.0f } }, 0.0f, false));
    float minClear = 1e9f;
    for (int i = 0; i < 50 * 400 && p.ended < 0; i++) {
        s.step(0.02f);
        minClear = std::min(minClear, p.st.pos.z - top(p.st.pos.x, p.st.pos.y));
    }
    CHECK(p.ended == static_cast<int>(PilotEnd::Arrived));
    CHECK(minClear > 20.0f);
    std::printf("terrain: past the hill, %.0f m over it at the closest\n", minClear);
}

// A plane chasing a target that goes round in circles stays near it.
static void testChase()
{
    Sim s;
    Pilot& p = s.add(520);
    PilotEnv e = s.env();
    p.place(e, { -3000.0f, -3000.0f, 300.0f }, 0.0f, false, 0.0f);
    float t = 0.0f;
    auto where = [&](int, Vec3& pos, Vec3& vel) {
        pos = { 2000.0f * std::cos(t * 0.02f), 2000.0f * std::sin(t * 0.02f), 150.0f };
        vel = { -40.0f * std::sin(t * 0.02f), 40.0f * std::cos(t * 0.02f), 0.0f };
        return true;
    };
    CHECK(p.chase(e, 7, 0.0f, 50.0f));
    float far = 0.0f;
    for (int i = 0; i < 50 * 300; i++) {
        PilotEnv en = s.env();
        en.target = where;
        s.atc.update(s.db, {}, s.now);
        p.step(en, 0.02f);
        s.now += 0.02f;
        t += 0.02f;
        Vec3 tp, tv;
        where(7, tp, tv);
        if (t > 120.0f) far = std::max(far, std::hypot(tp.x - p.st.pos.x, tp.y - p.st.pos.y));
    }
    CHECK(p.phase() == PlanePhase::Chase);
    CHECK(far < 2500.0f);
    std::printf("chase: at most %.0f m from the target after the first two minutes\n", far);
}

// San Andreas' own airports (airports/gta_sa.txt): from every gate a plane that fits
// gets away, and one that lands taxis back to it.
static void testStockAirports()
{
    Airports db;
    std::string error;
    CHECK(db.loadText(AIRPORTS_GTA_SA, "gta_sa.txt", error));
    if (!error.empty()) std::fprintf(stderr, "%s\n", error.c_str());
    const char* codes[] = { "LSIA", "SFIA", "LVA", "VM" };
    int runs = 0;
    for (const char* code : codes) {
        const int ap = db.find(code);
        CHECK(ap >= 0);
        if (ap < 0) continue;
        const Airport& a = *db.get(ap);
        for (size_t g = 0; g < a.gates.size(); g++) {
            if (a.gates[g].removed) continue;
            for (int model : { 577, 553, 593 }) {
                const PlaneSpec spec = PlaneSpec::of(model);
                if (spec.span > a.gates[g].span) continue;
                bool runwayFits = false;
                for (const Runway& r : a.runways)
                    if (!r.removed && r.open && r.span >= spec.span && r.length >= spec.takeoffRun() * 1.2f) runwayFits = true;
                if (!runwayFits) continue;
                for (int arrive = 0; arrive < 2; arrive++) {
                    Atc atc;
                    atc.setDb(&db);
                    Pilot p(0, model);
                    PilotEnv e;
                    e.db = &db;
                    e.atc = &atc;
                    e.world.top = [](float, float) { return -50.0f; };
                    const char* what = arrive ? "land" : "take off";
                    bool ok;
                    if (arrive) {
                        const Vec3 c { (a.lo.x + a.hi.x) * 0.5f, (a.lo.y + a.hi.y) * 0.5f, (a.lo.z + a.hi.z) * 0.5f };
                        p.place(e, { c.x + 5000.0f, c.y + 3000.0f, c.z + 500.0f }, 45.0f, false, 0.0f);
                        ok = p.land(e, ap, "", static_cast<int>(g), {});
                    } else {
                        ok = p.park(e, ap, static_cast<int>(g)) && p.takeoff(e, "", {});
                    }
                    if (!ok) {
                        std::fprintf(stderr, "%s %s %s: cannot %s (%s)\n", code, a.gates[g].name.c_str(), spec.name.c_str(), what,
                            p.why.c_str());
                        failures++;
                        continue;
                    }
                    float now = 0.0f;
                    const bool trace = std::getenv("DNAV_TRACE") != nullptr;
                    for (int i = 0; i < 20 * 900 && p.ended < 0; i++) {
                        e.now = now;
                        atc.update(db, {}, now);
                        const size_t was = p.phases.size();
                        p.step(e, 0.05f);
                        if (trace && (p.phases.size() != was || i % 100 == 0))
                            std::printf("  %s %s t %6.1f %-10s (%7.0f %7.0f %5.0f) h %3.0f v %4.1f\n", code, a.gates[g].name.c_str(), now,
                                phaseName(p.phase()), p.st.pos.x, p.st.pos.y, p.st.pos.z, p.st.heading, p.st.speed);
                        now += 0.05f;
                    }
                    const bool done = p.ended == static_cast<int>(PilotEnd::Arrived) && sane(p.st)
                        && (arrive ? p.phase() == PlanePhase::AtGate && p.gate() == static_cast<int>(g) : saw(p, PlanePhase::Climb));
                    if (!done) {
                        std::fprintf(stderr, "%s %s %s: did not %s (%s after %.0f s at %.0f %.0f %.0f; %s)\n", code,
                            a.gates[g].name.c_str(), spec.name.c_str(), what, phaseName(p.phase()), now, p.st.pos.x, p.st.pos.y,
                            p.st.pos.z, p.why.c_str());
                        failures++;
                    }
                    runs++;
                }
            }
        }
    }
    std::printf("stock airports: %d departures and arrivals flown\n", runs);
}

int main()
{
    testData();
    testDeparture();
    testArrival();
    testTraffic();
    testGround();
    testHeadOn();
    testTerrain();
    testChase();
    testStockAirports();
    if (failures) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("aircraft: all passed\n");
    return 0;
}
