// Airports: runways, taxiways, gates and flight routes (airports.hpp).

#include "airports.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <queue>
#include <sstream>

namespace dnav {

const char* const AIRPORTS_HELP = R"(An airports file (navmesh/airports/*.txt) is lines of words; # starts a comment.
Coordinates are GTA's, on the ground (the runway or taxiway surface); headings are
GTA's (degrees, 0 north, turning left: 90 is west).

  airport CODE Label words            starts (or goes on with) an airport; what follows is its
  prefer HEADING                      planes land and take off this way when the runway allows
  runway 09/27 X1 Y1 Z1 X2 Y2 Z2 [width M] [span M] [displaced END M] [noland END]
         [nodepart END] [closed]      a runway's centre line from its 09 end to its 27 end
  node NAME X Y Z                     a point on a taxiway (or on a runway's centre line)
  taxi NAME [span M] [oneway] N1 N2 N3 ...
                                      a taxiway through these nodes, both ways unless oneway;
                                      span is the widest wings that fit along it
  gate NAME X Y Z HEADING [span M] [lead NODE] [push NODE]
                                      a parked plane's centre and heading; it taxis in and out
                                      through `lead` (nose first), or is pushed back to `push`
  approach END X Y Z [X Y Z ...]      fixes flown before the final to the END end
  departure END X Y Z [X Y Z ...]     fixes flown after taking off from the END end
  route NAME X Y Z [X Y Z ...]        a flight route (any airport, or none); route+ adds points
  remove airport CODE | runway NAME | node NAME | taxi NAME | edge N1 N2 | gate NAME | route NAME

Runway nodes, the holds short of each runway (the last node before it) and the runways a
taxiway crosses are worked out. A runway with no nodes on it gets them every 150 m, so
planes can backtrack along it; nodes on a runway are joined along it
in order. Given again, a runway, node, gate or route replaces the
old one, and a taxi line between two nodes already joined replaces that edge.)";

namespace {

constexpr float AUTO_NODE_EVERY = 150.0f;
constexpr float RUNWAY_COST = 3.0f;         // taxiing along a runway costs this many times more
constexpr float CROSS_COST = 60.0f;         // m a runway crossing costs

float d2(const Vec3& a, const Vec3& b)
{
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

bool num(const std::string& s, float& out)
{
    try {
        size_t k = 0;
        out = std::stof(s, &k);
        return k == s.size();
    } catch (...) {
        return false;
    }
}

// Whether segments ab and cd cross (2D, not just touching at an end).
bool segmentsCross(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d)
{
    auto orient = [](const Vec3& p, const Vec3& q, const Vec3& r) {
        return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
    };
    const float o1 = orient(a, b, c), o2 = orient(a, b, d), o3 = orient(c, d, a), o4 = orient(c, d, b);
    return ((o1 > 0) != (o2 > 0)) && ((o3 > 0) != (o4 > 0)) && std::fabs(o1) > 1e-3f && std::fabs(o2) > 1e-3f;
}

} // namespace

// ── Runway ───────────────────────────────────────────────────────────────────

Vec3 Runway::pointAt(float along) const
{
    const float k = length > 0.0f ? along / length : 0.0f;
    return { end[0].x + dir.x * along, end[0].y + dir.y * along, end[0].z + (end[1].z - end[0].z) * k };
}

void Runway::place(const Vec3& p, float& along, float& off) const
{
    const float px = p.x - end[0].x, py = p.y - end[0].y;
    along = px * dir.x + py * dir.y;
    off = -px * dir.y + py * dir.x;
}

float Runway::headingFrom(int e) const
{
    const float dx = e == 0 ? dir.x : -dir.x, dy = e == 0 ? dir.y : -dir.y;
    float h = std::atan2(-dx, dy) * 180.0f / 3.14159265f;
    if (h < 0.0f) h += 360.0f;
    return h;
}

bool Runway::contains(const Vec3& p, float pad) const
{
    float along, off;
    place(p, along, off);
    return along >= -pad && along <= length + pad && std::fabs(off) <= width * 0.5f + pad;
}

// ── Airport ──────────────────────────────────────────────────────────────────

int Airport::node(const std::string& name) const
{
    for (size_t i = 0; i < nodes.size(); i++)
        if (!nodes[i].removed && nodes[i].name == name) return static_cast<int>(i);
    return -1;
}

int Airport::runway(const std::string& name) const
{
    for (size_t i = 0; i < runways.size(); i++) {
        const Runway& r = runways[i];
        if (!r.removed && (r.name == name || r.endName[0] == name || r.endName[1] == name)) return static_cast<int>(i);
    }
    return -1;
}

int Airport::gate(const std::string& name) const
{
    for (size_t i = 0; i < gates.size(); i++)
        if (!gates[i].removed && gates[i].name == name) return static_cast<int>(i);
    return -1;
}

bool Airport::contains(const Vec3& p, float pad) const
{
    return p.x >= lo.x - pad && p.x <= hi.x + pad && p.y >= lo.y - pad && p.y <= hi.y + pad;
}

// ── Loading ──────────────────────────────────────────────────────────────────

bool Airports::loadFile(const std::string& path, std::string& error)
{
    std::ifstream in(path);
    if (!in) {
        error = path + ": cannot be read";
        return false;
    }
    std::stringstream ss;
    ss << in.rdbuf();
    return loadText(ss.str(), path, error);
}

bool Airports::loadText(const std::string& text, const std::string& source, std::string& error)
{
    std::vector<Airport> aps = airports_;
    std::vector<FlightRoute> routes = routes_;
    Airport* cur = nullptr;
    std::istringstream lines(text);
    std::string line;
    int lineNo = 0;
    auto fail = [&](const std::string& why) {
        error = source + ":" + std::to_string(lineNo) + ": " + why;
        return false;
    };
    auto findAirport = [&](const std::string& code) -> Airport* {
        for (auto& a : aps)
            if (!a.removed && a.code == code) return &a;
        return nullptr;
    };
    auto routeOf = [&](const std::string& name) -> FlightRoute& {
        for (auto& r : routes)
            if (r.name == name) return r;
        routes.push_back({ name, {}, false });
        return routes.back();
    };
    while (std::getline(lines, line)) {
        lineNo++;
        const size_t hash = line.find('#');
        if (hash != std::string::npos) line.erase(hash);
        std::istringstream words(line);
        std::vector<std::string> w;
        for (std::string t; words >> t;) w.push_back(t);
        if (w.empty()) continue;
        const std::string& kw = w[0];
        auto needAirport = [&]() { return cur != nullptr; };
        auto points = [&](size_t from, std::vector<Vec3>& out) {
            if ((w.size() - from) % 3 != 0 || w.size() == from) return false;
            for (size_t i = from; i + 2 < w.size(); i += 3) {
                Vec3 p;
                if (!num(w[i], p.x) || !num(w[i + 1], p.y) || !num(w[i + 2], p.z)) return false;
                out.push_back(p);
            }
            return true;
        };
        if (kw == "airport") {
            if (w.size() < 2) return fail("airport needs a code");
            cur = findAirport(w[1]);
            if (!cur) {
                aps.push_back(Airport {});
                cur = &aps.back();
                cur->code = w[1];
            }
            if (w.size() > 2) {
                std::string label;
                for (size_t i = 2; i < w.size(); i++) label += (i > 2 ? " " : "") + w[i];
                cur->label = label;
            }
        } else if (kw == "prefer") {
            if (!needAirport()) return fail("prefer before any airport");
            if (w.size() != 2 || !num(w[1], cur->prefer)) return fail("prefer HEADING");
        } else if (kw == "runway") {
            if (!needAirport()) return fail("runway before any airport");
            if (w.size() < 8) return fail("runway NAME X1 Y1 Z1 X2 Y2 Z2 ...");
            Runway r;
            r.name = w[1];
            const size_t slash = r.name.find('/');
            if (slash == std::string::npos || slash == 0 || slash + 1 == r.name.size())
                return fail("a runway's name is its two ends, 09/27");
            r.endName[0] = r.name.substr(0, slash);
            r.endName[1] = r.name.substr(slash + 1);
            if (!num(w[2], r.end[0].x) || !num(w[3], r.end[0].y) || !num(w[4], r.end[0].z) || !num(w[5], r.end[1].x)
                || !num(w[6], r.end[1].y) || !num(w[7], r.end[1].z))
                return fail("runway: bad coordinates");
            auto endOf = [&](const std::string& n) { return n == r.endName[0] ? 0 : n == r.endName[1] ? 1 : -1; };
            for (size_t i = 8; i < w.size(); i++) {
                if (w[i] == "width" && i + 1 < w.size() && num(w[i + 1], r.width)) i++;
                else if (w[i] == "span" && i + 1 < w.size() && num(w[i + 1], r.span)) i++;
                else if (w[i] == "displaced" && i + 2 < w.size() && endOf(w[i + 1]) >= 0) {
                    if (!num(w[i + 2], r.displaced[endOf(w[i + 1])])) return fail("displaced END M");
                    i += 2;
                } else if (w[i] == "noland" && i + 1 < w.size() && endOf(w[i + 1]) >= 0) r.land[endOf(w[++i])] = false;
                else if (w[i] == "nodepart" && i + 1 < w.size() && endOf(w[i + 1]) >= 0) r.depart[endOf(w[++i])] = false;
                else if (w[i] == "closed") r.open = false;
                else return fail("runway: what is '" + w[i] + "'?");
            }
            const int was = cur->runway(r.name);
            if (was >= 0) {
                r.approach[0] = cur->runways[was].approach[0];
                r.approach[1] = cur->runways[was].approach[1];
                r.departure[0] = cur->runways[was].departure[0];
                r.departure[1] = cur->runways[was].departure[1];
                cur->runways[was] = r;
            } else {
                cur->runways.push_back(r);
            }
        } else if (kw == "node") {
            if (!needAirport()) return fail("node before any airport");
            TaxiNode n;
            if (w.size() != 5 || !num(w[2], n.p.x) || !num(w[3], n.p.y) || !num(w[4], n.p.z)) return fail("node NAME X Y Z");
            n.name = w[1];
            const int was = cur->node(n.name);
            if (was >= 0) cur->nodes[was].p = n.p;
            else cur->nodes.push_back(n);
        } else if (kw == "taxi") {
            if (!needAirport()) return fail("taxi before any airport");
            if (w.size() < 4) return fail("taxi NAME [span M] [oneway] N1 N2 ...");
            float span = 1e9f;
            bool oneway = false;
            std::vector<int> chain;
            for (size_t i = 2; i < w.size(); i++) {
                if (w[i] == "span" && i + 1 < w.size()) {
                    if (!num(w[++i], span)) return fail("taxi: span M");
                } else if (w[i] == "oneway") {
                    oneway = true;
                } else {
                    const int n = cur->node(w[i]);
                    if (n < 0) return fail("taxi: no node '" + w[i] + "' (define it with a node line first)");
                    chain.push_back(n);
                }
            }
            if (chain.size() < 2) return fail("taxi: at least two nodes");
            for (size_t i = 0; i + 1 < chain.size(); i++) {
                TaxiEdge e;
                e.a = chain[i];
                e.b = chain[i + 1];
                e.taxiway = w[1];
                e.span = span;
                e.oneway = oneway;
                bool replaced = false;
                for (auto& o : cur->edges) {
                    if (!o.removed && ((o.a == e.a && o.b == e.b) || (o.a == e.b && o.b == e.a))) {
                        o = e;
                        replaced = true;
                        break;
                    }
                }
                if (!replaced) cur->edges.push_back(e);
            }
        } else if (kw == "gate") {
            if (!needAirport()) return fail("gate before any airport");
            Gate g;
            if (w.size() < 6 || !num(w[2], g.p.x) || !num(w[3], g.p.y) || !num(w[4], g.p.z) || !num(w[5], g.heading))
                return fail("gate NAME X Y Z HEADING [span M] [lead NODE] [push NODE]");
            g.name = w[1];
            for (size_t i = 6; i < w.size(); i++) {
                if (w[i] == "span" && i + 1 < w.size()) {
                    if (!num(w[++i], g.span)) return fail("gate: span M");
                } else if ((w[i] == "lead" || w[i] == "push") && i + 1 < w.size()) {
                    const int n = cur->node(w[i + 1]);
                    if (n < 0) return fail("gate: no node '" + w[i + 1] + "'");
                    (w[i] == "lead" ? g.lead : g.push) = n;
                    i++;
                } else {
                    return fail("gate: what is '" + w[i] + "'?");
                }
            }
            if (g.lead < 0) return fail("gate " + g.name + " needs a lead node (where it joins the taxiways)");
            const int was = cur->gate(g.name);
            if (was >= 0) cur->gates[was] = g;
            else cur->gates.push_back(g);
        } else if (kw == "approach" || kw == "departure") {
            if (!needAirport()) return fail(kw + " before any airport");
            if (w.size() < 5) return fail(kw + " END X Y Z [X Y Z ...]");
            int rw = -1, e = -1;
            for (size_t i = 0; i < cur->runways.size(); i++) {
                if (cur->runways[i].removed) continue;
                for (int k = 0; k < 2; k++)
                    if (cur->runways[i].endName[k] == w[1]) {
                        rw = static_cast<int>(i);
                        e = k;
                    }
            }
            if (rw < 0) return fail(kw + ": no runway end '" + w[1] + "'");
            std::vector<Vec3> pts;
            if (!points(2, pts)) return fail(kw + ": coordinates come in threes");
            (kw == "approach" ? cur->runways[rw].approach[e] : cur->runways[rw].departure[e]) = pts;
        } else if (kw == "route" || kw == "route+") {
            if (w.size() < 2) return fail("route NAME X Y Z ...");
            std::vector<Vec3> pts;
            if (w.size() > 2 && !points(2, pts)) return fail("route: coordinates come in threes");
            FlightRoute& r = routeOf(w[1]);
            if (kw == "route") r.points.clear();
            r.points.insert(r.points.end(), pts.begin(), pts.end());
            r.removed = false;
        } else if (kw == "remove") {
            if (w.size() < 3) return fail("remove WHAT NAME");
            const std::string& what = w[1];
            if (what == "airport") {
                Airport* a = findAirport(w[2]);
                if (!a) return fail("remove airport: no airport '" + w[2] + "'");
                a->removed = true;
                if (a == cur) cur = nullptr;
            } else if (what == "route") {
                for (auto& r : routes)
                    if (r.name == w[2]) r.removed = true;
            } else {
                if (!needAirport()) return fail("remove " + what + " before any airport");
                if (what == "runway") {
                    const int r = cur->runway(w[2]);
                    if (r < 0) return fail("remove runway: no runway '" + w[2] + "'");
                    cur->runways[r].removed = true;
                } else if (what == "node") {
                    const int n = cur->node(w[2]);
                    if (n < 0) return fail("remove node: no node '" + w[2] + "'");
                    cur->nodes[n].removed = true;
                    for (auto& e : cur->edges)
                        if (e.a == n || e.b == n) e.removed = true;
                } else if (what == "taxi") {
                    bool any = false;
                    for (auto& e : cur->edges)
                        if (e.taxiway == w[2]) e.removed = any = true;
                    if (!any) return fail("remove taxi: no taxiway '" + w[2] + "'");
                } else if (what == "edge") {
                    if (w.size() < 4) return fail("remove edge N1 N2");
                    const int a = cur->node(w[2]), b = cur->node(w[3]);
                    bool any = false;
                    for (auto& e : cur->edges)
                        if ((e.a == a && e.b == b) || (e.a == b && e.b == a)) e.removed = any = true;
                    if (!any) return fail("remove edge: " + w[2] + " and " + w[3] + " are not joined");
                } else if (what == "gate") {
                    const int g = cur->gate(w[2]);
                    if (g < 0) return fail("remove gate: no gate '" + w[2] + "'");
                    cur->gates[g].removed = true;
                } else {
                    return fail("remove: what is '" + what + "'?");
                }
            }
        } else {
            return fail("what is '" + kw + "'? (" + std::string("airport, prefer, runway, node, taxi, gate, approach, departure, route, remove") + ")");
        }
    }
    airports_ = std::move(aps);
    routes_ = std::move(routes);
    finish();
    return true;
}

void Airports::finish()
{
    for (Airport& ap : airports_) {
        if (ap.removed) continue;
        // What an earlier finish() made is worked out again.
        for (auto& n : ap.nodes)
            if (n.made) n.removed = true;
        for (auto& e : ap.edges)
            if (e.made) e.removed = true;
        for (auto& r : ap.runways) {
            r.nodes.clear();
            const float dx = r.end[1].x - r.end[0].x, dy = r.end[1].y - r.end[0].y;
            r.length = std::sqrt(dx * dx + dy * dy);
            r.dir = r.length > 0.0f ? Vec3 { dx / r.length, dy / r.length, 0.0f } : Vec3 { 0.0f, 1.0f, 0.0f };
        }
        for (auto& n : ap.nodes) {
            n.runway = -1;
            n.holdFor = -1;
            n.edges.clear();
            if (n.removed) continue;
            for (size_t r = 0; r < ap.runways.size(); r++) {
                const Runway& rw = ap.runways[r];
                if (rw.removed) continue;
                float along, off;
                rw.place(n.p, along, off);
                if (std::fabs(off) <= std::max(3.0f, rw.width * 0.25f) && along >= -5.0f && along <= rw.length + 5.0f) {
                    n.runway = static_cast<int>(r);
                    n.along = along;
                    break;
                }
            }
        }
        // A runway with no node on it: nodes along it, so planes can use it at all.
        for (size_t r = 0; r < ap.runways.size(); r++) {
            Runway& rw = ap.runways[r];
            if (rw.removed) continue;
            bool any = false;
            for (const auto& n : ap.nodes)
                if (!n.removed && n.runway == static_cast<int>(r)) any = true;
            if (any) continue;
            const int count = std::max(2, static_cast<int>(std::ceil(rw.length / AUTO_NODE_EVERY)) + 1);
            for (int k = 0; k < count; k++) {
                TaxiNode n;
                n.name = rw.name + "@" + std::to_string(k);
                n.along = rw.length * k / (count - 1);
                n.p = rw.pointAt(n.along);
                n.runway = static_cast<int>(r);
                n.made = true;
                ap.nodes.push_back(n);
            }
        }
        for (size_t i = 0; i < ap.nodes.size(); i++)
            if (!ap.nodes[i].removed && ap.nodes[i].runway >= 0) ap.runways[ap.nodes[i].runway].nodes.push_back(static_cast<int>(i));
        for (auto& rw : ap.runways)
            std::sort(rw.nodes.begin(), rw.nodes.end(), [&](int a, int b) { return ap.nodes[a].along < ap.nodes[b].along; });
        // The nodes on a runway are joined along it, in order, where a file has not.
        for (size_t r = 0; r < ap.runways.size(); r++) {
            const Runway& rw = ap.runways[r];
            if (rw.removed) continue;
            for (size_t k = 0; k + 1 < rw.nodes.size(); k++) {
                const int a = rw.nodes[k], b = rw.nodes[k + 1];
                bool joined = false;
                for (const auto& e : ap.edges)
                    if (!e.removed && ((e.a == a && e.b == b) || (e.a == b && e.b == a))) joined = true;
                if (joined) continue;
                TaxiEdge e;
                e.a = a;
                e.b = b;
                e.taxiway = rw.name;
                e.made = true;
                ap.edges.push_back(e);
            }
        }
        // Edges: their length, the runway they run along, the runways they cross.
        for (size_t i = 0; i < ap.edges.size(); i++) {
            TaxiEdge& e = ap.edges[i];
            e.runway = -1;
            e.crosses.clear();
            if (e.removed || ap.nodes[e.a].removed || ap.nodes[e.b].removed) {
                e.removed = true;
                continue;
            }
            const TaxiNode& a = ap.nodes[e.a];
            const TaxiNode& b = ap.nodes[e.b];
            e.length = d2(a.p, b.p);
            if (a.runway >= 0 && a.runway == b.runway) e.runway = a.runway;
            for (size_t r = 0; r < ap.runways.size(); r++) {
                const Runway& rw = ap.runways[r];
                if (rw.removed || static_cast<int>(r) == e.runway || static_cast<int>(r) == a.runway
                    || static_cast<int>(r) == b.runway)
                    continue;
                if (segmentsCross(a.p, b.p, rw.end[0], rw.end[1])) e.crosses.push_back(static_cast<int>(r));
            }
            ap.nodes[e.a].edges.push_back(static_cast<int>(i));
            ap.nodes[e.b].edges.push_back(static_cast<int>(i));
        }
        // Holds: the last node before a runway, on the way onto it.
        for (const auto& e : ap.edges) {
            if (e.removed) continue;
            TaxiNode& a = ap.nodes[e.a];
            TaxiNode& b = ap.nodes[e.b];
            if (a.runway >= 0 && b.runway < 0 && b.holdFor < 0) b.holdFor = a.runway;
            if (b.runway >= 0 && a.runway < 0 && a.holdFor < 0) a.holdFor = b.runway;
        }
        // The box round it.
        bool first = true;
        auto grow = [&](const Vec3& p) {
            if (first) {
                ap.lo = ap.hi = p;
                first = false;
                return;
            }
            ap.lo.x = std::min(ap.lo.x, p.x);
            ap.lo.y = std::min(ap.lo.y, p.y);
            ap.lo.z = std::min(ap.lo.z, p.z);
            ap.hi.x = std::max(ap.hi.x, p.x);
            ap.hi.y = std::max(ap.hi.y, p.y);
            ap.hi.z = std::max(ap.hi.z, p.z);
        };
        for (const auto& n : ap.nodes)
            if (!n.removed) grow(n.p);
        for (const auto& r : ap.runways)
            if (!r.removed) {
                grow(r.end[0]);
                grow(r.end[1]);
            }
        for (const auto& g : ap.gates)
            if (!g.removed) grow(g.p);
    }
}

// ── Finding things ───────────────────────────────────────────────────────────

int Airports::find(const std::string& code) const
{
    for (size_t i = 0; i < airports_.size(); i++)
        if (!airports_[i].removed && airports_[i].code == code) return static_cast<int>(i);
    // Case doesn't matter.
    for (size_t i = 0; i < airports_.size(); i++) {
        if (airports_[i].removed || airports_[i].code.size() != code.size()) continue;
        bool same = true;
        for (size_t k = 0; k < code.size() && same; k++)
            same = std::tolower(static_cast<unsigned char>(code[k])) == std::tolower(static_cast<unsigned char>(airports_[i].code[k]));
        if (same) return static_cast<int>(i);
    }
    return -1;
}

int Airports::at(const Vec3& p, float pad) const
{
    int best = -1;
    float bestArea = 1e30f;
    for (size_t i = 0; i < airports_.size(); i++) {
        const Airport& ap = airports_[i];
        if (ap.removed || !ap.contains(p, pad)) continue;
        const float area = (ap.hi.x - ap.lo.x) * (ap.hi.y - ap.lo.y);
        if (area < bestArea) {
            bestArea = area;
            best = static_cast<int>(i);
        }
    }
    return best;
}

int Airports::liveCount() const
{
    int n = 0;
    for (const auto& a : airports_)
        if (!a.removed) n++;
    return n;
}

const FlightRoute* Airports::route(const std::string& name) const
{
    for (const auto& r : routes_)
        if (!r.removed && r.name == name) return &r;
    return nullptr;
}

void Airports::setRoute(const std::string& name, const std::vector<Vec3>& points)
{
    for (auto& r : routes_)
        if (r.name == name) {
            r.points = points;
            r.removed = false;
            return;
        }
    routes_.push_back({ name, points, false });
}

void Airports::addRoutePoint(const std::string& name, const Vec3& p)
{
    for (auto& r : routes_)
        if (r.name == name) {
            if (r.removed) r.points.clear();
            r.points.push_back(p);
            r.removed = false;
            return;
        }
    routes_.push_back({ name, { p }, false });
}

TaxiRoute Airports::taxiRoute(const Airport& ap, int from, const std::vector<int>& goals, float span,
    const std::function<bool(int, int)>& blocked, const std::function<float(int, int)>& penalty) const
{
    TaxiRoute out;
    const int n = static_cast<int>(ap.nodes.size());
    if (from < 0 || from >= n || goals.empty()) return out;
    std::vector<char> isGoal(n, 0);
    for (int g : goals)
        if (g >= 0 && g < n) isGoal[g] = 1;
    auto h = [&](int i) {
        float best = 1e30f;
        for (int g : goals)
            if (g >= 0 && g < n) best = std::min(best, d2(ap.nodes[i].p, ap.nodes[g].p));
        return best;
    };
    std::vector<float> cost(n, 1e30f);
    std::vector<int> prevNode(n, -1), prevEdge(n, -1);
    using Q = std::pair<float, int>;
    std::priority_queue<Q, std::vector<Q>, std::greater<Q>> open;
    cost[from] = 0.0f;
    open.push({ h(from), from });
    int reached = -1;
    while (!open.empty()) {
        const auto [est, c] = open.top();
        open.pop();
        if (est - h(c) > cost[c] + 1e-3f) continue;
        if (isGoal[c]) {
            reached = c;
            break;
        }
        for (int ei : ap.nodes[c].edges) {
            const TaxiEdge& e = ap.edges[ei];
            if (e.removed || e.span < span) continue;
            if (e.oneway && e.a != c) continue;
            if (e.runway >= 0 && (ap.runways[e.runway].removed || ap.runways[e.runway].span < span)) continue;
            const int o = e.other(c);
            if (ap.nodes[o].removed) continue;
            if (blocked && blocked(ei, c)) continue;
            float step = e.length * (e.runway >= 0 ? RUNWAY_COST : 1.0f) + CROSS_COST * e.crosses.size();
            if (penalty) step += penalty(ei, c);
            if (cost[c] + step < cost[o]) {
                cost[o] = cost[c] + step;
                prevNode[o] = c;
                prevEdge[o] = ei;
                open.push({ cost[o] + h(o), o });
            }
        }
    }
    if (reached < 0) return out;
    for (int c = reached; c != -1; c = prevNode[c]) {
        out.nodes.push_back(c);
        if (prevEdge[c] >= 0) out.edges.push_back(prevEdge[c]);
    }
    std::reverse(out.nodes.begin(), out.nodes.end());
    std::reverse(out.edges.begin(), out.edges.end());
    out.cost = cost[reached];
    return out;
}

int Airports::nearestNode(const Airport& ap, const Vec3& p, float span, float reach, bool offRunway) const
{
    int best = -1;
    float bestD = reach;
    for (size_t i = 0; i < ap.nodes.size(); i++) {
        const TaxiNode& n = ap.nodes[i];
        if (n.removed || (offRunway && n.runway >= 0)) continue;
        bool fits = false;
        for (int ei : n.edges)
            if (!ap.edges[ei].removed && ap.edges[ei].span >= span) fits = true;
        if (!fits) continue;
        const float d = d2(n.p, p) + std::fabs(n.p.z - p.z) * 2.0f;
        if (d < bestD) {
            bestD = d;
            best = static_cast<int>(i);
        }
    }
    return best;
}

} // namespace dnav
