// Airports: runways, taxiways, gates and the routes planes fly between them
// (navmesh/airports/*.txt, read in name order; Nav_LoadAirports reads more).
//
// The built-in file, airports/gta_sa.txt, has San Andreas' own four: Los Santos
// International, San Fierro (Easter Bay), Las Venturas and the Verdant Meadows
// airstrip. A server adds its own airstrips and taxi routes in files of its own,
// and changes the built-in ones the same way: a later `airport LSIA` line goes on
// with that airport, a node, taxiway, gate or runway given again replaces the old
// one, and `remove ...` takes one away. AIRPORTS_HELP says how a file is written.
//
// The graph is small (a few hundred nodes an airport) and searched on the game
// thread. Nothing here touches open.mp.

#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "aircraft.hpp"

namespace dnav {

extern const char* const AIRPORTS_HELP;
// The built-in airports: airports/gta_sa.txt, compiled in (CMakeLists.txt).
extern const char* const AIRPORTS_GTA_SA;

struct TaxiNode {
    std::string name;
    Vec3 p {};
    std::vector<int> edges;     // indices into Airport::edges
    int runway = -1;            // on this runway (its centre line)...
    float along = 0.0f;         // ...this far from its first end
    int holdFor = -1;           // a hold short of this runway: the last node off it on the way on
    bool removed = false;
    bool made = false;          // worked out by finish(), not from a file
};

struct TaxiEdge {
    int a = -1, b = -1;         // nodes
    std::string taxiway;        // its name ("A", "B2"; the runway's name along a runway)
    float span = 1e9f;          // the widest wings that fit along it (m)
    bool oneway = false;        // only from a to b
    int runway = -1;            // along this runway
    std::vector<int> crosses;   // runways it crosses (planes need a clearance to cross)
    float length = 0.0f;
    bool removed = false;
    bool made = false;          // worked out by finish() (along a runway), not from a file
    int other(int n) const { return n == a ? b : a; }
};

struct Runway {
    std::string name;           // "09/27"
    std::string endName[2];     // "09", "27"
    Vec3 end[2] {};             // the two ends of its centre line, on the surface
    float width = 45.0f;
    float span = 1e9f;          // the widest wings that may use it
    float displaced[2] = { 0.0f, 0.0f }; // m the landing threshold is in from each end
    bool land[2] = { true, true };       // which ends planes land at...
    bool depart[2] = { true, true };     // ...and take off from
    bool open = true;           // Nav_SetRunwayOpen
    bool removed = false;
    float length = 0.0f;
    Vec3 dir {};                // from end 0 to end 1, unit
    // Fixes flown before the final, and after take-off, by end (landing at / taking
    // off from that end).
    std::vector<Vec3> approach[2], departure[2];
    std::vector<int> nodes;     // taxi nodes on it, by `along`

    // A point `along` metres from end 0 on the centre line.
    Vec3 pointAt(float along) const;
    // How far along and how far off the centre line (left of end 0 -> end 1 positive) p is.
    void place(const Vec3& p, float& along, float& off) const;
    // The heading planes land and take off at, from end e (towards the other).
    float headingFrom(int e) const;
    // The landing threshold at end e, and where a plane aims to touch down.
    float thresholdAlong(int e) const { return e == 0 ? displaced[0] : length - displaced[1]; }
    // Whether p is on the runway (with `pad` metres round it).
    bool contains(const Vec3& p, float pad) const;
};

struct Gate {
    std::string name;
    Vec3 p {};                  // the ground under the parked plane's centre
    float heading = 0.0f;       // the parked plane's
    float span = 1e9f;          // the widest wings it takes
    int lead = -1;              // the taxi node a plane comes in through (and leaves by, nose first)
    int push = -1;              // where a pushback ends (-1: it taxis out forwards, a stand)
    bool removed = false;
};

struct Airport {
    std::string code;           // "LSIA"
    std::string label;          // "Los Santos International"
    float prefer = -1.0f;       // the heading planes land and take off at when they can (-1: any)
    std::vector<TaxiNode> nodes;
    std::vector<TaxiEdge> edges;
    std::vector<Runway> runways;
    std::vector<Gate> gates;
    Vec3 lo {}, hi {};          // a box round it all (after finish())
    bool removed = false;

    int node(const std::string& name) const;
    int runway(const std::string& name) const;     // by "09/27", or by either end's name
    int gate(const std::string& name) const;
    // Whether p is inside its box, `pad` metres round.
    bool contains(const Vec3& p, float pad) const;
};

struct FlightRoute {
    std::string name;
    std::vector<Vec3> points;
    bool removed = false;
};

// A taxi search's answer: the nodes in order and the edges between them.
struct TaxiRoute {
    std::vector<int> nodes;
    std::vector<int> edges;     // edges[i] joins nodes[i] and nodes[i + 1]
    float cost = 0.0f;
    bool ok() const { return !nodes.empty(); }
};

class Airports {
public:
    // Reads a file of airports; what it adds goes on top of what is loaded. False with
    // `error` (and nothing changed) if it cannot be read or a line is wrong.
    bool loadFile(const std::string& path, std::string& error);
    bool loadText(const std::string& text, const std::string& source, std::string& error);
    // Works out what the files leave implicit (nodes on runways, holds, crossings, the
    // box round each airport). Called after loading; safe to call again.
    void finish();

    size_t count() const { return airports_.size(); }
    Airport* get(int i) { return i >= 0 && i < static_cast<int>(airports_.size()) && !airports_[i].removed ? &airports_[i] : nullptr; }
    const Airport* get(int i) const { return i >= 0 && i < static_cast<int>(airports_.size()) && !airports_[i].removed ? &airports_[i] : nullptr; }
    int find(const std::string& code) const;
    // The airport p is on (inside its box), or -1.
    int at(const Vec3& p, float pad = 50.0f) const;
    const FlightRoute* route(const std::string& name) const;
    void setRoute(const std::string& name, const std::vector<Vec3>& points);
    void addRoutePoint(const std::string& name, const Vec3& p);
    size_t routeCount() const { return routes_.size(); }
    int liveCount() const;

    // The cheapest way along the taxiways from `from` to any of `goals` for wings of
    // `span`: runways cost more than taxiways (crossing one costs a little), edges for
    // which `blocked` says true are left out, and `penalty` adds to an edge's cost.
    TaxiRoute taxiRoute(const Airport& ap, int from, const std::vector<int>& goals, float span,
        const std::function<bool(int edge, int fromNode)>& blocked = {},
        const std::function<float(int edge, int fromNode)>& penalty = {}) const;
    // The node nearest p that wings of `span` can use (-1: none within reach).
    int nearestNode(const Airport& ap, const Vec3& p, float span, float reach = 200.0f, bool offRunway = false) const;

private:
    std::vector<Airport> airports_;
    std::vector<FlightRoute> routes_;
};

} // namespace dnav
