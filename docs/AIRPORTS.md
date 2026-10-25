# Airports

NPC planes (see the Planes section of [the README](../README.md#planes)) taxi, take
off and land on the airports DiamondNav knows. San Andreas' own four are built in;
your own go in `navmesh/airports/*.txt`, and change or add to them.

## What is built in

`airports/gta_sa.txt`, compiled into the component. Measured from the game's
collision, with every taxiway and gate checked clear of objects for the wings it
allows.

| Code | Airport | Runways | Gates |
| --- | --- | --- | --- |
| `LSIA` | Los Santos International | `09L/27R`, `09R/27L` (660 m each) | `P1`-`P4` (small planes, at the terminal's pods), `E1`, `E2` (any size, east apron) |
| `SFIA` | San Fierro International | `04/22` (795 m) | `G1`-`G3` (jet bridges, any size), `S1`, `S2` (small, by the hangars) |
| `LVA` | Las Venturas Airport | `18/36` (630 m) | `C1`-`C4` (up to 30 m wings, at the terminal) |
| `VM` | Verdant Meadows | `09/27` (525 m, dirt, up to 30 m wings) | `V1`, `V2` (small) |

At LSIA the crossovers between the two runways are mostly walled off in the game, so
big planes go round the loop at the east end; small ones use the gaps east of the
middle (`X1`, `X2`). Verdant Meadows has hills to the west: planes land westbound
there and take off eastbound.

To use your own version of the stock file instead, put a `gta_sa.txt` in
`navmesh/airports/`: the built-in one is then not loaded.

## Writing an airports file

Lines of words; `#` starts a comment. Coordinates are GTA's, on the ground (the
runway or taxiway surface, not a plane's centre). Headings are GTA's: degrees, 0
north, turning left, so 90 is west and 270 east.

```
airport CODE Label words            starts (or goes on with) an airport
prefer HEADING                      planes land and take off this way when the runway allows
runway 09/27 X1 Y1 Z1 X2 Y2 Z2 [width M] [span M] [displaced END M] [noland END]
       [nodepart END] [closed]      a runway's centre line from its 09 end to its 27 end
node NAME X Y Z                     a point on a taxiway (or on a runway's centre line)
taxi NAME [span M] [oneway] N1 N2 N3 ...
                                    a taxiway through these nodes, both ways unless oneway
gate NAME X Y Z HEADING [span M] [lead NODE] [push NODE]
                                    where a parked plane's centre stands and which way it faces
approach END X Y Z [X Y Z ...]      fixes flown before the final to the END end
departure END X Y Z [X Y Z ...]     fixes flown after taking off from the END end
route NAME X Y Z [X Y Z ...]        a flight route (route+ adds points to one)
remove airport CODE | runway NAME | node NAME | taxi NAME | edge N1 N2 | gate NAME | route NAME
```

- **Runways.** The name is its two ends, the first end being where the line starts:
  planes taking off from the `09` end roll towards the `27` end. `width` is the paved
  width (30 m if left out), `span` the widest wings allowed on it. `displaced END M`
  moves that end's landing threshold M metres in (planes still use the whole runway
  to take off). `noland` and `nodepart` stop landings towards, or take-offs from, an
  end. A `closed` runway is a taxiway only: planes taxi along and across it with no
  clearance.
- **Nodes and taxiways.** A taxiway joins its nodes in order. `span` is the widest
  wings that clear everything along it; leave it out when anything fits. A node on a
  runway's centre line (within a quarter of its width) is on that runway: nodes on a
  runway are joined along it, the node before one is its hold short, and ATC clears
  a plane before it enters. A runway with no nodes on it gets them every 150 m.
- **Gates.** `lead` is the node a plane taxis in from, nose first; with `push`, it
  leaves backwards behind a tug to that node (usually the same one) and turns there.
  Without `push` it drives out forwards, so leave room in front. `span` is the widest
  wings that fit (anything, if left out). Planes pick the smallest free gate they
  fit.
- **Approach and departure fixes** are optional: without them a plane flies a
  pattern onto a straight final (longer for bigger planes) and climbs out straight
  ahead.
- **Routes** belong to no airport. Scripts fly them with `Nav_FlyRoute`, or name one
  in `Nav_Takeoff` (flown after departing) and `Nav_Land` (flown before the
  approach). `Nav_AddRoutePoint` builds them in a script.

Files load in name order, each on top of what came before. Given again, a runway,
node, gate or route replaces the old one, and a `taxi` line between two nodes that
are already joined replaces that stretch. So a later file can reshape an airport:

```
# navmesh/airports/my_server.txt
airport LSIA
runway 09L/27R 1400 -2492.5 12.55 2060 -2492.5 12.55 width 31 closed   # a taxiway now
remove gate P1
node B1 1628 -2523 12.55
...
```

A wrong line stops that file loading (nothing in it is used, and the log says which
line); the other files still load.

## How big each plane is

DiamondNav keeps each plane to the runways, taxiways and gates its wings fit.

| Model | Plane | Wingspan | Length | Runway it needs to take off / land |
| --- | --- | --- | --- | --- |
| 577 | AT-400 | 62.5 m | 70 m | 430 m / 380 m |
| 592 | Andromada | 60 m | 66 m | 410 m / 340 m |
| 553 | Nevada | 27 m | 22 m | 320 m / 260 m |
| 519 | Shamal | 19 m | 20 m | 270 m / 260 m |
| 520 | Hydra | 12 m | 17 m | 200 m / 250 m |
| 476 | Rustler | 12 m | 10 m | 180 m / 170 m |
| 511 | Beagle | 15 m | 11 m | 180 m / 160 m |
| 512, 513, 593, 460 | Cropduster, Stuntplane, Dodo, Skimmer | 10.5-14 m | 8-9.5 m | 140-160 m / 140 m |

The take-off figure includes the margin a departure wants; a landing also needs
room to touch down, so a runway somewhat longer than the landing figure is best.

## Air traffic control

- One plane on a runway at a time: landing, taking off or crossing it. Runways that
  cross count as one.
- Departures wait at the hold short while an arrival is due in; an arrival that
  will find its runway busy holds (circles), and goes around if the runway is still
  not clear at the decision point, or if it is not lined up on the glide path.
- On the ground a plane stops short of other planes (by their wings and fuselages)
  and of vehicles on its way, and waits before a stretch of taxiway another plane is
  about to come along the other way. A car standing on a runway closes it.
- `Nav_SetRunwayOpen` closes a runway (or opens it again); `Nav_GetRunwayUser` says
  who is on it.
