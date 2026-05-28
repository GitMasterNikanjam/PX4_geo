# PX4 Geo Library

A lightweight geodesic library for precise position calculations on the Earth's surface using a spherical Earth model.  
Originally developed for the PX4 autopilot, it provides essential functions for waypoint navigation, map projections, and cross‑track error computation.

## Features

- Great‑circle distance and bearing between two geographic points
- Waypoint creation from line segments or heading + distance
- Local north/east vectors (exact and fast small‑angle approximations)
- Cross‑track error for line segments and circular arcs
- 3D distance (horizontal + vertical) between WGS‑84 points
- Map projection between geographic (lat/lon) and local azimuthal equidistant plane (north/east)

All angles are in radians or degrees as documented; distances are in meters.

## Dependencies

- `mathlib` – basic math utilities (angle conversion, wrap functions)
- `matrix` – vector/matrix operations (for `Vector2f`)

Both are part of the PX4 firmware. For standalone use, you can replace them with standard `<cmath>` and a simple 2‑vector struct.

## API Overview

### Core Functions

| Function | Description |
|----------|-------------|
| `get_distance_to_next_waypoint()` | Great‑circle distance between two points. |
| `get_bearing_to_next_waypoint()` | Initial bearing (forward azimuth) from one point to another. |
| `waypoint_from_heading_and_distance()` | Compute new point given start, bearing, and distance. |
| `get_vector_to_next_waypoint()` | North/east components of the vector between two points. |
| `add_vector_to_global_position()` | Add local north/east offset to a geographic position. |

### Cross‑track Error

Cross‑track error is the perpendicular distance from a vehicle’s current position to a desired path (line or arc).  
It is positive when the vehicle is to the **right** of the path (relative to the direction of travel) and negative when to the left.

#### Line Segment (`get_distance_to_line`)

Given a line from A to B, the function returns:
- `distance` – signed distance to the closest point on the line.
- `past_end` – `true` if the closest point lies beyond B.
- `bearing` – bearing from the current position to that closest point.

The algorithm first checks if the position is past the end by comparing bearings. If not, it uses the sine of the bearing difference to compute the perpendicular distance.

#### Circular Arc (`get_distance_to_arc`)

For an arc defined by:
- center point (`lat_center`, `lon_center`)
- radius (meters)
- start bearing (from center to arc start, radians)
- sweep angle (positive = counter‑clockwise)

The function determines whether the current position lies within the angular sector of the arc:
- **Inside sector** – distance to the arc is `|radius - distance_to_center|` (outside) or `radius - distance_to_center` (inside the circle).
- **Outside sector** – distance is the shorter of the distances to the arc’s endpoints.

The result includes signed distance, bearing to the closest point, and a `past_end` flag when the nearest point is at an endpoint.

### Map Projection (`MapProjection`)

Implements the azimuthal equidistant projection.  
Initialize with a reference point; then `project(lat, lon)` gives local north/east coordinates (meters), and `reproject(x, y)` returns geographic coordinates.

The projection is accurate for short distances (error < 1% up to 2000 km) and maintains true distances and directions from the reference point.

## Example Usage

```cpp
#include "geo.h"

// Waypoint navigation
double current_lat = 47.398, current_lon = 8.545;
double wp_lat = 47.400, wp_lon = 8.550;

float dist = get_distance_to_next_waypoint(current_lat, current_lon, wp_lat, wp_lon);
float bearing = get_bearing_to_next_waypoint(current_lat, current_lon, wp_lat, wp_lon);

// Cross‑track error to a line from A to B
double latA = 47.398, lonA = 8.545;
double latB = 47.402, lonB = 8.555;
crosstrack_error_s cte;
int ret = get_distance_to_line(cte, current_lat, current_lon, latA, lonA, latB, lonB);
if (ret == 0 && !cte.past_end) {
    printf("Cross‑track distance: %.2f m\n", cte.distance);
}

// Map projection
MapProjection proj(47.398, 8.545); // reference point
float north, east;
proj.project(current_lat, current_lon, north, east);
printf("Local position: N=%.1f m, E=%.1f m\n", north, east);
```

## Building

When used inside PX4, simply add `geo.cpp` to your module’s source list.  
For standalone projects, copy `geo.h` and `geo.cpp` and provide compatible `mathlib` and `matrix` headers (or replace with standard math functions).

## References

- [Movable Type – Calculate distance, bearing and more between Latitude/Longitude points](http://www.movable-type.co.uk/scripts/latlong.html)
- [Azimuthal Equidistant Projection – Wolfram MathWorld](http://mathworld.wolfram.com/AzimuthalEquidistantProjection.html)

## License

BSD 3‑Clause – see copyright headers in source files.

## Authors

Thomas Gubler, Julian Oes, Lorenz Meier, Anton Babushkin, Doug Weibel – PX4 Development Team.
```