/****************************************************************************
 *
 *   Copyright (C) 2012-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

 /**
 * @file geo.h
 * @brief Geodesic math functions for global positioning calculations.
 *
 * This file provides functions to compute distances, bearings, waypoint projections,
 * and cross‑track errors using the WGS‑84 ellipsoid model (approximated as a sphere).
 * It also defines a MapProjection class for converting between geographic coordinates
 * (latitude/longitude) and a local azimuthal equidistant plane.
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Doug Weibel <douglas.weibel@colorado.edu> (additional functions)
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#if defined(__has_include) && __has_include(<lib/mathlib/mathlib.h>)
#include <lib/mathlib/mathlib.h>
#elif defined(__has_include) && __has_include(<mathlib/mathlib.h>)
#include <mathlib/mathlib.h>
#else
// In the extracted PX4_mathlib repository this header is at the repository root.
#include <mathlib.h>
#endif

#if defined(__has_include) && __has_include(<lib/matrix/src/math.hpp>)
#include <lib/matrix/src/math.hpp>
#else
#include <src/math.hpp>
#endif

/** @brief Standard gravity acceleration [m/s^2] */
static constexpr float CONSTANTS_ONE_G = 9.80665f;						

/** @brief Mean Earth radius [m] (spherical approximation) */
static constexpr double CONSTANTS_RADIUS_OF_EARTH = 6371000;	

/** @brief Mean Earth radius [m] (float version) */
static constexpr float  CONSTANTS_RADIUS_OF_EARTH_F = CONSTANTS_RADIUS_OF_EARTH;

/** @brief Earth's spin rate [rad/s] */
static constexpr float CONSTANTS_EARTH_SPIN_RATE = 7.2921150e-5f;				


// XXX remove – temporary structure, subject to removal.
/**
 * @brief Cross‑track error information for a line or arc segment.
 */
struct crosstrack_error_s {
	bool past_end;		// Flag indicating we are past the end of the line/arc segment
	float distance;		// Distance in meters to closest point on line/arc
	float bearing;		// Bearing in radians to closest point on line/arc
} ;



float get_distance_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next);

/**
 * @brief Creates a new waypoint C on the line of two given waypoints (A, B) at certain distance
 * from waypoint A
 *
 * @param lat_A waypoint A latitude in degrees (47.1234567°, not 471234567°)
 * @param lon_A waypoint A longitude in degrees (8.1234567°, not 81234567°)
 * @param lat_B waypoint B latitude in degrees (47.1234567°, not 471234567°)
 * @param lon_B waypoint B longitude in degrees (8.1234567°, not 81234567°)
 * @param dist distance of target waypoint from waypoint A in meters (can be negative)
 * @param lat_target latitude of target waypoint C in degrees (47.1234567°, not 471234567°)
 * @param lon_target longitude of target waypoint C in degrees (47.1234567°, not 471234567°)
 */
void create_waypoint_from_line_and_dist(double lat_A, double lon_A, double lat_B, double lon_B, float dist,
					double *lat_target, double *lon_target);

/**
 * @brief Creates a waypoint from given waypoint, distance and bearing
 * see http://www.movable-type.co.uk/scripts/latlong.html
 *
 * @param lat_start latitude of starting waypoint in degrees (47.1234567°, not 471234567°)
 * @param lon_start longitude of starting waypoint in degrees (8.1234567°, not 81234567°)
 * @param bearing in rad
 * @param distance in meters
 * @param lat_target latitude of target waypoint in degrees (47.1234567°, not 471234567°)
 * @param lon_target longitude of target waypoint in degrees (47.1234567°, not 471234567°)
 */
void waypoint_from_heading_and_distance(double lat_start, double lon_start, float bearing, float dist,
					double *lat_target, double *lon_target);

/**
 * @brief Returns the bearing to the next waypoint in radians.
 *
 * @param lat_now current position in degrees (47.1234567°, not 471234567°)
 * @param lon_now current position in degrees (8.1234567°, not 81234567°)
 * @param lat_next next waypoint position in degrees (47.1234567°, not 471234567°)
 * @param lon_next next waypoint position in degrees (8.1234567°, not 81234567°)
 */
float get_bearing_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next);

/**
 * @brief Get the local north/east vector from current position to next waypoint.
 *
 * @param lat_now Current latitude [degrees].
 * @param lon_now Current longitude [degrees].
 * @param lat_next Next waypoint latitude [degrees].
 * @param lon_next Next waypoint longitude [degrees].
 * @param[out] v_n North component [m].
 * @param[out] v_e East component [m].
 */
void get_vector_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next, float *v_n,
				 float *v_e);

/**
 * @brief Fast (approximate) version of get_vector_to_next_waypoint().
 *
 * Uses small‑angle approximations. Suitable for short distances only.
 *
 * @param lat_now Current latitude [degrees].
 * @param lon_now Current longitude [degrees].
 * @param lat_next Next waypoint latitude [degrees].
 * @param lon_next Next waypoint longitude [degrees].
 * @param[out] v_n North component [m].
 * @param[out] v_e East component [m].
 */
void get_vector_to_next_waypoint_fast(double lat_now, double lon_now, double lat_next, double lon_next, float *v_n,
				      float *v_e);

/**
 * @brief Add a local north/east vector to a global position.
 *
 * @param lat_now Starting latitude [degrees].
 * @param lon_now Starting longitude [degrees].
 * @param v_n North offset [m].
 * @param v_e East offset [m].
 * @param[out] lat_res Result latitude [degrees].
 * @param[out] lon_res Result longitude [degrees].
 */
void add_vector_to_global_position(double lat_now, double lon_now, float v_n, float v_e, double *lat_res,
				   double *lon_res);

/**
 * @brief Compute the cross‑track error to a line segment.
 *
 * @param[out] crosstrack_error Structure containing error distance, bearing, and end flag.
 * @param lat_now Current latitude [degrees].
 * @param lon_now Current longitude [degrees].
 * @param lat_start Segment start latitude [degrees].
 * @param lon_start Segment start longitude [degrees].
 * @param lat_end Segment end latitude [degrees].
 * @param lon_end Segment end longitude [degrees].
 * @return 0 on success, -1 on error (e.g., invalid segment).
 */
int get_distance_to_line(struct crosstrack_error_s &crosstrack_error, double lat_now, double lon_now,
			 double lat_start, double lon_start, double lat_end, double lon_end);

/**
 * @brief Compute the cross‑track error to a circular arc.
 *
 * @param[out] crosstrack_error Pointer to structure to fill (may be nullptr if only the return value is needed).
 * @param lat_now Current latitude [degrees].
 * @param lon_now Current longitude [degrees].
 * @param lat_center Arc center latitude [degrees].
 * @param lon_center Arc center longitude [degrees].
 * @param radius Arc radius [m].
 * @param arc_start_bearing Bearing of the start of the arc from the center [rad].
 * @param arc_sweep Sweep angle of the arc [rad] (positive = counter‑clockwise).
 * @return 0 on success, -1 on error.
 */
int get_distance_to_arc(struct crosstrack_error_s *crosstrack_error, double lat_now, double lon_now,
			double lat_center, double lon_center,
			float radius, float arc_start_bearing, float arc_sweep);

/**
 * @brief Calculate global 3D distance between two WGS‑84 points.
 *
 * @param lat_now Current latitude [degrees].
 * @param lon_now Current longitude [degrees].
 * @param alt_now Current altitude [m] (AMSL).
 * @param lat_next Next point latitude [degrees].
 * @param lon_next Next point longitude [degrees].
 * @param alt_next Next point altitude [m] (AMSL).
 * @param[out] dist_xy Horizontal (great‑circle) distance [m] (may be nullptr).
 * @param[out] dist_z Vertical distance [m] (may be nullptr).
 * @return Total 3D distance [m].
 */
float get_distance_to_point_global_wgs84(double lat_now, double lon_now, float alt_now,
		double lat_next, double lon_next, float alt_next,
		float *dist_xy, float *dist_z);

/**
 * @brief Calculate distance between two points in local NED frame.
 *
 * @param x_now Current X (north) [m].
 * @param y_now Current Y (east) [m].
 * @param z_now Current Z (down) [m].
 * @param x_next Next point X (north) [m].
 * @param y_next Next point Y (east) [m].
 * @param z_next Next point Z (down) [m].
 * @param[out] dist_xy Horizontal (N‑E) distance [m] (may be nullptr).
 * @param[out] dist_z Vertical (down) distance [m] (may be nullptr).
 * @return Total 3D distance [m].
 */
float mavlink_wpm_distance_to_point_local(float x_now, float y_now, float z_now,
		float x_next, float y_next, float z_next,
		float *dist_xy, float *dist_z);

/**
 * @brief Map projection converting between geographic coordinates (lat/lon) and local azimuthal equidistant plane (north/east).
 *
 * The class maintains a reference point (lat0, lon0) and allows forward/inverse transformations.
 * The local plane is defined such that:
 * - North (x) axis points toward geographic north.
 * - East (y) axis points toward geographic east.
 * - Distances are preserved for short ranges (error < 1% for distances up to 2000 km).
 */
class MapProjection final
{
private:

	uint64_t _ref_timestamp{0};   ///< Timestamp of the reference position [μs].
	double _ref_lat{0.0};          ///< Reference latitude [rad].
	double _ref_lon{0.0};          ///< Reference longitude [rad].
	double _ref_sin_lat{0.0};      ///< Sine of reference latitude.
	double _ref_cos_lat{0.0};      ///< Cosine of reference latitude.
	bool _ref_init_done{false};    ///< True if reference has been initialized.

public:
	/**
	 * @brief Construct an uninitialized MapProjection object.
	 *
	 * Call initReference() before using the projection.
	 */
	MapProjection() = default;

	/**
	 * @brief Construct and immediately initialize the projection with a reference point.
	 *
	 * @param lat_0 Reference latitude [degrees].
	 * @param lon_0 Reference longitude [degrees].
	 * @param timestamp Timestamp of the reference position [μs] (default = 0).
	 */
	MapProjection(double lat_0, double lon_0, uint64_t timestamp = 0)
	{
		initReference(lat_0, lon_0, timestamp);
	}

	/**
	 * @brief Initialize (or reinitialize) the map projection with a new reference point.
	 *
	 * @param lat_0 Reference latitude [degrees].
	 * @param lon_0 Reference longitude [degrees].
	 * @param timestamp Timestamp of the reference position [μs] (default = 0).
	 */
	void initReference(double lat_0, double lon_0, uint64_t timestamp = 0);

	/**
	 * @brief Check whether the projection has been initialized.
	 * @return true if reference is set, false otherwise.
	 */
	bool isInitialized() const { return _ref_init_done; };

	/**
	 * @brief Get the timestamp of the reference position.
	 * @return Timestamp [μs].
	 */
	uint64_t getProjectionReferenceTimestamp() const { return _ref_timestamp; };

	/**
	 * @brief Get the reference latitude.
	 * @return Latitude [degrees].
	 */	
	double getProjectionReferenceLat() const { return math::degrees(_ref_lat); };

	/**
	 * @brief Get the reference longitude.
	 * @return Longitude [degrees].
	 */
	double getProjectionReferenceLon() const { return math::degrees(_ref_lon); };

	/**
	 * @brief Convert geographic coordinates to local north/east coordinates.
	 *
	 * @param lat Latitude [degrees].
	 * @param lon Longitude [degrees].
	 * @param[out] x North coordinate [m].
	 * @param[out] y East coordinate [m].
	 */
	void project(double lat, double lon, float &x, float &y) const;

	/**
	 * @brief Convert geographic coordinates to local north/east (vector version).
	 *
	 * @param lat Latitude [degrees].
	 * @param lon Longitude [degrees].
	 * @return 2D vector containing (north, east) [m].
	 */
	inline matrix::Vector2f project(double lat, double lon) const
	{
		matrix::Vector2f res;
		project(lat, lon, res(0), res(1));
		return res;
	}

	/**
	 * @brief Convert local north/east coordinates back to geographic coordinates.
	 *
	 * @param x North coordinate [m].
	 * @param y East coordinate [m].
	 * @param[out] lat Result latitude [degrees].
	 * @param[out] lon Result longitude [degrees].
	 */
	void reproject(float x, float y, double &lat, double &lon) const;
};
