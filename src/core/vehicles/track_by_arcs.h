#ifndef TRACK_BY_ARCS_H
#define TRACK_BY_ARCS_H

#include "lion/io/Xml_document.h"
#include "lion/thirdparty/include/logger.hpp"
#include "lion/math/euler_angles.h"

//! A class to compute the characteristics of a track built by circumference arcs connected by straight lines
class Track_by_arcs
{
 public:

    //! Default constructor: not usable track
    Track_by_arcs();

    //! Constructor from Xml_document containing either <straight length="X"/> or <corner radius="X" sweep="Y" direction = "left/right"/> nodes
    //! @param[in] doc: Xml document with track geometry
    //! @param[in] scale: scalar to amplify or reduce the circuit
    //! @param[in] closed: true if track is closed
    Track_by_arcs(Xml_document& doc, const scalar scale, const bool closed);

    Track_by_arcs(Xml_document& doc, const bool closed) : Track_by_arcs(doc, 1.0, closed) {}

    //! operator(): Compute the position, velocity, and acceleration of a point in the centerline at a given arclength
    //! @param[in] s: arclength
    struct Frenet_frame
    {
        sVector3d position;
        Euler_angles<scalar> euler_angles;
        Euler_angles<scalar> deuler_angles_ds;
    };

    Frenet_frame operator()(const scalar s) const;

    //! Compute the position, velocity, acceleration and jerk of a point in the centerline at a given arclength
    //! @param[in] s: arclength
    std::tuple<sVector3d,sVector3d,sVector3d,sVector3d> at(const scalar s) const;


    scalar get_left_track_limit(scalar s) const { return _w; }

    scalar get_right_track_limit(scalar s) const { return _w; }

    //! Compute the position, velocity, and acceleration of a point at a given position
    //! @param[in] s: arclength
    //! @param[in] w: lateral position
    //! @param[in] dw: lateral position first derivative
    //! @param[in] d2w: lateral position second derivative
    template<typename Timeseries_t>
    std::tuple<Vector3d<Timeseries_t>,Vector3d<Timeseries_t>,Vector3d<Timeseries_t>> position_at(const scalar s, const Timeseries_t& w, const Timeseries_t& dw,
                                                          const Timeseries_t& d2w) const;

    //! Compute the pseudo curvature at a given point
    //! @param[in] s: arclength
    //! @param[in] w: lateral position
    //! @param[in] dw: lateral position first derivative
    //! @param[in] d2w: lateral position second derivative
    template<typename Timeseries_t>
    Timeseries_t pseudo_curvature2_at(const scalar s, const Timeseries_t& w, const Timeseries_t& dw, const Timeseries_t& d2w) const;

    //! Return the track total length
    const scalar& get_total_length() const { return total_length; }

    //! Check if the track has elevation (false for track by arcs)
    constexpr const bool has_elevation() const { return false; }

 private:

    size_t                  n_segments;             //! [c] The number of segments (1 segment = 1 straight or 1 corner)
    std::vector<sVector3d>  starting_points;        //! [c] The starting point for each segment
    std::vector<sVector3d>  starting_directions;    //! [c] The starting direction for each segment
    std::vector<scalar>     starting_arclength;     //! [c] The starting arclength for each segment
    std::vector<scalar>     curvature;              //! [c] The starting curvature for each segment
    scalar                  total_length;           //! [c] The track total length
    scalar                  _w;                     //! [c] Track limit distance
    
};

#include "track_by_arcs.hpp"

#endif
