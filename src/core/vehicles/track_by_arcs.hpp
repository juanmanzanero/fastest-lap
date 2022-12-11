
#include "src/core/foundation/fastest_lap_exception.h"

inline Track_by_arcs::Track_by_arcs()
: n_segments(0),
  total_length(0.0),
  _w(0.0)
{}

inline Track_by_arcs::Track_by_arcs(Xml_document& doc, const scalar scale, const bool closed) 
: n_segments(doc.get_root_element().get_children().size()), 
  starting_points(n_segments,{0.0,0.0,0.0}),
  starting_directions(n_segments,{1.0,0.0,0.0}),
  starting_arclength(n_segments,0.0),
  curvature(n_segments,0.0),
  total_length(0.0),
  _w(0.5*scale*doc.get_root_element().get_attribute("width",scalar()))
{
    std::vector<Xml_element> children = doc.get_root_element().get_children();

    assert(children.size() > 0);

    for (size_t i = 0; i < children.size()-1; ++i)
    {
        Xml_element& child = children[i];
        if ( child.get_name() == "straight" )
        {
            const scalar L           = scale*child.get_attribute("length",scalar());
            starting_points[i+1]     = starting_points[i] + starting_directions[i]*L;
            starting_directions[i+1] = starting_directions[i];
            starting_arclength[i+1]  = starting_arclength[i] + L;
            total_length += L;
        }
        else if ( child.get_name() == "corner" )
        {
            // Read data from XML node attributes
            const std::string direction = child.get_attribute("direction");
            const scalar R = scale*child.get_attribute("radius",scalar());
            const scalar theta = child.get_attribute("sweep",scalar());

            scalar k = 1.0/R;

            // Left hand corner: negative curvature
            if ( direction == "left" ) k = -k;
                    
            curvature[i] = k;
            const sVector3d n = {-starting_directions[i][1],starting_directions[i][0],0.0};
            const sVector3d center_to_start = -n/k;
            const sVector3d center = starting_points[i] - center_to_start;
                    
            const sVector3d d1 = center_to_start/norm(center_to_start);
            const sVector3d d2 = starting_directions[i]/norm(starting_directions[i]);
            const sVector3d center_to_end = d1*cos(theta) + d2*sin(theta);
                    
            starting_points[i+1] = center + R*center_to_end;
            starting_arclength[i+1] = starting_arclength[i] + R*theta;
            starting_directions[i+1] = d2*cos(theta) - d1*sin(theta);
            total_length += R*theta;
        }
        else
            throw fastest_lap_exception("Node not recognized");
    }

    // Add the last segment
    if ( children.back().get_name() == "straight" )
    {
        if ( closed )
        {
            // Do not read the input data, just connect the last two points
            const scalar L = norm(starting_points.back() - starting_points.front());
            total_length += L;
        }
        else
        {
            total_length += scale*children.back().get_attribute("length",scalar());
        }
    }
    else if ( children.back().get_name() == "corner" )
    {
        const std::string direction = children.back().get_attribute("direction");
        const scalar R = scale*children.back().get_attribute("radius",scalar());
        const scalar theta = children.back().get_attribute("sweep",scalar());

        // Trust the input, for now
        scalar k = 1.0/R;
                
        if ( direction == "left" ) k = -k;
        
        curvature.back() = k;
        total_length += R*theta;
    }

}


template<typename Timeseries_t>
inline Timeseries_t Track_by_arcs::pseudo_curvature2_at(const scalar s, const Timeseries_t& w, const Timeseries_t& dw, const Timeseries_t& d2w) const
{
    auto [_,drc,d2rc]   = position_at(s,w,dw,d2w);
    auto [__,drc0,d2rc0] = position_at(s,0.0,0.0,0.0);
    const auto drc_times_d2rc = drc0[0]*d2rc[1] - drc0[1]*d2rc[0];
    const auto drc_times_drc = dot(drc0,drc0);
    return (drc_times_d2rc*drc_times_d2rc / (drc_times_drc*drc_times_drc*drc_times_drc));
}


template<typename Timeseries_t>
inline std::tuple<Vector3d<Timeseries_t>,Vector3d<Timeseries_t>,Vector3d<Timeseries_t>> Track_by_arcs::position_at(const scalar s, const Timeseries_t& w, const Timeseries_t& dw, const Timeseries_t& d2w) const
{
    auto [r,dr,d2r,d3r] = at(s);
            
    return 
    {  
        r + w*Vector3d<Timeseries_t>(-dr[1],dr[0],0.0),
        dr + w*Vector3d<Timeseries_t>(-d2r[1],d2r[0],0.0) + dw*Vector3d<Timeseries_t>(-dr[1],dr[0],0.0),
        d2r + w*Vector3d<Timeseries_t>(-d3r[1],d3r[0],0.0) + 2.0*dw*Vector3d<Timeseries_t>(-d2r[1],d2r[0],0.0) + d2w*Vector3d<Timeseries_t>(-dr[1],dr[0],0.0)
    };
}


inline auto Track_by_arcs::operator()(const scalar s) const -> Frenet_frame
{
    auto [r,dr,d2r,d3r] = at(s);

    return Frenet_frame
    {
        .position = r,
        .euler_angles = {atan2(dr.y(), dr.x()), 0.0, 0.0},
        .deuler_angles_ds = {dot(sVector3d(-dr.y(), dr.x(), 0.0), d2r)/dot(dr,dr), 0.0, 0.0}
    };
}


inline std::tuple<sVector3d,sVector3d,sVector3d,sVector3d> Track_by_arcs::at(const scalar s) const
{
    if ( n_segments < 1 )
    {
        throw fastest_lap_exception("Track is empty, and it cannot be used.");
    }

    assert(s <= (total_length + 1.0e-10));
    
    // Find the appropriate segment
    size_t i = 0;
    while (i < (n_segments-1))
    {
        if ( s < starting_arclength[i+1] ) break;

        ++i;
    }
    
    // Define relative segment
    const scalar s0 = s - starting_arclength[i];
    
    if ( std::abs(curvature[i]) < 1.0e-10 )
    {
        return
        {
            starting_points[i] + s0*starting_directions[i],
            starting_directions[i],
            {0.0,0.0,0.0},
            {0.0,0.0,0.0} 
        };
    }
    else
    {
        const scalar theta = s0*std::abs(curvature[i]);
        const sVector3d n = sVector3d(-starting_directions[i][1],starting_directions[i][0],0.0)/curvature[i];
        const sVector3d center = starting_points[i] + n;
        const sVector3d d1 = -n/norm(n);
        const sVector3d d2 = starting_directions[i];
        const sVector3d center_to_end = (d1*cos(theta) + d2*sin(theta))/std::abs(curvature[i]);
        
        const sVector3d dr = d2*cos(theta) - d1*sin(theta);

        return 
        {
            center + center_to_end,
            dr,
            sVector3d(-dr[1],dr[0],0.0)*curvature[i],
            -dr*curvature[i]*curvature[i]
        };
    }
}
