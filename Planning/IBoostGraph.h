//! Copyright 2016 Carrie Rebhuhn
#ifndef PLANNING_IBOOSTGRAPH_H_
#define PLANNING_IBOOSTGRAPH_H_


//! Interface for a class to use the astar planning
template <class G, class vertex_base, class vertex_hash,
class vertex_equal = std::equal_to<boost::graph_traits<G>::vertex_descriptor> >
class IBoostGraph {
 public:
    IBoostGraph() : pred_pmap(predecessor), dist_pmap(distance),
        weight(boost::static_property_map<double>(1)) {}
    typedef typename boost::graph_traits<G>::vertex_descriptor vertex_descriptor;

    //! Maps are vertex-to-vertex mapping.
    typedef typename boost::unordered_map<vertex_descriptor, vertex_descriptor,
        vertex_hash, vertex_equal> pred_map;
    typedef typename boost::unordered_map<vertex_descriptor, double,
        vertex_hash, vertex_equal> dist_map;
    virtual vertex_descriptor get_descriptor(vertex_base) = 0;
    virtual vertex_base get_vertex_base(vertex_descriptor) = 0;
    virtual double get_x(vertex_descriptor) = 0;
    virtual double get_y(vertex_descriptor) = 0;

    pred_map predecessor;
    boost::associative_property_map<pred_map> pred_pmap;
    dist_map distance;
    boost::associative_property_map<dist_map> dist_pmap;
    boost::static_property_map<double> weight;
};
#endif  // PLANNING_IBOOSTGRAPH_H_
