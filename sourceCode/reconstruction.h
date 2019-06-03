#ifndef RECONTRUCTION_H_INCLUDED
#define RECONTRUCTION_H_INCLUDED
#include "cloud.h"
#include <QtCore/QObject>

//BOOOST
#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
//#include <boost/graph/depth_first_search.hpp>

typedef float Weight;
typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;
typedef boost::property<boost::vertex_name_t, std::string> NameProperty;
typedef boost::adjacency_list< boost::listS, boost::vecS, boost::undirectedS, NameProperty, EdgeWeightProperty > mygraph;

typedef boost::property_map< mygraph, boost::vertex_index_t>::type VertexIndexMap;

typedef boost::graph_traits <mygraph >::edge_descriptor Edges;
typedef boost::graph_traits <mygraph >::vertex_descriptor Vertex;

typedef boost::property_map < mygraph, boost::vertex_index_t >::type IndexMap;
typedef boost::property_map < mygraph, boost::vertex_name_t >::type NameMap;

typedef boost::iterator_property_map < Vertex*, IndexMap, Vertex, Vertex& > PredecessorMap;
typedef boost::iterator_property_map < Weight*, IndexMap, Weight, Weight& > DistanceMap;
typedef std::vector<mygraph::edge_descriptor> PathType;


class TreeReconstruction : public QObject
{
  Q_OBJECT

public:
  TreeReconstruction();
  TreeReconstruction( pcl::PointCloud<pcl::PointXYZI>::Ptr tree);
  ~TreeReconstruction();
public slots:
  // sets
  void setDistance(float i);
  void setMinimalPoint(int i);
  void setTree(pcl::PointCloud<pcl::PointXYZI>::Ptr tree);
  void setCentroidsName(QString name);

  int euclSegmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr input,std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr > &clusters, int min_pt);
  void cluster();
  void stemSkeleton();
  void setVertex();
  void execute();

  //gets
  void sendData();

signals:
  void started();
  void finished();
  void sendingTree( Cloud *);
  void sendingCentr( Cloud *);
  void hotovo();
  void percentage(int);

private:
  Cloud *m_tree;
  Cloud *m_centroids;
  Cloud *m_stem;

  std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr > m_segments;
  std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr > m_clusters;
  std::vector<bool> m_usedCentroid;

  int m_minimal_pnt;
  QString m_treeName;
  int m_percent;
  float m_cm;

  mygraph m_graph;                                  /**< graph structure */
  std::vector<Vertex> m_vertexNames;
  IndexMap m_indexMap;
  PredecessorMap m_predecessorMap;
  DistanceMap m_distanceMap;

  std::vector<Vertex> m_predecessors; // To store parents
  std::vector<Weight> m_distances; // To store distances

};

#endif // RECONTRUCTION_H_INCLUDED
