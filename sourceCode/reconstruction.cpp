
#include "reconstruction.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/common/pca.h>
#include <pcl/octree/octree_search.h>
#include <pcl/search/octree.h>

//BOOOST
#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/array.hpp>



TreeReconstruction::TreeReconstruction()
{
  m_tree = new Cloud();
  m_centroids= new Cloud();
  m_stem = new Cloud();
  m_stem->set_name("stemCloud");

  m_minimal_pnt = 10;
  m_treeName = "centroids";
  m_percent = 0;
  m_cm = 0.05;
}
TreeReconstruction::TreeReconstruction(pcl::PointCloud<pcl::PointXYZI>::Ptr tree)
{
  m_tree = new Cloud();
  m_tree->set_Cloud(tree);
  m_centroids= new Cloud();
  m_stem = new Cloud();
  m_stem->set_name("stemCloud");

  m_minimal_pnt = 10;
  m_treeName = "centroids";
  m_percent = 0;
  m_cm = 0.05;
}
TreeReconstruction::~TreeReconstruction()
{
  delete m_tree;
  delete m_stem;
  delete m_centroids;
}
void TreeReconstruction::setDistance(float i)
{
  m_cm = i;
}
void TreeReconstruction::setMinimalPoint(int i)
{
  m_minimal_pnt = i;
}
void TreeReconstruction::setTree(pcl::PointCloud<pcl::PointXYZI>::Ptr tree)
{
  m_tree->set_Cloud( tree);
}
void TreeReconstruction::setCentroidsName(QString name)
{
  m_centroids->set_name(name);
}
int TreeReconstruction::euclSegmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr input, std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr > &clusters, int min_pt )
{
  if(input->points.size()==0)
    return -1;
  pcl::search::Octree<pcl::PointXYZI>::Ptr tree (new pcl::search::Octree<pcl::PointXYZI>(m_cm));
  tree->setInputCloud (input);

  std::vector<pcl::PointIndices> cluster_indices;
//  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
//
//  ec.setClusterTolerance (m_cm); // 5cm
//  ec.setMinClusterSize (min_pt );
//  ec.setMaxClusterSize (25000000);
//  ec.setSearchMethod (tree);
//  ec.setInputCloud (input);
//  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
        cloud_cluster->points.push_back (input->points[*pit]);
    }
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    clusters.push_back(cloud_cluster);
    clusters[j]->width = cloud_cluster->points.size ();
    clusters[j]->height = 1;
    clusters[j]->is_dense = true;
    j++;
  }
  //clusters = clu;
  return j;
}
void TreeReconstruction::cluster()
{
// vem m_tree a udelat segmenty podle zadaneho výškového profilu

  pcl::PointXYZI minp, maxp; // body ohranicujici vegetaci
  pcl::getMinMax3D(*m_tree->get_Cloud(),minp, maxp);

  float cut = minp.z;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> os (m_cm);
  os.setInputCloud(m_tree->get_Cloud());
  os.addPointsFromInputCloud();
  // pokud neni dosazeno vysky vegetace

  while(cut <= maxp.z)
  {
    std::vector <int> pID;
    std::vector< float > dist;
    Eigen::Vector3f minb (minp.x,minp.y, cut);
    Eigen::Vector3f maxb (maxp.x,maxp.y, cut+m_cm);
    pcl::PointCloud<pcl::PointXYZI>::Ptr segment (new pcl::PointCloud<pcl::PointXYZI>);

    if(os.boxSearch(minb,maxb,pID)> 0)
    {
      for(int j=0; j < pID.size(); j++)
      {
        pcl::PointXYZI bod = m_tree->get_Cloud()->points.at(pID.at(j));
        segment->points.push_back(bod);
      }
      segment->width = segment->points.size ();
      segment->height = 1;
      segment->is_dense = true;
      m_segments.push_back(segment);
    }
    cut+=m_cm;
  }
  for(int i=0; i < m_segments.size(); i++)
  {
    m_segments.at(i)->width = m_segments.at(i)->points.size ();
    m_segments.at(i)->height = 1;
    m_segments.at(i)->is_dense = true;
  }

// pro kazdy segment zjistit cluster
  for(int q=0; q < m_segments.size(); q++)
  {
    int a = euclSegmentation(m_segments.at(q), m_clusters, m_minimal_pnt);
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr centroidCloud (new pcl::PointCloud<pcl::PointXYZI>);// mracno s centroidy vsech clusterů
  for(int q = 0; q < m_clusters.size(); q++)
  {
    if(m_clusters.at(q)->points.size()> 0)
    {
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(	*m_clusters.at(q),	centroid );
      pcl::PointXYZI bod;
      bod.x =centroid[0];
      bod.y =centroid[1];
      bod.z =centroid[2];
      bod.intensity = centroid[3];

      centroidCloud->points.push_back(bod);
      m_usedCentroid.push_back(false);
    }
  }
  m_centroids->set_Cloud(centroidCloud);
  centroidCloud.reset();
}
void TreeReconstruction::setVertex()
{
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud (m_centroids->get_Cloud());
  for(int i = 0; i < m_centroids->get_Cloud()->points.size(); i++)
  {
    pcl::PointXYZI x = m_centroids->get_Cloud()->points.at(i);
    std::stringstream s;
    s << i;
    m_vertexNames.push_back(i);

    boost::add_vertex(s.str(), m_graph);
    std::vector<int> pointIDv;
    std::vector<float> pointSDv;

    int num = 5;
    float radius = m_cm*1.2;
    // search num nearest
    if( kdtree.radiusSearch(x,radius,pointIDv,pointSDv) > 3)
    {
      for(int j = 1; j < pointIDv.size(); j++)
      {
        float w = sqrt(pointSDv.at(j));
        boost::add_edge(i, pointIDv.at(j), w, m_graph);
      }
    }
    else
    {
      if( kdtree.nearestKSearch (x,num,pointIDv,pointSDv) > 0)
      {
        for(int j = 1; j < pointIDv.size(); j++)
        {
          float w = sqrt(pointSDv.at(j));
          boost::add_edge(i, pointIDv.at(j), w, m_graph);
        }
      }
    }
  }
}
void TreeReconstruction::stemSkeleton()
{
  m_indexMap = boost::get(boost::vertex_index, m_graph);
  m_predecessors.resize(boost::num_vertices(m_graph)); // To store parents
  m_distances.resize(boost::num_vertices(m_graph)); // To store distances
  PredecessorMap predecessorMap(&m_predecessors[0], m_indexMap);
  DistanceMap distanceMap(&m_distances[0], m_indexMap);

  boost::dijkstra_shortest_paths(m_graph, m_vertexNames.at(m_centroids->get_Cloud()->points.size()-1),
                                 boost::distance_map(distanceMap).predecessor_map(predecessorMap));

  m_predecessorMap = predecessorMap;
  m_distanceMap = distanceMap;

  PathType path;
  Vertex v = m_vertexNames.at(0);
  for(Vertex u = predecessorMap[v]; u != v;  v = u, u = predecessorMap[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
  {
    std::pair<mygraph::edge_descriptor, bool> edgePair = boost::edge(u, v, m_graph);
    mygraph::edge_descriptor edge = edgePair.first;
    path.push_back( edge );
  }

  int nod2;
  std::vector <int> nodes;

  //NameMap nameMap = boost::get(boost::vertex_name, g);
  for(PathType::reverse_iterator pathIterator = path.rbegin(); pathIterator != path.rend(); ++pathIterator)
  {
    nodes.push_back(source(*pathIterator,m_graph));
    nod2 = target(*pathIterator,m_graph);
  }
  nodes.push_back(nod2);

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud (m_centroids->get_Cloud());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_stem (new pcl::PointCloud<pcl::PointXYZI>);
  for(int a=0; a <nodes.size(); a++)
  {
    if(nodes.at(a) > m_centroids->get_Cloud()->points.size())
      continue;
    pcl::PointXYZI x = m_centroids->get_Cloud()->points.at(nodes.at(a));
    std::vector<int> pointIDv;
    std::vector<float> pointSDv;

    float radius = 0.08;
    // search num nearest
    if( kdtree.radiusSearch(x,radius,pointIDv,pointSDv) > 0)
    {
      for(int j = 0; j < pointIDv.size(); j++)
      {
        if(m_usedCentroid.at(pointIDv.at(j)) == false)
        {
          *cloud_stem += *m_clusters[pointIDv.at(j)];
          m_usedCentroid.at(pointIDv.at(j)) = true;
        }
      }
    }
  }
  cloud_stem->width = cloud_stem->points.size ();
  cloud_stem->height = 1;
  cloud_stem->is_dense = true;
  //*cloud_stem += *m_segments.at(source(*pathIterator,m_graph));

   m_stem->set_Cloud(cloud_stem );


}
void TreeReconstruction::execute()
{
  cluster();
  percentage(10);
  setVertex();
  percentage(30);
  stemSkeleton();
  percentage(80);
  sendData();
  // urcit patu stromu a oznacit jako zacatek vetve
  // urcit prvni valec
  //pridat dalsi
 //pokud jsou dva clustery rozdelit na hlavni(vetsi) a vetev
 //vetsi pripojit
 // mensi urcit jako zacatek vetve

}
void TreeReconstruction::sendData()
{
  QString a =QString("stemCloud_%1").arg(m_centroids->get_name());
  m_stem->set_name(a);
  sendingCentr( m_centroids);
  sendingTree( m_stem);
  finished();
}




