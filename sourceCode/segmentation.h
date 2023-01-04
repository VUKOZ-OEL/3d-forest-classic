#ifndef SEGMENTATION_H_INCLUDED
#define SEGMENTATION_H_INCLUDED

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/octree/octree_search.h>
#include "cloud.h"
#include <QtCore/QObject>


class Segmentation : public QObject
{
  Q_OBJECT

public:
  Segmentation();
  Segmentation( pcl::PointCloud<pcl::PointXYZI>::Ptr vegetation, pcl::PointCloud<pcl::PointXYZI>::Ptr terrain);
  ~Segmentation();

public slots:
  // sets
    void setDistance(float i);
    void setMinimalPoint(int i);
    void setVegetation(Cloud input);
    void setTerrain(Cloud input);
    void setRestCloudName(QString a);
    void setTreePrefix(QString a);
    void setMultiplicator(float multiplicator);
    void setTerrainDistance(float distaceFromTerrain);
    void setNumOfElements(int elements);
    void setResolution(float resolution);
    void setRange(float value);
    void setMethod(int method);
    int get_treeSize();
    int getNumTrees();
    void execute();
    void getData();
    void ready();

    void getRestcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr output);
    void getTree(int i, pcl::PointCloud<pcl::PointXYZI>::Ptr output);

signals:
    void started();
    void finished();
    void sendingTree( Cloud *);
    void sendingRest( Cloud *);
    void sendingCentr( Cloud *);
    void hotovo();
    void percentage(int);


protected:
   float m_resolution = 0.1;
    float m_multiplicator = 1;
    int m_numOfElements = 150;
    float m_terrainDistance = 1.9;
    int m_PCAValue = 70;
    float m_range =0.75;
    std::string m_prefix;
    int m_method =1;
    void computeRangePCAValue();
    void createOctree();
    void mergeClusters();
    void createComponents();
    void setTreeElements();
    bool findNeighborElements(float radius);
    bool findStructuralVoxels(float radius);
    bool findStructuralVoxelsTrees(float radius);
    int findAnyVoxelNeighborsTrees(float radius);
    bool mergeElements( std::vector<std::vector<int>>& vec);
    void sortVector(std::vector<int>& vec);
    void computeDescriptor();
    float computeSFFI(pcl::PointCloud<pcl::PointXYZI>::Ptr input);
    float computeIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointXYZI a);
    float computeSlope(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointXYZI a);
    float getNumUsedVoxels();
    void reconstructElement(std::vector<int> element);
    void reconstructElements();
    void reconstructVegetationRest();
    void setUsedVoxels();
    void setUsedVoxelsInTrees();
    void setOctreeVoxel();
    void setOctreeVegetation();
    void setOctreeTerrain();
    void cleanTreeBase();
    //void createOctree();

    Cloud *m_vegetation;
    Cloud *m_terrain;
    Cloud * m_restCloud;

    QString m_treeName;
    int percent;
    

    std::shared_ptr<Cloud> m_voxels;
    std::vector <std::vector<int> > m_clusters;
    std::vector< std::vector<int> > m_components;
    std::vector< std::vector<int> > m_elements;
    std::vector< std::vector<int> > m_trees;            // vector of voxels for each tree
    std::vector< std::vector<int> > m_connections;
    std::vector <pcl::PointCloud<pcl::PointXYZI>::Ptr > m_stems;  // clouds of segmented trees
    std::vector<int> m_usedVoxels;
    std::vector<int> m_usedVoxelsTrees;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> m_OctreeVoxel; //(m_resolution);
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> m_OctreeVegetation;//(m_resolution);
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> m_OctreeTerrain; //(m_resolution);

};

#endif // SEGMENTATION_H_INCLUDED
