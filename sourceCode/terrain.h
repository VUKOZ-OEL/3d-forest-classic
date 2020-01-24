#ifndef TERRAIN_H_INCLUDED
#define TERRAIN_H_INCLUDED

#include "cloud.h"
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <QtCore/QObject>


class OctreeTerrain : public QObject
{
  Q_OBJECT

public:
  OctreeTerrain();
  OctreeTerrain( Cloud input, float resolution);
  ~OctreeTerrain();

public slots:
  //sets
  void setResolution(float res);
  void setBaseCloud(Cloud input);
  void setVegetationName(QString name);
  void setTerrainName(QString name);
  void execute();
  void octree(float res, pcl::PointCloud<pcl::PointXYZI>::Ptr input,pcl::PointCloud<pcl::PointXYZI>::Ptr output_ground, pcl::PointCloud<pcl::PointXYZI>::Ptr output_vege);
  void sendData();
  void hotovo();

signals:
  void finished();
  void percentage(int);
  void sendingTerrain( Cloud *);
  void sendingVegetation( Cloud *);

private:
  float m_resolution;
  Cloud *m_baseCloud;
  Cloud *m_terrain;
  Cloud *m_vegetation;
};

class VoxelTerrain : public QObject
{
  Q_OBJECT
public:
    VoxelTerrain();
    ~VoxelTerrain();
public slots:
  void setResolution(float res);
  void setBaseCloud(Cloud input);
  void setVegetationName(QString name);
  void setTerrainName(QString name);
  void execute();
  void sendData();
  void hotovo();

signals:
  void finished();
  void percentage(int);
  void sendingTerrain( Cloud *);
  void sendingVegetation( Cloud *);

private:
  float m_resolution;
  Cloud *m_baseCloud;
  Cloud *m_terrain;
  Cloud *m_vegetation;

};

class IDW : public QObject
{
   Q_OBJECT
public:
    IDW();
    ~IDW();
public slots:
  void setResolution(float res);
  void setPointNumber(float num);
  void setBaseCloud(Cloud input);
  void setOutputName(QString name);

  void execute();
  void sendData();
  void hotovo();

signals:
  void finished();
  void percentage(int);
  void sendingoutput( Cloud *);


private:
  float m_resolution;
  int m_pointsnum;
  Cloud *m_baseCloud;
  Cloud *m_output;
};

class StatOutlierRemoval : public QObject
{
  Q_OBJECT
public:
    StatOutlierRemoval();
    ~StatOutlierRemoval();

public slots:
  void setMeanDistance(float dist);
  void setNeighborhood(int num);
  void setBaseCloud(Cloud input);
  void setOutputName(QString name);

  void execute();
  void sendData();
  void hotovo();

signals:
  void finished();
  void percentage(int);
  void sendingoutput( Cloud *);

private:
  float m_mDist;
  int m_neighbors;
  Cloud *m_baseCloud;
  Cloud *m_output;
};

class RadiusOutlierRemoval : public QObject
{
  Q_OBJECT
public:
    RadiusOutlierRemoval();
    ~RadiusOutlierRemoval();
public slots:
    void setRadius(float radius);
    void setNeighborhood(int num);
    void setBaseCloud(Cloud input);
    void setOutputName(QString name);
    void setType(QString);

    void execute();
    void sendData();
    void hotovo();

signals:
  void finished();
  void percentage(int);
  void sendingoutput( Cloud *, QString);

private:
    float m_radius;
    int m_neighbors;
    Cloud *m_baseCloud;
    Cloud *m_output;
    QString m_type;
};

class Slope : public QObject
{
    Q_OBJECT
public:
    Slope();
    ~Slope();
    public slots:
    void setRadius(float radius);
    void setNeighbors(int i);
    void setTerrainCloud(Cloud input);
    void setOutputName(QString name);
    void useRadius(bool radius);
    
    void execute();
    void sendData();
    void hotovo();
    
signals:
    void finished();
    void percentage(int);
    void sendingoutput( Cloud *);
    
    
private:
    float computeSlope(pcl::PointXYZI& a, pcl::PointXYZI& b);
    float m_Radius;
    int m_Neighbors;
    bool m_useRadius = false;
    Cloud *m_TerrainCloud;
    Cloud *m_Output;
};

 class Curvature : public QObject
{
    Q_OBJECT
public:
    Curvature();
    ~Curvature();
    public slots:
    void setRadius(float radius);
    void setNeighbors(int i);
    void setTerrainCloud(Cloud input);
    void setOutputName(QString name);
    void useRadius(bool radius);
    
    void execute();
    void sendData();
    void hotovo();
    
signals:
    void finished();
    void percentage(int);
    void sendingoutput( Cloud *);
    
    
private:
    float computeSlope(pcl::PointXYZI& a, pcl::PointXYZI& b);
    float computeCurvature(std::vector<int> vec);
    float m_Radius;
    int m_Neighbors;
    bool m_useRadius = false;
    Cloud *m_TerrainCloud;
    Cloud *m_Output;
};


class HillShade : public QObject
{
    Q_OBJECT
public:
    HillShade();
    ~HillShade();
    public slots:
    void setRadius(float radius);
    void setNeighbors(int i);
    void setTerrainCloud(Cloud input);
    void setOutputName(QString name);
    void useRadius(bool radius);
    
    void execute();
    void sendData();
    void hotovo();
    
signals:
    void finished();
    void percentage(int);
    void sendingoutput( Cloud *);
    
    
private:
    float computeSlope(std::vector<float> vec);
    float computeSlope(std::vector<int> pointsId);
    std::vector<float> computeSmallestPCA (std::vector<int> pointsId);
    float computeAspect(std::vector<float> vec);
    float m_Radius;
    int m_Neighbors;
    bool m_useRadius = false;
    Cloud *m_TerrainCloud;
    Cloud *m_Output;
};
class TerrainFeatures : public QObject
{
    Q_OBJECT
public:
    TerrainFeatures();
    ~TerrainFeatures();
    public slots:
    void setRadius(float radius);
    void setNeighbors(int i);
    void setTerrainCloud(Cloud input);
    void setOutputName(QString name);
    void useRadius(bool radius);
    void setlowerPointLimit (float limit);
    void setupperPointLimit (float limit);
    void setMinBinaryLimit (float limit);
    void setMaxBinaryLimit (float limit);
    void setMinLenghtLimit (float limit);
    void setMaxLenghtLimit (float limit);
    void setMaxAreaLimit (float limit);
    void setMinAreaLimit (float limit);
    void setAxisRatioLimit (float limit);
    void setAreaRatioLimit (float limit);
    
    void execute();
    void sendData();
    void hotovo();
    
signals:
    void finished();
    void percentage(int);
    void sendingoutput( Cloud *);
    
    
private:
    bool computeStatistics(std::vector<float>& vec, float& avg, float& sd, float& range);
    float computeSlope(std::vector<float> vec);
    float computeSlope(std::vector<int> vec);
    float computeCurvature(Cloud* cloud ,std::vector<int> vec,float& xleng, float& yleng);
    std::vector<float> computeSmallestPCA (std::vector<int> pointsId);
    float computeAspect(std::vector<float> vec);
    float getAverageZ(std::vector<int> vec);
    std::vector<float> getAverage (std::vector<int> pointsId);
    float computeRadius(std::vector<float> a,std::vector<float> b,std::vector<float> c);
    float computeDistance (std::vector<float> a,std::vector<float> b);
    float getConvexHullArea(std::vector<int> pId);
    int orientation(pcl::PointXYZI p, pcl::PointXYZI q, pcl::PointXYZI r);
    void computeClusters();
    float distSq(pcl::PointXYZI p1, pcl::PointXYZI p2);
    std::vector<pcl::PointXYZI> convex_hull(std::vector<pcl::PointXYZI> p);
    int comp(pcl::PointXYZI point1, pcl::PointXYZI point2);
    float polygonArea(std::vector<pcl::PointXYZI>& p);
    
    bool computeLimits(Cloud *input, Cloud *output);
    bool computeBinary(Cloud *input, Cloud *output);
    bool findClusters(Cloud *input, std::vector< std::vector<int> >& output);
    bool filterClustersBySize(std::vector< std::vector<int> >& input, std::vector< std::vector<int> >& output);
    bool filterClustersByPCA(Cloud *inputCloud, std::vector< std::vector<int> >& input, std::vector< std::vector<int> >& output);
    bool filterClustersByHull(Cloud *inputCloud, std::vector< std::vector<int> >& input, std::vector< std::vector<int> >& output);
    bool computePCA (pcl::PointCloud<pcl::PointXYZI>::Ptr input, float& Xlenght, float& Ylenght);
    bool computeHulls(pcl::PointCloud<pcl::PointXYZI>::Ptr input, float& convexArea, float& concaveArea );
    bool createCloudsFromClusters(Cloud *inputCloud,std::vector< std::vector<int> >& input);
    void printValues();
    
    float m_Radius;
    int m_Neighbors;
    bool m_useRadius = false;
    Cloud *m_TerrainCloud;
    Cloud *m_OutputAVG;
    Cloud *m_OutputSD;
    Cloud *m_OutputRange;
    pcl::PointXYZI m_p0;
    
    std::vector <pcl::PointCloud<pcl::PointXYZI>::Ptr > m_stems;
    
    float m_upperLimit = 4;
    float m_lowerLimit = -4;
    
    int m_lowerSizeLimit = 30;
    int m_upperSizeLimit = 3000;
    
    float m_lowerSideLimit = 6;
    float m_upperSideLimit = 30;
    float m_axisRatioLimit = 0.75;

    float m_lowerAreaLimit = 30;
    float m_upperAreaLimit = 400;
    float m_areaRatioLimit = 0.75;
    
};

#endif // TERRAIN_H_INCLUDED
