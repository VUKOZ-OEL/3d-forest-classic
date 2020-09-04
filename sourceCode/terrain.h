#ifndef TERRAIN_H_INCLUDED
#define TERRAIN_H_INCLUDED

#include "cloud.h"
#include "hull.h"
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
  void sendingoutput( Cloud *);

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
    void setPercent(bool percent);
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
    float computeSlopeDegrees(pcl::PointXYZI& a, pcl::PointXYZI& b);
    float m_Radius;
    int m_Neighbors;
    bool m_useRadius = false;
    Cloud *m_TerrainCloud;
    Cloud *m_Output;
    bool m_percent=false;
};
class Aspect : public QObject
{
    Q_OBJECT
public:
    Aspect();
    ~Aspect();
    public slots:
    void setRadius(float radius);
    void setNeighbors(int i);
    void setTerrainCloud(Cloud input);
    void setOutputName(QString name);
    void useRadius(bool radius);
    void setSmer(bool smer);
    int computeDiretion(float angle);
    
    void execute();
    void sendData();
    void hotovo();
    
signals:
    void finished();
    void percentage(int);
    void sendingoutput( Cloud *);
    
    
private:
    std::vector<float> computeSmallestPCA (std::vector<int> pointsId);
    float computeAspect(std::vector<float> vec);
    float m_Radius;
    int m_Neighbors;
    bool m_useRadius = false;
    Cloud *m_TerrainCloud;
    Cloud *m_Output;
    bool m_smer = false;
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
class Features : public Cloud
{
public:
    Features (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name, QColor col);
      //! Constructor.
      /*! Costructor of tree. \param cloud Cloud */
    Features (Cloud cloud);
      //! Constructor.
      /*! Copy Costructor of tree. \param kopie tree copy */
    Features ();
    Features operator=(Features &kopie);
    ~Features();
    void setConvexArea(float a);
    void setConcaveArea(float a);
    void setXlenght(float len);
    void setYlengtht(float len);
    void computeCentroid();
    void setCentroid(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void setCentroid(pcl::PointXYZI p);
    void setMeanCurvature(float curv);
    void setPointNumber(int n);
    void setConvexHull();
    void setconcaveHull();
    ConvexHull& getConvexHull();
    ConcaveHull& getConcaveHull();
    
    float getConvexArea();
    float getConcaveArea();
    float getXlenght();
    float getYlenght();
    float getMeanCurvature();
    int getPointNumber();
    Cloud* getPointCloud();
    pcl::PointXYZI getCentroid();
    ConvexHull *m_convexhull;           /**< cloud of points representing convex hull of tree */
    ConcaveHull *m_concavehull;         /**< cloud of points representing concave hull of tree */
    pcl::PolygonMesh *m_triangulatedConcaveHull; /**< */
    
    
private:
    float m_convexArea;
    float m_concaveArea;
    float m_pcaXLength;
    float m_pcaYLength;
    int m_pointCount;
    pcl::PointXYZI m_centroid;
    float m_meanCurvature;
    
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
    void setSlopeCloud(Cloud input);
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
    void sendingoutput( Features *);
    void sendingoutputCloud( Cloud *);
    
    
private:
    bool computeStatistics(std::vector<float>& vec, float& avg, float& sd, float& range);
    float computeSlope(std::vector<float> vec);
    float computeSlope(std::vector<int> vec);
    float computeCurvature(Features vec,float& xleng, float& yleng);
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
    bool computeBinary(Cloud *input, float lowerLimit, float upperLimit, Cloud *output);
    bool findClusters(Cloud *input,float radius, int minClusterSize, std::vector<Features>& output);
    bool filterClustersBySize(std::vector<Features>& input, float lowerSizeLimit, float upperSizeLimit, std::vector<Features>& output);
    bool filterClustersByPCA(std::vector<Features>& input, float ratio, float lowerSideLimit, float upperSideLimit,bool ratioLess, std::vector<Features>& output);
    bool filterClustersByHull(std::vector<Features>& input, float ratio, float lowerAreaLimit, float upperAreaLimit,bool ratioLess, std::vector<Features>& output);
    bool computePCA (pcl::PointCloud<pcl::PointXYZI>::Ptr input, float& Xlenght, float& Ylenght);
    bool computeHulls(pcl::PointCloud<pcl::PointXYZI>::Ptr input, float& convexArea, float& concaveArea );
    bool createCloudsFromClusters(Cloud *inputCloud,std::vector<Features>& input);
    void printValues();
    void noiseFilter(Cloud *input, Cloud *output);
    void computeHoughTransform(Cloud *input, Cloud *output);
    void removeNonBoundary(Cloud *input, Cloud *output);
    void computeInsiders(Cloud *input,std::vector<Features>& output);
    void computeBundaries(Cloud *input,std::vector<Features>& output);
    void computeFeatures(std::vector<Features>& insiders,std::vector<Features>& boundary,std::vector<Features>& output);
    void computePointDensity(Cloud *input,float radius, float minValue, float maxValue, Cloud *output );
    
    float m_Radius;
    int m_Neighbors;
    bool m_useRadius = false;
    Cloud *m_TerrainCloud;
    Cloud *m_slopeCloud;
    Cloud *m_binaryCloud;
    Cloud *m_filteredCloud;
    Cloud *m_OutputRange;
    pcl::PointXYZI m_p0;
    
    std::vector <pcl::PointCloud<pcl::PointXYZI>::Ptr > m_stems;
    std::vector <Features > m_features;
    
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
class PointDensity : public QObject
{
    Q_OBJECT
    public:
        PointDensity();
        ~PointDensity();
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
        float computeDensityValue(float valuemin, float valuemax, std::vector<int> points);
        float m_Radius;
        int m_Neighbors;
        bool m_useRadius = false;
        Cloud *m_TerrainCloud;
        Cloud *m_Output;
};

#endif // TERRAIN_H_INCLUDED
