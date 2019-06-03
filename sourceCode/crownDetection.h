#ifndef CROWNDETECTION_H_INCLUDED
#define CROWNDETECTION_H_INCLUDED

#include "geomcalculations.h"
#include "LeastSquareregression.h"

class CrownAutomaticDetection
{
protected:
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_crown;
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_stem;
    pcl::PointXYZI m_stemHighestPoint;
    float m_sectionHeight;
    float m_startHeightForDetailedSearch;
    std::vector<float> m_diff;


public:
    CrownAutomaticDetection(pcl::PointCloud<pcl::PointXYZI>::Ptr tree, stred dbhLSR,pcl::PointXYZI treePosition);
    pcl::PointCloud<pcl::PointXYZI>::Ptr getCrown();
    pcl::PointXYZI getStemHighestPoint();

private:
    void detectCrown(pcl::PointCloud<pcl::PointXYZI>::Ptr tree);
    pcl::PointXYZI predictNextStemCenter(stred A, stred B);
    void erasePointsUnder1_3m(pcl::PointCloud<pcl::PointXYZI>::Ptr tree, stred dbhLSR);
    stred getCenterByLSR(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    void findDilatation();
    void computeDiameters(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float cloudHighest, float cloudLowest);
    void separateStemPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float cloudLowest);

    float computeXAxisExtension(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    float computeYAxisExtension(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    stred getDiameterFrom(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float zmin, float zmax);

};



#endif // CROWNDETECTION_H_INCLUDED
