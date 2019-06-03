
#ifndef EXTERNALPOINTSBYSECTION_H_INCLUDED
#define EXTERNALPOINTSBYSECTION_H_INCLUDED

#include "geomcalculations.h"
#include "hull.h"


class ExternalPointsBySections
{
protected:
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> m_sections;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> m_sectionsExternalPts;
    std::vector<float> m_sectionsArea;
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_externalPtsAll;
    float m_sectionHeight;
    float m_concaveHullThresholdDist;

public:
    ExternalPointsBySections(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float sectionHeight, float concaveHullThreshlodDist);
    pcl::PointCloud<pcl::PointXYZI>::Ptr getSectionAt(int i);
    pcl::PointCloud<pcl::PointXYZI>::Ptr getSectionExternalPtsAt(int i);
    pcl::PointCloud<pcl::PointXYZI>::Ptr getExternalPtsAll();
    int getSectionsExternalPtsSize();
    float getAreaSectionAt(int i);
    float getSectionHeight();
    float getConcaveHullThresholdDist();

protected:
    void computeExternalPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float sectionHeight);
    void fillSectionCloud (pcl::PointCloud<pcl::PointXYZI>::Ptr crownBaseCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr sectionCloud,float sectionHeight, float maxZvalue);
    void lastPointZEqualToFirst();
};


#endif // EXTERNALPOINTSBYSECTION_H_INCLUDED
