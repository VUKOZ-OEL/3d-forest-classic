#include "externalPointsBySection.h"

// PUBLIC
ExternalPointsBySections::ExternalPointsBySections(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float sectionHeight, float concaveHullThresholdDist)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all(new pcl::PointCloud<pcl::PointXYZI>);
    m_externalPtsAll = cloud_all;
    m_concaveHullThresholdDist = concaveHullThresholdDist;
    computeExternalPoints(cloud,sectionHeight);
}
pcl::PointCloud<pcl::PointXYZI>::Ptr ExternalPointsBySections::getExternalPtsAll()
{
    return m_externalPtsAll;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr ExternalPointsBySections::getSectionAt(int i)
{
    return m_sections.at(i);
}
pcl::PointCloud<pcl::PointXYZI>::Ptr ExternalPointsBySections::getSectionExternalPtsAt(int i)
{
    return m_sectionsExternalPts.at(i);
}
float ExternalPointsBySections::getAreaSectionAt(int i)
{
    return m_sectionsArea.at(i);
}
int ExternalPointsBySections::getSectionsExternalPtsSize()
{
    return m_sectionsExternalPts.size();
}
float ExternalPointsBySections::getSectionHeight()
{
    return m_sectionHeight;
}
float ExternalPointsBySections::getConcaveHullThresholdDist()
{
    return m_concaveHullThresholdDist;
}
// PRIVATE
void ExternalPointsBySections::computeExternalPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float sectionHeight)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr externalPointsCloud (new pcl::PointCloud<pcl::PointXYZI>);
    //Find highest and lowest point
    cloudHighestAndLowestZValue HL = GeomCalc::findHighestAndLowestPoints(cloud);
    m_externalPtsAll->points.push_back(HL.highestPoint);
    m_externalPtsAll->points.push_back(HL.lowestPoint);
    //compute number of sections
    float height = HL.Highest-HL.Lowest;
    height = (height + 0.1);
    int loops = height/sectionHeight;
    //compute volume and set external points

    for (int i=0; i<loops; i++)
    {
        //Set section cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr sectionCloud (new pcl::PointCloud<pcl::PointXYZI>);
        fillSectionCloud(cloud,sectionCloud,sectionHeight,HL.Lowest);

        if(sectionCloud->points.size() > 3)
        {
            GeomCalc::cloudIntesityEqualToZCoordinate(sectionCloud);
            ConcaveHull ch(CloudOperations::getCloudCopy(sectionCloud),"name",m_concaveHullThresholdDist);
            m_sectionsExternalPts.push_back(ch.getPolygonSwappedZI().get_Cloud());
            m_sectionsArea.push_back(ch.getPolygonArea());
        }
        //set lower level of next section
        HL.Lowest +=sectionHeight;
    }
    // fill external points cloud all
    for (int i=0; i<m_sectionsExternalPts.size();i++)
    {
        for(int a=1;a<m_sectionsExternalPts.at(i)->points.size();a++)
        {
           m_externalPtsAll->points.push_back(m_sectionsExternalPts.at(i)->points.at(a));
        }
    }
}
void ExternalPointsBySections::fillSectionCloud (pcl::PointCloud<pcl::PointXYZI>::Ptr crownBaseCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr sectionCloud,float sectionHeight, float maxZvalue)
{
    for (int j =0; j<crownBaseCloud->points.size(); j++)
    {
    pcl::PointXYZI point =crownBaseCloud->points.at(j);
        if (point.z>maxZvalue && point.z<(maxZvalue + sectionHeight))
        {
            sectionCloud->points.push_back(point);
        }
    }
    m_sections.push_back(sectionCloud);
}
void ExternalPointsBySections::lastPointZEqualToFirst()
{

     pcl::PointCloud<pcl::PointXYZI>::Ptr section(m_sectionsExternalPts.at(m_sectionsExternalPts.size()-1));
  // pcl::PointXYZI p1 = section->points.at(0);
    int s = section->points.size();
//   pcl::PointXYZI p2 = section->points.at(s-1);
/*   // if(p1.z<p2.z && p1.z>p2.z)
    {
        section->points.at(0).z = p2.z;
          QString a = QString(" p1 %1  p2 %2").arg(p1.z).arg(p2.z);
          QMessageBox::information(0,("WARNING"),a);
    }
*/

}
