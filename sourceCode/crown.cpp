#include "crown.h"
#include <pcl/filters/voxel_grid.h>


Crown::Crown (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name, pcl::PointXYZI treePosition, pcl::PointXYZI stemHighestPoint)
: Cloud(cloud, name)
{
    m_crownCloud = new Cloud(cloud, name);
    m_treePosition = treePosition;
    m_volumeVoxels = 0;
    m_volumeSections = 0;
    m_sectionHeight = 1;//m
    m_thresholdDistance = 1;//m
    m_sectionsPolyhedron = 0;
    m_convexhull3D = 0;

    m_convexhull = new ConvexHull(cloud);
    m_externalSections = new ExternalPointsBySections(CloudOperations::getCloudCopy(cloud),m_sectionHeight,m_thresholdDistance);
    computeCrownPosition();

    computeCrownHeights(treePosition,stemHighestPoint);
    computeCrownXYLenghtAndWidth();
}
Crown::~Crown()
{
    delete m_crownCloud;
    delete m_convexhull;
    delete m_concavehull;
}
// COMPUTE
void Crown::computeConvexhull3D(bool useFullCrownCloud)
{
    if(useFullCrownCloud == true || m_externalSections->getExternalPtsAll()->points.size()<500){
        m_convexhull3D = new ConvexHull3D(m_crownCloud->get_Cloud());
    }else{
         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(CloudOperations::getCloudCopy(m_externalSections->getExternalPtsAll()));
         addFirstAndLastSectionPointsAll(cloud);
         m_convexhull3D = new ConvexHull3D(cloud);
    }
}
void Crown::addFirstAndLastSectionPointsAll(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    if(m_sectionHeight == 1){
        pcl::PointCloud<pcl::PointXYZI>::Ptr c0(CloudOperations::getCloudCopy(m_externalSections->getSectionAt(0)));
        pcl::PointCloud<pcl::PointXYZI>::Ptr c1(CloudOperations::getCloudCopy(m_externalSections->getSectionAt(1)));
        pcl::PointCloud<pcl::PointXYZI>::Ptr c2(CloudOperations::getCloudCopy(m_externalSections->getSectionAt(m_externalSections->getSectionsExternalPtsSize()-2)));
        pcl::PointCloud<pcl::PointXYZI>::Ptr c3(CloudOperations::getCloudCopy(m_externalSections->getSectionAt(m_externalSections->getSectionsExternalPtsSize()-1)));
        *cloud += *c0;
        *cloud += *c1;
        *cloud += *c2;
        *cloud += *c3;
    }else{
        ExternalPointsBySections *ep = new ExternalPointsBySections(CloudOperations::getCloudCopy(m_crownCloud->get_Cloud()),1,1);
        pcl::PointCloud<pcl::PointXYZI>::Ptr c0(ep->getSectionAt(0));
        pcl::PointCloud<pcl::PointXYZI>::Ptr c1(ep->getSectionAt(1));
        pcl::PointCloud<pcl::PointXYZI>::Ptr c2(ep->getSectionAt(ep->getSectionsExternalPtsSize()-2));
        pcl::PointCloud<pcl::PointXYZI>::Ptr c3(ep->getSectionAt(ep->getSectionsExternalPtsSize()-1));
        *cloud += *c0;
        *cloud += *c1;
        *cloud += *c2;
        *cloud += *c3;
    }
}
void Crown::computeSectionsAttributes(pcl::PointXYZI treePosition)
{
    if(m_sectionHeight != 1 && m_thresholdDistance !=1)
    {
        m_externalSections = new ExternalPointsBySections(CloudOperations::getCloudCopy(m_crownCloud->get_Cloud()),m_sectionHeight,m_thresholdDistance);
        computeCrownPosition();
    }
    cloudHighestAndLowestZValue HL = GeomCalc::findHighestAndLowestPoints(m_crownCloud->get_Cloud());
    m_sectionsPolyhedron = new PolyhedronFromSections(HL.lowestPoint,HL.highestPoint);
    computeSurfaceAndVolumeBySections();
}
void Crown::computeSurfaceAndVolumeBySections()
{
    ExternalPointsBySections *ep = new ExternalPointsBySections(m_crownCloud->get_Cloud(),m_sectionHeight,m_thresholdDistance);

    for(int i=0; i<ep->getSectionsExternalPtsSize();i++)
    {
        computeVolumeBySections(ep->getAreaSectionAt(i));
        m_sectionsPolyhedron->addSectionCloud(ep->getSectionExternalPtsAt(i));
    }
    m_sectionsPolyhedron->computeSurfaceTriangulation();
}
void Crown::computeCrownHeights(pcl::PointXYZI treePosition, pcl::PointXYZI stemHighestPoint)
{
    //Find highest and lowest point in cloud
    cloudHighestAndLowestZValue HL = GeomCalc::findHighestAndLowestPoints(m_crownCloud->get_Cloud());
    //Compute and set heights
    m_crownTotalHeight = HL.Highest-HL.Lowest;
    m_bottomHeight = stemHighestPoint.z-treePosition.z;
    m_crownHeight = HL.Highest-stemHighestPoint.z;

}
void Crown::computeCrownXYLenghtAndWidth()
{
    //copy points from m_crownCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    cloud = m_crownCloud->get_Cloud();
    //initialize hull object and create convexhull
    ConvexHull *h = new ConvexHull(cloud);
    //find point with longest distance and set it as m_crownLenghtXY
    pointsWithLongestDist PT = GeomCalc::findPointsWithLongestDistance(h->getPolygon());
    m_crownLenghtXY = GeomCalc::computeDistance2Dxy(PT.pointA,PT.pointB);
    //compoute crown width from convex polygon
    computeCrownWidth(h->getPolygon(),PT.pointA,PT.pointB);
}
void Crown::computeCrownWidth(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI &pointA, pcl::PointXYZI &pointB)
{
    //find sequence number of points with longest distance
    float itA, itB;
    for (int i=0; i<cloud->points.size();i++)
    {
        pcl::PointXYZI pointI = cloud->points.at(i);
        if (pointI.x == pointA.x && pointI.y == pointA.y) {itA = i;}
        if (pointI.x == pointB.x && pointI.y == pointB.y) {itB = i;}
    }
    //divide polygon into two polygons by points with longest distance
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZI>);
    for (int i=0; i<cloud->points.size();i++)
    {
        pcl::PointXYZI point = cloud->points.at(i);
        if (i<itA || i>itB){cloudA->push_back(point);}
        if (i>itA && i<itB){cloudB->push_back(point);}
    }
    //find longest perpendicular distance for each polygon
    float distA = GeomCalc::findLongestPerpendicularDistance(cloudA,pointA,pointB);
    float distB = GeomCalc::findLongestPerpendicularDistance(cloudB,pointA,pointB);

    m_crownWidthXY = distA+distB;
}
void Crown::computeCrownPosition()
{
    //compute average XY coordinates as crown position, Z coordinates is equal to tree Z position
    float x =0;
    float y =0;
    float z =0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(m_externalSections->getExternalPtsAll());
    for (int i =0; i<cloud->points.size(); i++)
    {
        pcl::PointXYZI bod = cloud->points.at(i);
        x +=bod.x;
        y +=bod.y;
        z +=bod.z;
    }
    pcl::PointXYZI average;
    average.x =x/cloud->points.size();
    average.y =y/cloud->points.size();
    average.z =z/cloud->points.size();
    //set crown position
    m_poseCrown = average;
    //compute crown position deviance from tree position
    computePosDeviation();
}
void Crown::computePosDeviation()
{
    // Crown position
    pcl::PointXYZI crownP = m_poseCrown;
    // point for computing back vector (0,1) from tree position
    pcl::PointXYZI pointA = m_treePosition;
    pointA.y +=1;
    //compute direction and distance
    m_positionAzimuth = GeomCalc::computeClockwiseAngle(pointA,m_treePosition,crownP);
    m_positionDist = GeomCalc::computeDistance2Dxy(crownP,m_treePosition);
}
void Crown::computeVolumeBySections(float area)
{
    float m = m_volumeSections;
    float sliceVol = area*m_sectionHeight;
    m_volumeSections = sliceVol+m;
}
void Crown::computeVolumeByVoxels(float resolution)
{
    // Copy base crown cloud for voxelization
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    cloud = m_crownCloud->get_Cloud();

    // Voxelization
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> vox;
    vox.setInputCloud (cloud);
    vox.setLeafSize (resolution, resolution, resolution);
    vox.filter (*cloud_filtered);

    // Compute voxels volume
    int Voxels = cloud_filtered->points.size();
    float volume = Voxels * (resolution*resolution*resolution);
    // set voxels volume
    m_volumeVoxels = volume;
    // !nastavuje voxelizovaný cloud jako korunu!
    //m_crownCloud->set_Cloud(cloud_filtered);
}
void Crown::recomputeHeightsAndPositionDev(pcl::PointXYZI newTreePosition)
{
    float zdiff = m_treePosition.z - newTreePosition.z;
    m_bottomHeight +=zdiff;
    m_treePosition = newTreePosition;
    computePosDeviation();
}
// GET
Cloud Crown::getCrownCloud()
{
    return *m_crownCloud;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr Crown::getExternalPoints()
{
    return m_externalSections->getExternalPtsAll();
}
float Crown::getCrownHeight()
{
    float h = ceilf(m_crownHeight * 100) / 100; //zaokrouhleni
    return h;
}
float Crown::getCrownBottomHeight()
{
    float h = ceilf(m_bottomHeight * 100) / 100; //zaokrouhleni
    return h;
}
float Crown::getCrownTotalHeight()
{
    float h = ceilf(m_crownTotalHeight * 100) / 100; //zaokrouhleni
    return h;
}
float Crown::getVolumeSections()
{
    return m_volumeSections;
}
float Crown::getVolumeVoxels()
{
    return m_volumeVoxels;
}
pcl::PointXYZI Crown::getCrownPosition()
{
    return m_poseCrown;
}
float Crown::getAzimuth()
{
    return m_positionAzimuth;
}
float Crown::getPosDist()
{
    float d = ceilf(m_positionDist * 100) / 100; //zaokrouhleni
    return d;
}
float Crown::getCrownLenghtXY()
{
    float d = ceilf(m_crownLenghtXY * 100) / 100; //zaokrouhleni
    return d;
}
float Crown::getCrownWidthXY()
{
    float d = ceilf(m_crownWidthXY * 100) / 100; //zaokrouhleni
    return d;
}
ConvexHull Crown::getCrownConvexhull()
{
    return *m_convexhull;
}
ConcaveHull Crown::getCrownConcavehull()
{
    return *m_concavehull;
}
PolyhedronFromSections Crown::getPolyhedronFromSections()
{
    return *m_sectionsPolyhedron;
}
float Crown::getSectionHeight()
{
    return m_sectionHeight*100;
}
float Crown::getThresholdDistanceForsectionsHull()
{
    return m_thresholdDistance*100;
}
ConvexHull3D Crown::get3DConvexhull()
{
    return *m_convexhull3D;
}
bool Crown::isConvexhull3DExist()
{
    if(m_convexhull3D == 0){
        return false;
    }else return true;
}
bool Crown::isSectionsPolyhedronExist()
{
   if(m_sectionsPolyhedron == 0){
        return false;
    }else return true;
}
//SET
void Crown::setCrownHeight(float height)
{
    m_crownHeight = height;
}
void Crown::setCrownTotalHeight(float totalHeight)
{
    m_crownTotalHeight = totalHeight;
}
void Crown::setCrownBottomHeight(float bottomHeight)
{
    m_bottomHeight = bottomHeight;
}
void Crown::setSectionHeight(float sectionHeight)
{
    m_sectionHeight = sectionHeight;
}
void Crown::setThresholdDistanceForSectionsHull(float thresholdDistance)
{
    m_thresholdDistance = thresholdDistance;
}
















