
#ifndef CROWN_H_INCLUDED
#define CROWN_H_INCLUDED

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include "cloud.h"
#include "hull.h"
#include "geomcalculations.h"
#include "alphaShapes.h"
#include "externalPointsBySection.h"



    //! Class for crown representation.
    /*! Class for holding information about tree crown with all parameters. */
class Crown : public Cloud
{
protected:
    Cloud *m_crownCloud;                                    /**< cloud representing voxelized crown */
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_externalPoints;  /**< cloud for points representing crown surface */
    pcl::PointXYZI m_poseCrown;                             /**< crown position */
    pcl::PointXYZI m_treePosition;                          /**< tree position for computing crown pos dev and bottom height*/
    ConvexHull *m_convexhull;                               /**< crown convex planar projection */
    ConcaveHull *m_concavehull;                             /**< crown concave planar projection */
    PolyhedronFromSections *m_sectionsPolyhedron;           /**< crown polyhedron from sections*/
    ExternalPointsBySections *m_externalSections;           /**< crown external points*/
    ConvexHull3D *m_convexhull3D;                           /**< crown 3D convexhull*/
    float m_volumeSections;                                 /**< crown volume obatin from sections*/
    float m_volumeVoxels;                                   /**< crown volume obtain from voxels*/
    float m_crownHeight;                                    /**< crown height */
    float m_bottomHeight;                                   /**< crown bottom height */
    float m_crownTotalHeight;                               /**< crown z axis lenght */
    float m_crownLenghtXY;                                  /**< crown lenght in xy plane */
    float m_crownWidthXY;                                   /**< crown width in xy plane*/
    float m_positionDist;                                   /**< crown position distance from tree position */
    float m_positionAzimuth;                                /**< crown position azimuth from tree position*/
    float m_sectionHeight;                                  /**< height of section used for section volume */
    float m_thresholdDistance;                              /**< */

public:
    //! Crown constructor.
    /*! Parametric costructor of crown. \param cloud crown pointCloud \param name of the crown  \param tree position \param z value of stem highest point */
    Crown(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name, pcl::PointXYZI treePosition, pcl::PointXYZI stemHighestPoint);
    //! Destructor.
    /*! Destructor of Crown. */
    ~Crown();
    //! Compute crown volume by voxels.
    /*! Compute crown volume by voxels. \param voxel size */
    void computeVolumeByVoxels(float resolution);
    //! Compute surface triangulation.
    /*! Compute surface triangulation mesh and mesh area using external point cloud and greedy triangulation. */
    Cloud getCrownCloud();
    //! Return crown external points cloud.
    /*! Return crown external points cloud. \return reference to external points cloud */
    pcl::PointCloud<pcl::PointXYZI>::Ptr getExternalPoints();
    //! Return crown position.
    /*! Return crown position, point with position coordinates. \return pcl::point */
    pcl::PointXYZI getCrownPosition();
    //! Return crown height.
    /*! Return crown height in m. \return float */
    float getCrownHeight();
    //! Return crown bottom height.
    /*! Return crown bottom height in m. \return float */
    float getCrownBottomHeight();
    //! Return crown z axis lenght.
    /*! Return crown z axis lenght in m. \return float */
    float getCrownTotalHeight();
    //! Return volume by sections.
    /*! Return crown volume computed by section. \return float */
    float getVolumeSections();
    //! Return voluem by voxels.
    /*! Return crown volume computed by voxels. \return float */
    float getVolumeVoxels();
    //! Return distance.
    /*! Return distance between crown and tree position. \return float */
    float getPosDist();
    //! Return azimuth.
    /*! Return azimuth angle from tree position to crown position. \return float */
    float getAzimuth();
    //! Return surface mesh.
    /*! Return polygon mesh representing crown surface. \return pcl::PolygonMesh */
    float getCrownLenghtXY();
    //! Return crown width in xy plane.
    /*! Return crown width in xy plane in m. \return float */
    float getCrownWidthXY();
    //! Return crown convex hull.
    /*! Return crown convex planar projection. \return *ConvexHull */
    ConvexHull getCrownConvexhull ();
    //! Return crown concave hull.
    /*! Return crown concave planar projection. \return *ConcaveHull */
    ConcaveHull getCrownConcavehull ();
    void setCrownHeight(float height);
    void setCrownTotalHeight(float totalHeight);
    void setCrownBottomHeight(float bottomHeight);
    void setSectionHeight(float sectionHeight);
    void setThresholdDistanceForSectionsHull(float thresholdDistance);
    float getSectionHeight();
    float getThresholdDistanceForsectionsHull();
    void computeSectionsAttributes(pcl::PointXYZI treePosition);
    void recomputeHeightsAndPositionDev(pcl::PointXYZI newTreePosition);
    bool isConvexhull3DExist();
    bool isSectionsPolyhedronExist();

// NEHOTOVE
    PolyhedronFromSections getPolyhedronFromSections();
    void computeSurfaceAndVolumeBySections();
    void computeConvexhull3D(bool useFullCrownCloud);
    ConvexHull3D get3DConvexhull();

private:
    //! Compute crown position.
    /*! Compute crown position as average value of crown external points with z valua equal to tree position. \param pcl::point tree position */
    void computeCrownPosition();
    //! Compute crown position deviation.
    /*! Compute crown position distance and azimuth from tree position. \param pcl::point tree position */
    void computePosDeviation();
    //! Compute crown heights.
    /*! Compute crown height and crown bottom height. \param pcl::point tree position */
    void computeCrownHeights(pcl::PointXYZI treePosition, pcl::PointXYZI stemHighestPoint);
    //! Compute crown lenght and width in xy plane.
    /*! Compute crown lenght and width in xy plane. */
    void computeCrownXYLenghtAndWidth ();
    //! Compute crown width in xy plane.
    /*! Compute crown width in xy plane as parpendicular distance to a line defined by points. \param pcl::point cloud \param pcl::point \param pcl::point */
    void computeCrownWidth (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI &pointA, pcl::PointXYZI &pointB);
    //! Compute crown volume by sections.
    /*! Compute crown volume by sections. \param float section area in square m \param float section height in m */
    void computeVolumeBySections (float area);
    void addFirstAndLastSectionPointsAll(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

};

#endif // CROWN_H_INCLUDED
