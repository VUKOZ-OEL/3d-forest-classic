

#ifndef HULL_H_INCLUDED
#define HULL_H_INCLUDED

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include "cloud.h"
#include <pcl/PolygonMesh.h>

//! Class for convex hull cumputing.
/*! Class for computinh convex polygon and holding its attributes. */
class ConvexHull
{
protected:
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_convexhull;    /**< cloud representing convex hull */
    float m_polygonArea;    /**< area of convex hull */
    pcl::PolygonMesh *m_mesh;

public:
    //! Constructor.
    /*! Costructor of ConvexHull \param pcl point cloud \param zh */
    ConvexHull(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    //! Destructor.
    /*! Destructor of ConvexHull. */
    ~ConvexHull();
    //! Get polygon.
    /*! Get cloud cloud containing points in convex polygon \return pointer to cloud */
    pcl::PointCloud<pcl::PointXYZI>::Ptr getPolygon();
    //! Get polygon area.
    /*! Get convex polygon area in square meters \return float */
    float getPolygonArea();

private:
    //! Compute Attributes.
    /*! Compute convex hull and area */
    void computeAttributes(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,float zHeight);
    void pclCloudToVTKPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,vtkSmartPointer<vtkPoints> pointsVTK,float newZcoord);
    void makeConvexHullDelaunay2D (vtkSmartPointer<vtkPoints> input,pcl::PointCloud<pcl::PointXYZI>::Ptr outputVertices);
    void toPolygonOrderWithTestToConvexness(pcl::PointCloud<pcl::PointXYZI>::Ptr vertices,pcl::PointCloud<pcl::PointXYZI>::Ptr polygon);
    pcl::PointXYZI returnPointLowestYZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void findSecondpointInPolygon(pcl::PointCloud<pcl::PointXYZI>::Ptr vertices,pcl::PointCloud<pcl::PointXYZI>::Ptr polygon);
    void getBackIntensityFromOrigCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr originalCloud);

};

//! Class for concave hull cumputing.
/*! Class for computinh concave polygon and holding its attributes. */
class ConcaveHull : public Cloud
{
protected:
    Cloud *m_concavehull;       /**< cloud representing concave hull */
    float m_polygonArea;        /**< area of concave hull */
    float m_searchingDistance;  /**<Start searching distance to find edge breaking point */

public:
    //! Constructor.
    /*! Costructor of ConcaveHull \param name \param pcl pointcloud \param float */
    ConcaveHull(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name, float searchDist=1.0);
    //! Destructor.
    /*! Destructor of ConcaveHull. */
    ~ConcaveHull();
    //! Get polygon.
    /*! Get cloud cloud containing points in convex polygon \return pointer to cloud */
    Cloud getPolygon();
    //! Get polygon area.
    /*! Get concave polygon area in square meters \return float */
    float getPolygonArea();
    //! Get polygon swapped ZI.
    /*! Get cloud cloud containing points in concave polygon with swapped z and intensity values \return pointer to cloud */
    Cloud getPolygonSwappedZI();
    //! Get triangle at.
    /*! Get get triangle at iterator \return pcl::point cloud */
    pcl::PointCloud<pcl::PointXYZI>::Ptr getTriangleAt(int i);
    //! Get triangulated polygon.
    /*! Get pointer to triangulated polygon \return TriangulatedPolygon */

private:
    //! Compute Attributes.
    /*! Compute concave hull and area */
    void computeAttributes();
    //! Compute concave hull.
    /*! Compute concave hull */
    void computeConcaveHull();
    //! Edges breaking.
    /*! Walk areound polygon and break edges. \param pcl poin cloud \param pcl poin cloud \param float */
    void edgesBreaking (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr hullCloud,float maxEdgeLenght);
    //! Return edge breaking point.
    /*! Find best point to break edge, if any point suit for conditions. \param pcl poin cloud \param pcl point \param pcl point \param float \return pcl point */
    pcl::PointXYZI returnEdgeBreakingPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,pcl::PointXYZI boda, pcl::PointXYZI bodb,float maxEdgeLenght);

};

#endif // HULL_H_INCLUDED
