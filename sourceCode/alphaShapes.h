
#ifndef ALPHASHAPES_H_INCLUDED
#define ALPHASHAPES_H_INCLUDED

#include "geomcalculations.h"
#include "hull.h"
#include <pcl/common/common_headers.h>
//#include <pcl_exports.h>


    //! Convex hull 3D.
    /*! Class for computing 3D convex hull and its parameters. */
class ConvexHull3D
{
protected:
    pcl::PolygonMesh m_mesh;    /**< Surface mesh */
    float m_surface;            /**< Mesh surface area */
    float m_volume;             /**<  Mesh volume */

public:
    //! ConvexHull3D
    /*! Class constructor \parm pcl point cloud. */
    ConvexHull3D(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    //! Get mesh
    /*! Return surface mesh. \return pcl PolygonMesh */
    pcl::PolygonMesh getMesh();
    //! Get volume
    /*! Return mesh volume. \return float */
    float getVolume();
    //! Get surface
    /*! Return mesh surface area. \return float */
    float getSurfaceArea();

private:
    //! Create convex hull
    /*! Create convex hull of given cloud \param pcl point cloud. */
    void createConvexHull(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
};

    //! Triangle
    /*! Structure for triangle and its parameters. */
struct Triangle{
pcl::PointXYZI ptA;
pcl::PointXYZI ptB;
pcl::PointXYZI ptC;
float area;
float sideRatio;
float perimeter;
float bcDist;
float smallestAngle;
};

    //! Edge.
    /*! Structure for edge and its parameters. */
struct Edge{
pcl::PointXYZI ptA;
pcl::PointXYZI ptB;
pcl::PointXYZI middlePt;
int ptrToB;
int ptrToA;
};

    //! polyhedron from sections.
    /*! Class for creating polyhedron from sections external points. */
class PolyhedronFromSections
{
protected:
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> m_triangles;  /**< Vector of resulting triangles */
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> m_sections;   /**< Vector of sections exgteranl pts */
    std::vector<Edge> m_edges;                                      /**< Vector of edges for strip triangulation */
    std::vector<Edge> m_edgesB;                                     /**< Vector of possible edges */
    std::vector<Edge> m_edgesA;                                     /**< Vector of possible edges */
    pcl::PointXYZI m_lowestPoint;                                   /**< Lowest point from external points*/
    pcl::PointXYZI m_highestPoint;                                  /**< Highest point from external points*/
    bool m_isSurfaceComputed;                                       /**< true if surface is compouted*/
    float m_surfaceArea;                                            /**< Surface area*/
    pcl::PolygonMesh *m_mesh;                                       /**< Surface mesh*/

public:
    //! Constructor.
    /*! Parametric constructor of class. \param pcl point  \param pcl point */
    PolyhedronFromSections(pcl::PointXYZI lowestPoint, pcl::PointXYZI highestPoint);
    //! Add section cloud.
    /*! Push back section cloud to m_sections. \param pcl point cloud */
    void addSectionCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr section);
    //! Get triangle at.
    /*! Return triangle at int. \param int \return pcl point cloud */
    pcl::PointCloud<pcl::PointXYZI>::Ptr getTriangleAt(int i);
    //! Get triangle size.
    /*! Return amount of triangles in vector. \return int */
    int getTrianglesSize ();
    //! Compute surface triangulation.
    /*! Compute surface triangulation and surface area. */
    void computeSurfaceTriangulation();
    //! Get sections size.
    /*! Get number of sections \return float */
    float getSectionsSize();
    //! Get section at.
    /*! Get section at int. \return pcl point cloud \param int */
    pcl::PointCloud<pcl::PointXYZI>::Ptr getSectionsAt(int i);
    //! Get mesh.
    /*! Get polygon mesh \return PolygonMesh */
    pcl::PolygonMesh getMesh();
    //! Get surface area.
    /*! Get area od triangulated surface \return float */
    float getSurfaceArea();

private:
    //! Compute surface by triangles.
    /*! Compute surface */
    void computeSurfaceByTriangles();
    //! Create mesh.
    /*! Create pcl::PolygonMesh from triangles */
    void createMesh();
    //! Triangulate top and bottom.
    /*! Triangulate space between lowest section and lowest point and between highest section and highest point */
    void triangulateTopAndBottom();
    //! Triangulate strip.
    /*! Triangulate strip created by two sections */
    void triangulateStrip(pcl::PointCloud<pcl::PointXYZI>::Ptr sectionA, pcl::PointCloud<pcl::PointXYZI>::Ptr sectionB);
    //! Add triangle.
    /*! Add triangle to vector */
    void addTriangle(pcl::PointXYZI A,pcl::PointXYZI B,pcl::PointXYZI C);
    //! Add triangle.
    /*! Add triangle to vector */
    void addTriangle(Triangle t);
    //! Create ordered shortest edges A.
    /*! Create ordered shortest edges from section A */
    void createOrderedShortestEdgesA(pcl::PointCloud<pcl::PointXYZI>::Ptr sectionA, pcl::PointCloud<pcl::PointXYZI>::Ptr sectionB);
    //! Create ordered shortest edges B.
    /*! Create ordered shortest edges from section B */
    void createOrderedShortestEdgesB(pcl::PointCloud<pcl::PointXYZI>::Ptr sectionA, pcl::PointCloud<pcl::PointXYZI>::Ptr sectionB);
    //! Compare total edges lenght.
    /*! Compare total edges lenght in vectors  */
    void compareTotalEdgesLenght();
    //! Sort sections from nearest points.
    /*! Find nearest points of two sections and organize polygon */
    void sortSectionsFromNearestPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr sectionA, pcl::PointCloud<pcl::PointXYZI>::Ptr sectionB);
    //! Compare total edges lenght polygon from shortest edge.
    /*! Compare total edges lenght created from polygons ordered from shorest edge  */
    void compareTotalEdgesLenghtPolygonFromShortestEdge();
    //! Strip triangulation.
    /*! Create triangles from edges */
    void stripTriangulation(pcl::PointCloud<pcl::PointXYZI>::Ptr sectionA, pcl::PointCloud<pcl::PointXYZI>::Ptr sectionB);
    //! Add edge A.
    /*! Add edge to vector containing edges A */
    void addEdgeA(pcl::PointCloud<pcl::PointXYZI>::Ptr sectionA, pcl::PointCloud<pcl::PointXYZI>::Ptr sectionB, int ptrToA, int ptrToB);
    //! Add edge B.
    /*! Add edge to vector containing edges B */
    void addEdgeB(pcl::PointCloud<pcl::PointXYZI>::Ptr sectionA, pcl::PointCloud<pcl::PointXYZI>::Ptr sectionB, int ptrToA, int ptrToB);
    //! Add edge for skipped points A.
    /*! Add edge for points from B section which wasn´t connected with any points from A section */
    void addEdgeForSkippedPointsA(pcl::PointCloud<pcl::PointXYZI>::Ptr sectionA, pcl::PointCloud<pcl::PointXYZI>::Ptr sectionB,int ptrToALast, int ptrToBLast);
    //! Add edge for skipped points B.
    /*! Add edge for points from A section which wasn´t connected with any points from B section */
    void addEdgeForSkippedPointsB(pcl::PointCloud<pcl::PointXYZI>::Ptr sectionA, pcl::PointCloud<pcl::PointXYZI>::Ptr sectionB,int ptrToALast, int ptrToBLast);
};

class PolyhedronIntersections3D
{
protected:
    pcl::PolygonMesh *m_sheredSpace;
    float m_surface;
    float m_volume;
    QString m_name1;
    QString m_name2;

public:
    PolyhedronIntersections3D(pcl::PolygonMesh input1, pcl::PolygonMesh input2, QString name1, QString name2);
    pcl::PolygonMesh getMesh();
    float getVolume();
    float getSurface();
    QString getName1();
    QString getName2();

private:
    void computeIntersection(pcl::PolygonMesh input1, pcl::PolygonMesh input2);
};
#endif // ALPHASHAPES_H_INCLUDED
