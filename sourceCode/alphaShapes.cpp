#include "alphaShapes.h"
#include <vtkInteractorStyle.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkDelaunay3D.h>
#include <vtkUnstructuredGrid.h>
#include <vtkSmartPointer.h>
#include <vtkCellArray.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyDataWriter.h>
#include <vtkPolyDataMapper.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <vtkMassProperties.h>
#include <vtkInteractorStyle.h>

#include <vtkIntersectionPolyDataFilter.h>
#include <vtkBooleanOperationPolyDataFilter.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <vtkSurfaceReconstructionFilter.h>

// 3D CONVEXHULL
ConvexHull3D::ConvexHull3D(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    m_surface = 0;
    m_volume = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr clo (new pcl::PointCloud<pcl::PointXYZI>);
   // m_cloudProjected = clo;
    createConvexHull(cloud);
}
void ConvexHull3D::createConvexHull(pcl::PointCloud<pcl::PointXYZI>::Ptr cl)
{
    // pcl cloud to VTK points
    vtkSmartPointer<vtkPoints> pointsVTK = vtkSmartPointer< vtkPoints >::New();
    for(int i=0; i<cl->points.size();i++)
    {
        pcl::PointXYZI p = cl->points.at(i);
        pointsVTK->InsertNextPoint(p.x, p.y, p.z);
    }
    //VTK Points to polydata
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(pointsVTK);
    //DELAUNAY
    vtkSmartPointer<vtkDelaunay3D> delaunay3D =vtkSmartPointer<vtkDelaunay3D>::New();
    delaunay3D->SetInputData(polydata);
    delaunay3D->SetAlpha(0);
    delaunay3D->Update();
    //Create surface from
    vtkSmartPointer<vtkDataSetSurfaceFilter> surfaceFilter = vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
    surfaceFilter->SetInputConnection(delaunay3D->GetOutputPort());
    surfaceFilter->Update();
    //To polydata
    vtkPolyData* polydataOut = surfaceFilter->GetOutput();
    //volume and surface
    vtkSmartPointer<vtkMassProperties> mp =  vtkSmartPointer<vtkMassProperties>::New();
    mp->SetInputData(polydataOut);
    m_surface = mp->GetSurfaceArea();
    m_volume = mp->GetVolume();
    //Convert to pclMesh
    pcl::PolygonMesh triangles;
    pcl::VTKUtils::vtk2mesh(polydataOut,triangles);
    m_mesh = triangles;
}
//GET
pcl::PolygonMesh ConvexHull3D::getMesh()
{
    return m_mesh;
}
float ConvexHull3D::getVolume()
{
    return m_volume;
}
float ConvexHull3D::getSurfaceArea()
{
    return m_surface;
}

//POLYHEDRON FROM SECTIONS
PolyhedronFromSections::PolyhedronFromSections(pcl::PointXYZI lowestPoint, pcl::PointXYZI highestPoint)
{
    m_surfaceArea = 0;
    m_highestPoint = highestPoint;
    m_lowestPoint = lowestPoint;
    m_mesh = new pcl::PolygonMesh;
    m_isSurfaceComputed = false;
}
void PolyhedronFromSections::addSectionCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr section)
{
    m_sections.push_back(section);
}
//GET
pcl::PointCloud<pcl::PointXYZI>::Ptr PolyhedronFromSections::getSectionsAt(int i)
{
    return m_sections.at(i);
}
float PolyhedronFromSections::getSectionsSize()
{
    return m_sections.size();
}
pcl::PointCloud<pcl::PointXYZI>::Ptr PolyhedronFromSections::getTriangleAt(int i)
{
    return m_triangles.at(i);
}
int PolyhedronFromSections::getTrianglesSize()
{
    return m_triangles.size();
}
//COMPUTE
void PolyhedronFromSections::addTriangle(pcl::PointXYZI A,pcl::PointXYZI B,pcl::PointXYZI C)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr triangle (new pcl::PointCloud<pcl::PointXYZI>);
    triangle->points.push_back(A);
    triangle->points.push_back(B);
    triangle->points.push_back(C);
    m_triangles.push_back(triangle);
}
void PolyhedronFromSections::addTriangle(Triangle t)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr triangle (new pcl::PointCloud<pcl::PointXYZI>);
    triangle->points.push_back(t.ptA);
    triangle->points.push_back(t.ptB);
    triangle->points.push_back(t.ptC);
    m_triangles.push_back(triangle);
}
void PolyhedronFromSections::computeSurfaceTriangulation()
{
    triangulateTopAndBottom();
    for(int i=1; i < m_sections.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr sectionA (CloudOperations::getCloudCopy(m_sections.at(i-1)));
        pcl::PointCloud<pcl::PointXYZI>::Ptr sectionB (CloudOperations::getCloudCopy(m_sections.at(i)));
        triangulateStrip(sectionA,sectionB);
    }
    computeSurfaceByTriangles();
    createMesh();
    m_isSurfaceComputed = true;
}
void PolyhedronFromSections::triangulateTopAndBottom()
{
    for(int i = 1; i<m_sections.at(0)->points.size();i++)
    {
        pcl::PointXYZI p = m_sections.at(0)->points.at(i-1);
        pcl::PointXYZI pp = m_sections.at(0)->points.at(i);
        addTriangle(m_lowestPoint,p,pp);
    }
    for(int i = 1; i<m_sections.at(m_sections.size()-1)->points.size();i++)
    {
        pcl::PointXYZI p = m_sections.at(m_sections.size()-1)->points.at(i-1);
        pcl::PointXYZI pp = m_sections.at(m_sections.size()-1)->points.at(i);
        addTriangle(m_highestPoint,p,pp);
    }
}
void PolyhedronFromSections::triangulateStrip(pcl::PointCloud<pcl::PointXYZI>::Ptr sectionA, pcl::PointCloud<pcl::PointXYZI>::Ptr sectionB)
{
        createOrderedShortestEdgesA(sectionA,sectionB);
        createOrderedShortestEdgesB(sectionB,sectionA);
        compareTotalEdgesLenght();
        m_edgesA.erase(m_edgesA.begin(),m_edgesA.end());
        m_edgesB.erase(m_edgesB.begin(),m_edgesB.end());
        sortSectionsFromNearestPoints(sectionA,sectionB);
        createOrderedShortestEdgesA(sectionA,sectionB);
        createOrderedShortestEdgesB(sectionB,sectionA);
        compareTotalEdgesLenghtPolygonFromShortestEdge();

        stripTriangulation(sectionA,sectionB);
        m_edges.erase(m_edges.begin(),m_edges.end());
        m_edgesA.erase(m_edgesA.begin(),m_edgesA.end());
        m_edgesB.erase(m_edgesB.begin(),m_edgesB.end());
}
void PolyhedronFromSections::compareTotalEdgesLenght()
{
    float A = 0;
    for(int i=0;i<m_edgesA.size();i++)
    {
        A +=GeomCalc::computeDistance3D(m_edgesA.at(i).ptA,m_edgesA.at(i).ptB);
    }
    float B = 0;
    for(int i=0;i<m_edgesB.size();i++)
    {
        B +=GeomCalc::computeDistance3D(m_edgesB.at(i).ptA,m_edgesB.at(i).ptB);
    }
//////////////////////////////////////////////////
//
    if(B<A){
        for(int i=0;i<m_edgesB.size();i++)
        {
            m_edges.push_back(m_edgesB.at(i));
        }
    }else{
        for(int i=0;i<m_edgesA.size();i++)
        {
            m_edges.push_back(m_edgesA.at(i));
        }
    }
}
void PolyhedronFromSections::compareTotalEdgesLenghtPolygonFromShortestEdge()
{
    float A = 0;
    for(int i=0;i<m_edgesA.size();i++)
    {
        A +=GeomCalc::computeDistance3D(m_edgesA.at(i).ptA,m_edgesA.at(i).ptB);
    }
    float B = 0;
    for(int i=0;i<m_edgesB.size();i++)
    {
        B +=GeomCalc::computeDistance3D(m_edgesB.at(i).ptA,m_edgesB.at(i).ptB);
    }
    float E = 0;
    for(int i=0;i<m_edges.size();i++)
    {
        E +=GeomCalc::computeDistance3D(m_edges.at(i).ptA,m_edges.at(i).ptB);
    }
//////////////////////////////////////////////////
//
    if(B<A && B<E){
        m_edges.erase(m_edges.begin(),m_edges.end());
        for(int i=0;i<m_edgesB.size();i++)
        {
            m_edges.push_back(m_edgesB.at(i));
        }
    }else if(A<B && A<E){
        m_edges.erase(m_edges.begin(),m_edges.end());
        for(int i=0;i<m_edgesA.size();i++)
        {
            m_edges.push_back(m_edgesA.at(i));
        }
    }
}
void PolyhedronFromSections::stripTriangulation(pcl::PointCloud<pcl::PointXYZI>::Ptr sectionA, pcl::PointCloud<pcl::PointXYZI>::Ptr sectionB)
{
    for(int i=1;i<m_edges.size();i++)
    {
        Edge e1 = m_edges.at(i-1);
        Edge e2 = m_edges.at(i);
        if(e1.ptrToA == e2.ptrToA){
            addTriangle(e1.ptA,e1.ptB,e2.ptB);
        }else if(e1.ptrToB == e2.ptrToB){
            addTriangle(e1.ptA,e1.ptB,e2.ptA);
        }else{
            if(GeomCalc::computeDistance3D(e1.ptA,e2.ptB)<GeomCalc::computeDistance3D(e1.ptB,e2.ptA)){
                addTriangle(e1.ptA,e1.ptB,e2.ptB);
                addTriangle(e1.ptA,e2.ptA,e2.ptB);
            }else{
                addTriangle(e1.ptA,e1.ptB,e2.ptA);
                addTriangle(e1.ptB,e2.ptA,e2.ptB);
            }
        }
    }
}
void PolyhedronFromSections::createOrderedShortestEdgesA(pcl::PointCloud<pcl::PointXYZI>::Ptr sectionA, pcl::PointCloud<pcl::PointXYZI>::Ptr sectionB)
{
    addEdgeA(sectionA,sectionB,0,0);
    int ptrToB = 0;
    for(int ptrToA=1;ptrToA<sectionA->points.size();ptrToA++)
    {
        pcl::PointXYZI A = sectionA->points.at(ptrToA);
        float dMax = 9999;
        for(int b=ptrToB;b<sectionB->points.size();b++)
        {
            pcl::PointXYZI B = sectionB->points.at(b);
            float d = GeomCalc::computeDistance2Dxy(A,B);
            if(d<dMax)
            {
                dMax = d;
                ptrToB = b;
            }
        }
        if(ptrToB > (m_edgesA.at(m_edgesA.size()-1).ptrToB +1))
        {
            addEdgeForSkippedPointsA(sectionA,sectionB,ptrToA,ptrToB);
        }
        addEdgeA(sectionA,sectionB,ptrToA,ptrToB);
    }
    if(ptrToB < sectionB->points.size()-1){
        addEdgeForSkippedPointsA(sectionA,sectionB,sectionA->points.size()-1,sectionB->points.size()-1);
    }
    addEdgeA(sectionA,sectionB,sectionA->points.size()-1,sectionB->points.size()-1);
}
void PolyhedronFromSections::createOrderedShortestEdgesB(pcl::PointCloud<pcl::PointXYZI>::Ptr sectionA, pcl::PointCloud<pcl::PointXYZI>::Ptr sectionB)
{
    addEdgeB(sectionA,sectionB,0,0);
    int ptrToB = 0;
    for(int ptrToA=1;ptrToA<sectionA->points.size();ptrToA++)
    {
        pcl::PointXYZI A = sectionA->points.at(ptrToA);
        float dMax = 9999;
        for(int b=ptrToB;b<sectionB->points.size();b++)
        {
            pcl::PointXYZI B = sectionB->points.at(b);
            float d = GeomCalc::computeDistance2Dxy(A,B);
            if(d<dMax)
            {
                dMax = d;
                ptrToB = b;
            }
        }
        if(ptrToB > (m_edgesB.at(m_edgesB.size()-1).ptrToB +1))
        {
            addEdgeForSkippedPointsB(sectionA,sectionB,ptrToA,ptrToB);
        }
        addEdgeB(sectionA,sectionB,ptrToA,ptrToB);
    }
    if(ptrToB < sectionB->points.size()-1){
        addEdgeForSkippedPointsB(sectionA,sectionB,sectionA->points.size()-1,sectionB->points.size()-1);
    }
    addEdgeB(sectionA,sectionB,sectionA->points.size()-1,sectionB->points.size()-1);
}
void PolyhedronFromSections::addEdgeForSkippedPointsA(pcl::PointCloud<pcl::PointXYZI>::Ptr sectionA, pcl::PointCloud<pcl::PointXYZI>::Ptr sectionB,int ptrToALast, int ptrToBLast)
{
        for(int i=m_edgesA.at(m_edgesA.size()-1).ptrToB+1;i < ptrToBLast;i++)
        {
            pcl::PointXYZI a1 = sectionA->points.at(ptrToALast-1);
            pcl::PointXYZI a2 = sectionA->points.at(ptrToALast);
            if(GeomCalc::computeDistance2Dxy(sectionB->points.at(i),a1)<GeomCalc::computeDistance2Dxy(sectionB->points.at(i),a2)){
                addEdgeA(sectionA,sectionB,ptrToALast-1,i);
            }else addEdgeA(sectionA,sectionB,ptrToALast,i);
        }
}
void PolyhedronFromSections::addEdgeForSkippedPointsB(pcl::PointCloud<pcl::PointXYZI>::Ptr sectionA, pcl::PointCloud<pcl::PointXYZI>::Ptr sectionB,int ptrToALast, int ptrToBLast)
{
        for(int i=m_edgesB.at(m_edgesB.size()-1).ptrToB+1;i < ptrToBLast;i++)
        {
            pcl::PointXYZI a1 = sectionA->points.at(ptrToALast-1);
            pcl::PointXYZI a2 = sectionA->points.at(ptrToALast);
            if(GeomCalc::computeDistance2Dxy(sectionB->points.at(i),a1)<GeomCalc::computeDistance2Dxy(sectionB->points.at(i),a2)){
                addEdgeB(sectionA,sectionB,ptrToALast-1,i);
            }else addEdgeB(sectionA,sectionB,ptrToALast,i);
        }
}
void PolyhedronFromSections::addEdgeA(pcl::PointCloud<pcl::PointXYZI>::Ptr sectionA, pcl::PointCloud<pcl::PointXYZI>::Ptr sectionB, int ptrToA, int ptrToB)
{
    Edge e;
    e.ptA=sectionA->points.at(ptrToA);
    e.ptB=sectionB->points.at(ptrToB);
    e.ptrToA = ptrToA;
    e.ptrToB = ptrToB;
    float vx = e.ptB.x - e.ptA.x;
    float vy = e.ptB.y - e.ptA.y;
    float vz = e.ptB.z - e.ptA.z;
    e.middlePt.x = e.ptA.x + (vx/2);
    e.middlePt.y = e.ptA.y + (vy/2);
    e.middlePt.z = e.ptA.z + (vz/2);
    m_edgesA.push_back(e);
}
void PolyhedronFromSections::addEdgeB(pcl::PointCloud<pcl::PointXYZI>::Ptr sectionA, pcl::PointCloud<pcl::PointXYZI>::Ptr sectionB, int ptrToA, int ptrToB)
{
    Edge e;
    e.ptA=sectionA->points.at(ptrToA);
    e.ptB=sectionB->points.at(ptrToB);
    e.ptrToA = ptrToA;
    e.ptrToB = ptrToB;
    float vx = e.ptB.x - e.ptA.x;
    float vy = e.ptB.y - e.ptA.y;
    float vz = e.ptB.z - e.ptA.z;
    e.middlePt.x = e.ptA.x + (vx/2);
    e.middlePt.y = e.ptA.y + (vy/2);
    e.middlePt.z = e.ptA.z + (vz/2);
    m_edgesB.push_back(e);
}
void PolyhedronFromSections::sortSectionsFromNearestPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr sectionA, pcl::PointCloud<pcl::PointXYZI>::Ptr sectionB)
{
    int ita = 0;
    int itb = 0;
    float distmin = 9999;
    for(int i=0; i<sectionA->points.size();i++)
    {
        pcl::PointXYZI ptA = sectionA->points.at(i);
        for(int j=0;j<sectionB->points.size();j++)
        {
            pcl::PointXYZI ptB = sectionB->points.at(j);
            float dist = GeomCalc::computeDistance3D(ptA,ptB);
            if(dist < distmin)
            {
                ita = i;
                itb = j;
                distmin = dist;
            }
        }
    }
    CloudOperations::sortPolygonFromIterator(sectionA,ita);
    CloudOperations::sortPolygonFromIterator(sectionB,itb);
}
void PolyhedronFromSections::computeSurfaceByTriangles()
{
    for(int i=0;i<m_triangles.size();i++)
    {
        pcl::PointXYZI a = m_triangles.at(i)->points.at(0);
        pcl::PointXYZI b = m_triangles.at(i)->points.at(1);
        pcl::PointXYZI c = m_triangles.at(i)->points.at(2);
        m_surfaceArea += GeomCalc::computeTriangleArea(a,b,c);
    }
}
void PolyhedronFromSections::createMesh()
{
  //  pcl::PointCloud<pcl::PointXYZI> cloudAll;
    pcl::PointCloud<pcl::PointXYZI> cloudAll;// (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PCLPointCloud2 cloudAll2;
    for(int i=0; i<m_triangles.size();i++)
    {
        pcl::PointXYZI a = m_triangles.at(i)->points.at(0);
        pcl::PointXYZI b = m_triangles.at(i)->points.at(1);
        pcl::PointXYZI c = m_triangles.at(i)->points.at(2);
        cloudAll.points.push_back(a);
        cloudAll.points.push_back(b);
        cloudAll.points.push_back(c);
    }
    pcl::toPCLPointCloud2(cloudAll,cloudAll2);
    pcl::PolygonMesh mesh;
    mesh.cloud = cloudAll2;

    for(int i=2; i<mesh.cloud.width; i++)
    {
        pcl::Vertices vertice;
        vertice.vertices.push_back(i-2);
        vertice.vertices.push_back(i-1);
        vertice.vertices.push_back(i);
        mesh.polygons.push_back(vertice);
        i +=2;
    }
  *m_mesh=mesh;/*

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
CloudOperations::cloudXYZItoXYZ(cloudAll,cloud);
// Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  // normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
 // cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (10);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);
    *m_mesh = triangles;
    QString ax = QString(" MESH %1").arg(m_mesh->polygons.size());
    QMessageBox::information(0,("WARNING"),ax);*/
}
float PolyhedronFromSections::getSurfaceArea()
{
    return m_surfaceArea;
}
pcl::PolygonMesh PolyhedronFromSections::getMesh()
{
    return *m_mesh;
}
//POLYHEDRON Intersections
PolyhedronIntersections3D::PolyhedronIntersections3D(pcl::PolygonMesh input1, pcl::PolygonMesh input2, QString name1, QString name2)
{
    m_sheredSpace = new pcl::PolygonMesh;
    m_surface = 0;
    m_volume = 0;
    m_name1 = name1;
    m_name2 = name2;

    computeIntersection(input1,input2);
}
pcl::PolygonMesh PolyhedronIntersections3D::getMesh()
{
    return *m_sheredSpace;
}
float PolyhedronIntersections3D::getVolume()
{
    return m_volume;
}
float PolyhedronIntersections3D::getSurface()
{
    return m_surface;
}
QString PolyhedronIntersections3D::getName1()
{
    return m_name1;
}
QString PolyhedronIntersections3D::getName2()
{
    return m_name2;
}
//COMPUTE
void PolyhedronIntersections3D::computeIntersection(pcl::PolygonMesh input1, pcl::PolygonMesh input2)
{
    //pcl to vtk
    vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> polydata2 = vtkSmartPointer<vtkPolyData>::New();
    pcl::VTKUtils::mesh2vtk(input1,polydata1);
    pcl::VTKUtils::mesh2vtk(input2,polydata2);
    //compute intersection
     vtkSmartPointer<vtkBooleanOperationPolyDataFilter> booleanOperation = vtkSmartPointer<vtkBooleanOperationPolyDataFilter>::New();
     booleanOperation->SetOperationToIntersection();
     booleanOperation->SetInputData( 0, polydata1 );
     booleanOperation->SetInputData( 1, polydata2 );
     booleanOperation->Update();
    //Create surface from
    vtkSmartPointer<vtkDataSetSurfaceFilter> surfaceFilter = vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
    surfaceFilter->SetInputConnection(booleanOperation->GetOutputPort());
    surfaceFilter->Update();
    //To polydata
    vtkPolyData* polydataOut = surfaceFilter->GetOutput();
    //volume and surface
    vtkSmartPointer<vtkMassProperties> mp =  vtkSmartPointer<vtkMassProperties>::New();
    mp->SetInputData(polydataOut);
    m_surface = mp->GetSurfaceArea();
    m_volume = mp->GetVolume();
    //Convert to pclMesh
    if(m_surface > 0 && m_volume > 0)
    {
        pcl::PolygonMesh triangles;
        pcl::VTKUtils::vtk2mesh(polydataOut,triangles);
        *m_sheredSpace = triangles;
    }
}

