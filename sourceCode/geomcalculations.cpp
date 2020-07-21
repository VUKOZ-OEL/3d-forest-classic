#include "geomcalculations.h"
#include <stdlib.h>
#include "hull.h"

#include <vtkCellArray.h>
#include <vtkProperty.h>
#include <vtkPolyDataMapper.h>
#include <vtkPoints.h>
#include <vtkActor.h>
#include <vtkPolyData.h>
#include <vtkPolygon.h>
#include <vtkSmartPointer.h>
#include <vtkDelaunay2D.h>
#include <pcl/filters/voxel_grid.h>
#include <QtWidgets/QMessageBox>

// GEOMETRIC CALCULATIONS
float GeomCalc::computeDistance2Dxy(pcl::PointXYZI boda, pcl::PointXYZI bodb)
{
    float xv =boda.x-bodb.x;
    float yv =boda.y-bodb.y;
    float dist =sqrt(xv*xv+yv*yv);

    return dist;
}
float GeomCalc::computeDistance3D(pcl::PointXYZI boda, pcl::PointXYZI bodb)
{
float xv =boda.x-bodb.x;
float yv =boda.y-bodb.y;
float zv =boda.z-bodb.z;
float dist =sqrt(xv*xv+yv*yv+zv*zv);

return dist;
}
float GeomCalc::computeClockwiseAngle(pcl::PointXYZI pointA, pcl::PointXYZI pointB,pcl::PointXYZI pointC)
{
    //vectors
    float xnext = pointB.x - pointC.x;
    float ynext = pointB.y - pointC.y;
    float xback = pointB.x - pointA.x;
    float yback = pointB.y - pointA.y;
    //angle in degrees
    float A = (((atan2(xback*ynext-xnext*yback,xback*xnext+yback*ynext)))*180/3.14159265359);
            if (A > 0){A -= 360;}
            if (A < 0){A = A * (-1);}
            //if (A == 0){A = 180;}
    return A;
}
cloudHighestAndLowestZValue GeomCalc::findHighestAndLowestPoints (pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI)
{
    cloudHighestAndLowestZValue HL;
    HL.Highest = -9999;
    HL.Lowest = 9999;
    for(int i=0; i<cloudXYZI->points.size();i++)
       {
           if (cloudXYZI->points.at(i).z < HL.Lowest){
                HL.Lowest = cloudXYZI->points.at(i).z;
                HL.lowestPoint = cloudXYZI->points.at(i);
           }
           if (cloudXYZI->points.at(i).z > HL.Highest){
                HL.Highest = cloudXYZI->points.at(i).z;
                HL.highestPoint = cloudXYZI->points.at(i);
           }
       }
   return HL;
}
void GeomCalc::cloudIntesityEqualToZCoordinate(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{

    for(pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud->begin(); it != cloud->end(); it++)
       {
           it->intensity = it->z;
       }
}
pointsWithLongestDist GeomCalc::findPointsWithLongestDistance (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    pointsWithLongestDist PT;
    float maxDist = 0;
    for (int i=0; i<cloud->points.size(); i++)
    {
        pcl::PointXYZI pointI = cloud->points.at(i);
        for (int j=0; j<cloud->points.size(); j++)
        {
            pcl::PointXYZI pointJ = cloud->points.at(j);
            float Dist = GeomCalc::computeDistance2Dxy(pointI,pointJ);
            if (Dist > maxDist)
            {
                maxDist = Dist;
                PT.pointA = pointI;
                PT.pointB = pointJ;
            }
        }
    }
    return PT;
}
float GeomCalc::computePerpendicularDistanceFromPointC (pcl::PointXYZI pointA, pcl::PointXYZI pointB,pcl::PointXYZI pointC)
{
    float AB = GeomCalc::computeDistance2Dxy(pointA,pointB);
    float S = GeomCalc::computeTriangleArea(pointA,pointB,pointC);
    return (2*S)/AB;
}
float GeomCalc::computeTrianglePerimeter (pcl::PointXYZI pointA, pcl::PointXYZI pointB,pcl::PointXYZI pointC)
{
    float perimeter = GeomCalc::computeDistance2Dxy(pointA,pointB);
    perimeter += GeomCalc::computeDistance2Dxy(pointB,pointC);
    perimeter += GeomCalc::computeDistance2Dxy(pointA,pointC);
    return perimeter;
}
float GeomCalc::findLongestPerpendicularDistance (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI &pointA, pcl::PointXYZI &pointB)
{
    float distMax =0;
    for (int i=0; i<cloud->points.size(); i++)
    {
        pcl::PointXYZI pointC = cloud->points.at(i);
        float dist = GeomCalc::computePerpendicularDistanceFromPointC(pointA,pointB,pointC);
        if (dist > distMax) {distMax = dist;}
    }
    return distMax;
}
float GeomCalc::computeTriangleArea (pcl::PointXYZI pointA, pcl::PointXYZI pointB,pcl::PointXYZI pointC)
{
    float AB = GeomCalc::computeDistance3D(pointA,pointB);
    float BC = GeomCalc::computeDistance3D(pointB,pointC);
    float AC = GeomCalc::computeDistance3D(pointA,pointC);
    float s = (AB+BC+AC)/2;
    return sqrt(s*(s-AB)*(s-BC)*(s-AC));
}
float GeomCalc::computePolygonArea (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    // for each point
    float sum=0;
    pcl::PointXYZI A,B;
    for(int i = 1; i < cloud->points.size(); i++)
    {

        A = cloud->points.at(i);
        B = cloud->points.at(i-1);

        sum+= (A.x*B.y - B.x*A.y);
    }
    A = cloud->points.at(0);
    B = cloud->points.at(cloud->points.size()-1);
    sum+= (A.x*B.y - B.x*A.y);
    return std::fabs(sum/2);
}
bool GeomCalc::isLineIntersection(pcl::PointXYZI a1, pcl::PointXYZI a2,pcl::PointXYZI b1,pcl::PointXYZI b2)
{
    float s1x, s1y, s2x, s2y;
    s1x = a2.x-a1.x;
    s1y = a2.y-a1.y;
    s2x = b2.x-b1.x;
    s2y = b2.y-b1.y;

    float s = (-s1y*(a1.x-b1.x)+s1x*(a1.y-b1.y))/(-s2x*s1y+s1x*s2y);
    float t = (-s2x*(a1.y-b1.y)-s2y*(a1.x-b2.x))/(-s2x*s1y+s1x*s2y);
    if(s>0 && s<1 && t>-1 && t<0)
    {return true;
    }else return false;
}
bool GeomCalc::isXYequal (pcl::PointXYZI pointA, pcl::PointXYZI pointB)
{
    if(pointA.x == pointB.x && pointA.y == pointB.y){
    return true;
    }else return false;
}
bool GeomCalc::isAnyEdgeIntersect(pcl::PointCloud<pcl::PointXYZI>::Ptr listOfEdges, pcl::PointXYZI newEdgePtA, pcl::PointXYZI newEdgePtB)
{
    for (int i = 1; i<listOfEdges->points.size();i++)
    {
        pcl::PointXYZI a = listOfEdges->points.at(i-1);
        pcl::PointXYZI b = listOfEdges->points.at(i);
        if(GeomCalc::isLineIntersection(newEdgePtA,newEdgePtB,a,b)== true)
        {
            return true;
        }
    }
    return false;
}
bool GeomCalc::isAnyPointInTriangle(pcl::PointCloud<pcl::PointXYZI>::Ptr points,pcl::PointXYZI a,pcl::PointXYZI b,pcl::PointXYZI c)
{
    for(int i=0;i<points->points.size();i++)
    {
        pcl::PointXYZI testedPoint = points->points.at(i);
        if (GeomCalc::isPointInTriangle(testedPoint,a,b,c) == true)
        {
            return true;
        }else continue;
    }
    return false;
}
bool GeomCalc::isPointInTriangle(pcl::PointXYZI testedPoint,pcl::PointXYZI a,pcl::PointXYZI b,pcl::PointXYZI c)
{
    if (GeomCalc::isXYequal(testedPoint,a) != true && GeomCalc::isXYequal(testedPoint,b) != true && GeomCalc::isXYequal(testedPoint,c) != true)
    {
        float alfa = GeomCalc::computeClockwiseAngle(a,b,testedPoint);
        float beta = GeomCalc::computeClockwiseAngle(b,c,testedPoint);
        float gama = GeomCalc::computeClockwiseAngle(c,a,testedPoint);
        if(alfa <180 && beta < 180 && gama < 180)
        {
            return true;
        }
    }
    return false;
}
float GeomCalc::getTriangleSideRatio(pcl::PointXYZI pointA, pcl::PointXYZI pointB,pcl::PointXYZI pointC)
{
    float a = GeomCalc::computeDistance3D(pointA,pointB);
    float b = GeomCalc::computeDistance3D(pointB,pointC);
    float c = GeomCalc::computeDistance3D(pointA,pointC);
    float sideRatioSum = 0;

    if(a>b && a>c){sideRatioSum = 1+(a/b)+(a/c);}
    else if(b>a && b>c){sideRatioSum = 1+(b/a)+(b/c);}
    else {sideRatioSum = 1+(c/a)+(c/b);}

    return sideRatioSum;
}
float GeomCalc::computePolygonLenght(pcl::PointCloud<pcl::PointXYZI>::Ptr polygon)
{
    float lenght=0;
    for(int i=1; i<polygon->points.size();i++)
    {
        lenght +=GeomCalc::computeDistance2Dxy(polygon->points.at(i-1),polygon->points.at(i));
    }
    return lenght;
}
pcl::PolygonMesh GeomCalc::triangulatePolygon(pcl::PointCloud<pcl::PointXYZI>::Ptr polygon)
{
 /*   pcl::PointCloud<pcl::PointXYZI> cloudAll;
    for(int i=0; i<polygon->points.size();i++)
    {
        cloudAll.points.push_back(polygon->points.at(i));
    }
    pcl::PCLPointCloud2 cloudAll2;
    pcl::toPCLPointCloud2(cloudAll,cloudAll2);

    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    pcl::PolygonMesh::Ptr in;
    mesh->cloud = cloudAll2;
    pcl::Vertices vertice;
    for(int i=0; i<mesh->cloud.width; i++)
    {
        vertice.vertices.push_back(i);
        i++;
    }
    mesh->polygons.push_back(vertice);

    pcl::EarClipping ec;

    ec.setInputMesh(mesh);

    pcl::PolygonMesh out;
    ec.process(out);
  return out;*/
/*
   vtkSmartPointer<vtkPoints> pointsVTK = vtkSmartPointer< vtkPoints >::New();
    for(int i=0; i<polygon->points.size();i++)
    {
        pcl::PointXYZI p = polygon->points.at(i);
        pointsVTK->InsertNextPoint(p.x, p.y, p.z);
    }
     // Create the polygon
    vtkSmartPointer<vtkPolygon> polygonVTK = vtkSmartPointer<vtkPolygon>::New();
    polygonVTK->GetPointIds()->SetNumberOfIds(4);
    for(int i=0; i<polygon->points.size();i++)
    {
        polygonVTK->GetPointIds()->SetId(i, i);
    }
  // Add the polygon to a list of polygons
    vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();
    polygons->InsertNextCell(polygonVTK);

    //VTK Points to polydata
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(pointsVTK);
    polydata->SetPolys(polygons);






    //To polydata
    vtkPolyData* polydataOut = surfaceFilter->GetOutput();
    //volume and surface
    //Convert to pclMesh
    pcl::PolygonMesh triangles;
    pcl::VTKUtils::vtk2mesh(polydataOut,triangles);*/
    pcl::PolygonMesh triangles;
    return triangles;
}
//CLOUD OPERATIONS
void CloudOperations::cloudXYZItoXYZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ)
{

    cloudXYZ->points.resize(cloudXYZI->points.size());
    for (size_t i = 0; i < cloudXYZI->points.size(); i++)
        {
            cloudXYZ->points[i].x = cloudXYZI->points[i].x;
            cloudXYZ->points[i].y = cloudXYZI->points[i].y;
            cloudXYZ->points[i].z = cloudXYZI->points[i].z;
        }
}
void CloudOperations::transformCloudToPlane (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float direction, float vertical)
{
    //transform angles from degree to rad
    vertical /= (180/3.14159265359);
    direction /= (180/3.14159265359);
    //find lowest point of the tree
    cloudHighestAndLowestZValue HL = GeomCalc::findHighestAndLowestPoints(cloud);
    //cloud transformation
    for (int i =0; i < cloud->points.size(); i++)
    {
        pcl::PointXYZI bod = cloud->points.at(0);
        float a = bod.z - HL.Lowest;
        float prepona = a/(sin(vertical));
        float c = prepona*(cos(vertical));
        bod.x = bod.x +(c*sin(direction));
        bod.y = bod.y +(c*cos(direction));
        bod.z = HL.Lowest;
        cloud->points.push_back(bod);
        cloud->points.erase(cloud->points.begin());
    }
}
void CloudOperations::erasePointFromCloudXY (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI bod)
{
                float q = (float) cloud->points.size();
                for (int h=0; h<q; h++)
                {
                    pcl::PointXYZI bodx =cloud->points.at(h);
                    if (bodx.x == bod.x && bodx.y == bod.y)
                    {
                        cloud->points.erase(cloud->points.begin()+h);
                        q--;
                    }
                }
}
pcl::PointCloud<pcl::PointXYZI>::Ptr CloudOperations::getCloudCopy (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr copyCloud (new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; i < cloud->points.size(); i++)
    {
      pcl::PointXYZI point = cloud->points.at(i);
      copyCloud->points.push_back(point);
    }
    return copyCloud;
}
void CloudOperations::ifFirstLastAreEqualEraseOne(pcl::PointCloud<pcl::PointXYZI>::Ptr polygon)
{
    pcl::PointXYZI first = polygon->points.at(0);
    pcl::PointXYZI last = polygon->points.at(polygon->points.size()-1);
    if(first.x == last.x && first.y == last.y)
    {
        polygon->points.erase(polygon->points.begin());
    }
}
void CloudOperations::sortPolygonFromIterator(pcl::PointCloud<pcl::PointXYZI>::Ptr polygon, int it)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    for(int i=0; i<it; i++)
    {
        pcl::PointXYZI pt = polygon->points.at(i);
        cloud->points.push_back(pt);
    }
    polygon->points.erase(polygon->points.begin(),polygon->points.begin()+it);
    for(int i=1; i<cloud->points.size(); i++)
    {
        pcl::PointXYZI pt = cloud->points.at(i);
        polygon->points.push_back(pt);
    }
    pcl::PointXYZI first = polygon->points.at(0);
    polygon->points.push_back(first);
}
pcl::PolygonMesh CloudOperations::PolygonToMesh(pcl::PointCloud<pcl::PointXYZI>::Ptr polygon)
{
    // pcl cloud to VTK points
    vtkSmartPointer<vtkPoints> pointsVTK = vtkSmartPointer< vtkPoints >::New();
    for(int i=0; i<polygon->points.size();i++)
    {
        pcl::PointXYZI p = polygon->points.at(i);
        pointsVTK->InsertNextPoint(p.x, p.y, p.z);
    }
    //VTK Points to polydata
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(pointsVTK);

    vtkSmartPointer<vtkCellArray> aCellArray = vtkSmartPointer<vtkCellArray>::New();

    vtkSmartPointer<vtkPolygon> aPolygon = vtkSmartPointer<vtkPolygon>::New();

    for(int i=0;i<polygon->points.size()-1;i++)
    {
        aPolygon->GetPointIds()->InsertNextId(i);
    }
    aCellArray->InsertNextCell(aPolygon);

    vtkSmartPointer<vtkPolyData> boundary = vtkSmartPointer<vtkPolyData>::New();
    boundary->SetPoints(polydata->GetPoints());
    boundary->SetPolys(aCellArray);

    vtkSmartPointer<vtkDelaunay2D> delaunay = vtkSmartPointer<vtkDelaunay2D>::New();
    delaunay->SetInputData(polydata);
    delaunay->SetSourceData(boundary);
    delaunay->Update();

    vtkPolyData* polydataOut = delaunay->GetOutput();

    pcl::PolygonMesh triangles;
    pcl::VTKUtils::vtk2mesh(polydataOut,triangles);

    return triangles;
}
void CloudOperations::voxelizeCloud (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr output, float x, float y, float z)
{
  pcl::VoxelGrid<pcl::PointXYZI> vox;
  vox.setInputCloud (cloud);
  vox.setLeafSize (x, y, z);
  vox.filter (*output);
}
// TRIANGULATED POLYGON
TriangulatedPolygon::TriangulatedPolygon(pcl::PointCloud<pcl::PointXYZI>::Ptr polygon)
{
    polygonTriangulation(polygon);
    QString xx = QString("triangles %1").arg(m_triangles.size());
    QMessageBox::information(0,("WARNING"),xx);
}
//GET
pcl::PointCloud<pcl::PointXYZI>::Ptr TriangulatedPolygon::getTriangleAt(int i)
{
    return m_triangles.at(i);
}
int TriangulatedPolygon::getTrianglesSize()
{
    return m_triangles.size();
}
//COMPUTE
void TriangulatedPolygon::polygonTriangulation (pcl::PointCloud<pcl::PointXYZI>::Ptr polygon)
{
    CloudOperations::ifFirstLastAreEqualEraseOne(polygon);
    int polsize = polygon->points.size();
    do{
        for(int i=0; i<polygon->points.size()-1;i++)
        {
            // Add new triangle into m_triangles if fulfill the conditions
            addNewTriangle(polygon,i);
        }
        if (polsize == polygon->points.size()) {break;}
            polsize = polygon->points.size();
    }while(polygon->points.size()>3);
    //add last remaining triangle to point cloud vector
    if(polygon->points.size()==3){
        m_triangles.push_back(polygon);
    }
    //m_triangles.push_back(polygon);
}
void TriangulatedPolygon::addNewTriangle(pcl::PointCloud<pcl::PointXYZI>::Ptr polygon,int iter)
{
    pcl::PointXYZI a;
    if(iter==0){
    a = polygon->points.at(polygon->points.size()-1);
    }else {a = polygon->points.at(iter-1);}
    pcl::PointXYZI b = polygon->points.at(iter);
    pcl::PointXYZI c = polygon->points.at(iter+1);

    if(GeomCalc::computeClockwiseAngle(a,b,c) <= 180 && GeomCalc::isAnyPointInTriangle(polygon,a,b,c)== false)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr triangle (new pcl::PointCloud<pcl::PointXYZI>);
        //push points to triangle cloud and push that cloud to point cloud vector
        triangle->points.push_back(a);
        triangle->points.push_back(b);
        triangle->points.push_back(c);
        m_triangles.push_back(triangle);
        //erase ear central point from polygon
        polygon->points.erase(polygon->points.begin()+iter);
    }
}

// POLYGONS TO PCL MESH TRANSFORMATION
TrianglesToPclMeshTransformation::TrianglesToPclMeshTransformation()
{
    m_mesh = new pcl::PolygonMesh;
}
TrianglesToPclMeshTransformation::TrianglesToPclMeshTransformation(pcl::PointCloud<pcl::PointXYZI>::Ptr cl)
{
     m_mesh = new pcl::PolygonMesh;
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

    vtkSmartPointer<vtkCellArray> aCellArray = vtkSmartPointer<vtkCellArray>::New();

    vtkSmartPointer<vtkPolygon> aPolygon = vtkSmartPointer<vtkPolygon>::New();

    for(int i=0;i<cl->points.size()-1;i++)
    {
        aPolygon->GetPointIds()->InsertNextId(i);
    }
    aCellArray->InsertNextCell(aPolygon);



    vtkSmartPointer<vtkPolyData> boundary = vtkSmartPointer<vtkPolyData>::New();
    boundary->SetPoints(polydata->GetPoints());
    boundary->SetPolys(aCellArray);

    vtkSmartPointer<vtkDelaunay2D> delaunay = vtkSmartPointer<vtkDelaunay2D>::New();
    delaunay->SetInputData(polydata);
    delaunay->SetSourceData(boundary);
    delaunay->Update();

    vtkPolyData* polydataOut = delaunay->GetOutput();

    pcl::PolygonMesh triangles;
    pcl::VTKUtils::vtk2mesh(polydataOut,triangles);

    *m_mesh = triangles;
}
void TrianglesToPclMeshTransformation::addTriangle(pcl::PointCloud<pcl::PointXYZI>::Ptr triangle)
{
    m_triangles.push_back(triangle);
}
void TrianglesToPclMeshTransformation::createMesh()
{
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
    m_mesh->cloud = cloudAll2;
    for(int i=2; i<=m_mesh->cloud.width; i++)
    {
        pcl::Vertices vertice;
        vertice.vertices.push_back(i-2);
        vertice.vertices.push_back(i-1);
        vertice.vertices.push_back(i);
        m_mesh->polygons.push_back(vertice);
        i +=2;
    }
}
pcl::PolygonMesh TrianglesToPclMeshTransformation::getMesh()
{
    return *m_mesh;
}


//QString ax = QString(" EDGES %1  Gsize %2").arg(m_edges.size()).arg(greaterSection->points.size());
//QMessageBox::information(0,("WARNING"),ax);




