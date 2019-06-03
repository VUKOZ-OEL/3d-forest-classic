#include "hull.h"
#include <vtkFeatureEdges.h>

//CLASS CONVEXHULL*/
ConvexHull::ConvexHull (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  //  QString hullName = QString("%1_hull").arg(name);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_h(new pcl::PointCloud<pcl::PointXYZI>);
    m_convexhull = cloud_h;
    m_mesh = new pcl::PolygonMesh;
    m_polygonArea = 0;

    pcl::PointXYZI minp,maxp;
    pcl::getMinMax3D(*cloud,minp, maxp);
    computeAttributes(cloud,minp.z);
}
ConvexHull::~ConvexHull()
{
    //delete m_convexhull;
}
//COMPUTE
void ConvexHull::computeAttributes(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,float zHeight)
{
    // voxelize to 1 cm
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZI>);
    CloudOperations::voxelizeCloud(cloud,cloudFiltered,0.01f,0.01f,100.0f);
    //pcl to VTK points
    vtkSmartPointer<vtkPoints> pointsVTK = vtkSmartPointer< vtkPoints >::New();
    pclCloudToVTKPoints(cloudFiltered,pointsVTK,zHeight);
    //Delaunay
    pcl::PointCloud<pcl::PointXYZI>::Ptr verticesCloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr polygon (new pcl::PointCloud<pcl::PointXYZI>);
    makeConvexHullDelaunay2D(pointsVTK,verticesCloud);
    //Vertices to polygon order
    toPolygonOrderWithTestToConvexness(verticesCloud,polygon);
    m_convexhull = polygon;
    getBackIntensityFromOrigCloud(cloudFiltered);

   // QString info = QString("END %1 %2").arg(m_convexhull->points.size()).arg(triangles.polygons.size());
   // QMessageBox::information(0,("ConvexHullDelaunay"),info);
}
void ConvexHull::getBackIntensityFromOrigCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr originalCloud)
{
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (originalCloud);
    for(int i=0;i<m_convexhull->points.size();i++)
    {
        pcl::PointXYZI searchPoint = m_convexhull->points.at(i);
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        kdtree.nearestKSearch (searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
        m_convexhull->points.at(i).intensity = originalCloud->points[ pointIdxNKNSearch[0] ].intensity;
    }
}
//GET
pcl::PointCloud<pcl::PointXYZI>::Ptr ConvexHull::getPolygon()
{
    /*
    Cloud *c = new Cloud(m_convexhull,"convexhull");
    return c;
    */
    return m_convexhull;
}
float ConvexHull::getPolygonArea()
{
    return m_polygonArea;
}
void ConvexHull::pclCloudToVTKPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,vtkSmartPointer<vtkPoints> pointsVTK,float newZcoord)
{
    for(int i=0; i<cloud->points.size(); i++)
    {
        pcl::PointXYZI p = cloud->points.at(i);
        pointsVTK->InsertNextPoint(p.x, p.y, newZcoord);
    }
}
void ConvexHull::makeConvexHullDelaunay2D (vtkSmartPointer<vtkPoints> input,pcl::PointCloud<pcl::PointXYZI>::Ptr outputVertices)
{
    //VTK Points to polydata
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(input);
    //delaunay
    vtkSmartPointer<vtkDelaunay2D> delaunay2D =vtkSmartPointer<vtkDelaunay2D>::New();
    delaunay2D->SetInputData(polydata);
    delaunay2D->SetAlpha(100);
    delaunay2D->Update();
    //Create surface from
    vtkSmartPointer<vtkDataSetSurfaceFilter> surfaceFilter = vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
    surfaceFilter->SetInputConnection(delaunay2D->GetOutputPort());
    surfaceFilter->Update();
    //volume and surface
    vtkSmartPointer<vtkMassProperties> mp =  vtkSmartPointer<vtkMassProperties>::New();
    vtkPolyData* polydataToSurface = surfaceFilter->GetOutput();
    mp->SetInputData(polydataToSurface);
    m_polygonArea = mp->GetSurfaceArea();
    //get boundary polygon vertices
    vtkSmartPointer<vtkFeatureEdges> featureEdges =
    vtkSmartPointer<vtkFeatureEdges>::New();
    featureEdges->SetInputConnection(delaunay2D->GetOutputPort());
    featureEdges->BoundaryEdgesOn();
    featureEdges->FeatureEdgesOff();
    featureEdges->ManifoldEdgesOff();
    featureEdges->NonManifoldEdgesOff();
    featureEdges->Update();
    vtkPolyData* polydataVertices = featureEdges->GetOutput();
    // to pcl mesh
    pcl::PolygonMesh triangles;
    pcl::VTKUtils::vtk2mesh(polydataVertices,triangles);
    //set surface to M_MESH
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudVertices (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(triangles.cloud,*outputVertices);
}
void ConvexHull::toPolygonOrderWithTestToConvexness(pcl::PointCloud<pcl::PointXYZI>::Ptr vertices,pcl::PointCloud<pcl::PointXYZI>::Ptr polygon)
{
    pcl::PointXYZI first = returnPointLowestYZ(vertices);
    polygon->points.push_back(first);
    findSecondpointInPolygon(vertices,polygon);

    do{

        float alfa = 0;
        int a = polygon->points.size();
        a-=1;
        int iter = 0;

        for(int i=0;i<vertices->points.size();i++)
        {
            float beta = GeomCalc::computeClockwiseAngle(polygon->points.at(a-1),polygon->points.at(a),vertices->points.at(i));
            if(beta > alfa)
            {
                alfa = beta;
                iter = i;
            }
        }
        float alfaTest = GeomCalc::computeClockwiseAngle(polygon->points.at(a-1),polygon->points.at(a),first);

        if(alfa > alfaTest){
            polygon->points.push_back(vertices->points.at(iter));
            vertices->points.erase(vertices->points.begin()+iter);
        }else{
                break;
        }

    }while(vertices->points.size()>0);

   // polygon->points.push_back(first); //add first point to the end of polygon
}
void ConvexHull::findSecondpointInPolygon(pcl::PointCloud<pcl::PointXYZI>::Ptr vertices,pcl::PointCloud<pcl::PointXYZI>::Ptr polygon)
{
    pcl::PointXYZI first = polygon->points.at(0);
    pcl::PointXYZI backVec = first;
    backVec.y -=1;

    float alfa = 0;
    float iter = 0;
    //iterate over cloud points and select that one with biggest clockwise angle
    pcl::PointXYZI second;
    for(int i=0; i < vertices->points.size();i++)
    {
        pcl::PointXYZI point = vertices->points.at(i);
        float beta = GeomCalc::computeClockwiseAngle(backVec,first,point);
        if (beta > alfa)
        {
            alfa = beta;
            second = point;
            iter = i;
        }
    }
    polygon->points.push_back(second);
    vertices->points.erase(vertices->points.begin()+iter);
}
pcl::PointXYZI ConvexHull::returnPointLowestYZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    float yMin = cloud->points.at(0).y;
    int iter = 0;
    for (int i=0; i < cloud->points.size(); i++)
    {
        pcl::PointXYZI bod =cloud->points.at(i);
        if (bod.y < yMin)
        {
            yMin = bod.y;
            iter = i;
        }
    }
    pcl::PointXYZI bodYmin = cloud->points.at(iter);
    cloud->points.erase(cloud->points.begin()+iter);
    return bodYmin;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/* CLASS CONCAVEHULL */
ConcaveHull::ConcaveHull(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name, float searchDist)
: Cloud(cloud, name)
{
    QString hullName = QString("%1_hull").arg(name);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_h(new pcl::PointCloud<pcl::PointXYZI>);
    m_concavehull = new Cloud(cloud_h, hullName);

    m_searchingDistance = searchDist;
    computeAttributes();
}
ConcaveHull::~ConcaveHull()
{
    delete m_concavehull;
}
//COMPUTE
void ConcaveHull::computeAttributes()
{
    computeConcaveHull();
    m_polygonArea = GeomCalc::computePolygonArea(m_concavehull->get_Cloud());
}
void ConcaveHull::computeConcaveHull()
{
    //Copy cloud and create convex hull
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(m_Cloud);
    int sizeOfCloud = cloud->points.size();
    ConvexHull *c = new ConvexHull(cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr hullCloud (new pcl::PointCloud<pcl::PointXYZI>);
    hullCloud = c->getPolygon();

    if(sizeOfCloud <= hullCloud->points.size() || cloud->points.size() == 1)
    {
        m_concavehull->set_Cloud(hullCloud);
        return;
    }else{
        //Edges breaking with incremental max edge lenght, start at 0.5m
        float searchDist = m_searchingDistance;
        int polygonSize = hullCloud->points.size();
        for (int i=0; i<11; i++)
        {
            edgesBreaking(cloud,hullCloud,searchDist);
            //If all edges are shorter than searchDist stop the loop
            if(polygonSize == hullCloud->points.size()){break;}
            polygonSize = hullCloud->points.size();
            //Increase serching distance
            searchDist +=0.1;
        }
    m_concavehull->set_Cloud(hullCloud);
    }
}
void ConcaveHull::edgesBreaking(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr hullCloud,float searchDist)
{
    float n = (float) hullCloud->points.size();
    int a =0;
    do{
        int j = a+1;
        pcl::PointXYZI boda =hullCloud->points.at(a);
        pcl::PointXYZI bodb =hullCloud->points.at(j);
        pcl::PointXYZI bodx;
        float distAB =GeomCalc::computeDistance2Dxy(boda,bodb);
        if (distAB > searchDist)
        {
            bodx = returnEdgeBreakingPoint(cloud,boda,bodb,searchDist);
            if(bodx.z != 0 and bodx.x != 0)
            {
                hullCloud->points.insert(hullCloud->points.begin()+j,bodx);
                CloudOperations::erasePointFromCloudXY(cloud,bodx);
                n++;
                a--;
            }
        }
    a++;
    }while (a < n-1);
}
pcl::PointXYZI ConcaveHull::returnEdgeBreakingPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,pcl::PointXYZI boda, pcl::PointXYZI bodb,float searchDist)
{
    float Amax = 9999;
    pcl::PointXYZI bodx;
    float distAB =GeomCalc::computeDistance2Dxy(boda,bodb);
    for (int g=0; g<cloud->points.size(); g++)
    {
        pcl::PointXYZI bodc =cloud->points.at(g);
        if(GeomCalc::isXYequal(boda,bodc)!=true || GeomCalc::isXYequal(bodb,bodc)!=true)
        {
            if ((GeomCalc::computeDistance2Dxy(boda,bodc) < searchDist and GeomCalc::computeDistance2Dxy(bodb,bodc) < distAB) or
                (GeomCalc::computeDistance2Dxy(bodb,bodc) < searchDist and GeomCalc::computeDistance2Dxy(boda,bodc) < distAB))
            {
                float A =GeomCalc::computeClockwiseAngle(boda,bodc,bodb);
                if (A < Amax)
                {
                    Amax = A;
                    bodx =bodc;
                    bodx.z =boda.z;
                    bodx.intensity=0;
                }
            }
        }
    }
    return bodx;
}

//GET
Cloud ConcaveHull::getPolygon()
{
    return *m_concavehull;
}
float ConcaveHull::getPolygonArea()
{
    return m_polygonArea;
}
Cloud ConcaveHull::getPolygonSwappedZI()
{
     pcl::PointCloud<pcl::PointXYZI>::Ptr c (new pcl::PointCloud<pcl::PointXYZI>);
     c = m_concavehull->get_Cloud();
     for(pcl::PointCloud<pcl::PointXYZI>::iterator it = c->begin(); it != c->end(); it++)
    {
        float z = it->z;
        it->z = it->intensity;
        it->intensity = z;
    }
     pcl::PointXYZI p1 = c->points.at(0);
    int s = c->points.size();
   pcl::PointXYZI p2 = c->points.at(s-1);
    if(p1.z<p2.z)
    {
          c->points.at(s-1).z = p1.z;
          //QString a = QString(" p1 %1  p2 %2").arg(p1.z).arg(p2.z);
          //QMessageBox::information(0,("WARNING"),a);
    }

    QString a = QString("%1_swapppedConcavehull").arg(m_name);
    Cloud *cloud = new Cloud(c, a);
    return *cloud;
}




