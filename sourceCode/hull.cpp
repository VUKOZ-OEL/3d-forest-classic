#include "hull.h"
#include <vtkFeatureEdges.h>
#include <iomanip>

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
            if(bodx.z != 0 && bodx.x != 0)
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
            if ((GeomCalc::computeDistance2Dxy(boda,bodc) < searchDist && GeomCalc::computeDistance2Dxy(bodb,bodc) < distAB) ||
                (GeomCalc::computeDistance2Dxy(bodb,bodc) < searchDist && GeomCalc::computeDistance2Dxy(boda,bodc) < distAB))
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



ConcaveHull2::ConcaveHull2()
{}
ConcaveHull2::ConcaveHull2(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    setCloud(cloud);
}
ConcaveHull2::ConcaveHull2(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr convexhull)
{
    setCloud(cloud);
    setConvexHull(convexhull);
}
ConcaveHull2::ConcaveHull2(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float limit)
{
    setCloud(cloud);
    setSearchLimit(limit);
}
ConcaveHull2::~ConcaveHull2()
{}
void ConcaveHull2::compute()
{
    //test();
    //return;
// najit spodni levy bod
   // std::cout<< "m_cloud  size " << m_cloud->points.size() << "\n";
    m_convexhull = computeHull(m_cloud, m_cloud->points.size());
   // std::cout<< "m_convexhull  size " << m_convexhull->points.size() << "\n";
    computeAreaConvex();
  //  std::cout<< "m_cloud  size po convexhullu " << m_cloud->points.size() << "\n";
    m_concavehull= computeHull(m_cloud, 15);
    //std::cout<< "m_concavehull  size " << m_concavehull->points.size() << "\n";
    computeAreaConcave();
   // std::cout<< "m_cloud  size po concavehullu " << m_cloud->points.size() << "\n";
    //computeConvexHull();
    // pro kazdy bod spocitat uhel
    // vložit uhel s nejvyšší hodnotou do hullu.
    
    //pokud nema hull vic jak dva body, pouzit osu x
    //std::cout<<"convexArea: "<< m_convexArea << " concaveAree: " << m_concaveArea <<"\n";
}
void ConcaveHull2::setUsedPoints()
{
    for(int i=0; i < m_cloud->points.size();i++)
    {
        pcl::PointXYZI bod = m_cloud->points.at(i);
        bool used = false;
        for(int u=0; u < m_convexhull->points.size(); u++)
        {
            pcl::PointXYZI p = m_convexhull->points.at(u);
            if(p.x == bod.x && p.y == bod.y)// && p.z == bod.z)
                used=true;
        }
        m_usedPoint.push_back(used);
    }
}
float ConcaveHull2::getConvexArea()
{return m_convexArea;}
float ConcaveHull2::getConcaveArea()
{return m_concaveArea;}

void ConcaveHull2::setCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
   // pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZI>);
   // CloudOperations::voxelizeCloud(cloud,cloudFiltered,0.01f,0.01f,100.0f);
   // std::cout<<"cLOUD: "<< cloud->points.size() << " cloudFiltered: " << cloudFiltered->points.size() <<"\n";
    m_cloud = cloud;
}
void ConcaveHull2::setConvexHull(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){m_convexhull = cloud;}
void ConcaveHull2::setSearchLimit(float s){m_searchLimit = s;}
float ConcaveHull2::computeAreaConvex()
{
    float area = computeArea(m_convexhull);
    m_convexArea = area;
    return area;
}
float ConcaveHull2::computeAreaConcave()
{
    float area = computeArea(m_concavehull);
    m_concaveArea = area;
    return area;
    
}
float ConcaveHull2::computeArea(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    // Initialze area
      float area = 0.0;
    
      // Calculate value of shoelace formula
      int j = cloud->points.size() - 1;
      for (int i = 0; i < cloud->points.size(); i++)
      {
          area += (cloud->points.at(j).x + cloud->points.at(i).x)*(cloud->points.at(j).y-cloud->points.at(i).y);
          //area += (X[j] + X[i]) * (Y[j] - Y[i]);
          j = i;  // j is previous vertex to i
      }
      // Return absolute value
      return std::abs(area / 2.0);
}
float ConcaveHull2::computeDistance(pcl::PointXYZI lineA,pcl::PointXYZI lineB,pcl::PointXYZI point)
{
    pcl::PointXYZI AB;
    AB.x =lineB.x - lineA.x;
    AB.y =lineB.y - lineA.y;
    AB.z =lineB.z - lineA.z;
    
    pcl::PointXYZI AC;
    AC.x =point.x - lineA.x;
    AC.y =point.y - lineA.y;
    AC.z =point.z - lineA.z;
    pcl::PointXYZI ACAB;
    ACAB.x =AB.y * AC.z - AB.z * AC.y;
    ACAB.y =AB.z * AC.x - AB.x * AC.z;
    ACAB.z =AB.x * AC.y - AB.y * AC.x;
    float area = std::sqrt(ACAB.x*ACAB.x + ACAB.y*ACAB.y + ACAB.z*ACAB.z);
    float dist = area/std::sqrt(AB.x*AB.x + AB.y*AB.y + AB.z*AB.z);
    
    
   // float a = lineA.y - lineB.y;
   // float b = lineB.x - lineA.x;
   // float c = (lineA.x * lineB.y) - (lineB.x * lineA.y);

    //float dist = std::abs(a*point.x + b*point.y + c)/ std::sqrt(a*a + b*b);
   // std::cout<<std::fixed << std::setprecision(3)<< " dist "<< dist <<"\n";
    return dist;
}
float ConcaveHull2::computeConvexHull(){
    
    ConvexHull *c = new ConvexHull(m_cloud);
    m_convexhull = c->getPolygon();
    m_convexArea = c->getPolygonArea();
    return m_convexArea;
    
}
float ConcaveHull2::computeTwoPointsDist(pcl::PointXYZI lineA, pcl::PointXYZI lineB)
{
    if(lineA.x == lineB.x && lineA.y == lineB.y && lineA.z == lineB.z)
        return 0;
    
    return std::sqrt((lineA.x - lineB.x)*(lineA.x - lineB.x) + (lineA.y - lineB.y)*(lineA.y - lineB.y) + (lineA.z - lineB.z)*(lineA.z - lineB.z));
}
pcl::PointCloud<pcl::PointXYZI>::Ptr ConcaveHull2::getConvexHull()
{
    return m_convexhull;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr ConcaveHull2::getCloud()
{
    return m_cloudP->get_Cloud();
}
pcl::PointCloud<pcl::PointXYZI>::Ptr ConcaveHull2::getConcaveHull(){
    return m_concavehull;
}
float ConcaveHull2::computeAngle(pcl::PointXYZI& a, pcl::PointXYZI& b, pcl::PointXYZI& c)
{
    // compute angle between vector AB and AC;
    //std::cout<<std::fixed << std::setprecision(3)<< " a.x "<< a.x<< " a.y "<< a.y <<"\n";
    //std::cout<<std::fixed << std::setprecision(3)<< " b.x "<< b.x<< " b.y "<< b.y <<"\n";
    //std::cout<<std::fixed << std::setprecision(3)<< " c.x "<< c.x<< " c.y "<< c.y <<"\n";
    pcl::PointXYZI u;
    u.x = b.x-a.x;
    u.y= b.y-a.y;
    u.z= b.z-a.z;
//std::cout<< " u.x "<< u.x<< " u.y "<< u.y <<"\n";
    pcl::PointXYZI v;
    v.x = c.x-a.x;
    v.y= c.y-a.y;
    v.z= c.z-a.z;
    //std::cout<< " v.x "<< v.x<< " u.y "<< v.y <<"\n";
    float dot = (u.x * v.x + u.y * v.y); // dot product
    float cross = (u.x * v.y - u.y * v.x); // cross product
    float alpha = atan2(cross, dot);
    float uh = alpha * 180./ M_PI;
    //std::cout<<"uhel: "<< uh << " alpha: " << alpha <<"\n";
    if(uh < 0)
        uh = 360+uh;
    return uh;
}
void ConcaveHull2::setPlaneCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr c)
{
    //pcl::PointCloud<pcl::PointXYZI>::Ptr c (new pcl::PointCloud<pcl::PointXYZI>);
    //m_cloudP = c;
    for(int i=0; i < cloud->points.size(); i++)
    {
        pcl::PointXYZI v;
        v.x = cloud->points.at(i).x;
        v.y = cloud->points.at(i).y;
        v.z = 1;
        c->points.push_back(v);
    }
}
pcl::PointCloud<pcl::PointXYZI>::Ptr ConcaveHull2::computeHull(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int k)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr clo (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cl (new pcl::PointCloud<pcl::PointXYZI>);
    m_cloudP = new Cloud();
   // *m_cloudP (new pcl::PointCloud<pcl::PointXYZI>);
   // std::cout<< "cloud size: "<< cloud->points.size()<< "\n";
    //removeDuplicateXYpoints(cloud, clo);
   // std::cout<< "removeDuplicateXYpoints: "<< clo->points.size()<< "\n";
    setPlaneCloud(cloud, cl);
    //std::cout<< "setPlaneCloud: "<< cl->points.size()<< "\n";
    m_cloudP->set_Cloud(cl);
    pcl::PointXYZI minp,maxp,leftM, firstPoint;
     int startId=-1;
     pcl::getMinMax3D(*cl,minp, maxp);
    //std::cout<<std::fixed << std::setprecision(5)<<"min.x "<< minp.x << " min.y " << minp.y << " min.z "<< minp.z << "\n";
   // std::cout<<std::fixed << std::setprecision(5)<<"max.x "<< maxp.x << " max.y " << maxp.y << " max.z "<< maxp.z << "\n";
    float left =99999999999;
     for(int i=0;i<cl->points.size();i++)
     {
         if(cl->points.at(i).x <= minp.x && cl->points.at(i).y < left)
         {
             startId = i;
             left = cl->points.at(i).y;
         }
     }
     firstPoint = cl->points.at(startId);
     //setPlaneCloud();
     //std::cout<< " startID: "<< startId << "\n";
     // ulozit bod do hullu
     pcl::PointCloud<pcl::PointXYZI>::Ptr hull (new pcl::PointCloud<pcl::PointXYZI>);
     hull->points.push_back(cl->points.at(startId));
   // std::cout<<std::fixed << std::setprecision(5)<<"del.x "<< cl->points.begin()+(startId-1)).x << " del.y " << firstPoint.y << " p.z "<< firstPoint.z << "\n";
        cl->points.erase(cl->points.begin()+startId);
  //  std::cout<<std::fixed << std::setprecision(5)<<"p.x "<< firstPoint.x << " p.y " << firstPoint.y << " p.z "<< firstPoint.z << "\n";
     // hledani nejblizsich bodu
    // std::cout<< " vyhledavani \n";
     for(int u =0; u < hull->points.size();u++)
     {
         if(hull->points.size()==3)
             cl->points.push_back(firstPoint);
             //m_indi.insert(m_indi.begin()+startId,startId);
        
         //std::cout<< " velikost hullu: "<< hull->points.size()<< "\n";
         
         pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
         kdtree.setInputCloud (cl);
         pcl::PointXYZI searchPoint = hull->points.at(u);
         std::vector<int> pointID;
         std::vector<float> pointDist;
         int id =startId;
        // int k=20;
         // najdi nejbližbí body
         kdtree.nearestKSearch (searchPoint, k, pointID, pointDist);
         //std::cout<< " nalezeno bodu "<< pointID.size() <<"\n";
         // pro kazdy bod spocitat uhel
         float leftAngle=-180;
         int leftID=-1;
         //std::cout<< "searchPoint: "<< id<< "\n";
      //   std::cout<<std::fixed << std::setprecision(5)<< " search.x "<< searchPoint.x<< " search.y "<< searchPoint.y << "\n";
         for(int t=0; t < pointID.size(); t++)
         {
             
            // std::cout<<" vyhledavani "<<  t<< " pointID: "<< pointID.at(t)<< "\n";
             
             if(cl->points.at(pointID.at(t)).x == searchPoint.x && searchPoint.y == cl->points.at(pointID.at(t)).y)
                 continue;
             pcl::PointXYZI a,b,c;
             if(hull->points.size()<2)
             {
                 b.x = searchPoint.x;
                 b.y= searchPoint.y-1;
                 b.z = searchPoint.z;
             }
             else{
                 b.x = hull->points.at(u-1).x;
                 b.y= hull->points.at(u-1).y;
                 b.z = hull->points.at(u-1).z;
             }
             c.x =cl->points.at(pointID.at(t)).x;
             c.y =cl->points.at(pointID.at(t)).y;
             c.z =cl->points.at(pointID.at(t)).z;
             int ori = orientation(searchPoint,b ,c);
             float angle;
             angle = computeAngle(searchPoint, b, c);
             
             bool inter = false;
            // std::cout<< " krizeni \n";
             if(hull->points.size() > 2)
             {
                 for(int op=0; op < hull->points.size()-2; op++)
                 {
                    bool intersectLine = intersect(hull->points.at(op), hull->points.at(op+1), hull->points.at(hull->points.size()-1), cl->points.at(pointID.at(t)));
                      if(intersectLine == true)
                          inter = true;
                 }
                 //std::cout<< " křizení "<< inter<< "\n";
                 if(inter == true)
                    continue;
             }
          //   std::cout<<std::fixed << std::setprecision(5)<< "pointID.x "<< cl->points.at(pointID.at(t)).x<< " pointID.y "<< cl->points.at(pointID.at(t)).y <<"\n";
         //    std::cout<< "    uhel: "<< angle<< " krizeni " << inter <<" orientace: "<< ori<<"\n";
             if(angle > leftAngle && inter == false)
             {
                 leftAngle = angle;
                 leftID =pointID.at(t);
               //  std::cout<< "  uhel: "<< angle<< "\n";
             }
         }
        // indicesptrP.reset();
         if(leftID == -1)
             continue;
         if (firstPoint.x == cl->points.at(leftID).x && firstPoint.y == cl->points.at(leftID).y )
         {
         //    std::cout<< " HOTOVO \n";
              break;
         }
            
        // std::cout<< "   leftUhel: "<< leftAngle<< "\n";
        // std::cout<< "   leftID: "<< leftID<< "\n";
         id = leftID;
      //   std::cout<<std::fixed << std::setprecision(5)<<"p.x "<< cl->points.at(leftID).x << " p.y " << cl->points.at(leftID).y << " p.z "<< cl->points.at(leftID).z << "\n";
        // std::cout<<std::fixed << std::setprecision(5)<<"p.x "<< cl->points.at(cl->points.begin()+leftID).x << " p.y " << cl->points.at(cl->points.begin()+leftID).y << " p.z "<< cl->points.at(cl->points.begin()+leftID).z << "\n";
         hull->points.push_back(cl->points.at(leftID));
        
        // std::cout<<std::fixed << std::setprecision(5)<<"hull.x "<< hull->points.at(hull->points.size()-1).x << " hull.y " << hull->points.at(hull->points.size()-1).y << " hull.z "<< hull->points.at(hull->points.size()-1).z << "\n";
         
          cl->points.erase(cl->points.begin()+(leftID));
        // m_indi.erase(m_indi.begin()+leftID);
     }
    for(int s=0; s < hull->points.size();s++)
    {
        hull->points.at(s).intensity = s;
    }
  //  std::cout<< "hull final size: "<< hull->points.size()<<"\n";
    return hull;
    
}
pcl::PointCloud<pcl::PointXYZI>::Ptr ConcaveHull2::findPointZ(pcl::PointCloud<pcl::PointXYZI>::Ptr c, pcl::PointCloud<pcl::PointXYZI>::Ptr z)
{
    // c cloud bez Z souradnice
    // z cloud s Z souradnici
    pcl::PointCloud<pcl::PointXYZI>::Ptr result (new pcl::PointCloud<pcl::PointXYZI>);
    for(int i=0;i< c->points.size();i++)
    {
        pcl::PointXYZI p = c->points.at(i);
        for(int u=0; u < z->points.size();u++)
        {
            pcl::PointXYZI pz = z->points.at(u);
            
            if(p.x == pz.x && p.y == pz.y)
            {
                result->points.push_back(pz);
                break;
            }
        }
    }
    return result;
}
bool ConcaveHull2::intersect(pcl::PointXYZI& a, pcl::PointXYZI& b, pcl::PointXYZI& c, pcl::PointXYZI& d)
{
    // line equations
    float A1 = b.y - a.y;
    float B1 = a.x -b.x;
    float C1 = A1*a.x + B1*a.y;
    
    float A2 = d.y - c.y;
    float B2 = c.x -d.x;
    float C2 = A2*c.x + B2*c.y;
    // det
    float det = A1*B2 - A2*B1;
    
    if(det == 0)
        return false;
    
// intersect point
    double x = (B2 * C1 - B1 * C2) / det;
    double y = (A1 * C2 - A2 * C1) / det;
    
    float minx1 = b.x;
    float maxx1 = a.x;
    if(a.x < b.x)
    {
        minx1 = a.x;
        maxx1 = b.x;
    }
    float miny1 = b.y;
    float maxy1 = a.y;
    if(a.y < b.y)
    {
        minx1 = a.y;
        maxx1 = b.y;
    }
    float minx2 = d.x;
    float maxx2 = c.x;
    if(c.x < d.x)
    {
        minx2 = c.x;
        maxx2 = d.x;
    }
    float miny2 = d.y;
    float maxy2 = c.y;
    if(c.y < d.y)
    {
        minx1 = c.y;
        maxx1 = d.y;
    }
    
    // check if point is in range of each line.
    //line1
    bool lineA=false;
    if(x>=minx1 && x<= maxx1 && y>= miny1 && y<=maxy1 )
        lineA = true;
    
    bool lineB=false;
    if(x>=minx2 && x<= maxx2 && y>= miny2 && y<=maxy2 )
        lineB = true;
    
    if( lineA == true&& lineB==true)
        return true;
    return false;
}
void ConcaveHull2::test(){
    pcl::PointXYZI a,b,c;
    a.x = 0;
    a.y = 0;
    a.z = 0;
    
    b.x = 0;
    b.y = 1;
    b.z = 0;
    
    c.x = 5;
    c.y = 5;
    c.z = 0;
    
   // std::cout<<std::fixed << std::setprecision(3)<< " a.x "<< a.x<< " a.y "<< a.y <<"\n";
    //std::cout<<std::fixed << std::setprecision(3)<< " b.x "<< b.x<< " b.y "<< b.y <<"\n";
    //std::cout<<std::fixed << std::setprecision(3)<< " c.x "<< c.x<< " c.y "<< c.y <<"\n";
    computeAngle(a, b, c);
    
}
int ConcaveHull2::orientation(pcl::PointXYZI& a, pcl::PointXYZI& b, pcl::PointXYZI& c)
{
    int val = (a.y - b.y) * (c.x - a.x) - (a.x - b.x) * (c.y - a.y);
    
      if (val == 0) return 0;  // colinear
      return (val > 0)? 1: 2; // clock or counterclock wise
}
void ConcaveHull2::removeDuplicateXYpoints(pcl::PointCloud<pcl::PointXYZI>::Ptr c,pcl::PointCloud<pcl::PointXYZI>::Ptr d)
{
    std::vector<bool> used(c->points.size(),false);
    for(int i = 0; i< c->points.size();i++)
    {
        if(used.at(i) == true)
            continue;
        
        for(int u=i; u < c->points.size();u++)
        {
            if(i == u)
                continue;
            if(c->points.at(i).x == c->points.at(u).x && c->points.at(i).y == c->points.at(u).y)
            {
                used.at(u)= true;
            }
        }
    }
    for(int e=0; e< used.size();e++)
    {
        if( used.at(e) == false)
            d->points.push_back(c->points.at(e));
    }
}
