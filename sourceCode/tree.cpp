//    This file is part of 3DFOREST  www.3dforest.eu
//
//    3DFOREST is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    3DFOREST is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with 3DFOREST.  If not, see <http://www.gnu.org/licenses/>.
//////////////////////////////////////////////////////////////////////

#include "tree.h"
#include "skeleton.h"
#include "sortiment.h"
#include "ComputeSortiment.h"
#include "qsm.h"
#include "hull.h"
#include "HoughTransform.h"
#include "LeastSquareregression.h"
#include <QtWidgets/QMessageBox>

//Tree
Tree::Tree(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name, QColor col)
: Cloud(cloud, name, col)
{
  pcl::getMinMax3D(*get_Cloud(),m_minp,m_maxp);
  QString a = QString("%1_dbh").arg(m_name);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>);
  m_dbhCloud = new Cloud(cloud_, a);

  QString aaaa = QString("%1_skeleton").arg(m_name);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_4(new pcl::PointCloud<pcl::PointXYZI>);
  //m_skeleton = new Cloud(cloud_4, aaaa);

  pcl::PointXYZI bod;
  bod.x = -1;
  bod.y = -1;
  bod.z = -1;

  m_pose = bod;
  m_dbh_HT = {-1,-1,-1,-1,-0.5};
  m_dbh_LSR = {-1,-1,-1,-1,-0.5};
    m_dbh_QSM = {-1,-1,-1,-1,-0.5};
  m_height = -1;
  m_lenght = -1;
  m_convexhull = 0;
  m_concavehull = 0;
  m_crown = 0;
  m_triangulatedConcaveHull = new pcl::PolygonMesh;

}
Tree::Tree (Cloud cloud)
: Cloud(cloud)
{
  pcl::getMinMax3D(*get_Cloud(),m_minp,m_maxp);
  QString a = QString("%1_dbh").arg(m_name);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>);
  m_dbhCloud = new Cloud(cloud_, a);

  QString aaaa = QString("%1_skeleton").arg(m_name);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_4(new pcl::PointCloud<pcl::PointXYZI>);
  //m_skeleton = new Cloud(cloud_4, aaaa);

  pcl::PointXYZI bod;
  bod.x = -1;
  bod.y = -1;
  bod.z = -1;

  m_pose = bod;
  m_dbh_HT = {-1,-1,-1,-1,-0.5};
  m_dbh_LSR = {-1,-1,-1,-1,-0.5};
    m_dbh_QSM = {-1,-1,-1,-1,-0.5};
  m_height = -1;
  m_lenght =-1;
  m_convexhull = 0;
  m_concavehull = 0;
  m_crown = 0;
  m_triangulatedConcaveHull = new pcl::PolygonMesh;
  set_length();
}
Tree Tree::operator=(Tree &kopie)
{
  Tree t(kopie);
  t.m_Cloud = kopie.m_Cloud;
  t.m_dbhCloud = kopie.m_dbhCloud;
  t.m_name = kopie.m_name;
  t.m_pose = kopie.m_pose;
  t.m_dbh_HT = kopie.m_dbh_HT;
  t.m_dbh_LSR = kopie.m_dbh_LSR;
  t.m_height = kopie.m_height;
  t.m_lenght = kopie.m_lenght;
  t.m_areaconvex = kopie.m_areaconvex;
  t.m_areaconcave = kopie.m_areaconcave;
  t.m_minp = kopie.m_minp;
  t.m_maxp = kopie.m_maxp;
  t.m_lmax = kopie.m_lmax;
  t.m_lmin = kopie.m_lmin;
  t.m_stemCurvature = kopie.m_stemCurvature;
  t.m_convexhull = kopie.m_convexhull;
  t.m_concavehull = kopie.m_concavehull;
  t.m_triangulatedConcaveHull = kopie.m_triangulatedConcaveHull;
  t.m_crown = kopie.m_crown;
  return t;
}
void Tree::set_height() //check if tree is connected to terrain!!!
{
  if(m_pose.x == -1 && m_pose.y == -1 && m_pose.z == -1 )
    m_height = -1;
  else
    m_height = m_maxp.z - m_pose.z;
}
void Tree::set_dbhCloud()
{
 // m_dbhCloud = new Cloud();
//points 10 CM around 1.3 m
  pcl::PointCloud<pcl::PointXYZI>::Ptr dbhCloud (new pcl::PointCloud<pcl::PointXYZI>);
  for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> >::const_iterator ith = get_Cloud()->points.begin(); ith != get_Cloud()->points.end(); ith++)
  {
    if (ith->z > (m_pose.z + 1.25) && ith->z < (m_pose.z + 1.35))
    {
      pcl::PointXYZI bod;
      bod.x =ith->x;
      bod.y =ith->y;
      bod.z =ith->z;
      bod.intensity = ith->intensity;

      dbhCloud->points.push_back(bod);
    }
  }
  if(dbhCloud->points.size() > 50)
  {
    //voxelize
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_fil (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (dbhCloud);
    sor.setLeafSize (0.001f, 0.001f, 0.001f);
    sor.filter (*cloud_fil);

  //save cloud
    QString a = QString("%1_dbh").arg(m_name);
   // QColor col = QColor(255,0,0);
    m_dbhCloud->set_Cloud(cloud_fil);
    cloud_fil.reset();
  }
  else
  {
    //save cloud
    QString a = QString("%1_dbh").arg(m_name);
    //QColor col = QColor(255,0,0);
    m_dbhCloud->set_Cloud(dbhCloud);
  }
  dbhCloud.reset();
}
void Tree::set_dbhCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  m_dbhCloud->set_Cloud(cloud);
}
void Tree::set_dbhHT(int i)
{
  if(m_pose.x == -1 && m_pose.y == -1 &&m_pose.z == -1 )
    return;

  if(m_dbhCloud->get_Cloud()->points.size() > 2)
  {
    HoughTransform ht = m_dbhCloud->get_Cloud();
    ht.set_iterations(i);
    ht.compute();
    m_dbh_HT = ht.get_circle();
  }
  else
  {
    m_dbh_HT = {-1,-1,-1,-1,-0.5};
  }
}
stred Tree::get_dbhHT()
{
  return m_dbh_HT;
}
stred Tree::get_dbhLSR()
{
  return m_dbh_LSR;
}
void Tree::set_dbhLSR()
{
  if(m_pose.x == -1 && m_pose.y == -1 &&m_pose.z == -1 )
    return;

  if(m_dbhCloud->get_Cloud()->points.size() > 2)
  {
    LeastSquaredRegression lsr;
    lsr.setCloud(m_dbhCloud->get_Cloud());
    lsr.compute();
    //m_dbh_LSR = lsr.kamihoDBH();
    m_dbh_LSR = lsr.getCircle();
  }
  else
  {
    m_dbh_LSR = {1,-1,-1,-1,-0.5};
  }
}
void Tree::set_position(int height)
{
  float h = (float)height/100;

  if(m_dbhCloud->get_Cloud()->points.size() > 0)
    m_dbhCloud->get_Cloud()->points.clear();
  std::vector<float> x_coor;
  std::vector<float> y_coor;

  for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> >::const_iterator ith = get_Cloud()->points.begin(); ith != get_Cloud()->points.end(); ith++)
  {
    if (ith->z < (m_minp.z + h) )
    {
      x_coor.push_back(ith->x);
      y_coor.push_back(ith->y);

    }
  }

  if( x_coor.size() > 1)
  {
    std::sort(x_coor.begin(),x_coor.end());
    std::sort(y_coor.begin(),y_coor.end());


    m_pose.x = x_coor.at(x_coor.size()/2);
    m_pose.y = y_coor.at(y_coor.size()/2);
    m_pose.z = m_minp.z;
    m_pose.intensity = 1;
  }
  else
  {
    m_pose = m_minp;
  }
  set_length();
  set_dbhCloud();

  if(m_dbh_HT.a!= -1 && m_dbh_HT.b!= -1 && m_dbh_HT.r!= -1 )
    set_dbhHT();
  if(m_dbh_LSR.a!= -1 && m_dbh_LSR.b!= -1 && m_dbh_LSR.r!= -1 )
    set_dbhLSR();
  if(m_height != -1)
    set_height();
  if(!m_stemCurvature.empty())
    set_stemCurvature();
}
void Tree::set_position(Cloud terrain, int num_points, int height)
{
  float  h = (float)height/100;
  if(m_dbhCloud->get_Cloud()->points.size() > 0)
    m_dbhCloud->get_Cloud()->points.clear();
  std::vector<float> x_coor;
  std::vector<float> y_coor;

  pcl::PointXYZI posit;
  posit = m_minp;
  for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> >::const_iterator ith = get_Cloud()->points.begin(); ith != get_Cloud()->points.end(); ith++)
  {
    if (ith->z < (m_minp.z + h) )
    {
      x_coor.push_back(ith->x);
      y_coor.push_back(ith->y);

    }
  }

  if( x_coor.size() > 1)
  {
    std::sort(x_coor.begin(),x_coor.end());
    std::sort(y_coor.begin(),y_coor.end());


    m_pose.x = x_coor.at(x_coor.size()/2);
    m_pose.y = y_coor.at(y_coor.size()/2);
    m_pose.z = m_minp.z;
    m_pose.intensity = 1;
  }
  else
  {
    m_pose = m_minp;
  }

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud (terrain.get_Cloud());

  std::vector<int> pointId(num_points);
  std::vector<float> pointSD(num_points);

  if (kdtree.nearestKSearch (m_pose, num_points, pointId, pointSD) > 0 )
  {
    float med_Z = 0;
    for(int i = 0; i < num_points; i++)
    {
      med_Z += terrain.get_Cloud()->points.at(pointId.at(i)).z;
    }

    m_pose.z = med_Z / num_points;
  }
  set_length();
  set_dbhCloud();
  if(m_dbh_HT.a!= -1 && m_dbh_HT.b!= -1 && m_dbh_HT.r!= -1 )
    set_dbhHT();
  if(m_dbh_LSR.a!= -1 && m_dbh_LSR.b!= -1 && m_dbh_LSR.r!= -1 )
    set_dbhLSR();
  if(m_height != -1)
    set_height();
  if(!m_stemCurvature.empty())
    set_stemCurvature();
}
pcl::PointXYZI Tree::get_pose()
{
  return m_pose;
}

float Tree::get_height()
{
  if(m_height == -1)
    return m_height;
  float AA = ceilf(m_height * 100) / 100; //zaokrouhleni
  return AA;
}
void Tree::set_length()
{

  pcl::PointXYZI pmin,pmax;
  m_lmin.x=9000000;
  m_lmin.y=9000000;
  m_lmin.z=9000000;
  m_lmax.x=-9000000;
  m_lmax.y=-9000000;
  m_lmax.z=-9000000;
  //najdi nejdelsi osu
  //X axis
  if (std::abs(m_maxp.x - m_minp.x) > std::abs(m_maxp.y - m_minp.y) && std::abs(m_maxp.x - m_minp.x) > std::abs(m_maxp.z - m_minp.z))
  {

    for(int j = 0; j< m_Cloud->points.size(); j++)
    {

    // pokud je rozdil bodu a m_maxp mensi nez 1
      if(m_maxp.x - m_Cloud->points.at(j).x < 1  && m_Cloud->points.at(j).x > m_lmax.x)
      {
        m_lmax= m_Cloud->points.at(j);
      }

      if(m_Cloud->points.at(j).x - m_minp.x < 1 && m_Cloud->points.at(j).x < m_lmin.x)
      {
        m_lmin = m_Cloud->points.at(j);
      }
    }
  }
  // Y axis
  else if ( (m_maxp.y - m_minp.y) > (m_maxp.x - m_minp.x) && (m_maxp.y - m_minp.y) > (m_maxp.z - m_minp.z))
  {

    for(int j = 0; j< m_Cloud->points.size(); j++)
    {

    // pokud je rozdil bodu a m_maxp mensi nez 10
      if(m_maxp.y - m_Cloud->points.at(j).y < 1  && m_Cloud->points.at(j).y > m_lmax.y)
      {
        m_lmax= m_Cloud->points.at(j);
      }

      if(m_Cloud->points.at(j).y - m_minp.y < 1 && m_Cloud->points.at(j).y < m_lmin.y)
      {
        m_lmin = m_Cloud->points.at(j);
      }
    }
  }
  else //Z axis
  {// nejdelsi je osa z

    for(int j = 0; j< m_Cloud->points.size(); j++)
    {
    // pokud je rozdil bodu a m_maxp mensi nez 10
      if(m_maxp.z - m_Cloud->points.at(j).z < 1  && m_Cloud->points.at(j).z > m_lmax.z)
      {
        m_lmax= m_Cloud->points.at(j);
      }

      if(m_Cloud->points.at(j).z - m_minp.z < 1 && m_Cloud->points.at(j).z < m_lmin.z)
      {
        m_lmin = m_Cloud->points.at(j);
      }
    }
  }
  //compute lenght
  m_lenght = sqrt((m_lmax.x - m_lmin.x)*(m_lmax.x - m_lmin.x) + (m_lmax.y - m_lmin.y)*(m_lmax.y - m_lmin.y) + (m_lmax.z - m_lmin.z)*(m_lmax.z - m_lmin.z));
}
float Tree::get_length()
{
  if(m_lenght == -1)
    return m_lenght;
  float AA = ceilf(m_lenght * 100) / 100; //zaokrouhleni
  return AA;
}
pcl::PointXYZI Tree::get_lpoint(bool low)
{
  if (low ==false)
    {return m_lmin;}
  else
    {return m_lmax;}
}
pcl::PointCloud<pcl::PointXYZI>::Ptr Tree::get_dbhCloud()
{
  return m_dbhCloud->get_Cloud();
}
// CONVEX & CONCAVE HULL
void Tree::setConvexhull()
{
  if(m_pose.x == -1 && m_pose.y == -1 && m_pose.z == -1 )
    return;
   QString aa = QString("%1_convex").arg(m_name);
    m_convexhull = new ConvexHull(CloudOperations::getCloudCopy(m_Cloud));
}
ConvexHull& Tree::getConvexhull()
{
    if(m_convexhull == 0)
    {
       setConvexhull();
    }
    return *m_convexhull;
}
void Tree::setConcavehull(float searchDist)
{
  if(m_pose.x == -1 && m_pose.y == -1 && m_pose.z == -1 )
    return;
   if(m_concavehull!=0)
    {
        delete m_concavehull;
    }
    QString aa = QString("%1_concave").arg(m_name);
    m_concavehull = new ConcaveHull(CloudOperations::getCloudCopy(m_Cloud),aa,searchDist);

    //TriangulatedPolygon *t = new TriangulatedPolygon(CloudOperations::getCloudCopy(m_concavehull->getPolygon().get_Cloud()));
   // TrianglesToPclMeshTransformation *m = new TrianglesToPclMeshTransformation(m_concavehull->getPolygon().get_Cloud());

    *m_triangulatedConcaveHull = CloudOperations::PolygonToMesh(m_concavehull->getPolygon().get_Cloud());
}
ConcaveHull& Tree::getConcavehull()
{
    if(m_concavehull == 0)
    {
        setConcavehull(1.0);
    }
    return *m_concavehull;
}
pcl::PolygonMesh Tree::getTriangulatedConcaveHull()
{
    return *m_triangulatedConcaveHull;
}
float Tree::getConvexAreaToInfoLine()
{
    if(m_convexhull == 0)
    return 0;
    else return m_convexhull->getPolygonArea();
}
float Tree::getConcaveAreaToInfoLine()
{
    if(m_concavehull == 0)
    return 0;
    else return m_concavehull->getPolygonArea();
}
//Skeleton
void Tree::set_skeleton()
{

}
void Tree::set_skeleton(Cloud c)
{
  //m_skeleton->set_Cloud(c.get_Cloud());
}
Cloud Tree::get_skeleton()
{
  return *m_dbhCloud;
}
void Tree::set_positionHT(int iter)
{
  if(m_dbhCloud->get_Cloud()->points.size() > 0)
    m_dbhCloud->get_Cloud()->points.clear();
// vypocitat stred v 1,3 (m_dbh) a v 0,65 m nad pozici

  stred c13;
  bool c13_exist,c065_exist;
  c13_exist = c065_exist = false;
  stred c065;
  pcl::PointXYZI posit;

  if(m_pose.x == -1 && m_pose.y == -1 && m_pose.z == -1 )
    posit = m_minp;
  else
    posit = m_pose;


  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud13 (new pcl::PointCloud<pcl::PointXYZI>);
  for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> >::const_iterator it = m_Cloud->points.begin(); it != m_Cloud->points.end(); it++)
  {
    if (it->z > (posit.z + 1.2) && it->z < (posit.z + 1.4))
    {
      pcl::PointXYZI bod;
      bod.x =it->x;
      bod.y =it->y;
      bod.z =it->z;
      bod.intensity = it->intensity;

      cloud13->points.push_back(bod);
    }
  }

  if(cloud13->points.size() > 5)
  {
    HoughTransform ht13;
    ht13.set_Cloud(cloud13);
    ht13.set_iterations(iter);
    ht13.compute();
    c13 = ht13.get_circle();
    c13_exist = true;
  }

  //set cloud in 065 m above position
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud065 (new pcl::PointCloud<pcl::PointXYZI>);

  for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> >::const_iterator ith = get_Cloud()->points.begin(); ith != get_Cloud()->points.end(); ith++)
  {
    if (ith->z > (posit.z + 0.6) && ith->z < (posit.z + 0.7))
    {
      pcl::PointXYZI bod;
      bod.x =ith->x;
      bod.y =ith->y;
      bod.z =ith->z;
      bod.intensity = ith->intensity;

      cloud065->points.push_back(bod);
    }
  }

  // calculate circle
  if(cloud065->points.size() > 5)
  {
    HoughTransform ht065;
    ht065.set_Cloud(cloud065);
    ht065.set_iterations(iter);
    ht065.compute();
    c065 = ht065.get_circle();
    c065_exist = true;
  }

  float vx, vy, vz;
//if exist both circles
  if(c13_exist == true && c065_exist == true)
  {
    // urcit prusecik
    //parametricka primka
    vx = c065.a - c13.a;
    vy = c065.b - c13.b;
    vz = c065.z - c13.z;
  }
  else if(c13_exist != true && c065_exist == true)
  {
    pcl::PointXYZI pose;
    pose.x = c065.a;
    pose.y = c065.b;
    pose.z = m_minp.z;

    m_pose = pose;
    set_length();
    set_dbhCloud();
    return;
  }
  else if(c13_exist == true && c065_exist != true)
  {
    pcl::PointXYZI pose;
    pose.x = c13.a;
    pose.y = c13.b;
    pose.z = m_minp.z;

    m_pose = pose;
    set_length();
    set_dbhCloud();
    return;
  }
  else
  {
    QMessageBox::information(0, m_name,("Too few points were found for estimation of position using Randomized Hough Transform. Using lowest points method."));
    set_position();
    return;
  }
  // rovina
    // najit nejnizsi bod stromu a prolozit rovinu
  pcl::PointXYZ A,B,C;
  A.x = m_minp.x+1;
  A.y = m_minp.y+1;
  A.z = m_minp.z;

  B.x = m_minp.x-1;
  B.y = m_minp.y+2;
  B.z = m_minp.z;

  C.x = m_minp.x;
  C.y = m_minp.y;
  C.z = m_minp.z;


    //vytvoøit rovnici roviny
  pcl::PointXYZ AB;
  AB.x =B.x - A.x;
  AB.y =B.y - A.y;
  AB.z =B.z - A.z;

  pcl::PointXYZ AC;
  AC.x =C.x - A.x;
  AC.y =C.y - A.y;
  AC.z =C.z - A.z;

  float a = (AB.y*AC.z) - (AB.z*AC.y);
  float b = (AB.z*AC.x) - (AB.x*AC.z);
  float c = (AB.x*AC.y) - (AB.y*AC.x);
  float d = -(a*A.x) - (b*A.y) - (c*A.z);
  float up = -d - (a*c13.a) - (b*c13.b) - (c*c13.z);

  float down =  a*vx + b*vy + c*vz;
  if(down == 0)
  {
    QMessageBox::information(0,("df"),("rovnobezne" ));
    return;
  }
  float t = up/down;
    //dosadit do rovnic primky
  pcl::PointXYZI pos;

  pos.x = c13.a +t*vx;
  pos.y = c13.b +t*vy;
  pos.z = m_minp.z;
  pos.intensity = 1;
  m_pose = pos;

  set_length();
  set_dbhCloud();
  if(m_dbh_HT.a!= -1 && m_dbh_HT.b!= -1 && m_dbh_HT.r!= -1 )
    set_dbhHT();
  if(m_dbh_LSR.a!= -1 && m_dbh_LSR.b!= -1 && m_dbh_LSR.r!= -1 )
    set_dbhLSR();
  if(m_height != -1)
    set_height();
  if(!m_stemCurvature.empty())
    set_stemCurvature();
}
void Tree::set_positionHT(Cloud terrain, int iter,  int num_points)
{
  if(m_dbhCloud->get_Cloud()->points.size() > 0)
    m_dbhCloud->get_Cloud()->points.clear();
// vypocitat stred v 1,3 (m_dbh) a v 0,65 m nad pozici
  stred c13,c065;
  bool c13_exist,c065_exist;
  pcl::PointXYZI posit;
  c13_exist = c065_exist = false;

  if(m_pose.x == -1 && m_pose.y == -1 && m_pose.z == -1 )
    posit = m_minp;
  else
    posit = m_pose;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud13 (new pcl::PointCloud<pcl::PointXYZI>);
  for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> >::const_iterator it = m_Cloud->points.begin(); it != m_Cloud->points.end(); it++)
  {
    if (it->z > (posit.z + 1.25) && it->z < (posit.z + 1.45))
    {
      pcl::PointXYZI bod;
      bod.x =it->x;
      bod.y =it->y;
      bod.z =it->z;
      bod.intensity = it->intensity;

      cloud13->points.push_back(bod);
    }
  }
  if(cloud13->points.size() > 5)
  {
    HoughTransform ht13;
    ht13.set_Cloud(cloud13);
    ht13.set_iterations(iter);
    ht13.compute();
    c13 = ht13.get_circle();
    c13_exist = true;
  }
  cloud13.reset();
  //set cloud in 065 m above position
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud065 (new pcl::PointCloud<pcl::PointXYZI>);
  for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> >::const_iterator ith = get_Cloud()->points.begin(); ith != get_Cloud()->points.end(); ith++)
  {
    if (ith->z > (posit.z + 0.6) && ith->z < (posit.z + 0.7))
    {
      pcl::PointXYZI bod;
      bod.x =ith->x;
      bod.y =ith->y;
      bod.z =ith->z;
      bod.intensity = ith->intensity;

      cloud065->points.push_back(bod);
    }
  }

  // calculate circle
  if(cloud065->points.size() > 5)
  {
    HoughTransform ht065;
    ht065.set_Cloud(cloud065);
    ht065.set_iterations(iter);
    ht065.compute();
    c065 = ht065.get_circle();
    c065_exist = true;
  }
  cloud065.reset();

  float vx, vy, vz;
  pcl::PointXYZI pose;
//if exist both circles
  if(c13_exist == true && c065_exist == true)
  {
    // urcit prusecik
    //parametricka primka
    vx = c065.a - c13.a;
    vy = c065.b - c13.b;
    vz = c065.z - c13.z;
  }
  else if(c13_exist != true && c065_exist == true)
  {
    pose.x = c065.a;
    pose.y = c065.b;
    pose.intensity = 1;
  }
  else if(c13_exist == true && c065_exist != true)
  {
    pose.x = c13.a;
    pose.y = c13.b;
    pose.intensity = 1;
  }
  else
  {
    QMessageBox::information(0, m_name, ("Too few points were found for estimation of position using Hough Transform. Using lowest points method."));
    set_position(terrain);
    return;
  }
  // rovina
   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZI>);
  for(int e = 0; e < terrain.get_Cloud()->points.size(); e++)
  {
    pcl::PointXYZI a;
    a.x = terrain.get_Cloud()->points.at(e).x;
    a.y = terrain.get_Cloud()->points.at(e).y;
    a.z = m_minp.z;
    cloud_tmp->points.push_back(a);
  }
    // najit tøi nejbližší body terénu
  pcl::PointXYZ A,B,C;
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud (cloud_tmp);

  std::vector<int> pointId(num_points);
  std::vector<float> pointSD(num_points);
  float z = 0;
  if (kdtree.nearestKSearch (posit, num_points, pointId, pointSD) > 0 )
  {
    for(int i =0; i < num_points; i++)
    {
      z += terrain.get_Cloud()->points.at(pointId.at(i)).z;
    }
    z/=num_points;

  A.x = m_minp.x+1;
  A.y = m_minp.y+1;
  A.z = z;

  B.x = m_minp.x-1;
  B.y = m_minp.y+2;
  B.z = z;

  C.x = m_minp.x;
  C.y = m_minp.y;
  C.z = z;

  }
    else
  {
    QMessageBox::information(0,("tr"),("No terrain point found in selected cloud. using calculation without terrain cloud."));
    set_positionHT();
    return;
  }

    //vytvoøit rovnici roviny
  pcl::PointXYZ AB;
  AB.x =B.x - A.x;
  AB.y =B.y - A.y;
  AB.z =B.z - A.z;

  pcl::PointXYZ AC;
  AC.x =C.x - A.x;
  AC.y =C.y - A.y;
  AC.z =C.z - A.z;

  float a = (AB.y*AC.z) - (AB.z*AC.y);
  float b = (AB.z*AC.x) - (AB.x*AC.z);
  float c = (AB.x*AC.y) - (AB.y*AC.x);
  float d = -(a*A.x) - (b*A.y) - (c*A.z);

  if(c13_exist == true && c065_exist == true)
  {
    //vypocitat t
  float up = -d - (a*c13.a) - (b*c13.b) - (c*c13.z);
  float down =  a*vx + b*vy + c*vz;
  if(down == 0)
  {
    QMessageBox::information(0,("df"),("rovnobezne" ));
    return;
  }
  float t = up/down;
    //dosadit do rovnic primky
  pcl::PointXYZI pos;
  pos.x = c13.a +t*vx;
  pos.y = c13.b +t*vy;

  pos.z = (A.z + B.z + C.z)/3;
  pos.intensity = 1;

  m_pose = pos;
  set_length();
  set_dbhCloud();
  }
  else if(c13_exist != true && c065_exist == true)
  {
    pose.z = (A.z + B.z + C.z)/3;
    m_pose = pose;
    set_length();
    set_dbhCloud();
  }
  else if(c13_exist == true && c065_exist != true)
  {
    pose.z = (A.z + B.z + C.z)/3;

    m_pose = pose;
    set_length();
    set_dbhCloud();
  }
  if(m_dbh_HT.a!= -1 && m_dbh_HT.b!= -1 && m_dbh_HT.r!= -1 )
    set_dbhHT();
  if(m_dbh_LSR.a!= -1 && m_dbh_LSR.b!= -1 && m_dbh_LSR.r!= -1 )
    set_dbhLSR();
  if(m_height != -1)
    set_height();
  if(!m_stemCurvature.empty())
    set_stemCurvature();
}
void Tree::set_stemCurvature(int iter, int height)
{
  if(m_pose.x == -1 && m_pose.y == -1 &&m_pose.z == -1 )
    return;
  m_stemCurvature.clear();
  float section = (float)height/100.0;
  for(float g = 0.00; g < (m_maxp.z - m_minp.z); g += section)
  {

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ (new pcl::PointCloud<pcl::PointXYZI>);
    //vybrat body do mracna ktere jsou
    for(int i = 0; i < get_Cloud()->points.size(); i++)
    {
      pcl::PointXYZI ith;
      ith = get_Cloud()->points.at(i);
      if (ith.z > (m_pose.z + g - 0.035) && ith.z < (m_pose.z + g + 0.035))// && m_dbh_HT.a -ith.x <5 &&  ith.x - m_dbh_HT.a < 5&& m_dbh_HT.b -ith.y <5 &&   ith.y - m_dbh_HT.b < 5)
      {
        cloud_->points.push_back(ith);
      }
    }
    stred res;
// if the cloud is empty
    if(cloud_->points.size() > 5)
    {
      HoughTransform *ht = new HoughTransform();
      ht->set_Cloud(cloud_);
      ht->set_iterations(iter);
      ht->compute();
      res = ht->get_circle();
      delete ht;
    }
    else
      res = {-1,-1,-1,-1,-0.5};

// if the circle is two times greater than previous two circles
    if(m_stemCurvature.size() >= 2 && res.r > (2* m_stemCurvature.at(m_stemCurvature.size()-2).r) && res.r > (2* m_stemCurvature.at(m_stemCurvature.size()-1).r))
      res = {-1,-1,-1,-1,-0.5};

    m_stemCurvature.push_back(res);
    cloud_.reset();
  }
}
std::vector<stred> Tree::get_stemCurvature()
{
  return m_stemCurvature;
}
//CROWN
void Tree::set_TreeCrownAutomatic()
{
  if((m_pose.x == -1 && m_pose.y == -1 && m_pose.z == -1) || m_height == -1)
    return;

  if(m_crown != 0){delete m_crown;}
    //create new crown object and set as tree crown
  CrownAutomaticDetection *ad = new CrownAutomaticDetection(CloudOperations::getCloudCopy(m_Cloud),get_dbhLSR(),m_pose);
  QString name = QString("%1_crown").arg(m_name);
  m_crown = new Crown(ad->getCrown(),name,m_pose,ad->getStemHighestPoint());
}
void Tree::set_TreeCrownManual(pcl::PointCloud<pcl::PointXYZI>::Ptr crown,pcl::PointCloud<pcl::PointXYZI>::Ptr stem)
{
    if(m_crown != 0){delete m_crown;}
    //create new crown object and set as tree crown
    QString name = QString("%1_crown").arg(m_name);
    cloudHighestAndLowestZValue hl = GeomCalc::findHighestAndLowestPoints(stem);
    m_crown = new Crown(crown,name,m_pose,hl.highestPoint);
}
Crown& Tree::get_TreeCrown()
{
    return *m_crown;
}
bool Tree::isCrownExist()
{
    if(m_crown == 0)
    {
        return false;
    }else return true;
}

//PRIVATE
pcl::PointXYZI Tree::getMinP()
{
    return m_minp;
}
pcl::PointXYZI Tree::getMaxP()
{
    return m_maxp;
}
pcl::PointXYZI Tree::getMinL()
{
    return m_lmin;
}
pcl::PointXYZI Tree::getMaxL()
{
    return m_lmax;
}

void Tree::setSkeleton(float voxelSize, float multiplicator)
{
    Skeleton* s = new Skeleton(m_Cloud,  voxelSize, multiplicator);
    s->setPosition(get_pose());
    s->compute();
    setSegments(s->getSegments());
    set_Cloud(s->getCloud());
    delete s;
}

void Tree::setSegments(std::vector<std::shared_ptr<Segment>> branches)
{
    m_branches = branches;
}
std::vector<std::shared_ptr<Segment>> Tree::getBranches()
{
    return m_branches;
}

void Tree::setQSM(int iterations, float CylinderHeight, float limit, float branchLength, int order, bool stemCurve)
{
    CylinderModel * c = new CylinderModel(getBranches(), iterations, CylinderHeight);
    c->setTreecloud (m_Cloud);
    c->setLimit(limit);
    c->setTreeHeight(get_length());
    c->setTreePosition(get_pose());
    c->setBranchLength(branchLength);
    c->setOrder(order);
    c->compute();
    setCylinders(c->getCylinders());
    setVolume();
    setDBH_QSM();
    if(stemCurve == true)
    {
        m_stemCurvature.clear();
       m_stemCurvature = c->getStemCurve();
    }
    delete c;
}
void Tree::setDBH_QSM()
{
    // find branch with order 0
    float distance =1000000000;
    for(int i=0; i < m_branches.size();i++)
    {
        if(m_branches.at(i)->getOrder() ==0)
        {
            // get height
            for(int q=0; q < m_branches.at(i)->getCylinders().size();q++)
            {
                if(std::abs(m_branches.at(i)->getCylinders().at(q)->values.at(2) - (m_pose.z + 1.3)) < distance)
                {
                    distance = std::abs(m_branches.at(i)->getCylinders().at(q)->values.at(2) - (m_pose.z + 1.3));
                   // std::cout<<"distance " << distance << " q: "<< q <<" z celkoveho poctu " <<m_branches.at(i)->getCylinders().size()<<  "\n";
                    m_dbh_QSM.a =m_branches.at(i)->getCylinders().at(q)->values.at(0);
                    m_dbh_QSM.b =m_branches.at(i)->getCylinders().at(q)->values.at(1);
                    m_dbh_QSM.z =m_branches.at(i)->getCylinders().at(q)->values.at(2);
                    m_dbh_QSM.r =m_branches.at(i)->getCylinders().at(q)->values.at(6)*100;

                }
            }
            break;
        }
    }
}
stred Tree::getDBH_QSM()
{
    return m_dbh_QSM;
}
void Tree::setSortiment( bool finalSortiment)
{
    ComputeSortiment * s = new ComputeSortiment(getBranches(), finalSortiment);
    s->compute();
    //std::cout<<"pocet cylindru: " << getCylinderSize()<< "\n";
//    if(finalSortiment == false)
//    {
//        setCylinders(s->getCylinders());
//        setSortiments(s->getSortiments());
//    }
//    else{
//        for(int i=1; i < 7; i++)
//        {
//            setSortimentVolume(i, s->getSortimentVolume(i));
//            setSortimentLenght(i, s->getSortimentLenght(i));
//        }
//       // setCylinders(s->getCylinders());
//        //setSortiments(s->getSortiments());
//    }
    
   // std::cout<<"pocet cylindru: " << getCylinderSize()<< "\n";
   // std::cout<<"pocet sortimentu: " << getSortimentsSize()<< "\n";
    
    delete s;
}
void Tree::setSortiments(std::vector<int> s)
{
    m_sortiments = s;
}
int Tree::getSortimentsSize()
{
    int vel=0;
    for(int i=0; i < m_branches.size();i++)
      vel += m_branches.at(i)->getSortiments().size();
    return vel;
}
std::vector< std::shared_ptr <Sortiment>> Tree::getSortiments()
{
    std::vector< std::shared_ptr <Sortiment>> sortiments;
    for(int i=0; i < m_branches.size(); i++)
    {
        sortiments.insert(sortiments.end(), m_branches.at(i)->getSortiments().begin(), m_branches.at(i)->getSortiments().end());
    }
    return sortiments;
}
int Tree::getSortiment(int i)
{
    return m_sortiments.at(i);
}

void Tree::setVolume()
{
    m_volume = 0;
    // pro kazdy cylinder compute volume, and add to the m_volume
    for(int i = 0; i < m_cylinders.size(); i++)
    {
        m_volume+= cylinderVolume(m_cylinders.at(i));
    }
    
    std::cout<< "strom: "<< get_name().toStdString() << " volume: " << m_volume << "\n";
}
float Tree::cylinderVolume (pcl::ModelCoefficients::Ptr model )
{
    //m_PI * radius* radius * v
   
    float v = 0;
    float volume = 0;
    v = std::sqrt(
                  (model->values.at(3) * model->values.at(3)) +
                  (model->values.at(4) * model->values.at(4)) +
                  (model->values.at(5) * model->values.at(5)) );
    
    volume =(M_PI * (model->values.at(6) * model->values.at(6)) * v);
    
    if(std::isnan(v))
    {
        v=0;
        volume = 0;
    }
  //  std::cout<<"valce hodnoty: x: "<<model->values.at(3) << " y: " << model->values.at(4)<< " z: " << model->values.at(5)<< "\n";
//std::cout<< "prumer: " << model->values.at(6) << " vyska: "<< v <<" volume: "<<  volume<< " m_volume: "<< m_volume<< "\n";
    
    return volume;
}
float Tree::getVolume()
{
    return m_volume;
}
void Tree::setCylinder(pcl::ModelCoefficients::Ptr cylinder)
{
    m_cylinders.push_back(cylinder);
}
pcl::ModelCoefficients::Ptr Tree::getCylinder (int i)
{
    return m_cylinders.at(i);
}
int Tree::getCylinderSize()
{
    return m_cylinders.size();
}
void Tree::setCylinders(std::vector< pcl::ModelCoefficients::Ptr > cylinders)
{
    m_cylinders.clear();
    m_cylinders = cylinders;
}
float Tree::getSortimentVolume(int sortimentID)
{
    switch (sortimentID) {
        case 1:
            return m_sortiment1_volume;
            break;
        case 2:
            return m_sortiment2_volume;
            break;
        case 3:
            return m_sortiment3_volume;
            break;
        case 4:
            return m_sortiment4_volume;
            break;
        case 5:
            return m_sortiment5_volume;
            break;
        case 6:
            return m_sortiment6_volume;
            break;
            
        default:
            return -1;
            break;
    }
}
void Tree::setSortimentVolume(int sortimentID, float volume)
{
    switch (sortimentID) {
        case 1:
            m_sortiment1_volume = volume;
            break;
        case 2:
            m_sortiment2_volume = volume;
            break;
        case 3:
            m_sortiment3_volume = volume;
            break;
        case 4:
            m_sortiment4_volume = volume;
            break;
        case 5:
            m_sortiment5_volume = volume;
            break;
        case 6:
            m_sortiment6_volume = volume;
            break;
            
        default:
            m_sortiment1_lenght = -1;
            break;
    }
}
float Tree::getSortimentLenght(int sortimentID)
{
    switch (sortimentID) {
        case 1:
            return m_sortiment1_lenght;
            break;
        case 2:
            return m_sortiment2_lenght;
            break;
        case 3:
            return m_sortiment3_lenght;
            break;
        case 4:
            return m_sortiment4_lenght;
            break;
        case 5:
            return m_sortiment5_lenght;
            break;
        case 6:
            return m_sortiment6_lenght;
            break;
            
        default:
            return -1;
            break;
    }
}
void Tree::setSortimentLenght(int sortimentID, float lenght)
{
    switch (sortimentID) {
        case 1:
            m_sortiment1_lenght = lenght;
            break;
        case 2:
            m_sortiment2_lenght = lenght;
            break;
        case 3:
            m_sortiment3_lenght = lenght;
            break;
        case 4:
            m_sortiment4_lenght = lenght;
            break;
        case 5:
            m_sortiment5_lenght = lenght;
            break;
        case 6:
            m_sortiment6_lenght = lenght;
            break;
            
        default:
            m_sortiment1_lenght = -1;
            break;
    }
}
float Tree::getQSMDBH()
{
    return m_dbh_QSM.r * 20;
}
float Tree::getQSMVolume()
{
    float volume=0;
    for(int q=0; q < m_branches.size(); q++)
        volume+= m_branches.at(q)->getTotalVolume();
    return volume;
}
float Tree::getQSMVolumeHroubi()
{
    float volume=0;
    for(int q=0; q < m_branches.size(); q++)
        volume+= m_branches.at(q)->getHroubiVolume();
    return volume;
}
