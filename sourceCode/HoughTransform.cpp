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

#include "HoughTransform.h"
#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>

HoughTransform::HoughTransform()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  m_cloud = cloud;
  m_it= 200;
 // m_circle = {-1,-1,-1,-1,-0.5};
}
HoughTransform::HoughTransform(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  m_cloud = cloud;
  m_it= 200;
  //m_circle = {-1,-1,-1,-1,-0.5};
}
HoughTransform::~HoughTransform()
{

}
void HoughTransform::set_Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  m_cloud = cloud;
}
void  HoughTransform::compute()
{
  std::vector<float> acc_x(m_it,0);
  std::vector<int> acc_xc(m_it,0);
  std::vector<float> acc_y(m_it,0);
  std::vector<int> acc_yc(m_it,0);
  std::vector<int> acc_r(m_it,0);
  std::vector<int> acc_rc(m_it,0);

  boost::mt19937 rng;
  boost::uniform_int<> six(0,m_cloud->points.size() -1);
  boost::variate_generator<boost::mt19937&, boost::uniform_int<> >  die(rng, six);             // glues randomness with mapping

    //#pragma omp parallel for
  for(int xa = 0; xa < m_it; xa++)
  {
    int r1 =  die();
    int r2 =  die();
    int r3 =  die();

    while (r1 == r2)
    {
      r2 = die();
    }

    while (r1 == r3)
    {
      r3 = die();
    }

    while (r2 == r3)
    {
      r3 = die();
    }
      // vybrat nahodne tri body
    pcl::PointXYZI p1,p2,p3;
    p1 =m_cloud->points.at(r1);
    p2 =m_cloud->points.at(r2);
    p3 =m_cloud->points.at(r3);


// m1 - bod uprostřed (p1,p2), osa ním prochází
    float f1 = (p2.x - p1.x) / (p1.y - p2.y);
    float m1x = (p1.x + p2.x)/2;
    float m1y = (p1.y + p2.y)/2;
    float g1 = m1y - f1*m1x;

    float f2 = (p3.x - p2.x) / (p2.y - p3.y);
    float m2x = (p2.x + p3.x)/2;
    float m2y = (p2.y + p3.y)/2;
    float g2 = m2y - f2*m2x;

   // ošetření degenerovaných případů
   // - tři body na přímce
    float retx,rety;
    int radius;
    if     (f1 == f2)
      continue;
    else if(p1.y == p2.y)
    {
      retx = m1x;
      rety = f1*retx + g1;
      radius = ceil(sqrt((retx-p1.x)*(retx-p1.x) + (rety-p1.y)*(rety-p1.y))*100000)/100.0;
    }
    else if(p2.y == p3.y)
    {
      retx = m2x;
      rety = f1*retx + g1;
      radius = ceil(sqrt((retx-p1.x)*(retx-p1.x) + (rety-p1.y)*(rety-p1.y))*100000)/100.0;
    }
    else
    {
      retx = (g2-g1) / (f1 - f2);
      rety = f1*retx + g1;
      radius = ceil(sqrt((retx-p1.x)*(retx-p1.x) + (rety-p1.y)*(rety-p1.y))*100000)/100.0;
    }
      //jen ulozit
    acc_x.at(xa) = retx;
    acc_y.at(xa)=rety;
    acc_r.at(xa)=radius;
  }


//ACCUULATOR x

  for(int i =0; i < acc_x.size();i++)
  {
    int m=0;
    float qq = acc_x.at(i);
      //#pragma omp parallel for
    for(int j =0; j < acc_x.size();j++)
    {
      float ww = acc_x.at(j);
      if( std::fabs(ww) - std::fabs(qq) >-0.005 && std::fabs(ww) - std::fabs(qq) < 0.005)
      {
        m++;
      }
    }
    acc_xc.at(i) = m;
  }

  int pos_x=0;
  int nej_x = 0;
  for(int k = 0; k < acc_xc.size(); k++)
  {
    int ma = acc_xc.at(k);
    if( ma > nej_x)
    {
      nej_x = ma;
      pos_x = k;
    }
  }
//ACCUULATOR y

  for(int i =0; i < acc_y.size();i++)
  {//ACCUULATOR x
    for(int e =0; e < acc_x.size();e++)
    {
      int m=0;
      float qq = acc_x.at(i);
      //#pragma omp parallel for
      for(int j =0; j < acc_x.size();j++)
      {
        float ww = acc_x.at(j);
        if( std::fabs(ww) - std::fabs(qq) >-0.0049 && std::fabs(ww) - std::fabs(qq) < 0.0049)
        {
          m++;
        }
      }
      acc_xc.at(e) = m;
    }
    int pos_m=0;
    int nej_m = 0;
    for(int k = 0; k < acc_xc.size(); k++)
    {
      int ma = acc_xc.at(k);
      if( ma > nej_m)
      {
        nej_m = ma;
        pos_m = k;
      }
    }
    int m=0;
    float qq = acc_y.at(i);

      //#pragma omp parallel for
    for(int j =0; j < acc_y.size();j++)
    {
      float ww = acc_y.at(j);

      if( std::fabs(ww) - std::fabs(qq) >-0.0049 && std::fabs(ww) - std::fabs(qq) < 0.0049)
      {
        m++;
      }
    }
    acc_yc.at(i) = m;
  }

  int pos_y=0;
  int nej_y = 0;
  for(int k = 0; k < acc_yc.size(); k++)
  {
    int ma = acc_yc.at(k);
    if( ma > nej_y)
    {
      nej_y = ma;
      pos_y = k;
    }
  }
//ACCUULATOR radius

  for(int i =0; i < acc_r.size();i++)
  {
    int m=0;
    float qq = acc_r.at(i);

      //#pragma omp parallel for
    for(int j =0; j < acc_r.size();j++)
    {
      float ww = acc_r.at(j);
      if( std::fabs(ww) - std::fabs(qq) > -5 && std::fabs(ww) - std::fabs(qq) < 5)
      {
        m++;
      }
    }
    acc_rc.at(i) = m;
  }

  int pos_r=0;
  int nej_r = 0;
  for(int k = 0; k < acc_rc.size(); k++)
  {
    int ma = acc_rc.at(k);
    if( ma > nej_r)
    {
      nej_r = ma;
      pos_r = k;
    }
  }
  float rad_f = acc_r.at(pos_r)/10.0;
  if(rad_f < 0)
      rad_f = -0.5;

  pcl::PointXYZI minp,maxp;
  pcl::getMinMax3D(*m_cloud,minp,maxp);

  float z_coord = (maxp.z + minp.z)/2;
  stred c = {acc_x.at(pos_x),acc_y.at(pos_y),z_coord,1,rad_f};
  m_circle = c;
}

stred HoughTransform::get_circle()
{
  return m_circle;
}
void  HoughTransform::set_iterations( int i)
{
  m_it = i;
}
