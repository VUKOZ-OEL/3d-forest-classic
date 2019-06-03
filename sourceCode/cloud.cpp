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

#include "cloud.h"


//CLOUD
Cloud::Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name)
{
  m_name = name;
  m_Cloud=cloud;
  std::srand(time(0));
  set_color( QColor( rand()%255, rand()%255, rand()%255));
  m_PointSize=1;
}
Cloud::Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name,QColor col)
{
  m_name = name;
  m_Cloud=cloud;
  set_color(col);
  m_PointSize=1;
}
Cloud::Cloud()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  m_name = "";
  m_Cloud=cloud;
  m_PointSize=1;
  }

Cloud::Cloud(const Cloud& kopie)
{
  m_Cloud = kopie.m_Cloud;
  m_name = kopie.m_name;
  m_color = kopie.m_color;
  m_PointSize = kopie.m_PointSize;
}

Cloud::~Cloud()
{
m_Cloud.reset();
}
Cloud Cloud::operator=(Cloud &kopie)
{
  Cloud t;
  t.set_name( kopie.get_name());
  t.set_Cloud(kopie.get_Cloud());
  t.set_color( kopie.get_color());
  t.set_Psize(kopie.get_Psize());
  return t;
}

void Cloud::set_color (QColor col)
{
    m_color = col;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud::get_Cloud()
{
  return m_Cloud;
}
QString Cloud::get_name()
{
  return m_name;
}
QColor Cloud::get_color()
{
  return m_color;
}
void Cloud::set_Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  *m_Cloud = *cloud;
}
void Cloud::set_name(QString name)
{
  m_name = name;
}
void Cloud::set_Psize (int p)
{
  m_PointSize = p;
}
int Cloud::get_Psize()
{
  return m_PointSize;
}
