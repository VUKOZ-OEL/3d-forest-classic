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
#include "project.h"
#include "cloud.h"
#include "hull.h"

//include INPUT OUTPUT
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <iostream>
//#include <string>
#include <QtCore/qstring.h>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QGroupBox>




//Projekt a jeho metody
Project::Project()
{
m_x=0;
m_y=0; //coordinate system
m_z=0;
m_projectName = "default";
m_path = "c:/";
}
Project::~Project()
{

}
Project::Project( QString name)
{
m_x=0;
m_y=0; //coordinate system
m_z=0;
m_projectName = name;
}
Project::Project(double x, double y, QString name)
{
m_x=x;
m_y=y; //coordinate system
m_z=0;
m_projectName = name;

}
Project::Project(double x, double y, double z, QString name)
{
m_x=x;
m_y=y; //coordinate system
m_z=z;
m_projectName = name;
}
void Project::cleanAll()
{
  m_baseCloud.clear();
  m_terrainCloud.clear();
  m_vegeCloud.clear();
  m_ostCloud.clear();
  m_stromy.clear();
}
QString Project::get_ProjName()
{
return m_projectName;
}
void Project::set_xTransform(double x)
{
  m_x=x;
}
void Project::set_yTransform(double y)
{
  m_y=y;
}
void Project::set_zTransform(double z)
{
  m_z=z;
}
void Project::set_baseCloud(Cloud cloud)
{
  m_baseCloud.push_back(cloud);
}
void Project::set_path(QString path)
{
  m_path = path;
}
void Project::set_TerrainCloud(Cloud cloud)
{
  m_terrainCloud.push_back(cloud);
}
void Project::set_TerrainCloudat(int i, Cloud cloud)
{
  m_terrainCloud.at(i) = cloud;
}
void Project::set_TreeCloudat(int i, Cloud cloud)
{
  Tree t (cloud);
  m_stromy.at(i) = t;
}
void Project::set_VegeCloud(Cloud cloud)
{
  m_vegeCloud.push_back(cloud);
}
void Project::set_Tree(Cloud cloud)
{
  Tree t (cloud);
  m_stromy.push_back(t);
}
void Project::set_dbhCloud(QString name,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      m_stromy.at(i).set_dbhCloud(cloud);
    }
  }
}
void Project::setSkeleton(QString name, float voxelSize, float multiplicator)
{
    #pragma omp parallel for
    for(int i = 0; i< m_stromy.size(); i++)
    {
        if (get_TreeCloud(i).get_name() == name)
        {
            m_stromy.at(i).setSkeleton(voxelSize, multiplicator);
        }
    }
}
void Project::setCylinders(QString name, int iterations, float CylinderHeight, float limit, float branchLength, int order, bool stemCurve)
{
#pragma omp parallel for
    for(int i = 0; i< m_stromy.size(); i++)
    {
        if (get_TreeCloud(i).get_name() == name)
        {
            m_stromy.at(i).setQSM(iterations, CylinderHeight, limit, branchLength, order, stemCurve);
        }
    }
}
void Project::setSortimens(QString name, bool finalSortiment)
{
    for(int i = 0; i< m_stromy.size(); i++)
    {
        if (get_TreeCloud(i).get_name() == name)
        {
            m_stromy.at(i).setSortiment(finalSortiment);
        }
    }
}
void Project::set_treeConvexCloud(QString name)
{

}
int Project::set_treeConcaveCloud(QString name,float edge)
{
    return 0;
}
void Project::set_ConcaveCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,float edge, QString name, QColor color)
{

}
void Project::set_ConvexCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name, QColor color)
{

}
void Project::set_treePosition(QString name, int height)
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      m_stromy.at(i).set_position(height);
    }
  }
}
void Project::set_treePosition(QString name, Cloud terrain,  int num_points, int height)
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      m_stromy.at(i).set_position(terrain, num_points, height);
    }
  }
}
void Project::set_treePositionHT(QString name, Cloud terrain, int iter, int num_points)
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      m_stromy.at(i).set_positionHT(terrain, iter, num_points);
    }
  }
}
void Project::set_treePositionHT(QString name, int iter)
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      m_stromy.at(i).set_positionHT(iter);
    }
  }
}
void Project::set_treeheigth(QString name)
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      m_stromy.at(i).set_height();
    }
  }
}
void Project::set_treeDBHCloud(QString name)
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      m_stromy.at(i).set_dbhCloud();
      m_stromy.at(i).set_dbhHT(200);
      m_stromy.at(i).set_dbhLSR();
    }
  }
}

void Project::set_treeDBH_HT(QString name)
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      m_stromy.at(i).set_dbhHT(200);
    }
  }
}
void Project::set_treeDBH_HT(int i)
{
  m_stromy.at(i).set_dbhHT(200);
}
void Project::set_treeDBH_LSR(QString name)
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      m_stromy.at(i).set_dbhLSR();
    }
  }
}
void Project::set_treeDBH_LSR(int i)
{
  m_stromy.at(i).set_dbhLSR();
}
void Project::set_skeleton(QString name, Cloud c)
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      m_stromy.at(i).set_skeleton(c);
    }
  }
}
void Project::set_length(QString name)
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      m_stromy.at(i).set_length();
    }
  }
}
void Project::set_treeStemCurvature(QString name, int iter,int height )
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
    {
      m_stromy.at(i).set_stemCurvature(iter, height);
    }
  }
}
void Project::set_OstCloud(Cloud cloud)
{
  m_ostCloud.push_back(cloud);
}
Cloud Project::get_baseCloud(int i)
{
  return m_baseCloud.at(i);
}
double Project::get_Xtransform()
{
  return m_x;
}
double Project::get_Ytransform()
{
  return m_y;
}
double Project::get_Ztransform()
{
  return m_z;
}
QString Project::get_Path()
{
  return m_path;
}
Cloud Project::get_TerrainCloud(int i)
{
  return m_terrainCloud.at(i);
}
Cloud Project::get_TerrainCloud(QString name)
{
  for(int i = 0; i< m_terrainCloud.size(); i++)
  {
    if (get_TerrainCloud(i).get_name() == name)
      return m_terrainCloud.at(i);
  }
}
Cloud Project::get_VegeCloud(QString name)
{
  for(int i = 0; i< m_vegeCloud.size(); i++)
  {
    if (get_VegeCloud(i).get_name() == name)
      return m_vegeCloud.at(i);
  }
}
Cloud Project::get_VegeCloud(int i)
{
  return m_vegeCloud.at(i);
}
void Project::set_VegeCloud(QString name, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  for(int i = 0; i< m_vegeCloud.size(); i++)
  {
    if (get_VegeCloud(i).get_name() == name)
    {
      m_vegeCloud.at(i).set_Cloud(cloud);
      return;
    }
  }
  Cloud *c = new Cloud(cloud,name);
  m_vegeCloud.push_back(*c);
  delete c;
}
int Project::get_sizebaseCV()
{
  return m_baseCloud.size();
}
int Project::get_sizeTreeCV()
{
 return m_stromy.size();
}
int Project::get_sizeTerainCV()
{
  return m_terrainCloud.size();
}
int Project::get_sizeostCV()
{
  return m_ostCloud.size();
}
int Project::get_sizevegeCV()
{
  return m_vegeCloud.size();
}
Tree& Project::get_TreeCloud(int i)
{
return m_stromy.at(i);
}
Tree& Project::get_TreeCloud(QString name)
{
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
      return m_stromy.at(i);
  }
}
Cloud Project::get_ostCloud(int i)
{
return m_ostCloud.at(i);
}
Cloud Project::get_Cloud(QString name)
{
  for(int i = 0; i< m_baseCloud.size(); i++)
  {
    if (get_baseCloud(i).get_name()== name)
    return m_baseCloud.at(i);
  }
  for(int i = 0; i< m_terrainCloud.size(); i++)
  {
    if (get_TerrainCloud(i).get_name()== name)
      return m_terrainCloud.at(i);
      //c = new Cloud( get_TerrainCloud(i));
  }

  for(int i = 0; i< m_vegeCloud.size(); i++)
  {
    if (get_VegeCloud(i).get_name() == name)
      return m_vegeCloud.at(i) ;
  }

  for(int i = 0; i< m_ostCloud.size(); i++)
  {
  if (get_ostCloud(i).get_name() == name)
    return m_ostCloud.at(i); //c = new Cloud(get_ostCloud(i));
  }
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
     return m_stromy.at(i);
  }
    for(int i = 0; i< m_features.size(); i++)
    {
      if (getFeature(i).get_name() == name)
       return m_features.at(i);
    }
}
void Project::save_newCloud(QString type, QString path)
{
  //save cloud
  QString path1 = save_Cloud(path);
  //otevrit Proj.3df file pro pripsani
  QString projfile = QString("%1%2%3.3df").arg(get_Path()).arg(QDir::separator ()).arg(get_ProjName());
  QFile file (projfile);
  if(!file.exists())
  {
    projfile = QString("%1%2proj.3df").arg(get_Path()).arg(QDir::separator ()).arg(get_ProjName());
    QFile file (projfile);
  }

  file.open(QIODevice::Append | QIODevice::Text);
  QTextStream out(&file);
  out  << type << " " << path1<<"\n";
  file.close();
}
void Project::save_newCloud(QString type, QString name, pcl::PointCloud<pcl::PointXYZI>::Ptr c )
{
  QString path = save_Cloud(name,c);
    //otevrit Proj.3df file pro pripsani
  QString projfile = QString("%1%2%3.3df").arg(get_Path()).arg(QDir::separator ()).arg(get_ProjName());

  QFile file (projfile);
  if(!file.exists())
  {
    projfile = QString("%1%2proj.3df").arg(get_Path()).arg(QDir::separator ()).arg(get_ProjName());
    QFile file (projfile);
  }
  file.open(QIODevice::Append | QIODevice::Text);
  QTextStream out(&file);
  out  << type << " " << path<<"\n";
  file.close();
}
QString Project::save_Cloud(QString path)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile(path.toUtf8().constData(),*cloud);

  QStringList name = path.split(QDir::separator ());
  QString file = name.back();
  QStringList ext = file.split(".");

  return save_Cloud(ext.front(),cloud);
}
QString Project::save_Cloud(QString name, pcl::PointCloud<pcl::PointXYZI>::Ptr c)
{
  QString path_out = QString ("%1%2%3.pcd").arg(get_Path()).arg(QDir::separator ()).arg(name);
  pcl::io::savePCDFileBinaryCompressed(path_out.toUtf8().constData(), *c);
  return path_out;
}
bool Project::cloud_exists(QString name)
{
  // for kazdy cloud v project
  for(int i = 0; i< m_baseCloud.size(); i++)
  {
    if (get_baseCloud(i).get_name()== name)
    return true;
  }
  for(int i = 0; i< m_terrainCloud.size(); i++)
  {
    if (get_TerrainCloud(i).get_name()== name)
      return true;
      //c = new Cloud( get_TerrainCloud(i));
  }
  for(int i = 0; i< m_vegeCloud.size(); i++)
  {
    if (get_VegeCloud(i).get_name() == name)
      return true;
  }
  for(int i = 0; i< m_ostCloud.size(); i++)
  {
  if (get_ostCloud(i).get_name() == name)
    return true;
  }
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
     return true;
  }
  return false;
}
void Project::delete_Cloud(QString name)
{
  //otevrit soubor proj.3df vymazat radek vyhledany podle zadaneho textu
  QString filepath = QString ("%1%2%3.3df").arg(m_path).arg(QDir::separator ()).arg(m_projectName);
  QString fileout = QString ("%1%2%3_t.3df").arg(m_path).arg(QDir::separator ()).arg(m_projectName);

  QFile file(filepath);
  if(!file.exists())
  {
    filepath = QString("%1%2proj.3df").arg(get_Path()).arg(QDir::separator ());
    fileout = QString("%1%2proj_t.3df").arg(get_Path()).arg(QDir::separator ());
    QFile file (filepath);
  }
  QFile fileU(fileout);
  file.open(QIODevice::ReadWrite| QIODevice::Text);
  fileU.open(QIODevice::ReadWrite| QIODevice::Text);
  QTextStream in(&file);
  QTextStream out(&fileU);

  while(!in.atEnd())
  {
    QString line = in.readLine();
    if(!line.contains(name))
      out << line<<"\n";
  }
  //smazat proj a prejmenovat proj_t
  file.close();
  fileU.close();
  QFile::remove(filepath);
  QFile::rename(fileout,filepath);

  remove_file(name);

  std::vector<Cloud> baseCloud;
  for (int i = 0; i < m_baseCloud.size(); i++)
  {
     if(m_baseCloud.at(i).get_name() != name)
     {
       baseCloud.push_back(m_baseCloud.at(i));
     }
  }
  m_baseCloud.swap(baseCloud);

  std::vector<Cloud> terrainCloud;
  for (int i = 0; i < m_terrainCloud.size(); i++)
  {
     if(m_terrainCloud.at(i).get_name() != name)
     {
       terrainCloud.push_back(m_terrainCloud.at(i));
     }
  }
  m_terrainCloud.swap(terrainCloud);

  std::vector<Cloud> vegeCloud;
  for (int i = 0; i < m_vegeCloud.size(); i++)
  {
     if(m_vegeCloud.at(i).get_name() != name)
     {
       vegeCloud.push_back(m_vegeCloud.at(i));
     }
  }
  m_vegeCloud.swap(vegeCloud);

  std::vector<Cloud> ostCloud;
  for (int i = 0; i < m_ostCloud.size(); i++)
  {
     if(m_ostCloud.at(i).get_name() != name)
     {
       ostCloud.push_back(m_ostCloud.at(i));
     }
  }
  m_ostCloud.swap(ostCloud);

  std::vector<Tree> stromy;
  for (int i = 0; i < m_stromy.size(); i++)
  {
     if(m_stromy.at(i).get_name() != name)
     {
       stromy.push_back(m_stromy.at(i));
     }
  }
  m_stromy.swap(stromy);
}
void Project::remove_file(QString name)
{
  QMessageBox *msgBox =  new QMessageBox(0);
	msgBox->setText("DELETE");
	QString a = QString("Do you want delete also file from disc??");
	msgBox->setInformativeText(a);
	msgBox->setStandardButtons(QMessageBox::Yes | QMessageBox::No);
	msgBox->setDefaultButton(QMessageBox::Yes);

	if(msgBox->exec() == QMessageBox::Yes)
  {
    QString filep = QString ("%1%2%3").arg(m_path).arg(QDir::separator ()).arg(name);
    QFile::remove(filep);
  }
  delete msgBox;
}
void Project::set_color(QString name, QColor col)
{
  for(int i = 0; i< m_baseCloud.size(); i++)
  {
    if (get_baseCloud(i).get_name() == name)
      m_baseCloud.at(i).set_color(col);
  }

  for(int i = 0; i< m_terrainCloud.size(); i++)
  {
    if (get_TerrainCloud(i).get_name()== name)
       m_terrainCloud.at(i).set_color(col);

  }

  for(int i = 0; i< m_vegeCloud.size(); i++)
  {
    if (get_VegeCloud(i).get_name() == name)
       m_vegeCloud.at(i).set_color(col);
  }

  for(int i = 0; i< m_ostCloud.size(); i++)
  {
  if (get_ostCloud(i).get_name() == name)
     m_ostCloud.at(i).set_color(col); //c = new Cloud(get_ostCloud(i));
  }
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
      m_stromy.at(i).set_color(col);
  }
  save_color(name,col);
}
void Project::set_PointSize(QString name, int p)
{
  for(int i = 0; i< m_baseCloud.size(); i++)
  {
    if (get_baseCloud(i).get_name() == name)
      m_baseCloud.at(i).set_Psize(p);
  }

  for(int i = 0; i< m_terrainCloud.size(); i++)
  {
    if (get_TerrainCloud(i).get_name()== name)
       m_terrainCloud.at(i).set_Psize(p);

  }

  for(int i = 0; i< m_vegeCloud.size(); i++)
  {
    if (get_VegeCloud(i).get_name() == name)
       m_vegeCloud.at(i).set_Psize(p);
  }

  for(int i = 0; i< m_ostCloud.size(); i++)
  {
  if (get_ostCloud(i).get_name() == name)
     m_ostCloud.at(i).set_Psize(p); //c = new Cloud(get_ostCloud(i));
  }
  for(int i = 0; i< m_stromy.size(); i++)
  {
    if (get_TreeCloud(i).get_name() == name)
      m_stromy.at(i).set_Psize(p);
  }
}
void Project::save_color(QString name, QColor col)
{
  //open pro file,
  QString filepath = QString ("%1%2%3.3df").arg(m_path).arg(QDir::separator ()).arg(m_projectName);
  QString filepathtmp = QString ("%1%2%3.tmp").arg(m_path).arg(QDir::separator ()).arg(m_projectName);
  QFile file(filepath);
  QFile tmp (filepathtmp);
  file.open(QIODevice::ReadWrite);
  tmp.open(QIODevice::ReadWrite | QIODevice::Text);
  QTextStream in(&file);
  QTextStream out(&tmp);

  while(!in.atEnd())
  {
    QString line = in.readLine();
    if(line.contains(name))
    {
      QStringList plist = line.split(" ");
      out << plist.at(0)<< " " << plist.at(1)<< " " << col.red()<< " " << col.green()<< " " << col.blue()<< "\n";

    }
    else
    {
      out<<line<<"\n";
    }
  }
  file.close();
  tmp.close();
  //smazat proj.3df a prejmenovat proj.tmp na pro.3df
  QFile::remove(filepath);
  QFile::rename(filepathtmp,filepath);
}
//POLYHEDRON INTERSECTIONS 3D
int Project::getIntersectionsSize()
{
    return m_intersections3D.size();
}
PolyhedronIntersections3D& Project::getIntersectionsAt(int i)
{
    return m_intersections3D.at(i);
}
void Project::computeCrownIntersections()
{
    for(int i=0; i<get_sizeTreeCV()-1;i++)
    {
        if(get_TreeCloud(i).isCrownExist()==true && get_TreeCloud(i).get_TreeCrown().isConvexhull3DExist()==true)
        {
                pcl::PolygonMesh mesh1(get_TreeCloud(i).get_TreeCrown().get3DConvexhull().getMesh());
                QString name1 = get_TreeCloud(i).get_name();
            for(int j=i+1;j<get_sizeTreeCV();j++)
            {
                if(get_TreeCloud(j).isCrownExist()==true && get_TreeCloud(j).get_TreeCrown().isConvexhull3DExist()==true)
                {
                    pcl::PointXYZI c1 = get_TreeCloud(i).get_TreeCrown().getCrownPosition();
                    pcl::PointXYZI c2 = get_TreeCloud(j).get_TreeCrown().getCrownPosition();
                    float l1 = get_TreeCloud(i).get_TreeCrown().getCrownLenghtXY();
                    float l2 = get_TreeCloud(j).get_TreeCrown().getCrownLenghtXY();

                    if(isIntersectionPossible(c1,l1,c2,l2) == true )
                    {
                        pcl::PolygonMesh mesh2(get_TreeCloud(j).get_TreeCrown().get3DConvexhull().getMesh());
                        QString name2 = get_TreeCloud(j).get_name();
                        PolyhedronIntersections3D p(mesh1,mesh2,name1,name2);
                        if(p.getSurface() > 0 && p.getVolume()>0)
                        {
                            m_intersections3D.push_back(p);
                        }
                    }
                }
            }
        }
    }
}
bool Project::isIntersectionPossible(pcl::PointXYZI pos1, float lenght1, pcl::PointXYZI pos2, float lenght2)
{
    float posDist = GeomCalc::computeDistance2Dxy(pos1,pos2);
    float lenghtsHalf = (lenght1/2)+(lenght2/2);
    if(lenghtsHalf>posDist){
        return true;
    }else return false;
}
//////////////////////
void Project::mergeEraseCloudsByID()
{
    for(int i=0;i<get_sizeTreeCV();i++)
    {
        QString name = get_TreeCloud(i).get_name();
        if(isID(name) == true){
            QString treeID = getIDnumber(name);
            for(int j=i+1;j<get_sizeTreeCV();j++)
            {
                QString id2 = getIDnumber(get_TreeCloud(j).get_name());
                if (treeID == id2 && name != get_TreeCloud(j).get_name()){
                    mergeClouds(name,get_TreeCloud(j).get_name());
                }
            }
            for(int j=0;j<get_sizeostCV();j++)
            {
                QString id2 = getIDnumber(get_ostCloud(j).get_name());
                if (treeID == id2){
                    mergeClouds(name,get_ostCloud(j).get_name());
                }
            }
           /* for(int j=0;j<get_sizevegeCV();j++)
            {
                QString id2 = getIDnumber(get_VegeCloud(j).get_name());

                if (treeID == id2){
                    mergeClouds(name,get_VegeCloud(j).get_name());
                }
            }*/
        }

    }
}
void Project::mergeClouds(QString treeName, QString joinCloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr tree (new pcl::PointCloud<pcl::PointXYZI>);
    tree = get_Cloud(treeName).get_Cloud();
    pcl::PointCloud<pcl::PointXYZI>::Ptr join (new pcl::PointCloud<pcl::PointXYZI>);
    join = get_Cloud(joinCloud).get_Cloud();
    *tree += *join;

    get_Cloud(treeName).set_Cloud(tree);

    QString path_out = QString ("%1%2%3").arg(get_Path()).arg(QDir::separator ()).arg(treeName);
    pcl::io::savePCDFileBinaryCompressed(path_out.toUtf8().constData(), *tree);
}
bool Project::isID(QString &str)
{
    QString id = str.left(2);
    QString number = getIDnumber(str);
    if((id == "id" || id == "ID") && number.size() > 0 )
    {
        return true;
    }
    else return false;
}
QString Project::getIDnumber(QString str)
{
    QString id;
    for(int i=0; i<str.size();i++)
    {
        if( str.at(i).isNumber() == true){
           id.push_back(str.at(i));
        }
    }
    return id;
}
void Project::deleteCloudNoQuestions(QString name) // !!!!!!!!!!!!!!!!!!
{
    //otevrit soubor proj.3df vymazat radek vyhledany podle zadaneho textu
  QString filepath = QString ("%1%2%3.3df").arg(m_path).arg(QDir::separator ()).arg(m_projectName);
  QString fileout = QString ("%1%2%3_t.3df").arg(m_path).arg(QDir::separator ()).arg(m_projectName);

  QFile file(filepath);
  if(!file.exists())
  {
    filepath = QString("%1%2proj.3df").arg(get_Path()).arg(QDir::separator ());
    fileout = QString("%1%2proj_t.3df").arg(get_Path()).arg(QDir::separator ());
    QFile file (filepath);
  }
  QFile fileU(fileout);
  file.open(QIODevice::ReadWrite| QIODevice::Text);
  fileU.open(QIODevice::ReadWrite| QIODevice::Text);
  QTextStream in(&file);
  QTextStream out(&fileU);

  while(!in.atEnd())
  {
    QString line = in.readLine();
    if(!line.contains(name))
      out << line<<"\n";
  }
  //smazat proj a prejmenovat proj_t
  file.close();
  fileU.close();
  QFile::remove(filepath);
  QFile::rename(fileout,filepath);

  QString filep = QString ("%1%2%3").arg(m_path).arg(QDir::separator ()).arg(name);
  QFile::remove(filep);

  std::vector<Cloud> baseCloud;
  for (int i = 0; i < m_baseCloud.size(); i++)
  {
     if(m_baseCloud.at(i).get_name() != name)
     {
       baseCloud.push_back(m_baseCloud.at(i));
     }
  }
  m_baseCloud.swap(baseCloud);

  std::vector<Cloud> terrainCloud;
  for (int i = 0; i < m_terrainCloud.size(); i++)
  {
     if(m_terrainCloud.at(i).get_name() != name)
     {
       terrainCloud.push_back(m_terrainCloud.at(i));
     }
  }
  m_terrainCloud.swap(terrainCloud);

  std::vector<Cloud> vegeCloud;
  for (int i = 0; i < m_vegeCloud.size(); i++)
  {
     if(m_vegeCloud.at(i).get_name() != name)
     {
       vegeCloud.push_back(m_vegeCloud.at(i));
     }
  }
  m_vegeCloud.swap(vegeCloud);

  std::vector<Cloud> ostCloud;
  for (int i = 0; i < m_ostCloud.size(); i++)
  {
     if(m_ostCloud.at(i).get_name() != name)
     {
       ostCloud.push_back(m_ostCloud.at(i));
     }
  }
  m_ostCloud.swap(ostCloud);

  std::vector<Tree> stromy;
  for (int i = 0; i < m_stromy.size(); i++)
  {
     if(m_stromy.at(i).get_name() != name)
     {
       stromy.push_back(m_stromy.at(i));
     }
  }
  m_stromy.swap(stromy);
}
void Project::setFeature(Features feature){
    m_features.push_back(feature);
}

Features Project::getFeature(int i)
{
    return m_features.at(i);
}
int Project::getFeatureSize(){
    return m_features.size();
}

ProjFile::ProjFile(QString name)
{
  m_name = name;
  m_x=m_y=m_z=0;
  m_version=1;
}
ProjFile::~ProjFile()
{

}
void ProjFile::setPath(QString path)
{
  m_path = path;
}
void ProjFile::setTransformMatrix(double x, double y, double z)
{
  m_x=x;
  m_y=y;
  m_z=z;
}
void ProjFile::setVersion(int i)
{
  m_version = i;
}
void ProjFile::writeHeader()
{
  //check if the file do not exist
  //open new file
  // write version
  // write path to projectfile
  //write transformation matrix
  //emit header ready

}
void ProjFile::writeNewCloud(QString name)
{
  //open proj file
  // append new line with name of the cloud
}
void ProjFile::removeCloud(QString name)
{
  //open proj file
  // find line with given name and remove it
}
void ProjFile::readOldFile()
{

}





