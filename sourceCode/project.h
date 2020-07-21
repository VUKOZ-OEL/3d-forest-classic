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
#ifndef PROJECT_H_INCLUDED
#define PROJECT_H_INCLUDED

#include <QtGui/QtGui>
//include BASE
#include <string>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include "skeleton.h"
#include "cloud.h"
#include "tree.h"
#include "terrain.h"
#include "alphaShapes.h"


  //! Basic class representing pointCloud data.
  /*! Basic structure for holding point cloud data. Class consist of pointcloud, name of pointcloud, color and pointsize. */
  //! Class for storing cloud data in one place .
  /*! Base class for data handling. */
class Project
{
  double m_x, m_y, m_z;                 /**< Transformation matrix values */
  QString m_projectName;                /**< Project name */
  QString m_path;                       /**< Path to the project */

  std::vector<Cloud> m_baseCloud;       /**< Vector of base clouds */
  std::vector<Cloud> m_terrainCloud;    /**< Vector of terrain clouds */
  std::vector<Cloud> m_vegeCloud;       /**< Vector of vegetation clouds */
  std::vector<Cloud> m_ostCloud;        /**< Vector of ost clouds */
  std::vector<Tree> m_stromy;           /**< Vector of Tree clouds */
    std::vector<Features> m_features;           /**< Vector of Tree clouds */
  std::vector<PolyhedronIntersections3D> m_intersections3D;           /**< Vector of Tree clouds */

public:
    //! Constructor.
    /*! Costructor of empty project. */
  Project();
    //! Destructor.
    /*! Destructor of project. */
  ~Project();
    //! Constructor.
    /*! Costructor of Project. \param name project name  */
  Project( QString name);
    //! Constructor.
    /*! Costructor of Project. Only for old versions of projects. \param x transformation matrix  value of X coordinate \param y transformation matrix  value of X coordinate
     \param name project name  */
  Project(double x, double y, QString name);
    //! Constructor.
    /*! Costructor of Project. \param x transformation matrix  value of X coordinate \param y transformation matrix  value of Y coordinate
     \param z transformation matrix  value of Z coordinate  \param name project name  */
  Project(double x, double y,double z, QString name);

//SET
    //! Set transformation matrix value.
    /*! Setting X value of the transformation matrix \param x transformation matrix  value of X coordinate  */
  void set_xTransform(double x);
    //! Set transformation matrix value.
    /*! Setting Y value of the transformation matrix \param y transformation matrix  value of Y coordinate  */
  void set_yTransform(double y);
    //! Set transformation matrix value.
    /*! Setting Z value of the transformation matrix \param z transformation matrix  value of Z coordinate  */
  void set_zTransform(double z);
    //! Set project path.
    /*! Setting project path to the folder of project \param path path to the proj file.  */
  void set_path(QString path);

    //! Save new cloud in project.
    /*! Save copy old file into project folder and add record in proj file \param type tape of the cloud \param path path to old file */
  void save_newCloud(QString type, QString path);
  //! Save new cloud in project.
    /*! Save copy old file into project folder and add record in proj file \param type tape of the cloud \param path path to old file */
  void save_newCloud(QString type, Cloud& input);
    //! Save new cloud in project.
    /*! Save new cloud in file and in proj file \param type type of the cloud \param name name of the cloud  \param c new pointCloud*/
  void save_newCloud(QString type, QString name, pcl::PointCloud<pcl::PointXYZI>::Ptr c );
    //! Save old pcd file in project.
    /*! Save copy old file into project folder \param path path to old file  \return path to the new file*/
  QString save_Cloud(QString path);
    //! Save new pointCloud into file .
    /*! Save new pointCloud into file with given name and save it into pcd file in project folder .
     \param name name of the pointCloud  \param c pointCloud \return path to newly created file */
  QString save_Cloud(QString name, pcl::PointCloud<pcl::PointXYZI>::Ptr c);
    //! Save color of the cloud into proj file.
    /*! Save color values of the given cloud to proj file. \param name name of the Cloud  \param  col color values */
  void save_color(QString name, QColor col);
    //! Set color to the cloud.
    /*! Save color values to the given cloud. \param name name of the Cloud  \param  col color values */
  void set_color(QString name, QColor col);
    //! Set point size to the cloud.
    /*! Save point size to the given cloud. \param name name of the Cloud  \param  p point size value */
  void set_PointSize(QString name, int p);
    //! Clear all vectors of clouds.
    /*! Clear all vectors of cloud in project  */
  void cleanAll();
    //! delete cloud from project.
    /*! Delete given cloud from project vector, proj file and from the folder \param name cloud name */
  void delete_Cloud(QString name);

    //GET
    //! Get X value of tranformation matrix.
    /*! \return value of X transformation matrix  */
  double get_Xtransform();
    //! Get Y value of tranformation matrix.
    /*! \return value of Y transformation matrix  */
  double get_Ytransform();
    //! Get Z value of tranformation matrix.
    /*! \return value of Z transformation matrix  */
  double get_Ztransform();
    //! Get project name.
    /*! \return project name  */
  QString get_ProjName();
    //! Get project path.
    /*! \return project path  */
  QString get_Path();
    //! Get cloud.
    /*! \param name name of the cloud \return cloud with given name  */
  Cloud get_Cloud(QString name);
    //! Check if cloud exist in project.
    /*! \param name name of the cloud \return bool value if cloud exist in project  */
  bool cloud_exists(QString name);

//BASECLOUD
    //! Set new base cloud.
    /*! \param cloud cloud of new base cloud  */
  void set_baseCloud(Cloud cloud);
    //! Get base cloud.
    /*! \param i get base cloud on ith position in vector \return cloud on given position  */
  Cloud get_baseCloud(int i);
    //! Get size of base cloud vector.
    /*! \return size of vector containing base clouds */
  int get_sizebaseCV();
    
    //TERRAIN FEATURE
      //! Set new base cloud.
      /*! \param cloud cloud of new base cloud  */
    void setFeature(Features feature);
      //! Get base cloud.
      /*! \param i get base cloud on ith position in vector \return cloud on given position  */
    Features getFeature(int i);
      //! Get size of base cloud vector.
      /*! \return size of vector containing base clouds */
    int getFeatureSize();

//TERRAINCLOUD
    //! Set new terrain cloud.
    /*! \param cloud cloud of new terrain cloud  */
  void set_TerrainCloud(Cloud cloud);
    //! Set new terrain cloud.
    /*! \param i set new cloud at ith position of vector \param cloud cloud of new terrain cloud  */
  void set_TerrainCloudat(int,Cloud);
    //! Get terrain cloud.
    /*! \param i get terrain cloud on ith position in vector \return cloud on given position  */
  Cloud get_TerrainCloud(int i);
  //! Get terrain cloud.
    /*! \param i get terrain cloud on ith position in vector \return cloud on given position  */
  Cloud get_TerrainCloud(QString name);
    //! Get size of terrain cloud vector.
    /*! \return size of vector containing terrain clouds */
  int get_sizeTerainCV();

//VEGECLOUD
    //! Set new vegetation cloud.
    /*! \param cloud cloud of new vegetation cloud  */
  void set_VegeCloud(Cloud cloud);
    //! Get vegetation cloud.
    /*! \param i get vegetation cloud on ith position in vector \return cloud on given position  */
  Cloud get_VegeCloud(int i);
    //! Get size of vegetation cloud vector.
    /*! \return size of vector containing vegetation clouds */
  int get_sizevegeCV();
    //! get vegetation cloud with given name.
    /*! \param QString name of the cloud \return Cloud with given name */
  Cloud get_VegeCloud(QString name);
   //! Set vegetation cloud with given name and pointcloud.
    /*! \param  name of the cloud \param cloud pointcloud */
  void set_VegeCloud(QString name, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

//TREECLOUD
    //! Set new tree cloud.
    /*! \param cloud cloud of new tree cloud  */
  void set_Tree(Cloud cloud);
    //! Set new tree cloud.
    /*! \param i set new cloud at ith position of vector \param cloud cloud of new tree cloud  */
  void set_TreeCloudat(int,Cloud);
    //! Get tree cloud.
    /*! \param i get tree cloud on ith position in vector \return Tree on given position  */
  Tree& get_TreeCloud(int i);
    //! Get tree cloud.
    /*! \param name name of the tree \return Tree with given name  */
  Tree& get_TreeCloud(QString name);
    //! Get size of tree cloud vector.
    /*! \return size of vector containing tree clouds */
  int get_sizeTreeCV();
    //! Set tree DBH_pointCloud.
    /*! \param name tree name \param cloud pointCloud representing points for DBH estimation  */
  void set_dbhCloud(QString name,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    //! Set convex planar projection pointCloud.
    /*! \param name tree name \param cloud pointCloud representing points of convex planar projection */
  void set_treeConvexCloud(QString name);
    //! Set concave planar projection pointCloud.
    /*! \param name tree name \param edge maximal length of distance between two points. \return number of edges with greates distance than maximal. */
  int set_treeConcaveCloud(QString name,float edge);
    //! Set tree position.
    /*! \param name tree name */
  void set_treePosition(QString name, int height);
    //! Set tree position based on terrain.
    /*! \param name tree name \param cloud terrain cloud */
  void set_treePosition(QString name, Cloud terrain,  int num_points, int height);
  void set_treePositionHT(QString name, Cloud terrain, int iter, int num_points );
  void set_treePositionHT(QString name, int iter);
    //! Set tree DBH cloud.
    /*! \param name tree name */
  void set_treeheigth(QString name);
    //! Set tree DBH cloud.
    /*! \param name tree name */
  void set_treeDBHCloud(QString name);
    //! Set tree DBH using RHT.
    /*! \param name tree name */
  void set_treeDBH_HT(QString name);
    //! Set tree DBH using RHT.
    /*! \param i for i-th tree in vector */
  void set_treeDBH_HT(int i);
    //! Set tree DBH using LSR.
    /*! \param name tree name */
  void set_treeDBH_LSR(QString name);
    //! Set tree DBH using LSR.
    /*! \param i for i-th tree in vector */
  void set_treeDBH_LSR(int i);
    //! Set tree length.
    /*! \param name tree name */
  void set_length(QString name);
    //! Set tree skeleton.
    /*! \param name tree name  \param c Cloud of point representing skeleton.*/
  void set_skeleton(QString name, Cloud c);
    void setSkeleton(QString name, float voxelSize, float multiplicator);
     void setCylinders(QString name, int iterations, float CylinderHeight, float limit, float branchLength, int order, bool stemCurve);
    void setSortimens(QString name, bool finalSortiment);
    //! Set Stem curve for given cloud name.
    /*! \param name tree name */
  void set_treeStemCurvature(QString name, int iter = 200, int height = 50);
//OSTCLOUDset_dbhCloud(cl);
    //! Set new ost cloud.
    /*! \param cloud cloud of new ost cloud  */
  void set_OstCloud(Cloud cloud);
    //! Get ost cloud.
    /*! \param i get tree cloud on ith position in vector \return Cloud on given position of vector  */
  Cloud get_ostCloud(int i);
    //! Get size of ost cloud vector.
    /*! \return size of vector containing ost clouds */
  int get_sizeostCV();
    //! Set concave planar projection pointCloud fro any pointcloud.
    /*! \param cloud input pointCloud \param edge length of maximal edge length \param name name of the cloud \param color color of the cloud */
    /*! \return Cloud with point of concave planar projection */
  void set_ConcaveCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,float edge, QString name, QColor color);
    //! Set convex planar projection pointCloud fro any pointcloud.
    /*! \param cloud input pointCloud \param name name of the cloud \param color color of the cloud */
    /*! \return Cloud with point of convex planar projection */
  void set_ConvexCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name, QColor color);

//POLYHEDRON INTERSECTIONS 3D
    int getIntersectionsSize();
    PolyhedronIntersections3D& getIntersectionsAt(int i);
    void computeCrownIntersections();
    bool isIntersectionPossible(pcl::PointXYZI pos1, float lenght1, pcl::PointXYZI pos2, float lenght2);
    void deleteCloudNoQuestions(QString name);
    void mergeClouds(QString treeName, QString joinCloud);
    void mergeEraseCloudsByID();
    QString getIDnumber(QString str);
    bool isID(QString &str);

private:
    //! Delete file from disc.
    /*!  Function ask user if wants to delete this cloud from disc \param name name of the cloud */
    void remove_file(QString name);

};


class ProjFile//: public QObject
{
  //Q_OBJECT
public:
  ProjFile(QString);
  ~ProjFile();

public slots:
  void writeHeader();
  void writeNewCloud(QString);
  void setPath(QString);
  void setTransformMatrix(double, double, double);
  void readHeader();
  void setVersion(int );
  void readOldFile();
  void removeCloud(QString name);

signals:
  void fileExist(bool);
  void percentage();
  void sendingoutput( Cloud *);


private:
 QString m_path;
 QString m_name;
 double m_x, m_y, m_z;                 /**< Transformation matrix values */
 int m_version;

};


#endif // PROJECT_H_INCLUDED
