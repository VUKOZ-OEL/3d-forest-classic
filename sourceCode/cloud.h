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

#ifndef CLOUD_H_INCLUDED
#define CLOUD_H_INCLUDED

#include <QtGui/QtGui>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include "geomcalculations.h"

  //! Struct consist for hold tree centre and diameter.
  /*! Struct of holding information about computed tree DBH with coordinates of centre (a,b,z) radius (r) and voting value when computing (i) . */
struct stred {
				float a;    /**< the X-coordinate of the center of the fitting circle */
				float b;    /**< the Y-coordinate of the center of the fitting circle */
				float z;    /**< the Z-coordinate of the center of the fitting circle */
        float i;    /**< the total number of outer iterations, can be removed */
        float r;    /**< the radius of the fitting circle */
   bool operator < (const stred& str) const
    {return (i < str.i);}
				};



  //! Basic data class of pointcloud representation.
  /*! Class represents points cloud with color, point size and name.  */
class Cloud
{
protected:
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_Cloud;   /**< pcl::pointcloud for points */
  QString m_name;                                 /**< cloud name */
  QColor m_color;                                 /**< cloud color */
  int m_PointSize;                                /**< size of point for display */

public:
    //! Constructor.
    /*! Costructor with all parameters \param cloud pointCloud itself \param name cloud name \param col color of cloud. */
  Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name,QColor col);
    //! Constructor.#include "mainwindow.h"
    /*! Costructor with parameters. color is randomly selected. \param cloud pointCloud itself \param name cloud name  */
  Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, QString name);
    //! Constructor.
    /*! Empty costructor  */
  Cloud();
  //! Copy Constructor.
    /*! Copy constructor  */
  Cloud(const Cloud& kopie);
    //! Destructor.
    /*! Empty destructor  */
  ~Cloud();
    //! Copy Constructor.
    /*! Copy contructor */
  Cloud operator=(Cloud &kopie);

    //! Set new cloud.
    /*! Set new pointCloud for cloud. \param cloud reference to the pointCloud. */
  void set_Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    //! Set new name.
    /*! Set new name for cloud. \param name name of the cloud. */
  void set_name(QString name);
    //! Set new color of pointCloud.
    /*! Set new color of pointCloud. \param col QColor value of color */
  void set_color (QColor col);
    //! Set point size.
    /*! Set point size for displaing in vizualizer. \param p size of point. */
  void set_Psize (int p);

    //! Get cloud.
    /*! Get cloud pointCloud. \return reference to the pointCloud of Cloud. */
  pcl::PointCloud<pcl::PointXYZI>::Ptr get_Cloud();
    //! Get name.
    /*! Get cloud name. \return name of the Cloud. */
  QString get_name();
    //! Get color.
    /*! Get cloud color. \return color of the Cloud. */
  QColor get_color();
    //! Get point size.
    /*! Get cloud point size. \return value of point size. */
  int get_Psize();
};
  //! Class for tree representation.
  /*! Class for holding information about single tree with all parameters. */


#endif // CLOUD_H_INCLUDED
