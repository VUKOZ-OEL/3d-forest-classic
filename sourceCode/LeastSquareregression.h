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
#ifndef LEASTSQUAREREGRESSION_H_INCLUDED
#define LEASTSQUAREREGRESSION_H_INCLUDED

#include "cloud.h"
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
  //! Struct consist for hold tree centre and diameter.
  /*! Struct of holding information about computed tree DBH with coordinates of centre (a,b) radius (r) and parameters of LSR computing . */
struct stredLSR {
				float a;  /**< the X-coordinate of the center of the fitting circle */
				float b;  /**< the Y-coordinate of the center of the fitting circle */
				float r;  /**< the radius of the fitting circle */
        float s;  /**< the root mean square error (the estimate of sigma) */
        float i;  /**< the total number of outer iterations (updating the parameters) */
        float j;  /**< the total number of inner iterations (adjusting lambda) */
        float g;  /**<  */
   bool operator < (const stred& str) const
    {return (i < str.i);}
				};
  //! Class for computing circle from input point cloud data using Least Square Regression method.
class LeastSquaredRegression
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud; /**< base point cloud for computing */
  stred m_circle;                               /**< result of calculating the ring */
  public:
    //! Constructor.
    /*! Empty Costructor. */
  LeastSquaredRegression ();
  //! Constructor.
    /*! Costructor of tree. \param cloud tree pointcloud  */
  LeastSquaredRegression (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    //! Destructor.
    /*! Destructor of the class */
  ~LeastSquaredRegression ();
    //! set cloud to the class.
    /*! set the point cloud for compute ring. \param cloud pointcloud*/
  void setCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    //! computation of the ring.
    /*! calculate the ring for given cloud and save result into m_circle*/
  void  compute();
   //! get the result circle.
    /*! \return stred ring computed by class. */
  stred getCircle();
    //! Compute algebraic circle fit and save it into m_circle
    /*! Compute lgebraic circle fit for given pointCloud.*/
  void algebraicCircle();
    //! Compute geometric circle fit and save it into m_circle
    /*! Compute geometric circle fit for given pointCloud.*/
  void geometricCirlce();
    //! Compute  the root mean square error - sigma
    /*! Compute the square root of the average square of the distance for given pointCloud and computed circle.
     \param circle computed stredLSR \return value of sigma */
  float sigma(stredLSR circle);
  stred kamihoDBH();
  void TaubinFit();
};

#endif // LEASTSQUAREREGRESSION_H_INCLUDED
