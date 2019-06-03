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
#ifndef HOUGHTRANSFORM_H_INCLUDED
#define HOUGHTRANSFORM_H_INCLUDED

#include "cloud.h"
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
//! Class for computing circle from input point cloud data.
class HoughTransform
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud; /**< base point cloud for computing */
  stred m_circle;                               /**< result of calculating the ring */
  int m_it;                                   /**< number of iterations */
  public:
    //! Constructor.
    /*! Empty Costructor. */
  HoughTransform();
  //! Constructor.
    /*! Costructor of tree. \param cloud tree pointCloud \param name of the tree \param col color of pointCloud  */
  HoughTransform(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    //! Destructor.
    /*! Destructor of the class */
  ~HoughTransform();
    //! set cloud to the class.
    /*! set the point cloud for compute ring. \param cloud point cloud*/
  void set_Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    //! computation of the ring.
    /*! calculate the ring for given cloud and save result into m_circle*/
  void  compute();
  //! setting number of iterations.
    /*! setting number of iterations for calculation*/
  void  set_iterations(int i = 200);
   //! get the result circle.
    /*! \return stred ring computed by class. */
  stred get_circle();
};

#endif // HOUGHTRANSFORM_H_INCLUDED
