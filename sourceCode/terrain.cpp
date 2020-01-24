#include "terrain.h"
#include "HoughTransform.h"
#include "hull.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
 OctreeTerrain::OctreeTerrain()
{
  m_baseCloud = new Cloud();
  m_vegetation = new Cloud();
  m_terrain = new Cloud();
  m_resolution = 0.1;
}
OctreeTerrain::OctreeTerrain( Cloud input, float resolution)
{
  m_baseCloud = new Cloud();
  m_vegetation = new Cloud();
  m_terrain = new Cloud();
  *m_baseCloud = input;
  m_resolution = resolution;
}
OctreeTerrain::~OctreeTerrain()
{
  delete m_baseCloud;
  delete m_vegetation;
  delete m_terrain;
}
void OctreeTerrain:: setResolution(float res)
{
  m_resolution = res;
}
void OctreeTerrain::setBaseCloud(Cloud input)
{
  m_baseCloud->set_Cloud(input.get_Cloud());
}
void OctreeTerrain::setVegetationName(QString name)
{
  m_vegetation->set_name(name);
}
void OctreeTerrain::setTerrainName(QString name)
{
  m_terrain->set_name(name);
}
void OctreeTerrain::octree(float res, pcl::PointCloud<pcl::PointXYZI>::Ptr input,pcl::PointCloud<pcl::PointXYZI>::Ptr output_ground, pcl::PointCloud<pcl::PointXYZI>::Ptr output_vege)
{

// udelat octree
  pcl::octree::OctreePointCloud<pcl::PointXYZI> oc (res);
  oc.setInputCloud (input);
  oc.addPointsFromInputCloud ();
  // zjistit vsechny voxely
  std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> > voxels;
  oc.getOccupiedVoxelCenters(voxels);

  // zjistit rozsah x y osy a podle toho hledat voxely ktere jsou nejníž
  double x_max,x_min,y_max,y_min,z_min,z_max;
  oc.getBoundingBox(x_min,y_min,z_min,x_max,y_max,z_max);

  oc.deleteTree();
  // z voxels udelat mracno bodu
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxels (new pcl::PointCloud<pcl::PointXYZI>);
  cloud_voxels->points.resize(voxels.size());
  #pragma omp parallel for
  for(int r=0; r < voxels.size(); r++)
  {
    cloud_voxels->points.at(r) = voxels.at(r);
  }
  cloud_voxels->width = cloud_voxels->points.size ();
  cloud_voxels->height = 1;
  cloud_voxels->is_dense = true;

  // spis boxsearch a pro kazdy voxel najit sousedy v danem boxu, pokud nenajde žadny bod niž než je on sam uložit jeho ID..
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> ocs (res);

  ocs.setInputCloud (cloud_voxels);
  ocs.addPointsFromInputCloud ();
   std::vector< int > low_voxels;
  for (int q =0; q < voxels.size(); q++)
  {
    std::vector< int > ind;
    Eigen::Vector3f low(voxels.at(q).x-res/2, voxels.at(q).y-res/2,z_min);
    Eigen::Vector3f high(voxels.at(q).x+res/2, voxels.at(q).y+res/2,voxels.at(q).z);
    if(ocs.boxSearch(low,high,ind) <3)
    {
      if(ind.size() == 0)
        continue;
      // pokud jsou voxely vyskove pouze res od sebe
      if(ind.size()==1)
        low_voxels.push_back(q);
      else
      {
        if(std::abs(voxels.at(ind.at(0)).z - voxels.at(ind.at(1)).z ) < (res*1.1) )
          low_voxels.push_back(q);
      }
    }
  }
  ocs.deleteTree();

// get point of lowest voxels
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> ocsearch (res);
  ocsearch.setInputCloud (input);
  ocsearch.addPointsFromInputCloud ();
  std::vector< int > low_voxels_indices;
  for(int u=0; u< low_voxels.size();u++)
  {
    ocsearch.voxelSearch(voxels.at(low_voxels.at(u)),low_voxels_indices);
  }
  ocsearch.deleteTree();
 // ocs.voxelSearch(voxels.at(q),low_voxels_indices);

    boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (low_voxels_indices));
    pcl::ExtractIndices<pcl::PointXYZI> extract;
     // Extract the inliers
    extract.setInputCloud (input);
    extract.setIndices (indicesptr);
    extract.setNegative (false);
    extract.filter (*output_ground);
    extract.setNegative (true);
    extract.filter (*output_vege);
}
void OctreeTerrain:: execute()
{

//qWarning()<<"octree terrain starts";
emit percentage( 5);
      //velky cyklus
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp2(new pcl::PointCloud<pcl::PointXYZI>);
    octree(m_resolution*5, m_baseCloud->get_Cloud(),cloud_tmp, cloud_tmp2);
    cloud_tmp2->points.clear();
emit percentage( 20);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp3(new pcl::PointCloud<pcl::PointXYZI>);
    octree(m_resolution/2, cloud_tmp,cloud_tmp3, cloud_tmp2);
    cloud_tmp2->points.clear();
    cloud_tmp->points.clear();
emit percentage( 35);
    //maly cyklus
    octree(m_resolution, m_baseCloud->get_Cloud(),cloud_tmp, cloud_tmp2);
    cloud_tmp2.reset();
emit percentage( 50);

// porovnat maly a velky cyklus
std::vector<int> pointID_ground;

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (cloud_tmp);

    #pragma omp parallel
    {
      std::vector<int> points_ground;
      #pragma omp for nowait //fill vec_private in parallel
      for(int i=0; i < cloud_tmp3->points.size();i++)
      {
        pcl::PointXYZI searchPointV;
        searchPointV=cloud_tmp3->points.at(i);
        std::vector<int> pointIDv;
        std::vector<float> pointSDv;
      if(kdtree.radiusSearch(searchPointV,0.001,pointIDv,pointSDv) > 0)
          points_ground.push_back(i);
      }
      #pragma omp critical
      {
        if(points_ground.size() > 0)
          pointID_ground.insert(pointID_ground.end(), points_ground.begin(), points_ground.end());
      }
    }
emit percentage( 60);
    boost::shared_ptr<std::vector<int> > indices_ground (new std::vector<int> (pointID_ground));
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ExtractIndices<pcl::PointXYZI> extract;
       // Extract the inliers
    extract.setInputCloud (cloud_tmp3);
    extract.setIndices (indices_ground );
//terrain
    extract.setNegative (false);
    extract.filter (*cloud_ground);
    cloud_ground->width = cloud_ground->points.size ();
    cloud_ground->height = 1;
    cloud_ground->is_dense = true;
    m_terrain->set_Cloud(cloud_ground);
    cloud_tmp3.reset();
    cloud_tmp.reset();
emit percentage( 70);

// vegetace
    std::vector<int> pointIDS;
    pcl::KdTreeFLANN<pcl::PointXYZI> k;
    k.setInputCloud (m_baseCloud->get_Cloud());

    #pragma omp parallel
    {
      std::vector<int> points_ground;
      #pragma omp for nowait //fill vec_private in parallel
      for(int i=0; i < cloud_ground->points.size();i++)
      {
        pcl::PointXYZI searchPointV;
        searchPointV=cloud_ground->points.at(i);
        std::vector<int> pointIDv;
        std::vector<float> pointSDv;
      if(k.radiusSearch(searchPointV,0.001,pointIDv,pointSDv) > 0)
          points_ground.push_back(pointIDv.at(0));
      }
      #pragma omp critical
      {
        if(points_ground.size() > 0)
          pointIDS.insert(pointIDS.end(), points_ground.begin(), points_ground.end());
      }
    }
emit percentage( 80);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_vege(new pcl::PointCloud<pcl::PointXYZI>);
    boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (pointIDS));
    pcl::ExtractIndices<pcl::PointXYZI> e;
       // Extract the inliers
    e.setInputCloud (m_baseCloud->get_Cloud());
    e.setIndices (indicesptr);
//vege
    e.setNegative (true);
    e.filter (*cloud_vege);
    cloud_vege->width = cloud_vege->points.size ();
    cloud_vege->height = 1;
    cloud_vege->is_dense = true;
    m_vegetation->set_Cloud(cloud_vege);
emit percentage( 90);
    cloud_vege.reset();
    cloud_ground.reset();
emit percentage( 95);
      sendData();
}
void OctreeTerrain::sendData()
{
  emit sendingVegetation(m_vegetation);
  emit sendingTerrain(m_terrain);
  emit percentage( 99);
}
void OctreeTerrain::hotovo()
{
  emit finished();
}

/////VOXELTERAIN
VoxelTerrain::VoxelTerrain()
{
  m_baseCloud = new Cloud();
  m_vegetation = new Cloud();
  m_terrain = new Cloud();
  m_resolution = 0.1;
}
VoxelTerrain:: ~VoxelTerrain()
{
  delete m_baseCloud;
  delete m_vegetation;
  delete m_terrain;
}
void VoxelTerrain:: setResolution(float res)
{
  m_resolution = res;
}
void VoxelTerrain::setBaseCloud(Cloud input)
{
  m_baseCloud->set_Cloud(input.get_Cloud());
}
void VoxelTerrain::setVegetationName(QString name)
{
  m_vegetation->set_name(name);
}
void VoxelTerrain::setTerrainName(QString name)
{
  m_terrain->set_name(name);
}
void VoxelTerrain:: execute()
{
  //pro vstupni cloud
// udelat octree
    pcl::octree::OctreePointCloud<pcl::PointXYZI> oc (m_resolution);
    oc.setInputCloud (m_baseCloud->get_Cloud());
    oc.addPointsFromInputCloud ();
// zjistit vsechny voxely
    std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> > voxels;
    oc.getOccupiedVoxelCenters(voxels);
// pro kazdy voxel zjistit  body a spocitat centroid
    pcl::PointCloud<pcl::PointXYZI>::Ptr centroidCloud (new pcl::PointCloud<pcl::PointXYZI>);// mracno s centroidy vsech clusterů
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> ocsC (m_resolution);
    
    ocsC.setInputCloud (m_baseCloud->get_Cloud());
    ocsC.addPointsFromInputCloud ();
    for (int q =0; q < voxels.size(); q++)
    {
        std::vector< int > ind;
        if(ocsC.voxelSearch(voxels.at(q), ind) >0)
        {
            // create cloud from found points
            pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud (new pcl::PointCloud<pcl::PointXYZI>);// mracno s centroidy vsech clusterů
            for(int w=0;w<ind.size();w++)
            {
                clusterCloud->points.push_back(m_baseCloud->get_Cloud()->points.at(ind.at(w)));
            }
            //compute centroid
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*clusterCloud, centroid);
            pcl::PointXYZI bod;
            bod.x =centroid[0];
            bod.y =centroid[1];
            bod.z =centroid[2];
            bod.intensity = centroid[3];
            centroidCloud->points.push_back(bod);
        }
    }


  // zjistit rozsah x y osy a podle toho hledat voxely ktere jsou nejníž
  double x_max,x_min,y_max,y_min,z_min,z_max;
  oc.getBoundingBox(x_min,y_min,z_min,x_max,y_max,z_max);

  oc.deleteTree();
emit percentage( 20);
  // z voxels udelat mracno bodu
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxels (new pcl::PointCloud<pcl::PointXYZI>);
  cloud_voxels->points.resize(voxels.size());
  #pragma omp parallel for
  for(int r=0; r < voxels.size(); r++)
  {
    cloud_voxels->points.at(r) = voxels.at(r);
  }
  cloud_voxels->width = cloud_voxels->points.size ();
  cloud_voxels->height = 1;
  cloud_voxels->is_dense = true;
emit percentage( 40);
    
  // spis boxsearch a pro kazdy voxel najit sousedy v danem boxu, pokud nenajde žadny bod niž než je on sam uložit jeho ID..
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> ocs (m_resolution);

  ocs.setInputCloud (cloud_voxels);
  ocs.addPointsFromInputCloud ();
  std::vector< int > low_voxels;
  for (int q =0; q < voxels.size(); q++)
  {
    std::vector< int > ind;
    Eigen::Vector3f low(voxels.at(q).x-m_resolution/2, voxels.at(q).y-m_resolution/2,z_min);
    Eigen::Vector3f high(voxels.at(q).x+m_resolution/2, voxels.at(q).y+m_resolution/2,voxels.at(q).z);
    if(ocs.boxSearch(low,high,ind) <3)
    {
        if(ind.size() == 0)
            continue;
      // pokud jsou voxely vyskove pouze res od sebe
        if(ind.size()==1)
            low_voxels.push_back(q);
        
        if(ind.size()==2)   // pokud jsou dva tak spocitat jejich vzdalenost centroidu
        {
            // mensi než resolution - vlozit
            float dist=0;
            pcl::PointXYZI a,b;
            a = centroidCloud->points.at(ind.at(0));
            b = centroidCloud->points.at(ind.at(1));
            dist = std::sqrt( (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.y-b.y)*(a.y-b.y));
            if(dist < m_resolution/2)
                low_voxels.push_back(q);
        }
    }
  }
  emit percentage( 80);
  ocs.deleteTree();

  // jeste by to chtelo trochu prefiltrovat aby byl opravdu jen voxely terenu

  boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (low_voxels));
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_vege (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_terrain (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::ExtractIndices<pcl::PointXYZI> extract;
     // Extract the inliers
  extract.setInputCloud (centroidCloud);
  extract.setIndices (indicesptr);
  extract.setNegative (false);
  extract.filter (*cloud_terrain);
  extract.setNegative (true);
  extract.filter (*cloud_vege);

  m_vegetation->set_Cloud(cloud_vege);
  m_terrain->set_Cloud(cloud_terrain);

   sendData();
emit percentage( 99);
}
void VoxelTerrain:: sendData()
{
  emit sendingVegetation(m_vegetation);
  emit sendingTerrain(m_terrain);
}
void VoxelTerrain:: hotovo()
{
  emit finished();
}

//IDW
IDW::IDW()
{
  m_baseCloud = new Cloud();
  m_output = new Cloud();
  m_resolution = 0.1;
  m_pointsnum = 12;
}
IDW::~IDW()
{
  delete m_baseCloud;
  delete m_output;
}
void IDW:: setResolution(float res)
{
  m_resolution = res;
}
void IDW:: setPointNumber(float num)
{
  m_pointsnum = num;
}
void IDW:: setBaseCloud(Cloud input)
{
  m_baseCloud->set_Cloud(input.get_Cloud());
}
void IDW:: setOutputName(QString name)
{
  m_output->set_name(name);
}
void IDW:: execute()
{
//get resolution of input cloud
  pcl::PointXYZI minp,maxp;
  pcl::getMinMax3D(*m_baseCloud->get_Cloud(),minp,maxp);
    //float res = in->get_intValue()/100.0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_idw (new pcl::PointCloud<pcl::PointXYZI>);


  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> ocs (m_resolution);
  ocs.setInputCloud (m_baseCloud->get_Cloud());
  ocs.addPointsFromInputCloud ();


  float lenght = maxp.x - minp.x + m_resolution;
  int per = 90/lenght;
  int percent =0;
emit percentage(percent+=5);
  for(float i = minp.x; i <  (maxp.x+m_resolution); i= i + m_resolution)
  {
    for(float j = minp.y; j <  (maxp.y+m_resolution);j = j + m_resolution)
    {
      std::vector<int> pIv;
      std::vector<float> pSv;
      pcl::PointXYZI spV;
      float z_coor=0;
      spV.x = i;
      spV.y = j;
      spV.z = (minp.z+maxp.z)/2;
      if(ocs.nearestKSearch(spV, m_pointsnum*3, pIv, pSv) > 0 )
      {

        for(int c=0; c< pIv.size(); c++)
        {
          z_coor +=m_baseCloud->get_Cloud()->points.at(pIv.at(c)).z;
        }
        z_coor/=pIv.size();
      }
      std::vector<int> pointIv;
      std::vector<float> pointSv;
      pcl::PointXYZI searchPointVV;
        // pro dany bod najdi 10 nejblizsich bodu
      searchPointVV.x = i;
      searchPointVV.y = j;
      searchPointVV.z = z_coor;


      if(ocs.nearestKSearch(searchPointVV, m_pointsnum, pointIv, pointSv) > 0 )
      {

        float w_sum = 0;
        float z_sum = 0;
        float intensity = m_baseCloud->get_Cloud()->points.at(pointIv.at(0)).intensity;

        // w_sum
        for(int q =0; q <pointIv.size(); q++)
        {
          float w = 1/pointSv.at(q);
          w_sum+= w;
        }
        //z_sum
        for(int e = 0; e < pointIv.size(); e++)
        {
          float w = 1/pointSv.at(e);
          float z= (w * m_baseCloud->get_Cloud()->points.at(pointIv.at(e)).z)/w_sum ;
          z_sum+= z;
        }
        pcl::PointXYZI bod;
        bod.x= searchPointVV.x;
        bod.y= searchPointVV.y;
        bod.z= z_sum;
        bod.intensity= intensity;
        //#pragma omp critical
        cloud_idw->points.push_back(bod);
        }
      }
      emit percentage(percent+= per);
    }
    cloud_idw->width = cloud_idw->points.size ();
    cloud_idw->height = 1;
    cloud_idw->is_dense = true;

    m_output->set_Cloud(cloud_idw);
    emit percentage(100);
    sendData();
}
void IDW::sendData()
{
  emit sendingoutput( m_output);

}
void IDW:: hotovo()
{
  emit finished();
}

//RadiusOutlierRemoval
RadiusOutlierRemoval:: RadiusOutlierRemoval()
{
  m_baseCloud = new Cloud();
  m_output = new Cloud();
  m_radius = 0.1;
  m_neighbors = 12;
}
RadiusOutlierRemoval:: ~RadiusOutlierRemoval()
{
  delete m_baseCloud;
  delete m_output;
}
void RadiusOutlierRemoval:: setRadius( float radius)
{
  m_radius = radius;
}
void RadiusOutlierRemoval::setType(QString type)
{
    m_type = type;
}
void RadiusOutlierRemoval::setNeighborhood(int n)
{
  m_neighbors = n;
}
void RadiusOutlierRemoval:: setBaseCloud(Cloud input)
{
  m_baseCloud->set_Cloud(input.get_Cloud());
}
void RadiusOutlierRemoval:: setOutputName(QString name)
{
  m_output->set_name(name);
}
void RadiusOutlierRemoval:: execute()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudNewTerrain (new pcl::PointCloud<pcl::PointXYZI>);
emit percentage(1);
    // Create the filtering object
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> ror;
    ror.setInputCloud (m_baseCloud->get_Cloud());
    ror.setRadiusSearch(m_radius);
    ror.setMinNeighborsInRadius(m_neighbors);
    ror.filter (*cloudNewTerrain);
emit percentage(50);
    cloudNewTerrain->width = cloudNewTerrain->points.size ();
    cloudNewTerrain->height = 1;
    cloudNewTerrain->is_dense = true;

    m_output->set_Cloud(cloudNewTerrain);
emit percentage(100);
    sendData();
    

}
void RadiusOutlierRemoval::sendData()
{
  emit sendingoutput( m_output, m_type);
    hotovo();
}
void RadiusOutlierRemoval:: hotovo()
{
  emit finished();
}

//StatOutlierRemoval
StatOutlierRemoval:: StatOutlierRemoval()
{
  m_baseCloud = new Cloud();
  m_output = new Cloud();
  m_mDist = 0.1;
  m_neighbors = 12;
}
StatOutlierRemoval:: ~StatOutlierRemoval()
{
  delete m_baseCloud;
  delete m_output;
}
void StatOutlierRemoval:: setMeanDistance( float dist)
{
  m_mDist = dist;
}
void StatOutlierRemoval::setNeighborhood(int n)
{
  m_neighbors = n;
}
void StatOutlierRemoval:: setBaseCloud(Cloud input)
{
  m_baseCloud->set_Cloud(input.get_Cloud());
}
void StatOutlierRemoval:: setOutputName(QString name)
{
  m_output->set_name(name);
}
void StatOutlierRemoval:: execute()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudNewTerrain (new pcl::PointCloud<pcl::PointXYZI>);
emit percentage(1);
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud (m_baseCloud->get_Cloud());
    sor.setMeanK (m_neighbors);
    sor.setStddevMulThresh (m_mDist);
    sor.filter (*cloudNewTerrain);
emit percentage(70);
    cloudNewTerrain->width = cloudNewTerrain->points.size ();
    cloudNewTerrain->height = 1;
    cloudNewTerrain->is_dense = true;

    m_output->set_Cloud(cloudNewTerrain);
emit percentage(100);
    sendData();

}
void StatOutlierRemoval::sendData()
{
  emit sendingoutput( m_output);
}
void StatOutlierRemoval:: hotovo()
{
  emit finished();
}
Slope::Slope()
{
    m_TerrainCloud = new Cloud();
    m_Output = new Cloud();
    m_Radius = 0.1;
    m_Neighbors = 8;
}
Slope::~Slope()
{
    
}
void Slope::setRadius(float radius)
{
    m_Radius = radius;
}
void Slope::setNeighbors(int i)
{
    m_Neighbors = i;
}
void Slope::setTerrainCloud(Cloud input)
{
    m_TerrainCloud->set_Cloud(input.get_Cloud());
}
void Slope::setOutputName(QString name)
{
    m_Output->set_name(name);
}

void Slope::execute()
{
    // vem mracno
    emit percentage(0);
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (m_TerrainCloud->get_Cloud());
   // m_Output->get_Cloud()->points.resize(m_TerrainCloud->get_Cloud()->points.size());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudNewTerrain (new pcl::PointCloud<pcl::PointXYZI>);
    cloudNewTerrain->points.resize(m_TerrainCloud->get_Cloud()->points.size());
    int procento = m_TerrainCloud->get_Cloud()->points.size()/100.0;
    std::cout<< "procento: " << procento << "\n";
    int step_size   = 100;
    int total_steps = m_TerrainCloud->get_Cloud()->points.size() / step_size + 1;
    
    int steps_completed = 0;
    int sum = 0;
    
#pragma omp parallel
    {
        int local_count = 0;
    
#pragma omp parallel for
        for(int i=0 ; i < m_TerrainCloud->get_Cloud()->points.size(); i++)
        {
            std::vector<int> pointIDv;
            std::vector<float> pointSDv;
            pcl::PointXYZI x = m_TerrainCloud->get_Cloud()->points.at(i);
            float sklon = 0.000000;
            // pro kazdy bod najdi sousedy
            if(m_useRadius == false)
            {
                kdtree.nearestKSearch(x, m_Neighbors, pointIDv, pointSDv);
            }
            else
            {
                kdtree.radiusSearch(x, m_Radius, pointIDv, pointSDv);
            }
            // spocitat skon
            for(int q=1; q < pointSDv.size(); q++)
            {
                float s = computeSlope(x, m_TerrainCloud->get_Cloud()->points.at(pointIDv.at(q)));
                sklon += s;
            }
            // udelat prumer
            float skl = sklon *100/pointSDv.size();
            //std::cout<<"sklon: " << skl<< "\n";
            // ulozit do bodu
            cloudNewTerrain->points.at(i).x = x.x;
            cloudNewTerrain->points.at(i).y = x.y;
            cloudNewTerrain->points.at(i).z = x.z;
            cloudNewTerrain->points.at(i).intensity = skl;
            if (local_count++ % step_size == step_size-1)
            {
                #pragma omp atomic
                ++steps_completed;
                
                if (steps_completed % 100 == 1)
                {
                    #pragma omp critical
                    emit percentage(100.0*steps_completed/total_steps);
                }
            }
        }
    }
    cloudNewTerrain->width = cloudNewTerrain->points.size ();
    cloudNewTerrain->height = 1;
    cloudNewTerrain->is_dense = true;
    m_Output->set_Cloud(cloudNewTerrain);
    sendData();

}
void Slope::sendData()
{
    emit sendingoutput( m_Output);
}
void Slope::hotovo()
{
    emit finished();
}
float Slope::computeSlope(pcl::PointXYZI& a, pcl::PointXYZI& b)
{
    float dist = std::sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) +(a.z-b.z)*(a.z-b.z) );
    if(dist ==0)
        return 0;
    float x =std::abs(a.z-b.z)/dist;
    //std::cout<< "computeslope: " << x << "\n";
    return x;
}
void Slope::useRadius(bool radius)
{
    m_useRadius = radius;
}

Curvature::Curvature()
{
    m_TerrainCloud = new Cloud();
    m_Output = new Cloud();
    m_Radius = 0.1;
    m_Neighbors = 8;
}
Curvature::~Curvature()
{
    
}
void Curvature::setRadius(float radius)
{
    m_Radius = radius;
}
void Curvature::setNeighbors(int i)
{
    m_Neighbors = i;
}
void Curvature::setTerrainCloud(Cloud input)
{
    m_TerrainCloud->set_Cloud(input.get_Cloud());
}
void Curvature::setOutputName(QString name)
{
    m_Output->set_name(name);
}
void Curvature::execute()
{
    // vem mracno
    emit percentage(0);
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (m_TerrainCloud->get_Cloud());
   // m_Output->get_Cloud()->points.resize(m_TerrainCloud->get_Cloud()->points.size());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudNewTerrain (new pcl::PointCloud<pcl::PointXYZI>);
    cloudNewTerrain->points.resize(m_TerrainCloud->get_Cloud()->points.size());
    int procento = m_TerrainCloud->get_Cloud()->points.size()/100.0;
    std::cout<< "procento: " << procento << "\n";
    int step_size   = 100;
    int total_steps = m_TerrainCloud->get_Cloud()->points.size() / step_size + 1;
    
    int steps_completed = 0;
    int sum = 0;
    
#pragma omp parallel
    {
        int local_count = 0;
    
#pragma omp parallel for
        for(int i=0 ; i < m_TerrainCloud->get_Cloud()->points.size(); i++)
        {
            std::vector<int> pointIDv;
            std::vector<float> pointSDv;
            pcl::PointXYZI x = m_TerrainCloud->get_Cloud()->points.at(i);
            float sklon = 0.000000;
            // pro kazdy bod najdi sousedy
            if(m_useRadius == false)
            {
                kdtree.nearestKSearch(x, m_Neighbors, pointIDv, pointSDv);
            }
            else
            {
                kdtree.radiusSearch(x, m_Radius, pointIDv, pointSDv);
            }
            float curv = computeCurvature(pointIDv);
            // ulozit do bodu
            cloudNewTerrain->points.at(i).x = x.x;
            cloudNewTerrain->points.at(i).y = x.y;
            cloudNewTerrain->points.at(i).z = x.z;
            cloudNewTerrain->points.at(i).intensity = curv;
            if (local_count++ % step_size == step_size-1)
            {
                #pragma omp atomic
                ++steps_completed;
                
                if (steps_completed % 100 == 1)
                {
                    #pragma omp critical
                    emit percentage(100.0*steps_completed/total_steps);
                }
            }
        }
    }
    cloudNewTerrain->width = cloudNewTerrain->points.size ();
    cloudNewTerrain->height = 1;
    cloudNewTerrain->is_dense = true;
    m_Output->set_Cloud(cloudNewTerrain);
    sendData();
}
void Curvature::sendData()
{
    emit sendingoutput( m_Output);
}
void Curvature::hotovo()
{
    emit finished();
}
float Curvature::computeSlope(pcl::PointXYZI& a, pcl::PointXYZI& b)
{
    float dist = std::sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) +(a.z-b.z)*(a.z-b.z) );
    if(dist ==0)
        return 0;
    float x =std::abs(a.z-b.z)/dist;
    //std::cout<< "computeslope: " << x << "\n";
    return x;
}
float Curvature::computeCurvature(std::vector<int> vec){
    // spocitat skon
       pcl::PointXYZI a = m_TerrainCloud->get_Cloud()->points.at(vec.at(0));
       float sk=0;
       float smax=a.intensity;
       float smin=a.intensity;
       for(int q=1; q < vec.size(); q++)
       {
           pcl::PointXYZI b = m_TerrainCloud->get_Cloud()->points.at(vec.at(q));
           
           float dist = std::sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) +(a.z-b.z)*(a.z-b.z) );
           float x=0;
           if(dist ==0)
           {x = 0;}
           else
           {x = a.intensity-b.intensity/dist;}
           
           sk += x;
           
           if(b.intensity > smax)
               smax = b.intensity;
           if(b.intensity < smin)
           smin = b.intensity;
       }
       // udelat prumer
       float skl = sk *100/vec.size();
       //skl = smax - smin;
       return skl*M_PI/180.0;// in radians
}
void Curvature::useRadius(bool radius)
{
    m_useRadius = radius;
}

HillShade::HillShade(){
    m_TerrainCloud = new Cloud();
    m_Output = new Cloud();
    m_Radius = 0.1;
    m_Neighbors = 8;
}
HillShade::~HillShade(){
    
}
void HillShade::setRadius(float radius){
    m_Radius = radius;
}
void HillShade::setNeighbors(int i){
    m_Neighbors = i;
}
void HillShade::setTerrainCloud(Cloud input){
    m_TerrainCloud->set_Cloud(input.get_Cloud());
}
void HillShade::setOutputName(QString name){
    m_Output->set_name(name);
}
void HillShade::execute(){
    // vem mracno
    emit percentage(0);
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (m_TerrainCloud->get_Cloud());
    // m_Output->get_Cloud()->points.resize(m_TerrainCloud->get_Cloud()->points.size());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudNewTerrain (new pcl::PointCloud<pcl::PointXYZI>);
    cloudNewTerrain->points.resize(m_TerrainCloud->get_Cloud()->points.size());
    int procento = m_TerrainCloud->get_Cloud()->points.size()/100.0;
    std::cout<< "procento: " << procento << "\n";
    int step_size   = 100;
    int total_steps = m_TerrainCloud->get_Cloud()->points.size() / step_size + 1;
    
    int steps_completed = 0;
    int sum = 0;
    
#pragma omp parallel
    {
        int local_count = 0;
        
#pragma omp parallel for
        for(int i=0 ; i < m_TerrainCloud->get_Cloud()->points.size(); i++)
        {
            std::vector<int> pointIDv;
            std::vector<float> pointSDv;
            pcl::PointXYZI x = m_TerrainCloud->get_Cloud()->points.at(i);
            
            // pro kazdy bod najdi sousedy
            if(m_useRadius == false)
            {
                kdtree.nearestKSearch(x, m_Neighbors, pointIDv, pointSDv);
            }
            else
            {
                kdtree.radiusSearch(x, m_Radius, pointIDv, pointSDv);
            }
            std::vector<float> vec;
            // take whole pointcloud and compute PCA
            // define main vectors
            // smallest should be normal vector of plane
            vec = computeSmallestPCA(pointIDv);
            float aspect = computeAspect(vec);
            // deviation from Z axis means slope - skalar
            //float sklon = computeSlope(vec);
            float slope = computeSlope(pointIDv);
            
            //HIllSAHDE
            float zenith =(90 - 45)*M_PI/180.0;
            float azimuth = 225*M_PI/180.0;// zero is to the right - East - and counter clockwise direction
            float hillShade = 255 *((std::cos(zenith)*std::cos(slope)) + (std::sin(zenith)*std::sin(slope)*std::cos(azimuth-aspect)));
            if(hillShade < 0)
                hillShade = 0;
            // std::cout<<"angle: " <<aspect<<" sklon: " <<sklon<<" hillshade: " <<hillShade<<"\n";
            // udelat prumer
            //float skl = sklon *100/pointSDv.size();
            //std::cout<<"sklon: " << skl<< "\n";
            // ulozit do bodu
            cloudNewTerrain->points.at(i).x = x.x;
            cloudNewTerrain->points.at(i).y = x.y;
            cloudNewTerrain->points.at(i).z = x.z;
            cloudNewTerrain->points.at(i).intensity = hillShade;
            if (local_count++ % step_size == step_size-1)
            {
#pragma omp atomic
                ++steps_completed;
                
                if (steps_completed % 100 == 1)
                {
#pragma omp critical
                    emit percentage(100.0*steps_completed/total_steps);
                }
            }
        }
    }
    cloudNewTerrain->width = cloudNewTerrain->points.size ();
    cloudNewTerrain->height = 1;
    cloudNewTerrain->is_dense = true;
    m_Output->set_Cloud(cloudNewTerrain);
    sendData();
    
}
std::vector<float> HillShade::computeSmallestPCA (std::vector<int> pointsId){
    // create cloud based on indices stored in pointIds from m_TerrainCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ (new pcl::PointCloud<pcl::PointXYZI>);
    cloud_->points.resize(pointsId.size());
    std::vector<float> v {0,0,0};
    if(pointsId.size() < 3)
        return v;
#pragma omp parallel for
    for(int q=0; q < pointsId.size(); q++)
        cloud_->points.at(q) = m_TerrainCloud->get_Cloud()->points.at(pointsId.at(q));
    
    // use PCA  to estimate  pca vectors
    //std::cout<<"PCA\n";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_translated (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PCA<pcl::PointXYZI> pca;
    pca.setInputCloud(cloud_);
    pca.project(*cloud_, *cloud_translated);
    
    pcl::PointXYZI proj_min,proj_max, proj_lmin, proj_lmax,lmin, lmax;
    pcl::getMinMax3D (*cloud_translated, proj_min, proj_max);
    // swap axes
    
    
    // estimate smallest vector
    float eX = std::abs(proj_max.x - proj_min.x);
    float eY = std::abs(proj_max.y - proj_min.y);
    float eZ = std::abs(proj_max.z - proj_min.z);
    
    //estimate two points in the middle of two longer sides
    if(eX < eZ && eX < eY) // if eX is the smallest
    {
        proj_lmin.x =proj_min.x;
        proj_lmin.y =(proj_max.y + proj_min.y)/2;
        proj_lmin.z =(proj_max.z + proj_min.z)/2;
        
        proj_lmax.x =proj_max.x;
        proj_lmax.y =(proj_max.y + proj_min.y)/2;
        proj_lmax.z =(proj_max.z + proj_min.z)/2;
    }
    else if (eY<eX && eY< eZ){
        proj_lmin.x =(proj_max.x + proj_min.x)/2;
        proj_lmin.y =proj_min.y;
        proj_lmin.z =(proj_max.z + proj_min.z)/2;
        
        proj_lmax.x =(proj_max.x + proj_min.x)/2;
        proj_lmax.y =proj_max.y;
        proj_lmax.z =(proj_max.z + proj_min.z)/2;
    }
    else{
        proj_lmin.x =(proj_max.x + proj_min.x)/2;
        proj_lmin.y =(proj_max.y + proj_min.y)/2;
        proj_lmin.z =proj_min.z;
        
        proj_lmax.x =(proj_max.x + proj_min.x)/2;
        proj_lmax.y =(proj_max.y + proj_min.y)/2;
        proj_lmax.z =proj_max.z;
    }
    //reconstruct vector into original space
    pca.reconstruct(proj_lmin, lmin);
    pca.reconstruct(proj_lmax, lmax);
    
    float x,y,z;
    //return smallest vector;
    if(lmin.z < lmax.z){
        x = lmax.x - lmin.x;
        y = lmax.y - lmin.y;
        z = lmax.z - lmin.z;
    }else{
        x = lmin.x - lmax.x;
        y = lmin.y - lmax.y;
        z = lmin.z - lmax.z;
    }
    // std::cout<< x <<" " << y << " " << z << "\n";
    
    v.at(0)= x;
    v.at(1) = y;
    v.at(2) = z;
    return v;
}
void HillShade::sendData(){
    emit sendingoutput( m_Output);
}
void HillShade::hotovo(){
    emit finished();
}
float HillShade::computeSlope(std::vector<float> vec){
    std::vector<float> axisZ{0,0,1};
    
    float del = vec.at(0)*axisZ.at(0) + vec.at(1)*axisZ.at(1) + vec.at(2)*axisZ.at(2);
    float det1 = std::sqrt(vec.at(0)*vec.at(0) + vec.at(1)*vec.at(1) + vec.at(2)*vec.at(2));
    float det2 = std::sqrt(axisZ.at(0)*axisZ.at(0) + axisZ.at(1)*axisZ.at(1) + axisZ.at(2)*axisZ.at(2));
    //std::cout<<"del: " <<del<<" det1: " <<det1<<" det2: " <<det2<<"\n";
    float det = det1*det2;
    float slope= M_PI/2 - del/det;
    if (det ==0)
        slope=0;
    return slope;
}
void HillShade::useRadius(bool radius){
    m_useRadius = radius;
}
float HillShade::computeAspect(std::vector<float> vec){
    
    // its projection into XY plane should give aspect
    if(vec.at(1)==0 && vec.at(0) ==0)
        return 0;
    float angle = std::atan2(vec.at(1), vec.at(0));  //# atan2(y, x) or atan2(sin, cos)
    float aspect = angle;// * 180/M_PI;
    return aspect;
}
float HillShade::computeSlope(std::vector<int> pointsId){
    // spocitat skon
    pcl::PointXYZI a = m_TerrainCloud->get_Cloud()->points.at(pointsId.at(0));
    float sk=0;
    for(int q=1; q < pointsId.size(); q++)
    {
        pcl::PointXYZI b = m_TerrainCloud->get_Cloud()->points.at(pointsId.at(q));
        
        float dist = std::sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) +(a.z-b.z)*(a.z-b.z) );
        float x=0;
        if(dist ==0)
        {x = 0;}
        else
        {x = std::abs(a.z-b.z)/dist;}
        
        sk += x;
    }
    // udelat prumer
    float skl = sk *100/pointsId.size();
    return skl*M_PI/180.0;// in radians
}


TerrainFeatures::TerrainFeatures(){
    m_TerrainCloud = new Cloud();
    m_OutputAVG = new Cloud();
    m_OutputSD = new Cloud();
    m_OutputRange = new Cloud();
    m_Radius = 1.0;
    m_Neighbors = 8;
}
TerrainFeatures::~TerrainFeatures(){
    
}
void TerrainFeatures::setRadius(float radius){
    m_Radius = radius;
}
void TerrainFeatures::setNeighbors(int i){
    m_Neighbors = i;
}
void TerrainFeatures::setTerrainCloud(Cloud input){
    m_TerrainCloud->set_Cloud(input.get_Cloud());
}
void TerrainFeatures::setOutputName(QString name){
    m_OutputAVG->set_name(QString("AVG"));
    m_OutputSD->set_name(QString("Sd"));
    m_OutputRange->set_name(QString("Range"));
}
void TerrainFeatures::setlowerPointLimit (float limit){ m_lowerSizeLimit = limit;}
void TerrainFeatures::setupperPointLimit (float limit){ m_upperSizeLimit = limit;}
void TerrainFeatures::setMinBinaryLimit (float limit){m_lowerLimit = limit;}
void TerrainFeatures::setMaxBinaryLimit (float limit){m_upperLimit = limit;}
void TerrainFeatures::setMinLenghtLimit (float limit){m_lowerSideLimit = limit;}
void TerrainFeatures::setMaxLenghtLimit (float limit){m_upperSideLimit = limit;}
void TerrainFeatures::setMaxAreaLimit (float limit){m_upperAreaLimit = limit;}
void TerrainFeatures::setMinAreaLimit (float limit){m_lowerAreaLimit = limit;}
void TerrainFeatures::setAxisRatioLimit (float limit){m_axisRatioLimit = limit;}
void TerrainFeatures::setAreaRatioLimit (float limit){m_areaRatioLimit = limit;}


void TerrainFeatures::execute(){
   // printValues();
    // vzit curvature terenu
    std::cout<< "terrainDiff execute: \n";
    emit percentage(0);
    // upravit podle limitu
    std::cout<< "computeLimits: \n";
    computeLimits(m_TerrainCloud, m_OutputRange);
    emit percentage(10);
    std::cout<< "computebinary: \n";
    computeBinary(m_TerrainCloud, m_OutputAVG);
    emit percentage(20);
    std::cout<< "findClusters: \n";
    std::vector< std::vector<int> >clusters;
    findClusters(m_OutputAVG, clusters);
    std::cout<< "pocet clusteru: "<< clusters.size() << "\n";
    emit percentage(30);
    
    std::cout<< "filterClustersBySize: \n";
    std::vector< std::vector<int> >clustersSize;
    filterClustersBySize(clusters, clustersSize);
    std::cout<< "pocet Filtered clusteru: "<< clustersSize.size() << "\n";
    emit percentage(40);
    
   // std::cout<< "create clouds from clusters: \n";
   // createCloudsFromClusters(m_OutputAVG, clustersSize);
    
    std::cout<< "filterClustersByPCA: \n";
    std::vector< std::vector<int> >clustersPCA;
    filterClustersByPCA(m_OutputAVG, clustersSize, clustersPCA);
    std::cout<< "pocet Filtered clusteru: "<< clustersPCA.size() << "\n";
    emit percentage(50);
    
    //std::cout<< "create clouds from clusters: \n";
    //createCloudsFromClusters(m_OutputAVG, clustersPCA);
    
    std::cout<< "filterClustersByHull: \n";
    std::vector< std::vector<int> >clustersHull;
    filterClustersByHull(m_OutputAVG, clustersPCA, clustersHull);
    std::cout<< "pocet Filtered clusteru: "<< clustersHull.size() << "\n";
    emit percentage(60);
    
    std::cout<< "create clouds from clusters: \n";
    createCloudsFromClusters(m_OutputAVG, clustersHull);
    emit percentage(70);
    std::cout<< "sendData() \n";
    sendData();
   // return;
  
}
bool TerrainFeatures::computeLimits(Cloud *input, Cloud *output)
{
    // nastavit hranice
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudNewTerrainRange (new pcl::PointCloud<pcl::PointXYZI>);
    cloudNewTerrainRange->points.resize(input->get_Cloud()->points.size());
#pragma omp parallel for
    for(int i=0; i < input->get_Cloud()->points.size(); i++)
    {
        pcl::PointXYZI x = input->get_Cloud()->points.at(i);
     
         cloudNewTerrainRange->points.at(i).x = x.x;
         cloudNewTerrainRange->points.at(i).y = x.y;
         cloudNewTerrainRange->points.at(i).z = x.z;
         
         if(x.intensity <= m_upperLimit && x.intensity >= m_lowerLimit )
            {cloudNewTerrainRange->points.at(i).intensity = x.intensity;}
         else
            {cloudNewTerrainRange->points.at(i).intensity = m_upperLimit;}
    }//for loop
    
    cloudNewTerrainRange->width = cloudNewTerrainRange->points.size ();
    cloudNewTerrainRange->height = 1;
    cloudNewTerrainRange->is_dense = true;
    output->set_Cloud(cloudNewTerrainRange);
    return true;
}
bool TerrainFeatures::computeBinary(Cloud *input, Cloud *output)
{
    // nastavit hranice
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudNewTerrainRange (new pcl::PointCloud<pcl::PointXYZI>);
    cloudNewTerrainRange->points.resize(input->get_Cloud()->points.size());
#pragma omp parallel for
    for(int i=0; i < input->get_Cloud()->points.size(); i++)
    {
        pcl::PointXYZI x = input->get_Cloud()->points.at(i);
     
         cloudNewTerrainRange->points.at(i).x = x.x;
         cloudNewTerrainRange->points.at(i).y = x.y;
         cloudNewTerrainRange->points.at(i).z = x.z;
         
         if(x.intensity <= m_upperLimit && x.intensity >= m_lowerLimit )
            {cloudNewTerrainRange->points.at(i).intensity = 0;} // true
         else
            {cloudNewTerrainRange->points.at(i).intensity = 1;}// false
    }//for loop
    
    cloudNewTerrainRange->width = cloudNewTerrainRange->points.size ();
    cloudNewTerrainRange->height = 1;
    cloudNewTerrainRange->is_dense = true;
    output->set_Cloud(cloudNewTerrainRange);
    return true;
}
bool TerrainFeatures::findClusters(Cloud *input, std::vector< std::vector<int> >& output)
{
    std::vector<bool> usedPoint(input->get_Cloud()->points.size(),false);
    //std::vector< std::vector<int> > clusters;
    
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (input->get_Cloud());
    
    for(int q=0; q < input->get_Cloud()->points.size(); q++)
    {
        if(input->get_Cloud()->points.at(q).intensity == 1 || usedPoint.at(q)==true)
            continue;
        
        std::vector<int> cluster;
        cluster.push_back(q);
        usedPoint.at(q)=true;
        
        for(int w=0; w< cluster.size();w++)
        {
            std::vector<int> pointIDv;
            std::vector<float> pointSDv;
            pcl::PointXYZI x = input->get_Cloud()->points.at(cluster.at(w));
            // search for neighbor points
            kdtree.nearestKSearch(x, m_Neighbors, pointIDv, pointSDv);
            // if neighbor point is not used in other cluster
            // and have true value
            for(int e=0; e < pointIDv.size();e++)
            {
                if(input->get_Cloud()->points.at(pointIDv.at(e)).intensity == 0 && usedPoint.at(pointIDv.at(e))==false)
                {
                    usedPoint.at(pointIDv.at(e))=true;
                    cluster.push_back(pointIDv.at(e));
                }
            }
        }
        output.push_back(cluster);
    }
    std::cout<< "clusters: "<< output.size() <<"\n";
    
    return true;
}
bool TerrainFeatures::filterClustersBySize(std::vector< std::vector<int> >& input, std::vector< std::vector<int> >& output)
{
    for(int i = 0; i < input.size();i++)
    {
        //std::cout<<"pocet bodu: " <<input.at(i).size() << "\n";
        if(input.at(i).size() > m_lowerSizeLimit && input.at(i).size() < m_upperSizeLimit )
            output.push_back(input.at(i));
    }
    return true;
}
bool TerrainFeatures::filterClustersByPCA(Cloud *inputCloud, std::vector< std::vector<int> >& input, std::vector< std::vector<int> >& output)
{
    for(int i =0; i < input.size(); i++)
    {
        float xlen=0, ylen=0;
        float ratio =  computeCurvature(inputCloud, input.at(i),xlen, ylen);

        std::cout<<"ratio: " << ratio << " xlen: " << xlen << " ylen: " << ylen << "\n";
        
        if(ratio < m_axisRatioLimit)
            continue;

        if(xlen > m_lowerSideLimit && xlen < m_upperSideLimit && ylen > m_lowerSideLimit && ylen < m_upperSideLimit)
        {
            output.push_back(input.at(i));
        }
    }
    return true;
}
bool TerrainFeatures::filterClustersByHull(Cloud *inputCloud, std::vector< std::vector<int> >& input, std::vector< std::vector<int> >& output)
{
    for(int i=0; i< input.size(); i++)
    {
        // create cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cl (new pcl::PointCloud<pcl::PointXYZI>);
        for(int q=0; q <input.at(i).size(); q++)
            cl->points.push_back(inputCloud->get_Cloud()->points.at(input.at(i).at(q)));
        
        float convex=0, concave=0;
        computeHulls(cl, convex, concave);
        float ratio = concave/convex;
        std::cout<<"ratio: " << ratio << " convex: " << convex << " concave: " << concave << "\n";
        if(ratio < m_areaRatioLimit)
            continue;
        if((convex > m_lowerAreaLimit && convex < m_upperAreaLimit) || (concave > m_lowerAreaLimit && concave < m_upperAreaLimit ))
            output.push_back(input.at(i));
    }
    return true;
}
bool TerrainFeatures::computeHulls(pcl::PointCloud<pcl::PointXYZI>::Ptr input, float& convexArea, float& concaveArea )
{
    ConcaveHull2 * rr = new ConcaveHull2(input);
    rr->compute();
    convexArea = rr->getConvexArea();
    concaveArea = rr->getConcaveArea();
    //std::cout<< "convex: " << convexArea << " concave: "<< concaveArea << "\n";
    return true;
    //float ratio = rr->getConcaveArea()/ rr->getConvexArea();
}
bool TerrainFeatures::createCloudsFromClusters(Cloud *inputCloud, std::vector< std::vector<int> >& input)
{
    for(int i = 0; i < input.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cl (new pcl::PointCloud<pcl::PointXYZI>);
        for(int q=0; q <input.at(i).size(); q++)
            cl->points.push_back(inputCloud->get_Cloud()->points.at(input.at(i).at(q)));
        
         m_stems.push_back(cl);
    }
    return true;
}

bool TerrainFeatures::computePCA (pcl::PointCloud<pcl::PointXYZI>::Ptr input, float& Xlenght, float& Ylenght)
{
   pcl::PointCloud<pcl::PointXYZI> cl ;
   pcl::PCA<pcl::PointXYZI> pca;
   pca.setInputCloud(input);
   pca.project(*input, cl);

   pcl::PointXYZI proj_min,proj_max;
   pcl::getMinMax3D (cl, proj_min, proj_max);

   float eL = std::abs(proj_max.x - proj_min.x); // delka osy
   float eI = std::abs(proj_max.y - proj_min.y); // delka osy
   float eS = std::abs(proj_max.z - proj_min.z);// delka osy
   float sL = (proj_max.x + proj_min.x)/2; // stred osy
   float sI = (proj_max.y + proj_min.y)/2;// stred osy
   float sS = (proj_max.z + proj_min.z)/2;// stred osy
   float xleng,yleng;
   // sorting
   if(eI < eS)
   {
       float p = eS;
       eS = eI;
       eI = p;
   }
   if(eL < eI)
   {
       float p = eL;
       eL = eI;
       eI = p;
   }
   if(eI < eS)
   {
       float p = eS;
       eS = eI;
       eI = p;
   }
   // compute index
   //float SFFIx = (proj_max.z - proj_min.z)/ (eL + eI + eS);
   xleng=eL;
   yleng=eI;
   float SFFIy = eI/eL;
    return true;
    
}
void TerrainFeatures::sendData(){
    emit sendingoutput( m_OutputAVG);
    emit sendingoutput( m_OutputSD);
    emit sendingoutput( m_OutputRange);
    for(int i=0; i < m_stems.size(); i++)
    {
    //    QString a = QString(m_prefix);
      QString name = QString("cluster_%1").arg(i);
      Cloud *c = new Cloud(m_stems.at(i), name);
      emit sendingoutput( c);
    }
}
void TerrainFeatures::hotovo(){
    emit finished();
}

bool TerrainFeatures::computeStatistics(std::vector<float>& vec, float& avg, float& sd, float& range)
{
    // pro hlavni bod
    pcl::PointXYZI p = m_TerrainCloud->get_Cloud()->points.at(vec.at(0));
    
    float average=0, pocet=0;
    float rangeMin=90, rangeMax=-90;
    // vzit bod a
    for (int a=1; a < vec.size(); a++)
    {
        pcl::PointXYZI pa = m_TerrainCloud->get_Cloud()->points.at(vec.at(a));
        //spocitat vektor smeru pa = u
        float u1 = p.x - pa.x;
        float u2 = p.y - pa.y;
        float u3 = p.z - pa.z;
        
        float x1 = p.x - 0;
        float x2 = p.y - 0;
        float x3 = p.z - 1;
        
        float delenec1 =std::abs(u1*x1 +u2*x2 + u3*x3);
        float delitel1 = std::sqrt(u1*u1 + u2*u2 + u3*u3) + std::sqrt(x1*x1 + x2*x2 + x3*x3);
        if(delitel1 ==0)
            delitel1=0.0000000001;
        
        float uhel1 = std::acos(delenec1/delitel1);
        // vzit bod b
        for (int b=a+1; b < vec.size(); b++)
        {
            pcl::PointXYZI pb = m_TerrainCloud->get_Cloud()->points.at(vec.at(b));
            //spocitat vektor smeru pb = v
            float v1 = p.x - pb.x;
            float v2 = p.y - pb.y;
            float v3 = p.z - pb.z;
                // spocitat vektor smeru pb
                // zjistit uhel
            
            float delenec2 =std::abs(x1*v1 +x2*v2 + x3*v3);
            float delitel2 = std::sqrt(x1*x1 + x2*x2 + x3*x3) + std::sqrt(v1*v1 + v2*v2 + v3*v3);
            if(delitel2 ==0)
                continue;
            float uhel2 = std::acos(delenec2/delitel2);
            
            
            float delenec3 = u1*v1 +u2*v2 + u3*v3;
            float delitel3 = std::sqrt(u1*u1 + u2*u2 + u3*u3) + std::sqrt(v1*v1 + v2*v2 + v3*v3);
            if(delitel3 ==0)
                continue;
            float uhel3 = std::acos(delenec2/delitel2);
            float k = uhel3*2/delitel3;
            
            float uhel = uhel1 - uhel2;
            
            // ukladat max min avg uhel
            pocet++;
            average += k;
            if (uhel3 > rangeMax)
                rangeMax = uhel3;
            if (uhel3 < rangeMin)
            rangeMin = uhel3;
        }
    }
    
    avg = (average/ pocet)*180/ M_PI;
    
    sd = rangeMin*180/ M_PI;
    range = rangeMax*180/ M_PI;
    
    
//
//
//    // avg
//
//
//    #pragma omp parallel for
//    for (int q=0; q < vec.size(); q++)
//    {
//        #pragma omp atomic
//        average += m_TerrainCloud->get_Cloud()->points.at(vec.at(q)).intensity;
//        avgZ += m_TerrainCloud->get_Cloud()->points.at(vec.at(q)).z;
//        if(m_TerrainCloud->get_Cloud()->points.at(vec.at(q)).intensity > rangeMax){
//            #pragma omp atomic
//            rangeMax = m_TerrainCloud->get_Cloud()->points.at(vec.at(q)).intensity;}
//        if(m_TerrainCloud->get_Cloud()->points.at(vec.at(q)).intensity < rangeMin){
//            #pragma omp atomic
//            rangeMin = m_TerrainCloud->get_Cloud()->points.at(vec.at(q)).intensity;}
//    }
//    avg = average / vec.size();
//    avgZ /= vec.size();
//    range = std::abs(rangeMax- rangeMin);
//
//    // standart deviation
//    float sdev2=0;
//     #pragma omp parallel for
//    for (int q=0; q < vec.size(); q++)
//    {
//        float diff = m_TerrainCloud->get_Cloud()->points.at(vec.at(q)).z - avgZ;
//    #pragma omp atomic
//        sdev2 += diff*diff;
//    }
//    sd = std::sqrt(sdev2/(vec.size()-1));
//    //std::cout<< "average: "<< avg << " range : " << range << " sd: " << sd<<"/n";
}
float TerrainFeatures::computeSlope(std::vector<float> vec){
    
}
void TerrainFeatures::useRadius(bool radius){
    m_useRadius = radius;
}
float TerrainFeatures::computeAspect(std::vector<float> vec){
    
}
float TerrainFeatures::computeSlope(std::vector<int> vec){
    // spocitat skon
    pcl::PointXYZI a = m_TerrainCloud->get_Cloud()->points.at(vec.at(0));
    float sk=0;
    float smax=a.intensity;
    float smin=a.intensity;
    for(int q=1; q < vec.size(); q++)
    {
        pcl::PointXYZI b = m_TerrainCloud->get_Cloud()->points.at(vec.at(q));
        
        float dist = std::sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) +(a.z-b.z)*(a.z-b.z) );
        float x=0;
        if(dist ==0)
        {x = 0;}
        else
        {x = a.intensity-b.intensity/dist;}
        
        sk += x;
        
        if(b.intensity > smax)
            smax = b.intensity;
        if(b.intensity < smin)
        smin = b.intensity;
    }
    // udelat prumer
    float skl = sk *100/vec.size();
    //skl = smax - smin;
    return skl*M_PI/180.0;// in radians
}
float TerrainFeatures::computeCurvature(Cloud* cloud ,std::vector<int> vec,float& xleng, float& yleng)
{
    
    // pro vsechny body jistit jejich umisteni k prvnimu bodu a roztridit do 4(8) skupin podle směru
    // pro kazdy směr spocitat krivost
    // najit nejvetsi krivist a kolmou  krivost
    pcl::PointCloud<pcl::PointXYZI>::Ptr input (new pcl::PointCloud<pcl::PointXYZI>);
    for(int i=0; i <vec.size(); i++)
        input->points.push_back(cloud->get_Cloud()->points.at(vec.at(i)));
    
    // pca bodu a urcit pomer os nejmenší / součet všech
    if(input->points.size() < 3)
        return -1;
    
    pcl::PointCloud<pcl::PointXYZI> cloud_ ;
    pcl::PCA<pcl::PointXYZI> pca;
    pca.setInputCloud(input);
    pca.project(*input, cloud_);

    pcl::PointXYZI proj_min,proj_max;
    pcl::getMinMax3D (cloud_, proj_min, proj_max);

    float eL = std::abs(proj_max.x - proj_min.x); // delka osy
    float eI = std::abs(proj_max.y - proj_min.y); // delka osy
    float eS = std::abs(proj_max.z - proj_min.z);// delka osy
    float sL = (proj_max.x + proj_min.x)/2; // stred osy
    float sI = (proj_max.y + proj_min.y)/2;// stred osy
    float sS = (proj_max.z + proj_min.z)/2;// stred osy
    // sorting
    if(eI < eS)
    {
        float p = eS;
        eS = eI;
        eI = p;
    }
    if(eL < eI)
    {
        float p = eL;
        eL = eI;
        eI = p;
    }
    if(eI < eS)
    {
        float p = eS;
        eS = eI;
        eI = p;
    }
    // compute index
    //float SFFIx = (proj_max.z - proj_min.z)/ (eL + eI + eS);
    xleng=eL;
    yleng=eI;
    float SFFIy = eI/eL;

    return SFFIy;
    
}
void TerrainFeatures::computeClusters(){
    std::cout<<"computeClusters \n";
    
    // pro kazdy bod
    // jestli je jednicka zacit prohledávat okoli
    //prohledávat sousedni body
    std::vector<bool> usedPoint(m_OutputAVG->get_Cloud()->points.size(),false);
    std::vector< std::vector<int> > clusters;
    
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (m_OutputAVG->get_Cloud());
    
    for(int q=0; q < m_OutputAVG->get_Cloud()->points.size(); q++)
    {
        if(m_OutputAVG->get_Cloud()->points.at(q).intensity >0 || usedPoint.at(q)==true)
            continue;
        
        std::vector<int> cluster;
        cluster.push_back(q);
        usedPoint.at(q)=true;
        
        for(int w=0; w< cluster.size();w++)
        {
            std::vector<int> pointIDv;
            std::vector<float> pointSDv;
            pcl::PointXYZI x = m_OutputAVG->get_Cloud()->points.at(cluster.at(w));
            
            kdtree.nearestKSearch(x, m_Neighbors, pointIDv, pointSDv);
            for(int e=0; e < pointIDv.size();e++)
            {
                if(m_OutputAVG->get_Cloud()->points.at(pointIDv.at(e)).intensity ==1 && usedPoint.at(pointIDv.at(e))==false)
                {
                    usedPoint.at(pointIDv.at(e))=true;
                    cluster.push_back(pointIDv.at(e));
                }
            }
        }
        clusters.push_back(cluster);
    }
    std::cout<< "clusters: "<< clusters.size() <<"\n";
    // pro kazdy cluster poskladat mracno a zjistit jeho rozmery, plochu, pomer stran
    int pocet=0;
    #pragma omp parallel for
    for(int r=0; r < clusters.size(); r++)
    {
        if (clusters.at(r).size() >6  )
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ (new pcl::PointCloud<pcl::PointXYZI>);
            pocet++;
            std::cout<< pocet <<" velikost clusteru "<< r << " z " << clusters.size() <<" : "<< clusters.at(r).size() <<"\n";
            for(int t=0; t < clusters.at(r).size();t++)
            {
                // mracno
                pcl::PointXYZI bod = m_TerrainCloud->get_Cloud()->points.at(clusters.at(r).at(t));
                cloud_->points.push_back(bod);
            }
            
            // pca a urcit pomer stran
            float xleng,yleng;
            float pomer = computeCurvature(m_TerrainCloud, clusters.at(r),xleng,yleng);
            std::cout<<"pomer: "<< pomer << " x: "<<xleng<< " y: "<<yleng<<"\n";
            
            // zjistit pměr stran
            if(pomer >0.75 && xleng> 6  && xleng < 18 )
            {
                ConcaveHull2 * rr = new ConcaveHull2(m_TerrainCloud->get_Cloud());
                rr->compute();
                float ratio = rr->getConcaveArea()/ rr->getConvexArea();
                
                if(ratio > 0.9)
                {
                    #pragma omp critical
                    m_stems.push_back(cloud_);
                    std::cout<<"ratio: "<< ratio << " x: "<<xleng<< " y: "<<yleng <<"\n";
                }
            }
        }
    }
    std::cout<< "pocet stemu: "<< m_stems.size() <<"\n";
}
float TerrainFeatures::getConvexHullArea(std::vector<int> pId){
    float area =0;
    if (pId.size()<3)
        return area;
    //najit hranicni bod
    int leftmost=0;
    float xCoor = 999999999999999;
std::vector<pcl::PointXYZI> body;
    for(int i=0; i<pId.size(); i++)
    {
        body.push_back(m_TerrainCloud->get_Cloud()->points.at(pId.at(i)));
        if(m_TerrainCloud->get_Cloud()->points.at(pId.at(i)).x < xCoor   )
        {
            xCoor = m_TerrainCloud->get_Cloud()->points.at(pId.at(i)).x;
            leftmost = pId.at(i);
            m_p0 = m_TerrainCloud->get_Cloud()->points.at(pId.at(i));
        }
    }
    std::vector<pcl::PointXYZI> hull;
    hull = convex_hull(body);
    area = polygonArea(body);

    return area;
}
int TerrainFeatures::orientation(pcl::PointXYZI p, pcl::PointXYZI q, pcl::PointXYZI r)
{
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);
  
    if (val == 0) return 0;  // colinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}
float TerrainFeatures::distSq(pcl::PointXYZI p1, pcl::PointXYZI p2)
{
    return (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y);
}
std::vector<pcl::PointXYZI> TerrainFeatures::convex_hull(std::vector<pcl::PointXYZI> p)
{
    int n = p.size(), k = 0;
    std::vector<pcl::PointXYZI> H(2*n);
 
    // Sort points lexicographically
    //sort(p.begin(), p.end(), comp);
    std::sort(p.begin(), p.end(), [](pcl::PointXYZI& a, pcl::PointXYZI& b){
        if (a.x < b.x) return true;
        return false;
    });
 
    // Build lower hull
    for (int i = 0; i < n; i++) {
        while (k >= 2 && orientation(H[k-2], H[k-1], p[i]) <= 0) k--;
        H[k++] = p[i];
    }
 
    // Build upper hull
    for (int i = n-2, t = k+1; i >= 0; i--) {
        while (k >= t && orientation(H[k-2], H[k-1], p[i]) <= 0) k--;
        H[k++] = p[i];
    }
    H.resize(k);
    return H;
}
int TerrainFeatures::comp(pcl::PointXYZI p1, pcl::PointXYZI p2) {
   int dir = orientation(m_p0, p1, p2);
   if(dir == 0)
      return (distSq(m_p0, p2) >= distSq(m_p0, p1))?-1 : 1;
   return (dir==2)? -1 : 1;
}
float TerrainFeatures::polygonArea(std::vector<pcl::PointXYZI>& p)
{
    float area = 0.0;
    int j = p.size()-1;
    for (int i = 0; i < p.size()-1; ++i)
    {
       float a= (p.at(i).x + p.at(j).x) *  (p.at(j).y - p.at(i).y);
        area += std::abs(a/2);
        //cout<<"x: "<< p.at(i).x << " y: "<< p.at(i).y << " x2: "<< p.at(j).x << " y2: " << p.at(j).y<< " area: "<< a <<"\n";
        j = i;
    }
   // cout<< " plocha: "<< area << "\n";
    return area/2;
}
void TerrainFeatures::printValues(){
    std::cout << "binary limit: "<< m_lowerLimit << " - " << m_upperLimit << "\n";
    std::cout << "Point size limit: "<< m_lowerSizeLimit << " - " << m_upperSizeLimit <<"\n";
    std::cout << "axis limit: "<< m_lowerSideLimit << " - " << m_upperSideLimit <<" ratio: "<<  m_axisRatioLimit << "\n";
    std::cout << "area limit: "<< m_lowerAreaLimit << " - " << m_upperAreaLimit <<" ratio: "<<  m_areaRatioLimit << "\n";
}
