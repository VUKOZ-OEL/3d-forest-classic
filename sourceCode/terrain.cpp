#include "terrain.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/centroid.h>

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
