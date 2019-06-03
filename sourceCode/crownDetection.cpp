#include "crownDetection.h"

CrownAutomaticDetection::CrownAutomaticDetection(pcl::PointCloud<pcl::PointXYZI>::Ptr tree,stred dbhLSR,pcl::PointXYZI treePosition)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(CloudOperations::getCloudCopy(tree));
    m_crown = cloud;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);
    m_stem = cloud2;

    m_sectionHeight = 0.5;
    cloudHighestAndLowestZValue hl = GeomCalc::findHighestAndLowestPoints(cloud);
    computeDiameters(cloud,hl.Highest,hl.Lowest);
    findDilatation();
    m_startHeightForDetailedSearch +=hl.Lowest;
    separateStemPoints(cloud,hl.Lowest);

    detectCrown(cloud);
}
pcl::PointCloud<pcl::PointXYZI>::Ptr CrownAutomaticDetection::getCrown()
{
    return m_crown;
}
pcl::PointXYZI CrownAutomaticDetection::getStemHighestPoint()
{
    return m_stemHighestPoint;
}
void CrownAutomaticDetection::detectCrown(pcl::PointCloud<pcl::PointXYZI>::Ptr tree)
{
    float zmax = m_startHeightForDetailedSearch +0.1;
    stred s1 = getDiameterFrom(tree,m_startHeightForDetailedSearch,zmax);
    zmax += 0.1;
    stred s = getDiameterFrom(tree,zmax-0.1,zmax);
    pcl::PointXYZI sPoint = predictNextStemCenter(s1,s);
    float a=0;
    do
    {

        if (a >0) {s1 =s;}
        pcl::PointCloud<pcl::PointXYZI>::Ptr selectedPoints (new pcl::PointCloud<pcl::PointXYZI>);
        float n = (float) tree->points.size();
        for (int i=0; i<n; i++)
        {
            pcl::PointXYZI bod = tree->points.at(i);
            if (bod.z < zmax)
            {
                float dist =GeomCalc::computeDistance2Dxy(bod,sPoint);
                dist *=100; //prevod na cm
                if (a == 0) {dist =0;}
                if (dist < (s1.r*2))
                {
                    selectedPoints->points.push_back(bod);
                    m_stem->points.push_back(bod);
                    tree->points.erase(tree->points.begin()+i);
                    n--;
                    if(i>0){i--;}
                }
            }
        }
        s = getCenterByLSR(selectedPoints);
        sPoint =predictNextStemCenter(s1,s);
        a++;
        zmax +=0.1;
    }while((s1.r*1.25) > s.r);
    cloudHighestAndLowestZValue hl = GeomCalc::findHighestAndLowestPoints(m_stem);
    m_stemHighestPoint = hl.highestPoint;
    //QString xxx = QString("stem highest %1\n l %2 h %3").arg(m_stemHighestPoint).arg(hl.Lowest).arg(hl.Highest);
    //QMessageBox::information(0,("WARNING"),xxx);
}
stred CrownAutomaticDetection::getDiameterFrom(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float zmin, float zmax)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr selectedPoints (new pcl::PointCloud<pcl::PointXYZI>);
    float n = (float) cloud->points.size();
    for(int i=0; i<n;i++)
    {
        pcl::PointXYZI p = cloud->points.at(i);
        if(p.z > zmin && p.z<zmax){
            selectedPoints->points.push_back(p);
            cloud->points.erase(cloud->points.begin()+i);
            n--;
            i--;
        }
    }
    stred s = getCenterByLSR(selectedPoints);
    return s;
}
void CrownAutomaticDetection::findDilatation()
{
    for(int i=1; i<m_diff.size(); i++)
    {
        if(m_diff.at(i)>(m_diff.at(i-1)*1.25))
        {
            if(i<m_diff.size()-1 && m_diff.at(i+1)<m_diff.at(i)){
                continue;
            }else if(i<m_diff.size()-2 && (m_diff.at(i+2)<m_diff.at(i-1) || m_diff.at(i+2)<m_diff.at(i-1)*1.25)){
               continue;
            }else{
                m_startHeightForDetailedSearch = (i-1)*m_sectionHeight;
                break;
            }
        }
    }
}
void CrownAutomaticDetection::computeDiameters(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float cloudHighest, float cloudLowest)
{
    float zmin = cloudLowest;
      do{
            pcl::PointCloud<pcl::PointXYZI>::Ptr test (new pcl::PointCloud<pcl::PointXYZI>);
            for (int i=0; i<cloud->points.size(); i++)
            {
                pcl::PointXYZI bod = cloud->points.at(i);
                if (bod.z > zmin && bod.z < (zmin+m_sectionHeight)){
                test->points.push_back(bod);
                }
            }
       // QString xx = QString("size %1").arg(test->points.size());
       // QMessageBox::information(0,("WARNING"),xx);
            zmin+=m_sectionHeight;
            if(test->points.size()>2){
            m_diff.push_back((computeXAxisExtension(test)+computeYAxisExtension(test))/2);
            }else m_diff.push_back(0);
       // QString x = QString("zmin %1 %2").arg(zmin).arg(cloudHighest);
       // QMessageBox::information(0,("WARNING"),x);
    }while(zmin<cloudHighest);
/*
    int rows = m_diff.size();
    QStandardItemModel *model = new QStandardItemModel(rows,3);
    for(int i=0;i<m_diff.size();i++)
    {
        QModelIndex index= model->index(i,0,QModelIndex());
        model->setData(index,m_diff.at(i));
    }
        QTableView *attributeTable = new QTableView;
        attributeTable->setModel(model);
        attributeTable->showNormal();*/
}
float CrownAutomaticDetection::computeXAxisExtension(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    float xmin = cloud->points.at(0).x;
    float xmax = cloud->points.at(0).x;
    for(int i=0; i<cloud->points.size();i++)
    {
        if(cloud->points.at(i).x > xmax){
            xmax = cloud->points.at(i).x;
        }
        if(cloud->points.at(i).x < xmin){
            xmin = cloud->points.at(i).x;
        }
    }
    return xmax-xmin;
}
float CrownAutomaticDetection::computeYAxisExtension(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    float ymin = cloud->points.at(0).y;
    float ymax = cloud->points.at(0).y;
    for(int i=0; i<cloud->points.size();i++)
    {
        if(cloud->points.at(i).y > ymax){
            ymax = cloud->points.at(i).y;
        }
        if(cloud->points.at(i).y < ymin){
            ymin = cloud->points.at(i).y;
        }
    }
    return ymax-ymin;
}
void CrownAutomaticDetection::separateStemPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float cloudLowest)
{
    int s = cloud->points.size();
    for(int i=0; i<s;i++)
    {
        pcl::PointXYZI p = cloud->points.at(i);
        if(p.z<m_startHeightForDetailedSearch)
        {
            cloud->points.erase(cloud->points.begin()+i);
            s--;
            i--;
            m_stem->points.push_back(p);
        }
    }
}
stred CrownAutomaticDetection::getCenterByLSR(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    LeastSquaredRegression *lr = new LeastSquaredRegression();
    lr->setCloud(cloud);
    lr->compute();
    return lr->getCircle();
}
void CrownAutomaticDetection::erasePointsUnder1_3m(pcl::PointCloud<pcl::PointXYZI>::Ptr tree, stred dbhLSR)
{
    float n = (float) tree->points.size();
    for (int i=0; i<n; i++)
        {
            pcl::PointXYZI bod = tree->points.at(i);
            if (bod.z < dbhLSR.z)
            {
                m_stem->points.push_back(bod);
                tree->points.erase(tree->points.begin()+i);
                n--;
                i--;
            }
        }
}
pcl::PointXYZI CrownAutomaticDetection::predictNextStemCenter(stred A, stred B)
{
    float vx =B.a-A.a;
    float vy =B.b-A.b;
    float vz =B.z-A.z;

    pcl::PointXYZI point;
    point.x =B.a+vx;
    point.y =B.b+vy;
    point.z =B.z+vz;

    return point;
}
/*void CrownAutomaticDetection::computeCrownHeights(pcl::PointXYZI treePosition)
{
    //Find highest and lowest point in cloud
    float Highest = 0;
    float Lowest = 0;
    cloudHighestAndLowestZValue crown = GeomCalc::findHighestAndLowestPoints(m_crown);
    cloudHighestAndLowestZValue stem = GeomCalc::findHighestAndLowestPoints(m_stem);
    //Compute and set heights
    m_totalHeight = crown.Highest-crown.Lowest;
    m_bottomHeight = stem.Highest-treePosition.z;
    m_height = crown.Highest-stem.Highest;
}*/




