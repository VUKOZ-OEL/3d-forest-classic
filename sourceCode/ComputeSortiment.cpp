//
//  ComputeSortiment.cpp
//  3DForest043
//
//  Created by Jan Trochta on 26/10/2018.
//

#include "ComputeSortiment.h"
#include "sortiment.h"


ComputeSortiment::ComputeSortiment()
{
    
}
ComputeSortiment::ComputeSortiment(std::vector<std::shared_ptr< Segment>> branches, bool finalSortiment)
{
    m_branches = branches;
    setFinalSortiment(finalSortiment);
    for(int i=0; i <m_branches.size(); i++)
        m_branches.at(i)->clearSortiments();
}
ComputeSortiment::~ComputeSortiment()
{
    
}
void ComputeSortiment::setFinalSortiment( bool finalSortiment)
{
    m_finalSortiment = finalSortiment;
}
void ComputeSortiment::compute()
{
   // std::cout<< "sortiment compute()\n";
    //std::cout << "m_finalSortiment: " << m_finalSortiment<< "\n";
    
    //  setSortimentsSize();
    // vem segment
    for(int i=0; i < m_branches.size(); i++)
    {
        //std::cout<<"branch: " << i << "\n";
        // pro kazdy segment
        //nastavit data a struktury
            // pro kazdy cylinder
                // spocitat parametry
            // pokud !finalSortiments
                //ulozit kazdy cylinder do sortimentu s jeho hodnotama
            // jinak spočitat final sortiments
                // ulozit az posledni vector cylinderu do sortiments
        
        
        // zakladni nastaveni vectoru a dat
        // vektor sortimentu, ktery se bude ukladat do branches
        std::vector<std::shared_ptr<Sortiment>> sorts;
        // vsechny cylindery segmentu
        std::vector<pcl::ModelCoefficientsPtr> cylinders;
        cylinders = m_branches.at(i)->getCylinders();
        // pokud malý segment, nic nepočítat
        if (cylinders.size() < 3)
            continue;
        // pruměrná výška válce
        float vyskaValce =0;
        for(int w=0; w < cylinders.size();w++)
        {
            vyskaValce+= std::sqrt(
                      (cylinders.at(w)->values.at(3) * cylinders.at(w)->values.at(3)) +
                      (cylinders.at(w)->values.at(4) * cylinders.at(w)->values.at(4)) +
                      (cylinders.at(w)->values.at(5) * cylinders.at(w)->values.at(5)) );
        }
        vyskaValce/=cylinders.size();
        
        
        // zjistit deti a kde se napojují
        std::vector<pcl::ModelCoefficientsPtr> suky;
        sukyVetve(suky, i);
        // pro kazdy cylinder  jeho hodnoty
        std::vector<int> sukatostC(cylinders.size(), -1);
        std::vector<int> krivostC(cylinders.size(),-1);
        std::vector<int> taperC(cylinders.size(),-1);
        std::vector<int> sbihavostC(cylinders.size(),-1);
        std::vector<int> qualityC(cylinders.size(),-1);
        
        //pro kazdy cylinder
        for(int q=0; q < cylinders.size();q++)
        {
            // cylinder.at(q); // pro tento cylinder
            std::vector<pcl::ModelCoefficientsPtr> metr; // vector valcu do vzdalenosti 1 m
            std::vector<int> cylinderIDs;
            std::vector<float> cylinderDist;
            // zjistit okolní cylindery ve vzdalenosti 1 m
            for(int w=0; w < cylinders.size();w++)
            {
                if(w==q)
                    continue;
                float dist = cylinderDistance(cylinders.at(q), cylinders.at(w));
                if(dist < 0.5+ vyskaValce/2)
                {
                    cylinderIDs.push_back(w);
                    cylinderDist.push_back(dist);
                }
            }
            
            // std::cout<<"velikost okolni cylinderu: "<< cylinderIDs.size()<<"\n";
            if(cylinderIDs.size() < 2)
                continue;
            // get biggest distance between cylinders
            int celo, cep;
            float d=0;
            for(int e=0; e < cylinderIDs.size(); e++)
            {
                // ulozit vsechny cylindery  do vectoru metr
                metr.push_back(cylinders.at(cylinderIDs.at(e)));
                
                //najit dva nejvzálenejsi cylindery
                for(int r=0; r < cylinderIDs.size(); r++)
                {
                    if(r==e)
                        continue;
                    float dist =cylinderDistance(cylinders.at(cylinderIDs.at(e)), cylinders.at(cylinderIDs.at(r)));
                    if(dist > d)
                    {
                        celo = cylinderIDs.at(e);
                        cep = cylinderIDs.at(r);
                        d = dist;
                    }
                }
            }
            
            //sukatost
            int suk = sukatost(metr, suky);
            sukatostC.at(q)=suk;
            // sbihavost
            int sbihavost = skewness(cylinders.at(cep), cylinders.at(celo));
            sbihavostC.at(q) = sbihavost;
            // krivost
            pcl::ModelCoefficientsPtr line = lineCoefficient(cylinders.at(cep), cylinders.at(celo));
            int krivost = curvature( line, cylinders.at(q));
            krivostC.at(q) = krivost;
            //velikost cela
            int celoRadius =diameterSize(cylinders.at(cep), cylinders.at(celo));
            taperC.at(q) = celoRadius;
            //quality
            int sortiment = sbihavost;
            
            if(krivost > sortiment)
                sortiment = krivost;
            if(celoRadius > sortiment)
                sortiment = celoRadius;
            if(suk > sortiment)
                sortiment = suk;
            qualityC.at(q) = sortiment;
        }
       // std::cout << "for(int q=0; q < cylinders.size();q++) hotovo\n";
        // pokud !finalSortiments
        //ulozit kazdy cylinder do sortimentu s jeho hodnotama
        if(m_finalSortiment == false)
        {
            //std::cout << "m_finalSortiment2: " << m_finalSortiment<<" cylinders.size " <<cylinders.size() << "\n";
            for(int r=0; r < cylinders.size(); r++)
            {
               // std::cout << "r: " << r<< "\n";
                if(sukatostC.at(r) == -1)
                    continue;
                std::vector<pcl::ModelCoefficientsPtr> cyl;
                cyl.push_back(cylinders.at(r));
                m_branches.at(i)->setSortiment(std::shared_ptr<Sortiment>(new Sortiment(cyl, krivostC.at(r), sukatostC.at(r), taperC.at(r), sbihavostC.at(r), qualityC.at(r))));
            }
            //std::cout << "m_finalSortiment == false hotovo\n";
        }
        else
        {
           // std::cout << "m_finalSortiment2: " << m_finalSortiment<< "\n";
            std::vector<bool> usedSortiment ( cylinders.size(), false);
            for(int sortiment = 1; sortiment < 7; sortiment++)
            {
                //std::cout<<"sortimenty quality: " << sortiment << "\n";
                for(int p=0; p < qualityC.size();p++)
                {
                    //std::cout<<"p: "<< p << " quality: "<< qualityC.at(p)<< " used: " << usedSortiment.at(p) << "\n";
                    if(qualityC.at(p) == sortiment && usedSortiment.at(p) == false)
                    {
                        int vel= continuosSortiment( qualityC, p, sortiment, usedSortiment);
                        float delka = std::abs(vel*vyskaValce);
                        int m=p;
                        //std::cout<< "vel: "<< vel << " delka: " << delka << " m: " << m << "\n";
                        
                        // std::cout<<"switch\n";
                        switch (sortiment)
                        {
                            case 1:
                                if(delka > 3)
                                {
                                    std::vector<pcl::ModelCoefficientsPtr> cyl;
                                    for(m; m < p+vel; m++)
                                    {
                                        cyl.push_back(cylinders.at(m));
                                        usedSortiment.at(m) = true;
                                    }
                                    m_branches.at(i)->setSortiment(std::shared_ptr<Sortiment>(new Sortiment(cyl, krivostC.at(p), sukatostC.at(p), taperC.at(p), sbihavostC.at(p), sortiment)));
                                }
                                else{
                                    for(m; m < p+vel; m++)
                                        qualityC.at(m) = 2;
                                    //std::cout<<"sortiment "<< sortiment << " velikost: " << delka <<"\n";
                                    
                                }
                                break;
                            case 2:
                                if(delka > 3)
                                {
                                    std::vector<pcl::ModelCoefficientsPtr> cyl;
                                    for(m; m < p+vel; m++)
                                    {
                                        cyl.push_back(cylinders.at(m));
                                        usedSortiment.at(m) = true;
                                    }
                                    m_branches.at(i)->setSortiment(std::shared_ptr<Sortiment>(new Sortiment(cyl, krivostC.at(p), sukatostC.at(p), taperC.at(p), sbihavostC.at(p), sortiment)));
                                }
                                else{
                                    for(m; m < p+vel; m++)
                                        qualityC.at(m) = 3;
                                    //std::cout<<"sortiment "<< sortiment << " velikost: " << delka <<"\n";
                                }
                                break;
                            case 3:
                                if(delka > 2.5)
                                {
                                    std::vector<pcl::ModelCoefficientsPtr> cyl;
                                    for(m; m < p+vel; m++)
                                    {
                                        cyl.push_back(cylinders.at(m));
                                        usedSortiment.at(m) = true;
                                    }
                                    m_branches.at(i)->setSortiment(std::shared_ptr<Sortiment>(new Sortiment(cyl, krivostC.at(p), sukatostC.at(p), taperC.at(p), sbihavostC.at(p), sortiment)));
                                }
                                else{
                                    for(m; m < p+vel; m++)
                                        qualityC.at(m) = 4;
                                    //std::cout<<"sortiment "<< sortiment << " velikost: " << delka <<"\n";
                                }
                                break;
                            case 4:
                                if(delka > 2)
                                {
                                    std::vector<pcl::ModelCoefficientsPtr> cyl;
                                    for(m; m < p+vel; m++)
                                    {
                                        cyl.push_back(cylinders.at(m));
                                        usedSortiment.at(m) = true;
                                    }
                                    m_branches.at(i)->setSortiment(std::shared_ptr<Sortiment>(new Sortiment(cyl, krivostC.at(p), sukatostC.at(p), taperC.at(p), sbihavostC.at(p), sortiment)));
                                }
                                else{
                                    for(m; m < p+vel; m++)
                                        qualityC.at(m) = 5;
                                    //std::cout<<"sortiment "<< sortiment << " velikost: " << delka <<"\n";
                                }
                                break;
                            case 5:
                                if(delka > 1)
                                {
                                    std::vector<pcl::ModelCoefficientsPtr> cyl;
                                    for(m; m < p+vel; m++)
                                    {
                                        cyl.push_back(cylinders.at(m));
                                        usedSortiment.at(m) = true;
                                    }
                                   // std::cout<< "cylinder.size: " << cyl.size()<<"\n";
                                    m_branches.at(i)->setSortiment(std::shared_ptr<Sortiment>(new Sortiment(cyl, krivostC.at(p), sukatostC.at(p), taperC.at(p), sbihavostC.at(p), sortiment)));
                                    //std::cout<<"branches cylinder size: " << m_branches.at(i)->getSortiments().at(m_branches.at(i)->getSortiments().size()-1)->getCylinders().size()<<"\n";
                                }
                                else{
                                    for(m; m < p+vel; m++)
                                        qualityC.at(m) = 6;
                                    // std::cout<<"sortiment "<< sortiment << " velikost: " << delka <<"\n";
                                }
                                break;
                            case 6:
                                if(delka > 0)
                                {
                                   // std::cout<< "vel: "<< vel << " m "<< m << " p "<< p << " i "<< i <<" pocet cylinderu: "<< cylinders.size() << "\n";
                                    std::vector<pcl::ModelCoefficientsPtr> cyl;
                                    for(m; m < p+vel; m++)
                                    {
                                        cyl.push_back(cylinders.at(m));
                                        usedSortiment.at(m) = true;
                                    }

                                    
                                    m_branches.at(i)->setSortiment(std::shared_ptr<Sortiment>(new Sortiment(cyl, krivostC.at(p), sukatostC.at(p), taperC.at(p), sbihavostC.at(p), sortiment)));
                                    //std::cout<< " ulozeno\n";
                                }
                                break;
                            default:
                                break;
                        } //SWITCH
                    } //if
                }//for sortiments
            }// for sortiment
           // std::cout << "m_finalSortiment == true hotovo\n";
        }
        //std::cout << "m_branch.at("<< i << ") hotovo\n";
    }
    //std::cout<< "sortiment compute() hotovo\n";
}
int ComputeSortiment::diameterSize(std::vector<pcl::ModelCoefficientsPtr> cylinders)
{
    // find smaller radius
    int sor=6;
    float cep =cylinders.at(0)->values.at(6);
    if (cylinders.at(cylinders.size()-1)->values.at(6)  < cep )
        cep =cylinders.at(cylinders.size()-1)->values.at(6);
    
    if(cep*2 > 0.03)
        sor = 5;
    if(cep*2 > 0.07)
        sor = 4;
    if(cep*2 > 0.2)
        sor = 3;
    if(cep*2 > 0.28)
        sor = 2;
    if(cep*2 > 0.45)
        sor = 1;
    
    return sor;
}
int ComputeSortiment::diameterSize(pcl::ModelCoefficientsPtr celo, pcl::ModelCoefficientsPtr cep)
{
    int sor=6;
    float radius = celo->values.at(6);
    if (celo->values.at(6)  < radius )
        radius =celo->values.at(6);
    
    if(radius*2 > 0.03)
        sor = 5;
    if(radius*2 > 0.07)
        sor = 4;
    if(radius*2 > 0.2)
        sor = 3;
    if(radius*2 > 0.28)
        sor = 2;
    if(radius*2 > 0.45)
        sor = 1;
    
    return sor;
}
int ComputeSortiment::curvature(std::vector<pcl::ModelCoefficientsPtr> cylinders)
{
    pcl::ModelCoefficientsPtr linie = lineCoefficient(cylinders.at(0), cylinders.at(cylinders.size()-1));
    float maxKrivost = 0;
    for(int r=0; r < cylinders.size(); r++)
    {
        float krivost = curvatureDiff(linie, cylinders.at(r));
        if(krivost > maxKrivost)
            maxKrivost = krivost;
    }
    int sor = 6;
    if(maxKrivost<0.1)
        sor = 5;
    if(maxKrivost<0.06)
        sor = 4;
    if(maxKrivost<0.03)
        sor = 3;
    if(maxKrivost<0.02)
        sor = 2;
    if(maxKrivost<0.015)
        sor = 1;
    
    return sor;
}
int ComputeSortiment::curvature(pcl::ModelCoefficientsPtr line, pcl::ModelCoefficientsPtr cylinder)
{
    float krivost = curvatureDiff(line, cylinder);
    int sor = 6;
    if(krivost<0.1)
        sor = 5;
    if(krivost<0.06)
        sor = 4;
    if(krivost<0.03)
        sor = 3;
    if(krivost<0.02)
        sor = 2;
    if(krivost<0.015)
        sor = 1;
    
    return sor;
}
int ComputeSortiment::sortimentCurvature(std::vector<pcl::ModelCoefficientsPtr> cylinders)
{
    pcl::ModelCoefficientsPtr linie = lineCoefficient(cylinders.at(0), cylinders.at(cylinders.size()-1));
    
    float maxKrivost = 0;
    float v=0;
    for(int r=0; r < cylinders.size(); r++)
    {
        v += std::sqrt((cylinders.at(r)->values.at(3) * cylinders.at(r)->values.at(3)) +
                      (cylinders.at(r)->values.at(4) * cylinders.at(r)->values.at(4)) +
                      (cylinders.at(r)->values.at(5) * cylinders.at(r)->values.at(5)) );
        float krivost = curvatureDiff(linie, cylinders.at(r));
        if(krivost > maxKrivost)
            maxKrivost = krivost;
    }
    int sor = 6;
    if(maxKrivost/v<0.1)
        sor = 5;
    if(maxKrivost/v<0.06)
        sor = 4;
    if(maxKrivost/v<0.03)
        sor = 3;
    if(maxKrivost/v<0.02)
        sor = 2;
    if(maxKrivost/v<0.015)
        sor = 1;
    
    return sor;
    
}
int ComputeSortiment::skewness(std::vector<pcl::ModelCoefficientsPtr> cylinders)
{
    
    float sbihavost =  radiusDiff(cylinders.at(0), cylinders.at(cylinders.size()-1));
    int sor = 3;
    if(sbihavost < 1)
        sor = 1;
    return sor;
}
int ComputeSortiment::skewness(pcl::ModelCoefficientsPtr celo, pcl::ModelCoefficientsPtr cep)
{
    float sbihavost = radiusDiff(celo, cep);
    int sor = 3;
    if(sbihavost < 1)
        sor = 1;
    return sor;
}

int ComputeSortiment::sukatost(std::vector<pcl::ModelCoefficientsPtr> cylinders, std::vector<pcl::ModelCoefficientsPtr> suky)
{
    int pocetSuku =0;
    int sukyDo3cm =0;
    int suky4cm = 0;
    int sukydo10cm = 0;
    int sukynad10cm=0;
    for(int q=0;q < cylinders.size();q++)
    {
        for(int w=0; w < suky.size();w++)
        {
            if(cylinderTouch(cylinders.at(q), suky.at(w)) == true)
            {
                pocetSuku++;
                if(suky.at(w)->values.at(6)*2 > 0.1 )
                {
                    sukynad10cm++;
                }
                else if (suky.at(w)->values.at(6)*2 < 0.1 && suky.at(w)->values.at(6)*2 > 0.04 )
                {
                    sukydo10cm++;
                }
                else if (suky.at(w)->values.at(6)*2 < 0.04 && suky.at(w)->values.at(6)*2 > 0.03)
                {
                    suky4cm++;
                }
                else
                {
                    sukyDo3cm++;
                }
            }
        }
    }
    
    int sor=4;
    if(pocetSuku >0 && sukydo10cm == 0 && sukynad10cm ==0)
        sor =3;
    if(pocetSuku >0 && suky4cm < 2 && sukydo10cm == 0 && sukynad10cm ==0)
        sor = 2;
    if(pocetSuku ==0)
        sor = 1;
    
    return sor;
    
    
    //pro kazdy cylinders
    // pokud je ve stejnem miste jako suky
    // ++ pocet suku
    // ulozit velikost
    
    
    
}
float ComputeSortiment::pointDiff(pcl::PointXYZI a, pcl::PointXYZI b)
{
    //return distance of two points
    return (std::sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z) ));
}
float ComputeSortiment::radiusDiff(pcl::ModelCoefficients::Ptr& coeff, pcl::ModelCoefficients::Ptr& coeff2)
{
    // retutn difference of two radius
    return std::abs(coeff->values.at(6) - coeff2->values.at(6));
}
float ComputeSortiment::curvatureDiff(pcl::ModelCoefficients::Ptr& line, pcl::ModelCoefficients::Ptr& point)
{
    // return distance of point from line
    float t=0;
    // std::cout << " linie: " << line->values.at(0) << " " <<line->values.at(1) << " " << line->values.at(2) << " " <<line->values.at(3) << " " <<line->values.at(4) << " " <<line->values.at(5) << "\n";
    
    // std::cout << " bod: " << point->values.at(0) << " " <<point->values.at(1) << " " << point->values.at(2) << " " <<point->values.at(3) << " " <<point->values.at(4) << " " <<point->values.at(5) << "\n";
    //x = line->values.at(0) + line->values.at(3)*t;
    //y = line->values.at(1) + line->values.at(4)*t;
    //z = line->values.at(2) + line->values.at(5)*t;
    
    //W=X-Q
    //Wx= x - point->values.at(0);
    //Wy= y - point->values.at(1);
    //Wz= z - point->values.at(2);
    
    //u*w = 0
    //0=line->values.at(0)* line->values.at(3) + line->values.at(3) * t * line->values.at(3) - point.x * line->values.at(3) +
    // line->values.at(1) * line->values.at(4) + line->values.at(4) * line->values.at(4) * t - point.y * line->values.at(4) +
    // line->values.at(2) * line->values.at(5) + line->values.at(5) * t * line->values.at(5) - point.z * line->values.at(5)
    float delenec = -1 *(line->values.at(0)* line->values.at(3) - point->values.at(0) * line->values.at(3) + line->values.at(1) * line->values.at(4) - point->values.at(1) * line->values.at(4) + line->values.at(2) * line->values.at(5) - point->values.at(2) * line->values.at(5));
    float delitel =line->values.at(3) * line->values.at(3) + line->values.at(4) * line->values.at(4) + line->values.at(5) * line->values.at(5);
    
    // std::cout<<"delenec: " << delenec <<  " delitel: "  << delitel<< "\n";
    if(delenec == 0 || delitel == 0)
        return 0;
    t =delenec / delitel;
    // std::cout<<"t: " << t << "\n";
    
    return  std::sqrt(
                      (line->values.at(0) - point->values.at(0) + line->values.at(3)*t) * (line->values.at(0) - point->values.at(0) + line->values.at(3)*t) +
                      (line->values.at(1) - point->values.at(1) + line->values.at(4)*t) * (line->values.at(1) - point->values.at(1) + line->values.at(4)*t) +
                      (line->values.at(2) - point->values.at(2) + line->values.at(5)*t) * (line->values.at(2) - point->values.at(2) + line->values.at(5)*t));
    // return distance;
}
pcl::ModelCoefficients::Ptr ComputeSortiment::lineCoefficient(pcl::ModelCoefficients::Ptr& coeff, pcl::ModelCoefficients::Ptr& coeff2)
{
    // create line from two points
    pcl::ModelCoefficients::Ptr coeffF(new pcl::ModelCoefficients);
    
    coeffF->values.push_back(coeff->values.at(0));
    coeffF->values.push_back(coeff->values.at(1));
    coeffF->values.push_back(coeff->values.at(2));
    coeffF->values.push_back(coeff2->values.at(0) - coeff->values.at(0));
    coeffF->values.push_back(coeff2->values.at(1) - coeff->values.at(1));
    coeffF->values.push_back(coeff2->values.at(2) - coeff->values.at(2));
    return coeffF;
}
std::vector<pcl::ModelCoefficients::Ptr> ComputeSortiment::getCylinders()
{
    return m_cylinders;
}
std::vector<int> ComputeSortiment::getSortiments()
{
    return m_sortiments;
}
void ComputeSortiment::setSortimentsSize()
{
    std::cout<<"cylinders size \n";
    int sizeC=0;
    for(int i =0; i < m_branches.size(); i++)
    {
        std::vector<pcl::ModelCoefficientsPtr> cylinders;
        cylinders = m_branches.at(i)->getCylinders();
        // zjistit celkovou delktu segmentu
        sizeC += cylinders.size();
    }
    m_sortiments.resize(sizeC);
}
float ComputeSortiment::cylinderDistance(pcl::ModelCoefficients::Ptr& cyl1, pcl::ModelCoefficients::Ptr& cyl2)
{
    
    return std::sqrt( (cyl1->values.at(0) - cyl2->values.at(0))*(cyl1->values.at(0) - cyl2->values.at(0)) + (cyl1->values.at(1) - cyl2->values.at(1))*(cyl1->values.at(1) - cyl2->values.at(1)) + (cyl1->values.at(2) - cyl2->values.at(2))*(cyl1->values.at(2) - cyl2->values.at(2))  );
}
int ComputeSortiment::continuosSortiment(std::vector<int> sortiments, int sortimentID, int sort, std::vector<bool>& usedSortiment)
{
    int number =0;
    int i = sortimentID;
    for(i; i< sortiments.size(); i++)
    {
        if(sortiments.at(i) == sort && usedSortiment.at(i) == false)
            number++;
        else
            break;
    }
    return number;
}
float ComputeSortiment::computeCylinderVolume(pcl::ModelCoefficients::Ptr& model)
{
    float v = 0;
    float volume = 0;
    v = std::sqrt(
                  (model->values.at(3) * model->values.at(3)) +
                  (model->values.at(4) * model->values.at(4)) +
                  (model->values.at(5) * model->values.at(5)) );
    
    volume =(M_PI * (model->values.at(6) * model->values.at(6)) * v);
    
    if(std::isnan(v))
    {
        v=0;
        volume = 0;
    }
    //  std::cout<<"valce hodnoty: x: "<<model->values.at(3) << " y: " << model->values.at(4)<< " z: " << model->values.at(5)<< "\n";
    //std::cout<< "prumer: " << model->values.at(6) << " vyska: "<< v <<" volume: "<<  volume<< " m_volume: "<< m_volume<< "\n";
    
    return volume;
}
float ComputeSortiment::getSortimentVolume(int sortimentID)
{
    switch (sortimentID) {
        case 1:
            return m_sortiment1_volume;
            break;
        case 2:
            return m_sortiment2_volume;
            break;
        case 3:
            return m_sortiment3_volume;
            break;
        case 4:
            return m_sortiment4_volume;
            break;
        case 5:
            return m_sortiment5_volume;
            break;
        case 6:
            return m_sortiment6_volume;
            break;
            
        default:
            return -1;
            break;
    }
}
float ComputeSortiment::getSortimentLenght(int sortimentID)
{
    switch (sortimentID) {
        case 1:
            return m_sortiment1_lenght;
            break;
        case 2:
            return m_sortiment2_lenght;
            break;
        case 3:
            return m_sortiment3_lenght;
            break;
        case 4:
            return m_sortiment4_lenght;
            break;
        case 5:
            return m_sortiment5_lenght;
            break;
        case 6:
            return m_sortiment6_lenght;
            break;
            
        default:
            return -1;
            break;
    }
}
void ComputeSortiment::sukyVetve(std::vector<pcl::ModelCoefficientsPtr>& cylinders, int branchID)
{
    std::vector<std::shared_ptr<Segment> > deti;
    m_branches.at(branchID)->getChildrens(deti);
    
    //pro kazdou vetev najit deti
    for(int q=0; q < deti.size();q++)
    {
        std::vector<pcl::ModelCoefficientsPtr> cyl;
        cyl = deti.at(q)->getCylinders();
        if(cyl.size() > 0)
            cylinders.push_back(cyl.at(0));
    }
    // pro kazde dite ulozit prvni cylinder - pozice stejna jako  u rodice, plus intensity - radius
}
bool ComputeSortiment::cylinderTouch(pcl::ModelCoefficients::Ptr& coeff, pcl::ModelCoefficients::Ptr& coeff2)
{
    // define for each cylinder points centre upper and lower
    pcl::PointXYZI upper1,lower1;
    upper1.x =  coeff->values.at(0);
    upper1.y =  coeff->values.at(1);
    upper1.z =  coeff->values.at(2);
    
    lower1.x = coeff->values.at(0) + coeff->values.at(3);
    lower1.y = coeff->values.at(1) + coeff->values.at(4);
    lower1.z = coeff->values.at(2) + coeff->values.at(5);
    
    pcl::PointXYZI upper2,lower2;
    upper2.x =  coeff2->values.at(0);
    upper2.y =  coeff2->values.at(1);
    upper2.z =  coeff2->values.at(2);
    
    lower2.x = coeff2->values.at(0) + coeff2->values.at(3);
    lower2.y = coeff2->values.at(1) + coeff2->values.at(4);
    lower2.z = coeff2->values.at(2) + coeff2->values.at(5);
    
    if(pointAreTheSame(upper1, upper2)== true || pointAreTheSame(upper1, lower2) == true || pointAreTheSame(lower1, upper2)== true || pointAreTheSame(lower1, lower2) == true )
        return true;
    
    return false;
}
bool ComputeSortiment::pointAreTheSame(pcl::PointXYZI a, pcl::PointXYZI b)
{
    if(a.x == b.x && a.y == b.y && a.z == b.z)
        return true;
    return false;
}

