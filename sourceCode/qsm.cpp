//
//  qsm.cpp
//  3D_Forest
//
//  Created by Jan Trochta on 16.05.18.
//

#include "qsm.h"
#include "HoughTransform.h"
#include "LeastSquareregression.h"
#include <pcl/common/pca.h>
#include <Eigen/QR>
#include <Eigen/Eigen>
#include <unsupported/Eigen/NonLinearOptimization>

struct LMFunctorStem
{
    // 'm' pairs of (x, f(x))
    Eigen::MatrixXf measuredValues;
    
    // Compute 'm' errors, one for each data point, for the given parameter values in 'x'
    int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
    {
        // 'x' has dimensions n x 1
        // It contains the current estimates for the parameters.
        
        // 'fvec' has dimensions m x 1
        // It will contain the error for each data point.
        
        float aParam = x(0);
        float bParam = x(1);
        float cParam = x(2);
        
        for (int i = 0; i < values(); i++) {
            float xValue = measuredValues(i, 0);
            float yValue = measuredValues(i, 1);
            
            // rovnice podle ktere se vypočítává  chyba - změnit
            fvec(i) = yValue - (aParam * std::pow(xValue,2) + bParam * xValue + cParam*std::pow(xValue,3)); //yValue - aParam + bParam * std::exp(cParam*xValue); yValue - bParam * std::pow(xValue, aParam);//
        }
        return 0;
    }
    
    // Compute the jacobian of the errors
    int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
    {
        // 'x' has dimensions n x 1
        // It contains the current estimates for the parameters.
        
        // 'fjac' has dimensions m x n
        // It will contain the jacobian of the errors, calculated numerically in this case.
        
        float epsilon;
        epsilon = 1e-5f;
        
        for (int i = 0; i < x.size(); i++) {
            Eigen::VectorXf xPlus(x);
            xPlus(i) += epsilon;
            Eigen::VectorXf xMinus(x);
            xMinus(i) -= epsilon;
            
            Eigen::VectorXf fvecPlus(values());
            operator()(xPlus, fvecPlus);
            
            Eigen::VectorXf fvecMinus(values());
            operator()(xMinus, fvecMinus);
            
            Eigen::VectorXf fvecDiff(values());
            fvecDiff = (fvecPlus - fvecMinus) / (2.0f * epsilon);
            
            fjac.block(0, i, values(), 1) = fvecDiff;
        }
        
        return 0;
    }
    
    // Number of data points, i.e. values.
    int m;
    
    // Returns 'm', the number of values.
    int values() const { return m; }
    
    // The number of parameters, i.e. inputs.
    int n;
    
    // Returns 'n', the number of inputs.
    int inputs() const { return n; }
    
};
struct LMFunctorBranches
{
    // 'm' pairs of (x, f(x))
    Eigen::MatrixXf measuredValues;
    
    // Compute 'm' errors, one for each data point, for the given parameter values in 'x'
    int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
    {
        // 'x' has dimensions n x 1
        // It contains the current estimates for the parameters.
        
        // 'fvec' has dimensions m x 1
        // It will contain the error for each data point.
        
        float aParam = x(0);
        float bParam = x(1);
        float cParam = x(2);
        
        for (int i = 0; i < values(); i++) {
            float xValue = measuredValues(i, 0);
            float yValue = measuredValues(i, 1);
            
            // rovnice podle ktere se vypočítává  chyba - změnit
            fvec(i) = yValue - (aParam * std::pow(xValue,2) + bParam * xValue  + cParam); //yValue - aParam + bParam * std::exp(cParam*xValue); yValue - bParam * std::pow(xValue, aParam);//
        }
        return 0;
    }
    
    // Compute the jacobian of the errors
    int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
    {
        // 'x' has dimensions n x 1
        // It contains the current estimates for the parameters.
        
        // 'fjac' has dimensions m x n
        // It will contain the jacobian of the errors, calculated numerically in this case.
        
        float epsilon;
        epsilon = 1e-5f;
        
        for (int i = 0; i < x.size(); i++) {
            Eigen::VectorXf xPlus(x);
            xPlus(i) += epsilon;
            Eigen::VectorXf xMinus(x);
            xMinus(i) -= epsilon;
            
            Eigen::VectorXf fvecPlus(values());
            operator()(xPlus, fvecPlus);
            
            Eigen::VectorXf fvecMinus(values());
            operator()(xMinus, fvecMinus);
            
            Eigen::VectorXf fvecDiff(values());
            fvecDiff = (fvecPlus - fvecMinus) / (2.0f * epsilon);
            
            fjac.block(0, i, values(), 1) = fvecDiff;
        }
        
        return 0;
    }
    
    // Number of data points, i.e. values.
    int m;
    
    // Returns 'm', the number of values.
    int values() const { return m; }
    
    // The number of parameters, i.e. inputs.
    int n;
    
    // Returns 'n', the number of inputs.
    int inputs() const { return n; }
    
};

CylinderModel::CylinderModel()
{
    
}
CylinderModel::CylinderModel(std::vector<std::shared_ptr<Segment>> branches, int iter, float cylinderSize)
{
    //setTree(tree);
    setBranches(branches);
    setIterations (iter);
    setCylinderSize(cylinderSize);
    //setLimit(cylinderSize);
   // std::cout<< "konstruktor\n";
}
CylinderModel::~CylinderModel()
{
}
std::vector<pcl::ModelCoefficients::Ptr> CylinderModel::getCylinders()
{
    return m_cylinders;
}
void CylinderModel::setCylinderSize(float cs)
{
    m_cylinderSize = cs;
}
void CylinderModel::setTreePosition(pcl::PointXYZI pose)
{
    m_position = pose;
}
//void CylinderModel::setTree(Tree* tree)
//{
    //m_tree = tree;
//}
void CylinderModel::setTreeHeight(float h)
{
    m_treeHeight = h;
}
void CylinderModel::setIterations(int iter)
{
    m_iterations = iter;
}
void CylinderModel::setLimit(float limit)
{
    m_limit = limit;
}
void CylinderModel::setTreecloud(pcl::PointCloud<pcl::PointXYZI>::Ptr treeCloud)
{
    m_treeCloud = treeCloud;
}
void CylinderModel::setBranches(std::vector<std::shared_ptr<Segment>> branches)
{
    m_branches = branches;
}
void CylinderModel::setBranchLength(float branchLength)
{
    m_branchLength = branchLength;
}
void CylinderModel::setOrder(int order)
{
    m_order = order;
}
std::vector<stred>& CylinderModel::getStemCurve()
{
    return m_stemCurve;
}
void CylinderModel::setStemCurve(std::vector<pcl::PointXYZI>& inVector)
{
    for(int q=0; q < inVector.size();q++)
    {
        stred s;
        s.a = inVector.at(q).x;
        s.b = inVector.at(q).y;
        s.z = inVector.at(q).z;
        s.r = inVector.at(q).intensity*100;
       // std::cout<< "s.x: " << s.a << " s.y: " << s.b << " s.z: " << s.z << " s.r: "<< s.r<<"\n";
        m_stemCurve.push_back(s);
    }
}

void CylinderModel::compute()
{
   // std::cout<< "compute()\n";
    // prepsat//
    // spocitat kmen
    computeStem();
    // spocitat jednotlivé vetve
    computeBranches();
    //coefficientData();
}

void CylinderModel::computeStem()
{
    std::cout << "computeStem()\n";
    
    std::vector<pcl::PointXYZI> points; // points representing cylinder
    
    // vzit mračnno segmentu
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);// make cloud from segment
    setSegmentCloud(m_branches.at(0), cloud);
    //PCA

    //std::cout<<"PCA\n";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_translated (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PCA<pcl::PointXYZI> pca;
    pca.setInputCloud(cloud);
    pca.project(*cloud, *cloud_translated);
    
    pcl::PointXYZI proj_min,proj_max, proj_ConnPoint, proj_minS,proj_maxS, proj_CPS;
    pcl::getMinMax3D (*cloud_translated, proj_min, proj_max);
    // PCA pozice
    pca.project(m_position, proj_ConnPoint);
   // std::cout<<"m_position.x "<< m_position.x << " m_position.y "<< m_position.y << " m_position.z "<< m_position.z << "\n";
    //std::cout<<"proj_ConnPoint.x "<< proj_ConnPoint.x << " proj_ConnPoint.y "<< proj_ConnPoint.y << " proj_ConnPoint.z "<< proj_ConnPoint.z << "\n";
    // swap axes
    float eX = std::abs(proj_max.x - proj_min.x);
    float eY = std::abs(proj_max.y - proj_min.y);
    float eZ = std::abs(proj_max.z - proj_min.z);
    
    int swap = 0; // Z  = longest
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_swap(new pcl::PointCloud<pcl::PointXYZI>);
    if(eX > eY && eX > eZ) // if eX is longest
    {
        swap = 1;
        swapXZ(cloud_translated, cloud_swap);
        swapXZ(proj_min,proj_minS);
        swapXZ(proj_max,proj_maxS);
        swapXZ(proj_ConnPoint,proj_CPS);

    }
    else if(eY> eX && eY > eZ)// if eY is longest
    {
        swap = 2;
        swapYZ(cloud_translated, cloud_swap);
        swapYZ(proj_min,proj_minS);
        swapYZ(proj_max,proj_maxS);
        swapYZ(proj_ConnPoint,proj_CPS);
    }
    else
    {
        swap = 0;
        *cloud_swap = *cloud_translated;
        proj_minS = proj_min;
        proj_maxS = proj_max;
        proj_CPS = proj_ConnPoint;
    }
   // std::cout << "swap hodnota: " << swap << "\n";
    //vzdalenost k proj_max
    float distMax = pointDistance (proj_maxS, proj_ConnPoint);
    
    //vzdalenost k proj_min
    float distMin = pointDistance (proj_minS, proj_ConnPoint);
  //  std::cout << "vzdalenosti distMAx: " << distMax << " distmin: " << distMin<< "\n";
    //otočeni
    float startHeight, step, stop;
    
   // std::cout<<"tree height: " << m_treeHeight << " procento: "  <<m_treeHeight/100.0 << "\n";
   // std::cout<<"proj_maxS.z: " << proj_maxS.z << "\n";
    //std::cout<<"proj_minS.z: " << proj_minS.z << "\n";
    if(distMax > distMin) //position close to distMin
    {
        startHeight = proj_CPS.z + m_treeHeight/100.0;
        step = m_cylinderSize;
        stop = proj_maxS.z;
    }
    else
    {
        startHeight = proj_CPS.z - m_treeHeight/100.0;
        step = -1 *m_cylinderSize;
        stop = proj_minS.z;
    }
    //std::cout << "podminky startHeight: " << startHeight << " step: " << step << " stop: " << stop << "\n";
    // cylindery pro cely kmen
    //std::cout<<" id vetve: "<< m_branches.at(0)->getCurrID()<< " fit: ";
    std::vector<float>fits;
    for(float w = startHeight; std::abs(w) < std::abs(stop); w += step )
    {
        //create cloud based on w and step
        //divide into two parts
        // for each compute circle
        pcl::PointXYZI u, u_swap, u_real;
        float fit;
        if(computeCylinder(cloud_swap, w, w+step, u_swap, fit, 0.02))
        {
            if(swap ==0) //Z
                {u = u_swap;}
            else if(swap ==1) // X
                {swapXZ(u_swap,u);}
            else //Y
                {swapYZ(u_swap,u);}
            u.intensity = u_swap.intensity;
            // reproject
            pca.reconstruct(u, u_real);
            u_real.intensity = u.intensity;
            // std::cout<< "u.intensity: " << u.intensity<< "\n";
            points.push_back(u_real);
            fits.push_back(fit);
            //std::cout<< fit<<" ";
        }
        //std::cout<<"parts y\n";
        // get points of cylinder
    }
   // std::cout<<"\n";
    std::cout<< "pocet points: " << points.size()<<"\n";
    std::vector<pcl::PointXYZI> pointsRegression; // points representing cylinder
    nonLinearRegressionStem(points); // ziskat coefficienty
    medianvalues();// spočitat mediany
   // m_median3 =0.01;
    regressionCylindersStem(points, pointsRegression);// vyhlazeni podle regrese
    
    
   // polynomialFit(points, pointsRegression);
   // smallerRadius(pointsRegression);
   // logFit(points, pointsRegression);
    // ulozit
    std::vector<pcl::ModelCoefficients::Ptr> cylinders;
    cylinders = getCylinders(pointsRegression,0);
    setStemCurve(pointsRegression);
    m_branches.at(0)->setCylinders(cylinders);
    m_branches.at(0)->setTotalVolume();
    m_branches.at(0)->setHroubiVolume();
    m_branches.at(0)->setHroubiLength();
    setSegmentParentCylinder(0);
    std::cout<< "x^2 * " <<m_median1 << " + X * " << m_median2 << " + " << m_median3 << " * X^3\n";
    std::cout<< "stem volume: " << m_branches.at(0)->getTotalVolume()<<"\n";
    clearCoeffVector();
}
void CylinderModel::computeBranches()
{
    std::cout << "computeBranches()\n";
    // pro kazdou vetev spocit valce
    // prokazdouvetev najit parametry regrese
    // urcit median value regresnich coef a ty pouzit na vetve

    for(int q=1;q < m_branches.size();q++)
    {
        //based on 1 order compute regression parameters.
        if(m_branches.at(q)->getLenght() < m_branchLength || m_branches.at(q)->getOrder() > m_order || m_branches.at(q)->getParentCP().intensity < m_limit)
            continue;
        
        std::vector<pcl::PointXYZI> points; // points representing cylinder
        computePoints(q,  points);
        
       // std::cout<<"\n";
       // std::cout<< "pocet points: " << points.size()<<"\n";
        std::vector<pcl::PointXYZI> pointsParent; // points representing cylinder
        std::vector<pcl::PointXYZI> pointsRegression; // points representing cylinder after filter
        // tady by to chtelo pridat spojeni bodu s parentem
        connectionWithParent(points, q, pointsParent);
        smallerThanParent(q, pointsParent);
        branchLinear(pointsParent);
        //smallerRadius(pointsParent);
        
        //regressionCylindersBranches(points, pointsRegression);// vyhlazeni podle regrese
        std::vector<pcl::ModelCoefficients::Ptr> cylinders;
        cylinders = getCylinders(pointsParent,q);
        m_branches.at(q)->setCylinders(cylinders);
        m_branches.at(q)->setTotalVolume();
        m_branches.at(q)->setHroubiVolume();
        m_branches.at(q)->setHroubiLength();
        setSegmentParentCylinder(q);
    }
}
void CylinderModel::computePoints(int branchID, std::vector<pcl::PointXYZI>& points)
{
    // vzit mračnno segmentu
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);// make cloud from segment
    setSegmentCloud(m_branches.at(branchID), cloud);
    
    //PCA
    
    //std::cout<<"PCA\n";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_translated (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PCA<pcl::PointXYZI> pca;
    pca.setInputCloud(cloud);
    pca.project(*cloud, *cloud_translated);
    
    pcl::PointXYZI proj_min,proj_max, proj_ConnPoint, proj_minS,proj_maxS, proj_CPS;
    pcl::getMinMax3D (*cloud_translated, proj_min, proj_max);
    // PCA pozice
    pca.project(m_branches.at(branchID)->getParentCP(), proj_ConnPoint);
    // std::cout<<"m_position.x "<< m_position.x << " m_position.y "<< m_position.y << " m_position.z "<< m_position.z << "\n";
    //std::cout<<"proj_ConnPoint.x "<< proj_ConnPoint.x << " proj_ConnPoint.y "<< proj_ConnPoint.y << " proj_ConnPoint.z "<< proj_ConnPoint.z << "\n";
    // swap axes
    float eX = std::abs(proj_max.x - proj_min.x);
    float eY = std::abs(proj_max.y - proj_min.y);
    float eZ = std::abs(proj_max.z - proj_min.z);
    
    int swap = 0; // Z  = longest
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_swap(new pcl::PointCloud<pcl::PointXYZI>);
    if(eX > eY && eX > eZ) // if eX is longest
    {
        swap = 1;
        swapXZ(cloud_translated, cloud_swap);
        swapXZ(proj_min,proj_minS);
        swapXZ(proj_max,proj_maxS);
        swapXZ(proj_ConnPoint,proj_CPS);
        
    }
    else if(eY > eX && eY > eZ)// if eY is longest
    {
        swap = 2;
        swapYZ(cloud_translated, cloud_swap);
        swapYZ(proj_min,proj_minS);
        swapYZ(proj_max,proj_maxS);
        swapYZ(proj_ConnPoint,proj_CPS);
    }
    else
    {
        swap = 0;
        *cloud_swap = *cloud_translated;
        proj_minS = proj_min;
        proj_maxS = proj_max;
        proj_CPS = proj_ConnPoint;
    }
    //std::cout << "\tswap hodnota: " << swap << "\n";
    //vzdalenost k proj_max
    //float distMax = pointDistance (proj_maxS, proj_ConnPoint);
    float distMax = std::abs(proj_maxS.z - proj_ConnPoint.z);
    
    //vzdalenost k proj_min
    //float distMin = pointDistance (proj_minS, proj_ConnPoint);
    float distMin = std::abs(proj_minS.z - proj_ConnPoint.z);
    //std::cout << "\tvzdalenosti distMAx: " << distMax << " distmin: " << distMin<< "\n";
    //otočeni
    float startHeight, step, stop;
    
    //std::cout<<"tree height: " << m_treeHeight << " procento: "  <<m_treeHeight/100.0 << "\n";
    //std::cout<<"proj_maxS.z: " << proj_maxS.z << "\n";
    //std::cout<<"proj_minS.z: " << proj_minS.z << "\n";
    if(distMax > distMin) //position close to distMin
    {
        startHeight = proj_minS.z;
        step = m_cylinderSize;
        stop = proj_maxS.z;
    }
    else
    {
        startHeight = proj_maxS.z;
        step = -1 *m_cylinderSize;
        stop = proj_minS.z;
    }
    std::vector<float>fits;
    
    
    
    if(step > 0) //
    {
        for(float w = startHeight; w < stop; w += step )
        {
            //create cloud based on w and step
            //divide into two parts
            // for each compute circle
            pcl::PointXYZI u, u_swap, u_real;
            float fit=0;
            
            if(computeCylinder(cloud_swap, w, w+step, u_swap, fit))
            {
                if(swap ==0) //Z
                {u = u_swap;}
                else if(swap ==1) // X
                {swapXZ(u_swap,u);}
                else //Y
                {swapYZ(u_swap,u);}
                
                u.intensity = u_swap.intensity;
                // reproject
                pca.reconstruct(u, u_real);
                u_real.intensity = u.intensity;
                //std::cout<< "u.intensity: " << u.intensity<< "\n";
                points.push_back(u_real);
                fits.push_back(fit);
                // std::cout<< fit<<" ";
            }
            //std::cout<<"parts y\n";
            // get points of cylinder
        }
    }
    else
    {
        for(float w = startHeight; w > stop; w += step )
        {
            //create cloud based on w and step
            //divide into two parts
            // for each compute circle
            pcl::PointXYZI u, u_swap, u_real;
            float fit=0;
            
            if(computeCylinder(cloud_swap, w, w+step, u_swap, fit))
            {
                if(swap ==0) //Z
                {u = u_swap;}
                else if(swap ==1) // X
                {swapXZ(u_swap,u);}
                else //Y
                {swapYZ(u_swap,u);}
                
                u.intensity = u_swap.intensity;
                // reproject
                pca.reconstruct(u, u_real);
                u_real.intensity = u.intensity;
                //std::cout<< "u.intensity: " << u.intensity<< "\n";
                points.push_back(u_real);
                fits.push_back(fit);
                // std::cout<< fit<<" ";
            }
            //std::cout<<"parts y\n";
            // get points of cylinder
        }
    }
}
void CylinderModel::branchLinear(std::vector<pcl::PointXYZI>& inVector)
{
    // for given branch get lenght and first diameter
    if(inVector.size()==0)
    {
        std::cout<< "invector emtpy!\n";
        return;
    }
    
    float diameter = inVector.at(0).intensity;
    float lenght = pointDistance(inVector.at(0), inVector.at(inVector.size()-1));
    for(int q=0;q < inVector.size();q++)
    {
        float len = pointDistance(inVector.at(0), inVector.at(q));
        float oldD = inVector.at(q).intensity;
        float c = -1*lenght*diameter;
        float newD = (diameter * len + c)/ (-1*lenght);
        if(newD < 0)
            newD = 0;
        inVector.at(q).intensity = newD;
       // std::cout<< "linear: d: " << diameter << " length: " << lenght << " len: " << len <<" oldD: "<< oldD << " newD: " << newD<< "\n";
    }
}
void CylinderModel::setSegmentCloud(std::shared_ptr<Segment> branch, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out )
{
    std::vector<int> indices = branch->getInliers();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    cloud_out->points.resize(indices.size());
    //
    //if(indices.size() < 4)
       // continue;
    
#pragma omp parallel for
    for(int q=0;q < indices.size();q++)
        cloud_out->points.at(q) = m_treeCloud->points.at(indices.at(q));
    
    //std::cout<< "velikost mracna: " << cloud_out->points.size() << "\n";
}
 void CylinderModel::connectionWithParent(std::vector<pcl::PointXYZI>& inVector, int branchId, std::vector<pcl::PointXYZI>& outVector)
{
    //vytvorit novy point s intesity nejbližšího cylinderu ze segmentu a odečíst vzdálenost cylinderu rodiče
    // pridat další body z invector
    //std::cout<<"connectionWithParent\n";
    // por segment zjisti rodice
    pcl::PointXYZI p = m_branches.at(branchId)->getConnPoint();
    // pro rodice ziskat cylindery
    std::vector<pcl::ModelCoefficients::Ptr> cyl = m_branches.at(branchId)->getParent()->getCylinders();
    if(cyl.size() == 0)
    {
        std::cout<<"parent has no cylinders!\n";
        return;
    }
    //closest parent cylinder
    // zjisti nejbližší cylinder rodice k connpoint segmentu
    float distance = 10000000000000;
    int cylinderID;
    
    for(int w=0; w < cyl.size(); w++)
    {
        float d = cylinderPointDistance(p, cyl.at(w));
       // std::cout<<"vzdalenosti: " << d << "\n";
        if(d < distance)
        {
            distance = d;
            cylinderID = w;
        }
    }
    pcl::PointXYZI parentCylinder;
    parentCylinder.x = cyl.at(cylinderID)->values.at(0);
    parentCylinder.y = cyl.at(cylinderID)->values.at(1);
    parentCylinder.z = cyl.at(cylinderID)->values.at(2);
    parentCylinder.intensity = cyl.at(cylinderID)->values.at(6);
    m_branches.at(branchId)->setParentCylinderPoint(parentCylinder);
    // zjisti nejbližší cylinder segmentu ke connpoint
    
    if(inVector.size()==0)
        return;
   // pcl::PointXYZI p =m_branches.at(branchId)->getConnPoint();
    float prvni = pointDistance(inVector.at(0), p);
    float posledni = pointDistance(inVector.at(inVector.size()-1), p);
    
    if(prvni < posledni) //prvni is close
    {
        for(int r=0; r < inVector.size();r++)
            outVector.push_back(inVector.at(r));
    }
    else
    {
        for(int r=inVector.size()-1; r >=0;r--)
            outVector.push_back(inVector.at(r));
    }
}
void CylinderModel::translateCloud()
{

}
void CylinderModel::setSegmentParentCylinder(int i)
{
    // pro segment  zjistit deti
    // pro kazde dite urcit z connpoint  nejbližší válec a uložit jej.
    
    // pro dany segment najdi vsechny deti, a pro kazde dite pridej connpoint
    std::vector<std::shared_ptr<Segment> > childs;
    m_branches.at(i)->getChildrens(childs);
   // std::cout<<"pocet deti: " << childs.size()<< "\n";
    
    std::vector<pcl::ModelCoefficients::Ptr> cyl = m_branches.at(i)->getCylinders();
    //std::cout<<"pocet cylindru - rodic: "<< m_branches.at(i)->getCurrID()<<"  pocet: " << cyl.size()<<"\n";
    
    for(int q=0; q < childs.size();q++)
    {
       // std::cout<< "dite ID "<< childs.at(q)->getCurrID()<<"\n";
        // get parent cylinders
        
        pcl::PointXYZI cp = childs.at(q)->getConnPoint();
        if(cyl.size() == 0)
            continue;

        float distance = 10000000000000;
        int cylinderID;
        for(int w=0; w < cyl.size(); w++)
        {
            float d = cylinderPointDistance(cp, cyl.at(w));
           // std::cout<<"vzdalenosti: " << d << "\n";
            if(d < distance)
            {
                distance = d;
                cylinderID = w;
            }
        }
       // std::cout<<"vybrana vzdalenost: " << distance << "\n";
        pcl::PointXYZI p;
        p.x = cyl.at(cylinderID)->values.at(0);
        p.y = cyl.at(cylinderID)->values.at(1);
        p.z = cyl.at(cylinderID)->values.at(2);
        p.intensity = cyl.at(cylinderID)->values.at(6);
      //   std::cout<< "pro vetev "<< childs.at(q)->getCurrID() << " maximalni radius: " << p.intensity << "\n";
        childs.at(q)->setParentCylinderPoint(p);
    }
}
void CylinderModel::smallerThanParent(int q, std::vector<pcl::PointXYZI>& points)
{
  //  std::cout<<"smallerthanParent2 " << q<< " rad vetve: "<< m_branches.at(q)->getOrder()<<  "\n";
    
    // pro danou vetev udelat regresi, pokud jsou v regresi hodnoty vetsi nez parent
    
    
    
    // poro danou vetev
    if(q < 0 || q > m_branches.size())
    {
        std::cout<< " no valid ID\n";
        return;
    }
    std::vector<pcl::ModelCoefficients::Ptr>  parentCyl = m_branches.at(q)->getParent()->getCylinders();
    if(parentCyl.size() ==0)
    {
        std::cout<< " neni rodic s cylindery\n";
        return;
    }
   // std::cout<<" velikost parentCyl: " << parentCyl.size()<<"\n";
    
    pcl::PointXYZI limit =  m_branches.at(q)->getParentCP();
    float prumer = points.at(0).intensity;
    if(points.size()>2)
        prumer = (points.at(0).intensity + points.at(1).intensity + points.at(2).intensity)/3;
    //std::cout<< "prumerna hodnota r: " << prumer << " limit: " << limit.intensity << " prvni hodnota: " << points.at(0).intensity<< "\n";
    points.at(0).intensity = prumer;
    for(int e=0; e < points.size(); e++)
    {
        if(points.at(e).intensity > limit.intensity*0.95)
            points.at(e).intensity = limit.intensity*0.95;
    }
    return;

}
void CylinderModel::smallerThanParent()
{

    
    
   // std::cout<< "smallerThanParent()\n ";
    for(int q=0; q < m_branches.size(); q++)
    {
        if(m_branches.at(q) == m_branches.at(q)->getParent())
            continue;
        // get parent cylinder size
        // for each cylinder
        // if radius is > parent size
        // parent size
        pcl::PointXYZI limit =  m_branches.at(q)->getParentCP();
        std::vector<pcl::ModelCoefficients::Ptr> cyl = m_branches.at(q)->getCylinders();
        for(int w=0; w < cyl.size(); w++)
        {
            if(cyl.at(w)->values.at(6) > limit.intensity)
                cyl.at(w)->values.at(6) = limit.intensity;
        }
        m_branches.at(q)->setCylinders(cyl);
    }
}
float CylinderModel::cylinderPointDistance( pcl::PointXYZI p, pcl::ModelCoefficients::Ptr cylinder)
{
    float dist = std::sqrt((p.x - cylinder->values.at (0))*(p.x - cylinder->values.at (0)) + (p.y - cylinder->values.at (1))*(p.y - cylinder->values.at (1)) + (p.z - cylinder->values.at (2))*(p.z - cylinder->values.at (2))  );
    return dist;
}
bool CylinderModel::segmentPartition(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI& u,  float height, bool reverseDirection)
{
   // std::cout<<"segmentPartition\n";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_low (new pcl::PointCloud<pcl::PointXYZI>);

    for(int i=0; i<cloud->points.size(); i++)
    {
        if(cloud->points.at(i).z > height && cloud->points.at(i).z <= height + m_cylinderSize)
            cloud_low->points.push_back(cloud->points.at(i));
    }
    // pro kazdy sub cloud spocitat kruh RHT a ziskat stred a prumer na zacatku a na konci
    if(cloud_low->points.size() < 4)
        return false;
    
    u = setCircle(cloud_low);
    return true;
}
bool CylinderModel::computeCylinder(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float startHeight, float stopHeight, pcl::PointXYZI& cylinder, float& fit, float tolerance)
{
   // std::cout<<"\t\tpcelkem bodu v cloudu: "<< cloud->points.size() << "\n";
    float startH, stopH;
    if(startHeight > stopHeight)
    {
        startH = stopHeight;
        stopH = startHeight;
    }
    else
    {
        startH = startHeight;
        stopH = stopHeight;
    }
   // std::cout<< "startH: " << startH << " stopH: " << stopH <<"\n";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ (new pcl::PointCloud<pcl::PointXYZI>);
    
    for(int i=0; i < cloud->points.size(); i++)
    {//create cloud based on startheight and stopHeight
        if(cloud->points.at(i).z  >= startH && cloud->points.at(i).z  < stopH)
            cloud_->points.push_back(cloud->points.at(i));
    }
   // std::cout<<"\t\tpocet bodů v delenem cloudu: "<< cloud_->points.size() << "\n";
    if(cloud_->points.size() < 4)
        return false;
    
    cylinder = setCircle(cloud_);
    fit = cylinderFit(cloud_, cylinder,tolerance);
    
    return true;
    
    // for each compute circle
}

pcl::PointXYZI CylinderModel::setCircle(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    //std::cout<< "setCircle(()\n";
    //std::cout<<"setCircle\n";
    stred res;
    //HOugh transform
    HoughTransform *ht = new HoughTransform();
    ht->set_Cloud(cloud);
    ht->set_iterations(m_iterations);
    ht->compute();
    res= ht->get_circle();
    
    // LSR
//    LeastSquaredRegression *lsr = new LeastSquaredRegression();
//    lsr->setCloud(cloud);
//    lsr->compute();
//    res = lsr->getCircle();
    
    pcl::PointXYZI c;
    c.x = res.a;
    c.y = res.b;
    c.z= res.z-m_cylinderSize/2;
    c.intensity = res.r/100;
    return c;
}
bool CylinderModel::logFit(std::vector<pcl::PointXYZI>& inVector, std::vector<pcl::PointXYZI>& outVector)
{
    std::vector<float> xv; // distance from bottom
    std::vector<float> yv;
    std::vector<float> coeff;
    std::vector<float> residuals;
    float distance =inVector.at(inVector.size()-1).z + m_cylinderSize ;
   // distance += inVector.at(inVector.size()-1).z;
    std::cout<<"distance "<< distance << "\n";
    for(int q=0; q < inVector.size(); q++)
    {
        std::cout<<"distance "<< distance << " inVector.at(q).z "  << inVector.at(q).z << "\n";
        xv.push_back(std::log(distance - inVector.at(q).z) );
        yv.push_back(inVector.at(q).intensity);
        std::cout<< "distance: " << xv.at(xv.size()-1) << " radius: " << yv.at(yv.size()-1) << "\n";
    }
    //polyfit(xv, yv, coeff, 1);
    std::cout<< "koeficienty: " << coeff.at(1) << "*ln(X) + " << coeff.at(0) <<"\n";
    
    outVector.resize(inVector.size());
    for(int q=0; q < inVector.size();q++)
    {
        // pro kazdy bod zjistit hodnotu regrese
        float model =std::exp(distance - inVector.at(q).z)  * coeff.at(1) + coeff.at(0);
        float diff = model - inVector.at(q).intensity;
        residuals.push_back(diff);
        
        pcl::PointXYZI bod;
        bod.x = inVector.at(q).x;
        bod.y = inVector.at(q).y;
        bod.z = inVector.at(q).z;
        bod.intensity = model;
        outVector.at(q) = bod;
        std::cout<< "modelova hodnota: "<< model<< " skutecna: " <<inVector.at(q).intensity <<  " odchylka: " << diff << "\n";
    }
    // zjistit prumernou odchylku
    float r=0,rAbs=0;
    for(int w=0;w <residuals.size(); w++)
    {
        r+=residuals.at(w);
        rAbs+= std::abs(residuals.at(w));
    }
    
    r/=residuals.size();
    rAbs/=residuals.size();
    std::cout<< "prumerna odchylka od modelu: " << r <<  " absolutni odchylka:  " << rAbs << "\n";
    //
    
    // jisti odchylu od modelu a skutecnych hodnot
    // take radius of each point
    // create polyfit and check data for ouliers
    
    return true;
}


bool CylinderModel::polynomialFit(std::vector<pcl::PointXYZI>& inVector, std::vector<pcl::PointXYZI>& outVector)
{
    // take distance from the first point
    std::vector<float> xv; // distance from bottom
    std::vector<float> yv;
    std::vector<float> weightsv;
    std::vector<float> coeff;
    std::vector<float> residuals;
    // pro kazdy bod udelej
    for(int i=0; i < inVector.size(); i++)
    {
        xv.push_back(inVector.at(i).z);
        yv.push_back(inVector.at(i).intensity);
    }
    polyfit(xv, yv, weightsv, coeff, 2);
    //std::cout<< "koeficienty: " << coeff.at(1) << "*X + " << coeff.at(0) <<"\n";// <<coeff.at(3) << "*X^3 + "<<coeff.at(2) << "*X^2 + "
    // spocitat confidence interval
    outVector.resize(inVector.size());
    for(int q=0; q < inVector.size();q++)
    {
        // pro kazdy bod zjistit hodnotu regrese
        float model =inVector.at(q).z * inVector.at(q).z * coeff.at(0) + inVector.at(q).z * coeff.at(1) + coeff.at(2);// inVector.at(q).z * inVector.at(q).z *inVector.at(q).z * coeff.at(3) + inVector.at(q).z * inVector.at(q).z * coeff.at(2) +
        float diff = model - inVector.at(q).intensity;
        residuals.push_back(diff);
        
        pcl::PointXYZI bod;
        bod.x = inVector.at(q).x;
        bod.y = inVector.at(q).y;
        bod.z = inVector.at(q).z;
        bod.intensity = model;
        outVector.at(q) = bod;
        float pokles = 0;
        if(q>0)
            pokles = outVector.at(q-1).intensity - model;
        //std::cout<< "modelova hodnota: "<< model<< " skutecna: " <<inVector.at(q).intensity <<  " odchylka: " << diff <<" pokles: "<< pokles <<"\n";
    }
    // zjistit prumernou odchylku
    float r=0,rAbs=0;
    for(int w=0;w <residuals.size(); w++)
    {
        r+=residuals.at(w);
        rAbs+= std::abs(residuals.at(w));
    }

    r/=residuals.size();
    rAbs/=residuals.size();
    //std::cout<< "prumerna odchylka od modelu: " << r <<  " absolutni odchylka:  " << rAbs << "\n";
    //
    
    // jisti odchylu od modelu a skutecnych hodnot
    // take radius of each point
    // create polyfit and check data for ouliers
    
    return true;
}
void CylinderModel::polyfit(const std::vector<float> &xv, const std::vector<float> &yv, const std::vector<float> &wv, std::vector<float> &coeff, int order)
{
    Eigen::MatrixXf A(xv.size(), order+1);
    Eigen::VectorXf yv_mapped = Eigen::VectorXf::Map(&yv.front(), yv.size());
    Eigen::VectorXf wv_mapped = Eigen::VectorXf::Map(&wv.front(), wv.size());
    Eigen::VectorXf result;
    
    //assert(xv.size() == yv.size());
   // assert(xv.size() >= order+1);
    
    // create matrix
    for (size_t i = 0; i < xv.size(); i++)
        for (size_t j = 0; j < order+1; j++)
            A(i, j) = pow(xv.at(i), j);
    
    // solve for linear least squares fit
    result = A.householderQr().solve(yv_mapped);
    
    coeff.resize(order+1);
    for (size_t i = 0; i < order+1; i++)
        coeff[i] = result[i];
}
bool CylinderModel::nonLinearRegressionStem(std::vector<pcl::PointXYZI>& inVector)
{
    std::vector<float> xv; // distance from bottom
    std::vector<float> yv;
    //std::vector<float> coeff;
    //std::vector<float> residuals;
    // pro kazdy bod udelej
    for(int i=0; i < inVector.size(); i++)
    {
        float dataX =std::abs(pointDistance(inVector.at(i), inVector.at(inVector.size()-1)));
        xv.push_back(dataX);
        yv.push_back(inVector.at(i).intensity);
       // std::cout<<"vstupni data: x " << xv.at(xv.size()-1) << " \ty "<< yv.at(yv.size()-1)<<"\n";
    }
    
    int m = xv.size();
    
    // Move the data into an Eigen Matrix.
    // The first column has the input values, x. The second column is the f(x) values.
    Eigen::MatrixXf measuredValues(m, 2);
    for (int i = 0; i < m; i++) {
        measuredValues(i, 0) = xv[i];
        measuredValues(i, 1) = yv[i];
    }
    
    // 'n' is the number of parameters in the function.
    // f(x) = a(x^2) + b(x) + c has 3 parameters: a, b, c
    int n = 3;
    
    // 'x' is vector of length 'n' containing the initial values for the parameters.
    // The parameters 'x' are also referred to as the 'inputs' in the context of LM optimization.
    // The LM optimization inputs should not be confused with the x input values.
    Eigen::VectorXf coeff(n);
    coeff(0) = 1.0;             // initial value for 'a'
    coeff(1) = 1.0;             // initial value for 'b'
    coeff(2) = 1.0;             // initial value for 'c'
    
    //
    // Run the LM optimization
    // Create a LevenbergMarquardt object and pass it the functor.
    //
        LMFunctorStem functor;

    
    functor.measuredValues = measuredValues;
    functor.m = m;
    functor.n = n;
    
    Eigen::LevenbergMarquardt<LMFunctorStem, float> lm(functor);
    int status = lm.minimize(coeff);
   // std::cout << "LM optimization status: " << status << std::endl;
    
    regressionCoeff(coeff);

    return true;
}
bool CylinderModel::nonLinearRegressionBranches(std::vector<pcl::PointXYZI>& inVector)
{
    std::vector<float> xv; // distance from bottom
    std::vector<float> yv;
    //std::vector<float> coeff;
    //std::vector<float> residuals;
    // pro kazdy bod udelej
    //std::cout<<"nonlinear vstup: " << inVector.size()<<"\n";
    for(int i=0; i < inVector.size(); i++)
    {
        float dataX =std::abs(pointDistance(inVector.at(i), inVector.at(inVector.size()-1)));
        xv.push_back(dataX);
        yv.push_back(inVector.at(i).intensity);
        // std::cout<<"vstupni data: x " << xv.at(xv.size()-1) << " \ty "<< yv.at(yv.size()-1)<<"\n";
    }
    
    int m = xv.size();
    
    // Move the data into an Eigen Matrix.
    // The first column has the input values, x. The second column is the f(x) values.
    Eigen::MatrixXf measuredValues(m, 2);
    for (int i = 0; i < m; i++) {
        measuredValues(i, 0) = xv[i];
        measuredValues(i, 1) = yv[i];
    }
    
    // 'n' is the number of parameters in the function.
    // f(x) = a(x^2) + b(x) + c has 3 parameters: a, b, c
    int n = 3;
    
    // 'x' is vector of length 'n' containing the initial values for the parameters.
    // The parameters 'x' are also referred to as the 'inputs' in the context of LM optimization.
    // The LM optimization inputs should not be confused with the x input values.
    Eigen::VectorXf coeff(n);
    coeff(0) = 0.002;             // initial value for 'a'
    coeff(1) = 0.002;             // initial value for 'b'
    coeff(2) = 0.0;             // initial value for 'c'
    
    //
    // Run the LM optimization
    // Create a LevenbergMarquardt object and pass it the functor.
    //

    LMFunctorBranches functor;
    
    functor.measuredValues = measuredValues;
    functor.m = m;
    functor.n = n;
    
    Eigen::LevenbergMarquardt<LMFunctorBranches, float> lm(functor);
    int status = lm.minimize(coeff);
    // std::cout << "LM optimization status: " << status << std::endl;
    
    regressionCoeff(coeff);
    
    return true;
}
void CylinderModel::regressionCylindersStem(std::vector<pcl::PointXYZI>& inVector, std::vector<pcl::PointXYZI>& outVector)
{
    std::vector<float> ser; // standart error of regression
    for(int q=0;q < inVector.size();q++)
    {
        pcl::PointXYZI p=inVector.at(q);
        float old = p.intensity;
        float xVal = std::abs(pointDistance(inVector.at(q), inVector.at(inVector.size()-1)));
        p.intensity = (m_median1* xVal*xVal + m_median2* xVal + m_median3*xVal*xVal*xVal);
        //p.intensity = 0.002* xVal*xVal - 0.002* xVal + 0.005;
        // p.intensity = (coeff(0) - coeff(1)*std::exp(coeff(2)*xVal));
        //p.intensity = coeff(1) * std::pow(xVal, coeff(0));
        ser.push_back(std::abs(p.intensity -old));
        outVector.push_back(p);
    }
    float s=0;
    for(int w=0;w < ser.size(); w++)
    {
        s+=ser.at(w);
    }
    s/=ser.size();
}
void CylinderModel::regressionCylindersBranches(std::vector<pcl::PointXYZI>& inVector, std::vector<pcl::PointXYZI>& outVector)
{
    std::vector<float> ser; // standart error of regression
    for(int q=0;q < inVector.size();q++)
    {
        pcl::PointXYZI p=inVector.at(q);
        float old = p.intensity;
        float xVal = std::abs(pointDistance(inVector.at(q), inVector.at(inVector.size()-1)));
        p.intensity = (m_median1* xVal*xVal + m_median2* xVal + m_median3);
       // p.intensity = 0.002* xVal*xVal - 0.002* xVal + 0.005;
        // p.intensity = (coeff(0) - coeff(1)*std::exp(coeff(2)*xVal));
        //p.intensity = coeff(1) * std::pow(xVal, coeff(0));
        ser.push_back(std::abs(p.intensity -old));
        outVector.push_back(p);
    }
    float s=0;
    for(int w=0;w < ser.size(); w++)
    {
        s+=ser.at(w);
    }
    s/=ser.size();
}
bool CylinderModel::regressionCoeff(Eigen::VectorXf coeff)
{
    // save regression coefficients into vectors
    m_coef1.push_back(coeff(0));
    m_coef2.push_back(coeff(1));
    m_coef3.push_back(coeff(2)); // initial value for 'a'
    return true;
}
void CylinderModel::medianvalues()
{
    int m;
    if (m_coef1.size() % 2 == 0)
    {
        m=(m_coef1.size()/2)-1;
    }
    else
    {
        m=(m_coef1.size()/2);
    }
    std::sort(m_coef1.begin(), m_coef1.end());
    std::sort(m_coef2.begin(), m_coef2.end());
    if(m_coef3.size() != 0)
    {
        std::sort(m_coef3.begin(), m_coef3.end());
        m_median3 = m_coef3[m];
    }
    else
        m_median3=0;
    
    m_median1 = m_coef1[m];
    m_median2 = m_coef2[m];
    
}
void CylinderModel::clearCoeffVector()
{
    m_coef1.clear();
    m_coef2.clear();
    m_coef3.clear();
}
bool CylinderModel::petras(std::vector<pcl::PointXYZI>& inVector, std::vector<pcl::PointXYZI>& outVector)
{
    // zjistit delku - m_treeheight
    //zjisti vysku valce
    float h=0;
    
    //zjistit dbh
    float dbh = getDBH(inVector);
    //vypocitat  koefs
    float p1 = 3;
    float p2 = 1/4;
    float p3 = 4/3;
    float k3 = -1.43 + 0.077*m_treeHeight - 0.085*dbh ;// tohot ještě určit
    float k2 = (std::pow(-dbh, 1/p3) - k3 * (std::pow(m_treeHeight,p2) - std::pow(1.3,p2))) / (std::pow(m_treeHeight,p1) - std::pow(1.3,p1)) ;
    float k1 = std::pow(dbh,1/p3) - k3 * std::pow(1.3,p2) - k2 * std::pow(1.3,p1);
    // dosadit do rovnice
    //float d = std::pow(k1 + k2 * std::pow(h,p1) + k3 * std::pow(h,p2),p3);
    
    std::vector<float> ser; // standart error of regression
    for(int i=0; i < inVector.size(); i++)
    {
        // zjistit vyšku od začatku
        h = std::abs(pointDistance(inVector.at(i), inVector.at(0)));
        // dosadit
        pcl::PointXYZI p=inVector.at(i);
        float old = p.intensity;
        p.intensity = std::pow(k1 + k2 * std::pow(h,p1) + k3 * std::pow(h,p2),p3);
        ser.push_back(std::abs(p.intensity - old));
        outVector.push_back(p);
        
    }
    float s=0;
    for(int w=0;w < ser.size(); w++)
    {
        s+=ser.at(w);
    }
    s/=ser.size();
    std::cout<<"prumrna odchylka za cely kmen: "<< s <<" m\n";
    return true;
    
}

std::vector<pcl::ModelCoefficients::Ptr> CylinderModel::getCylinders(std::vector<pcl::PointXYZI> points, int branchID)
{
    // pokud je aspon jeden bod
    // spojit s connpoint branches,
    //vytvorit cylinder
    //pro kazdy bod  spocitat nasledujici cylinder
    // pokud moc vybočuje - viz delka  tak tento bod ignorovat, a spojit dalsim.
    
    
    std::vector<pcl::ModelCoefficients::Ptr> cylinders;
    if(points.size() < 1)
        return cylinders;
    
    for(int q=0; q < points.size(); q++)
    {
        if(branchID == 0 && q==0)
            q++;
        
        pcl::PointXYZI p1,p2;
        p1 = points.at(q);
        
        if(q==0)
            {p2 = m_branches.at(branchID)->getParentCP(); p2.intensity = p1.intensity;}
        else
            {p2 = points.at(q-1);}
    // podminky
        float radius =(p1.intensity + p2.intensity) /2;
        // height of cylinder
        float v = std::sqrt(((p1.x - p2.x)*(p1.x - p2.x)) +((p1.y - p2.y)*(p1.y - p2.y)) + ((p1.z - p2.z)*(p1.z - p2.z)) );
        
        if (radius*2 < m_limit || v > m_cylinderSize*8)
            continue;
        
        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
        coeff = contructCylinder(p1, p2);
        
       // std::cout<<"cylinder radius: " << coeff->values.at(6) <<"\n";
        
        cylinders.push_back(coeff);
        m_cylinders.push_back(coeff);
    }
    return cylinders;
    
    
    
    
    
//   // std::cout<< "getCylinders(std::vector<pcl::PointXYZI> points)\n";
//    //std::vector<pcl::ModelCoefficients::Ptr> cylinders;
//    if(points.size() < 1)
//        return cylinders;
//
//    for(int i=0; i < points.size()-1; i++)
//    {
//        // radius
//        int a=i+1;
//        float radius =(points.at(i).intensity + points.at(a).intensity) /2;
//        // height of cylinder
//        float v = pointDistance(points.at(i), points.at(a));
//
//        if (radius < m_limit || v > m_cylinderSize*2)
//        {
//            a++;
//            if(a >= points.size())
//                continue;
//
//        }
//
//        // std::cout<< " cylinder radius: " << radius<< "\n";
//        //if(radius <= 0)
//        //return;
//        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
//        coeff->values.push_back(points.at(a).x);
//        coeff->values.push_back(points.at(a).y);
//        coeff->values.push_back(points.at(a).z);
//        coeff->values.push_back(points.at(i).x - points.at(a).x);
//        coeff->values.push_back(points.at(i).y - points.at(a).y);
//        coeff->values.push_back(points.at(i).z - points.at(a).z);
//        coeff->values.push_back(radius);
//       // std::cout<<"cylinder radius: " << radius <<"\n";
//        i=a;
//        i--;
//
//        cylinders.push_back(coeff);
//        m_cylinders.push_back(coeff);
//    }
//    return cylinders;
}
void CylinderModel::setCylinder(std::vector<pcl::PointXYZI> points)
{
    std::cout<< "setCylinder()\n";
    if(points.size() < 2)
        return;
    for(int i=1; i < points.size(); i++)
    {
        // radius
        float radius =(points.at(i-1).intensity + points.at(i).intensity) /2;
        // height of cylinder
        float v = std::sqrt(((points.at(i).x - points.at(i-1).x)*(points.at(i).x - points.at(i-1).x)) +((points.at(i).y - points.at(i-1).y)*(points.at(i).y - points.at(i-1).y)) + ((points.at(i).z - points.at(i-1).z)*(points.at(i).z - points.at(i-1).z)) );
        std::cout<<"m_limit: " << m_limit<< " radius*2: "<< radius*2<<"\n";
        if (radius*2 < m_limit || v > m_cylinderSize*2)
            continue;
       // std::cout<< " cylinder radius: " << radius<< "\n";
        //if(radius <= 0)
            //return;
        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
        coeff->values.push_back(points.at(i-1).x);
        coeff->values.push_back(points.at(i-1).y);
        coeff->values.push_back(points.at(i-1).z);
        coeff->values.push_back(points.at(i).x - points.at(i-1).x);
        coeff->values.push_back(points.at(i).y - points.at(i-1).y);
        coeff->values.push_back(points.at(i).z - points.at(i-1).z);
        coeff->values.push_back(radius);
        //std::cout<<"cylinder radius: " << radius <<"\n";
        m_cylinders.push_back(coeff);
        //m_tree->setCylinder(coeff);
    }
}

pcl::ModelCoefficients::Ptr CylinderModel::contructCylinder(pcl::PointXYZI p1, pcl::PointXYZI p2)
{
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    coeff->values.push_back(p1.x);
    coeff->values.push_back(p1.y);
    coeff->values.push_back(p1.z);
    coeff->values.push_back(p2.x - p1.x);
    coeff->values.push_back(p2.y - p1.y);
    coeff->values.push_back(p2.z - p1.z);
    coeff->values.push_back((p1.intensity + p2.intensity) /2);
    return coeff;
}
void CylinderModel::swapXZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out)
{
    cloud_out->points.resize(cloud_in->points.size());
    #pragma omp parallel for
    for(int i=0; i < cloud_in->points.size();i++)
    {
        pcl::PointXYZI swap;
        swap.x = cloud_in->points.at(i).z;
        swap.y = cloud_in->points.at(i).y;
        swap.z = cloud_in->points.at(i).x;
        swap.intensity = cloud_in->points.at(i).intensity;
        cloud_out->points.at(i) = swap;
    }
}
void CylinderModel::swapYZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out)
{
    cloud_out->points.resize(cloud_in->points.size());
    #pragma omp parallel for
    for(int i=0; i < cloud_in->points.size();i++)
    {
        pcl::PointXYZI swap;
        swap.x = cloud_in->points.at(i).x;
        swap.y = cloud_in->points.at(i).z;
        swap.z = cloud_in->points.at(i).y;
        swap.intensity = cloud_in->points.at(i).intensity;
        cloud_out->points.at(i) = swap;
    }
}
void CylinderModel::swapXZ(pcl::PointXYZI& input, pcl::PointXYZI& output)
{
    output.x = input.z;
    output.y = input.y;
    output.z = input.x;
    output.intensity = input.intensity;
}
void CylinderModel::swapYZ(pcl::PointXYZI& input, pcl::PointXYZI& output)
{
    output.x = input.x;
    output.y = input.z;
    output.z = input.y;
    output.intensity = input.intensity;
}
void CylinderModel::smallerRadius(std::vector<pcl::PointXYZI>& inVector)
{
   // std::cout<< "smallerRadius(()\n";
    if(inVector.size() < 2)
        return;
    for(int q=0; q < inVector.size()-1; q++)
    {
        if(inVector.at(q).intensity < inVector.at(q+1).intensity)
        {
            inVector.at(q+1).intensity =inVector.at(q).intensity;
        }
    }
}
float CylinderModel::cylinderVolume(pcl::ModelCoefficients::Ptr& coeff)
{
    float h = std::sqrt((coeff->values.at(3) * coeff->values.at(3)) +
                        (coeff->values.at(4) * coeff->values.at(4)) +
                        (coeff->values.at(5) * coeff->values.at(5)) );
    
    float volume =(M_PI * coeff->values.at(6) * coeff->values.at(6) * h);
   // std::cout<< "volume valce: "<< M_PI << " * " << h << " * " << coeff->values.at(6) << "^2 = " << volume<<"\n";
    return volume;
}
float CylinderModel::pointDistance (pcl::PointXYZI& u, pcl::PointXYZI& v)
{
    if(u.x == v.x && u.y == v.y && u.z == v.z)
        return 0;
    return std::sqrt( (u.x - v.x)*(u.x - v.x) + (u.y - v.y)*(u.y - v.y) + (u.z - v.z)*(u.z - v.z) );
}
void CylinderModel::branchParentConnection()
{
    std::cout<<"branchParentConnection()\n";
    // por kazdou vetev
    // spojit prvni cylinder s parent connpoint
    for(int q=1; q < m_branches.size(); q++)
    {
        // get parent conpoint
        pcl::PointXYZI c = m_branches.at(q)->getParentCP();
    //std:cout<< "spojeni valcem: " << m_branches.at(q)->getParentm()->getCurrID() << " dite: " << m_branches.at(q)->getCurrID()<<"\n";
        // get first clinder
        std::vector<pcl::ModelCoefficients::Ptr> cylinders = m_branches.at(q)->getCylinders();
       // std::cout<<"pocet cylinderu: " << cylinders.size()<< "\n";
        if(cylinders.size() == 0)
            continue;
        
        pcl::ModelCoefficients::Ptr firstCylinder = cylinders.at(0);
        //std::cout<<"make new cylinder\n";
        // make new cylinder
        pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
        coeff->values.push_back(firstCylinder->values.at(0));
        coeff->values.push_back(firstCylinder->values.at(1));
        coeff->values.push_back(firstCylinder->values.at(2));
        coeff->values.push_back(c.x - firstCylinder->values.at(0) );
        coeff->values.push_back(c.y - firstCylinder->values.at(1) );
        coeff->values.push_back(c.z - firstCylinder->values.at(2));
        coeff->values.push_back(firstCylinder->values.at(6));
       // std::cout<< "valec: "<< coeff->values.at(0)<< " "<< coeff->values.at(1)<< " "<< coeff->values.at(2)<< " "<< coeff->values.at(3)<< " "<< coeff->values.at(4)<< " " <<coeff->values.at(5)<< " "<< coeff->values.at(6)<<  "\n ";
       // m_branches.at(q)->addCylinder(coeff);
        m_cylinders.push_back(coeff);
        //std::vector<pcl::ModelCoefficients::Ptr> cylinders2 = m_branches.at(q)->getCylinders();
        //std::cout<<"pocet cylinderu2: " << cylinders2.size()<< "\n";
    }
}
float CylinderModel::cylinderFit(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI& point, float tolerance)
{
    // pro kazdy bod spocitat vodorovnou vzdalenost od point pokud je v blizke velikosti intensizity tak je to plus,
    float fit=0;
    for(int i=0; i< cloud->points.size(); i++)
    {
        float dist = std::sqrt((cloud->points.at(i).x-point.x)*(cloud->points.at(i).x-point.x) + (cloud->points.at(i).y-point.y)*(cloud->points.at(i).y-point.y));
        if(std::abs(dist - point.intensity) < tolerance)
        {
            fit++;
        }
    }
    fit/=cloud->points.size();
    return fit;
}
float CylinderModel::getDBH(std::vector<pcl::PointXYZI>& inVector)
{
    float dbh=0;
    float distance=10000000000000;
    for(int i=0; i < inVector.size(); i++)
    {
        if(std::abs(pointDistance(inVector.at(i), inVector.at(0)) - 1.3) < distance)
        {
            distance =std::abs(pointDistance(inVector.at(i), inVector.at(0)) - 1.3);
            dbh = inVector.at(i).intensity;
        }
    }
    return dbh;
}
void CylinderModel::coefficientData()
{
    // informace o regresních koeficientech za celý strom.
    int m;
    if (m_coef1.size() % 2 == 0)
    {
        m=(m_coef1.size()/2)-1;
    }
    else
    {
        m=(m_coef1.size()/2);
    }
   // std::cout << "korelacni rovnice: aX^2 + bX + c\n";
    std::sort(m_coef1.begin(), m_coef1.end());
    float average1 = accumulate( m_coef1.begin(), m_coef1.end(), 0.0)/m_coef1.size();
    float median1 =m_coef1[m];
   // std::cout << "hodnoty koeficientu a: min:"<< m_coef1.at(0)<< " max: "<< m_coef1.at(m_coef1.size()-1) << " prumer: "<< average1<< " median: "<< median1 << " \n";
    
    std::sort(m_coef2.begin(), m_coef2.end());
    float average2 = accumulate( m_coef2.begin(), m_coef2.end(), 0.0)/m_coef2.size();
    float median2 =m_coef2[m];
    //std::cout << "hodnoty koeficientu b: min:"<< m_coef2.at(0)<< " max: "<< m_coef2.at(m_coef2.size()-1) << " prumer: "<< average2<< " median: "<< median2 << " \n";
    
    std::sort(m_coef3.begin(), m_coef3.end());
    float average3 = accumulate( m_coef3.begin(), m_coef3.end(), 0.0)/m_coef3.size();
    float median3 =m_coef3[m];
   // std::cout << "hodnoty koeficientu c: min:"<< m_coef3.at(0)<< " max: "<< m_coef3.at(m_coef3.size()-1) << " prumer: "<< average3<< " median: "<< median3 << " \n";
    
}


