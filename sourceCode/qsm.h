//
//  qsm.h
//  3D_Forest
//
//  Created by Jan Trochta on 16.05.18.
//

#ifndef qsm_h
#define qsm_h

#include <stdio.h>
#include <pcl/common/common_headers.h>
#include <pcl/ModelCoefficients.h>
#include "tree.h"

class CylinderModel
{
public:
    CylinderModel();
    CylinderModel(std::vector<std::shared_ptr<Segment>> branches, int iter, float cylinderSize);
    ~CylinderModel();
    void compute();
    std::vector<pcl::ModelCoefficients::Ptr> getCylinders();
    void setTreecloud(pcl::PointCloud<pcl::PointXYZI>::Ptr treeCloud);
    void setLimit(float limit);
    void setBranches(std::vector<std::shared_ptr<Segment>> branches);
    void setIterations(int iter);
    void setCylinderSize(float cs);
    void setTreeHeight(float h);
    void setTreePosition(pcl::PointXYZI pose);
    void setBranchLength(float branchLength);
    void setOrder(int order);
    std::vector<stred>& getStemCurve();
    void setStemCurve(std::vector<pcl::PointXYZI>& inVector);
    
    
protected:
    void computeStem();
    void computeBranches();
    void computePoints(int branchID, std::vector<pcl::PointXYZI>& points);
    void branchLinear(std::vector<pcl::PointXYZI>& inVector);
    //void setTree(Tree* tree);
    void translateCloud();
    void setSegmentParentCylinder(int i);
    float cylinderPointDistance( pcl::PointXYZI p, pcl::ModelCoefficients::Ptr cylinder);
    bool segmentPartition(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI& u, float height, bool reverseDirection = false);
    bool computeCylinder(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float startHeight, float stopHeight, pcl::PointXYZI& cylinder, float& fit, float tolerance=0.01);
    void smallerRadius(std::vector<pcl::PointXYZI>& inVector);
    bool polynomialFit(std::vector<pcl::PointXYZI>& inVector, std::vector<pcl::PointXYZI>& outVector);
    
    bool petras(std::vector<pcl::PointXYZI>& inVector, std::vector<pcl::PointXYZI>& outVector);
    void smallerThanParent();
    void smallerThanParent(int q, std::vector<pcl::PointXYZI>& points);
    void branchParentConnection();
    void polyfit(const std::vector<float> &xv, const std::vector<float> &yv, const std::vector<float> &wv,std::vector<float> &coeff, int order);
    pcl::PointXYZI setCircle(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void setCylinder(std::vector<pcl::PointXYZI> points);
    void swapXZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out);
    void swapYZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out);
    void swapXZ(pcl::PointXYZI& u, pcl::PointXYZI& v);
    void swapYZ(pcl::PointXYZI& u, pcl::PointXYZI& v);
    float cylinderVolume(pcl::ModelCoefficients::Ptr& coeff);
    std::vector<pcl::ModelCoefficients::Ptr> getCylinders(std::vector<pcl::PointXYZI> points, int branchID);
    pcl::ModelCoefficients::Ptr contructCylinder(pcl::PointXYZI p1, pcl::PointXYZI p2);
    bool logFit(std::vector<pcl::PointXYZI>& inVector, std::vector<pcl::PointXYZI>& outVector);
    void connectionWithParent(std::vector<pcl::PointXYZI>& inVector, int branchId, std::vector<pcl::PointXYZI>& outVector);
    
    void setSegmentCloud(std::shared_ptr<Segment> branch, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out );
    float pointDistance (pcl::PointXYZI& u, pcl::PointXYZI& v);
    float cylinderFit(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI& point, float tolerance=0.01);
    float getDBH(std::vector<pcl::PointXYZI>& inVector);
    
    bool nonLinearRegressionStem(std::vector<pcl::PointXYZI>& inVector);
    bool nonLinearRegressionBranches(std::vector<pcl::PointXYZI>& inVector);
    bool regressionCoeff(Eigen::VectorXf coeff);
    void medianvalues();
    void coefficientData();
    void clearCoeffVector();
    void regressionCylindersStem(std::vector<pcl::PointXYZI>& inVector, std::vector<pcl::PointXYZI>& outVector);
    void regressionCylindersBranches(std::vector<pcl::PointXYZI>& inVector, std::vector<pcl::PointXYZI>& outVector);
    
    
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_treeCloud;
    std::vector<std::shared_ptr<Segment>> m_branches;
    std::vector<pcl::ModelCoefficients::Ptr> m_cylinders;
   // Tree* m_tree;
    float m_cylinderSize = 0.2;
    float m_limit =0.02;
    int m_iterations = 150;
    float m_theta = 0.02;
    float m_volume = 0;
    float m_treeHeight = 0;
    pcl::PointXYZI m_position;
    int m_swap =0; //0 == z longest, 1 = X longest, 2 = Y longest
    float m_branchLength=1;
    int m_order = 50;
    std::vector<stred> m_stemCurve;
    std::vector<float> m_coef1;
    std::vector<float> m_coef2;
    std::vector<float> m_coef3;
    float m_median1;
    float m_median2;
    float m_median3;
};

#endif /* qsm_h */
