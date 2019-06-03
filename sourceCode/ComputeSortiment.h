//
//  ComputeSortiment.hpp
//  3DForest043
//
//  Created by Jan Trochta on 26/10/2018.
//

#ifndef ComputeSortiment_h
#define ComputeSortiment_h

#include <stdio.h>
#include <pcl/common/common_headers.h>
#include <pcl/ModelCoefficients.h>
//#include "tree.h"
#include "segment.h"


class ComputeSortiment
{
public:
    ComputeSortiment();
    ComputeSortiment(std::vector<std::shared_ptr<Segment>> branches, bool finalSortiment);
    ~ComputeSortiment();
    void compute();
    std::vector<pcl::ModelCoefficients::Ptr> getCylinders();
    std::vector<int> getSortiments();
    void setSortimentsSize();
    void setFinalSortiment( bool finalSortiment);
    float getSortimentVolume(int sortimentID);
    float getSortimentLenght(int sortimentID);
    
protected:
    int curvature(std::vector<pcl::ModelCoefficientsPtr> cylinders);
    int diameterSize(std::vector<pcl::ModelCoefficientsPtr> cylinders);
    int skewness(std::vector<pcl::ModelCoefficientsPtr> cylinders);
    int sukatost(std::vector<pcl::ModelCoefficientsPtr> cylinders, std::vector<pcl::ModelCoefficientsPtr> suky);
    int sortimentCurvature(std::vector<pcl::ModelCoefficientsPtr> cylinders);
    
    
    
    void sukyVetve(std::vector<pcl::ModelCoefficientsPtr>& cylinders, int branchID);
    int curvature(pcl::ModelCoefficientsPtr line, pcl::ModelCoefficientsPtr cylinder);
    int diameterSize(pcl::ModelCoefficientsPtr celo, pcl::ModelCoefficientsPtr cep);
    int skewness(pcl::ModelCoefficientsPtr celo, pcl::ModelCoefficientsPtr cep);
    float pointDiff(pcl::PointXYZI a, pcl::PointXYZI b);
    float radiusDiff(pcl::ModelCoefficients::Ptr& coeff, pcl::ModelCoefficients::Ptr& coeff2);
    float curvatureDiff(pcl::ModelCoefficients::Ptr& line, pcl::ModelCoefficients::Ptr& point);
    float cylinderDistance(pcl::ModelCoefficients::Ptr& cyl1, pcl::ModelCoefficients::Ptr& cyl2);
    int continuosSortiment(std::vector<int> sortiments, int sortimentID, int sort, std::vector<bool>& usedSortiment);
    float computeCylinderVolume(pcl::ModelCoefficients::Ptr& model);
    bool cylinderTouch(pcl::ModelCoefficients::Ptr& coeff, pcl::ModelCoefficients::Ptr& coeff2);
    bool pointAreTheSame(pcl::PointXYZI a, pcl::PointXYZI b);
    
    pcl::ModelCoefficients::Ptr lineCoefficient(pcl::ModelCoefficients::Ptr& coeff, pcl::ModelCoefficients::Ptr& coeff2);
    
    std::vector<std::shared_ptr<Segment> > m_branches;
    std::vector<pcl::ModelCoefficientsPtr> m_cylinders;
    std::vector<int> m_sortiments;
    bool m_finalSortiment = false;
    float m_sortiment1_volume =0;
    float m_sortiment1_lenght =0;
    float m_sortiment2_volume =0;
    float m_sortiment2_lenght =0;
    float m_sortiment3_volume =0;
    float m_sortiment3_lenght =0;
    float m_sortiment4_volume =0;
    float m_sortiment4_lenght =0;
    float m_sortiment5_volume =0;
    float m_sortiment5_lenght =0;
    float m_sortiment6_volume =0;
    float m_sortiment6_lenght =0;
};

#endif /* ComputeSortiment_hpp */
