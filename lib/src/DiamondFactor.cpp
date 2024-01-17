#include "../include/DiamondFactor.h"
#include "../include/DiamondConstraintsFactor.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>


/*
 Camera: inv(R(2)) * R(1)
*/

int main(int argc, char *argv[])
{

    // Open the CSV file
    std::ifstream file("../../data/GT_data_and_joint_values_for_factor_graph_base_on_symforce_20.csv");
    std::vector<std::vector<double>> data;
    if (file) {
        std::string line;
        while (getline(file, line)) {
            std::stringstream ss(line);
            std::vector<double> row;
            std::string val;
            while (getline(ss, val, ',')) {
                row.push_back(stod(val));
            }
            data.push_back(row);
        }
        std::cout << "Number of data: " << data.size() << std::endl;
    } else {
        std::cout << "Unable to open file." << std::endl;
    }    
    // test data 
    // int i  = 1200;
    // int j = 9; // j=9 or j=0
    // std::cout << data[i][0+j] << std::endl; 
    // std::cout << data[i][1+j] << std::endl;    
    // std::cout << data[i][2+j] << std::endl;   
    // std::cout << data[i][3+j] << std::endl; 
    // std::cout << data[i][4+j] << std::endl;    
    // std::cout << data[i][5+j] << std::endl;
    // std::cout << data[i][6+j] << std::endl; 
    // std::cout << data[i][7+j] << std::endl;    
    // std::cout << data[i][8+j] << std::endl; 
    // std::cout << data[i][18] << std::endl;    
    // std::cout << data[i][19] << std::endl;    
    // std::cout << data[i][20] << std::endl; 
    // std::cout << data[i][21] << std::endl; 

    NonlinearFactorGraph graph;
    Values initial_estimate;

    gtsam::Rot3 init_estimate_Hand_Eye = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};

    Eigen::MatrixXd covariance_rot3 = Eigen::MatrixXd::Zero(3, 3);
    double DeltaorientationStdDev = 1.0 * M_PI / 180.0; // convert degrees to radians
    covariance_rot3.diagonal().array() = DeltaorientationStdDev;
    gtsam::noiseModel::Gaussian::shared_ptr noiseModel_rot3 = gtsam::noiseModel::Gaussian::Covariance(covariance_rot3);

    auto DiamondCalibrationNoise = noiseModel::Isotropic::Sigma(3, sqrt(19000));  
    auto EqualityNoiseModel = noiseModel::Isotropic::Sigma(1, sqrt(0.005));  
    auto BoundFactorNoiseModel = noiseModel::Isotropic::Sigma(1, sqrt(1));  
    auto InequalityNoiseModel = noiseModel::Isotropic::Sigma(1, sqrt(1));  
    auto PrirorFactorNoiseModel = noiseModel::Isotropic::Sigma(1, sqrt(0.01));  

    for (int i = 500; i <= data.size()-1 ; i++)
    {
        gtsam::Rot3 RotGT1 = { {data[i][0],  data[i][1], data[i][2]}, {data[i][3],  data[i][4], data[i][5]}, {data[i][6], data[i][7], data[i][8]} };
        gtsam::Rot3 RotGT2 = { {data[i][9],  data[i][10], data[i][11]}, {data[i][12],  data[i][13], data[i][14]}, {data[i][15], data[i][16], data[i][17]} };

        double theta11 = data[i][18];
        double theta12 = data[i][19];
        double theta21 = data[i][20];
        double theta22 = data[i][21];

        graph.add(std::make_shared<DiamondCalibrationFactor>(Symbol('o', 0), Symbol('o', 1), Symbol('x', 0), Symbol('x', 1), Symbol('r', 0), theta11, theta12, theta21, theta22, RotGT1,  RotGT2, DiamondCalibrationNoise));
    }

    graph.add(std::make_shared<Inequality>(Symbol('x', 0), Symbol('x', 1), InequalityNoiseModel)); // key second > key first
    graph.add(std::make_shared<Equality>(Symbol('x', 0), Symbol('x', 1), 90.0 * M_PI/180.0, EqualityNoiseModel));
    graph.add(std::make_shared<BoundFactor>(Symbol('x', 0), 40.0 * M_PI/180.0, 50.0 * M_PI/180.0, BoundFactorNoiseModel)); 
    graph.add(std::make_shared<BoundFactor>(Symbol('x', 1), 40.0 * M_PI/180.0, 50.0 * M_PI/180.0, BoundFactorNoiseModel)); 

    // graph.add(gtsam::PriorFactor<double>(Symbol('o', 0), 0 * M_PI/180, PrirorFactorNoiseModel));
    // graph.add(gtsam::PriorFactor<double>(Symbol('o', 1), 0 * M_PI/180, PrirorFactorNoiseModel));
    // graph.add(gtsam::PriorFactor<gtsam::Rot3>(Symbol('r', 0), init_estimate_Hand_Eye, noiseModel_rot3));

    double init_estimate_alpha = 45.0 * M_PI/180.0;
    double init_estimate_beta = 45.0 * M_PI/180.0;

    initial_estimate.insert(Symbol('x', 0), init_estimate_alpha); // x(0)
    initial_estimate.insert(Symbol('x', 1), init_estimate_beta); // x(1)
    initial_estimate.insert(Symbol('o', 0), 0.0 * M_PI/180); // offset(0)
    initial_estimate.insert(Symbol('o', 1), 0.0 * M_PI/180); // offset(1)
    initial_estimate.insert(Symbol('r', 0), init_estimate_Hand_Eye); // r(0)
    
    // graph.print();

    // Optimize using Levenberg-Marquardt optimization
    LevenbergMarquardtOptimizer optimizer(graph, initial_estimate);
    Values result_LM = optimizer.optimize();
    std::cout << "using Levenberg-Marquardt optimization:\n";
    result_LM.print();
    std::cout << "Total factor graph error is: " << optimizer.error() << std::endl;

    std::cout << "\nAlpha is: " << (result_LM.at<double>(Symbol('x', 0))) * 180/M_PI << std::endl;
    std::cout << "\nBeta is: " << (result_LM.at<double>(Symbol('x', 1))) * 180/M_PI << std::endl;
    std::cout << "\nBeta + Beta is: " << (result_LM.at<double>(Symbol('x', 1)) + result_LM.at<double>(Symbol('x', 0))) * 180/M_PI << std::endl;
    std::cout << "\nOffset0 is: " << (result_LM.at<double>(Symbol('o', 0))) * 180/M_PI << std::endl;
    std::cout << "\nOffset1 is: " << (result_LM.at<double>(Symbol('o', 1))) * 180/M_PI << std::endl;
    gtsam::Rot3 Hand_Eye_Matrix = result_LM.at<gtsam::Rot3>(Symbol('r', 0));
    std::pair<gtsam::Unit3, double> AngleAxis = Hand_Eye_Matrix.axisAngle();
    std::cout << "\nAxis of rotation matrix is: " << AngleAxis.first << std::endl;
    std::cout << "\nAngle of rotation matrix is: " << AngleAxis.second * 180/M_PI << std::endl;

    return 0;
}
