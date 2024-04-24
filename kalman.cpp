#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>
#include <string>
#include <memory>
#include <stdio.h>
#include <tuple>
#include <cmath>
#include <complex>
#include <math.h> 
#include <string.h>

// #include <opencv2/opencv.hpp>
#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/LinearizedSystemModel.hpp>


#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/LinearizedSystemModel.hpp>

#include "KalmanState.hpp"


typedef float M; 

double pymod(double a, double b) {
    double r = fmod(a, b);
    if (r!=0 && ((r<0) != (b<0))) {
        r += b;
    }
    return r;
}

double theta_addition(double theta_1, double theta_2) {
    double angle = pymod((theta_1 + theta_2 + M_PI), (2 * M_PI)) - M_PI;
    return angle;
}

int main() {
    std::ifstream file("listy.txt"); 
    std::string line;
    std::vector<std::vector<double>> vectors;  // Wektor przechowujący wszystkie wektory

    while (getline(file, line)) {
        std::istringstream iss(line);
        int size;
        char comma;
        iss >> size >> comma; // Odczytaj rozmiar listy i przecinek

        std::vector<double> currentVector(size);
        for (int i = 0; i < size; i++) {
            iss >> currentVector[i] >> comma; // Odczytaj elementy listy jako double
        }

        // Dodaj obecny wektor do wektora wektorów
        vectors.push_back(currentVector);
    }

    // // Wyświetlanie zawartości wszystkich wektorów
    // int vectorNumber = 1;
    // for (const auto& vec : vectors) {
    //     std::cout << "Vector " << vectorNumber << ": ";
    //     for (double num : vec) {
    //         std::cout << num << " ";
    //     }
    //     std::cout << "\n";
    //     vectorNumber++;
    // }

    std::cout << "Liczba wektorów: " << vectors.size() << std::endl;

    std::cout << "Rozmiar każdego wektora:" << std::endl;
    for (size_t i = 0; i < vectors.size(); ++i) {
        std::cout << "Wektor " << i + 1 << ": " << vectors[i].size() << std::endl;
    }
    // U, U_odom, U_imu, Q

    Kalman::ExtendedKalmanFilter<State<M> > ekf;
    SystemModel<M> sys;
    Control<M> u_;
    VelocityMeasurementModel<M> odom_vel_model;
    VelocityMeasurementModel<M> imu_vel_model;
    Control<M> x_odom, x_imu;

    double last_u1 = 0.0;
    double last_u2 = 0.0;

    double sampling_rate = 0.1;

    for(int i =0; i<400; i++){
        if(i>0){
            u_.du1() = (vectors[0][2*i] - last_u1)/sampling_rate; 
            u_.du2() = (vectors[0][(2*i)+1] - last_u2)/sampling_rate;

            auto x_ekf = ekf.predict(sys, u_);

            // std::cout<< "vectors[2][0] " << vectors[2][0] << std::endl;
            x_odom.du1() = vectors[2][2*i]; 
            x_odom.du2() = vectors[2][(2*i)+1]; 

            x_imu.du1() = vectors[1][2*i]; 
            x_imu.du2() = vectors[1][(2*i)+1]; 

            if(i<10){
               std::cout<< i << " x_ekf " << x_ekf << std::endl;  
            }
        }
        
        last_u1 = vectors[0][2*i];
        last_u1 = vectors[0][(2*i)+1];
    }

    
    
    // VelocityMeasurement<M> velocity;

    // velocity = imu_vel_model.h(x_imu);
    // Kalman::Covariance<State<M> > imu_covariance;
    // imu_covariance << 0.001, 0.0, 0.0, 0.001;
    // imu_vel_model.setCovariance(imu_covariance);

    // x_ekf = ekf.update(imu_vel_model, velocity);  //update by imu 
    // std::cout <<"update by imu " <<std::endl;
        

    // velocity = odom_vel_model.h(x_odom);
    // Kalman::Covariance<State<M> > odom_covariance;
    // odom_covariance << 0.0001, 0.0, 0.0, 0.0001; 
    // odom_vel_model.setCovariance(odom_covariance);

    // x_ekf = ekf.update(odom_vel_model, velocity);  //update by wheel
    // std::cout <<"update by wheel " <<std::endl;

    // std::cout<< "x_ekf " << x_ekf << std::endl;


    return 0;
}

