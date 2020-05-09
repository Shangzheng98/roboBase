//
// Created by eric on 4/10/20.
//

#ifndef ROBOTBASE_PRED_ALGRSM_H
#define ROBOTBASE_PRED_ALGRSM_H
#include <cmath>
#include <iostream>
using namespace std;
#define Cd 0.47
#define A 0.001
#define grivity 9.8
#define V_t sqrt((2*0.039/9.0*grivity)/(Cd*1.225*A))
#define PI 3.14159


class pred_algrsm {
private:
    static double horizontal_time(double u_0, double x);
    static double vertical_time(double v_0, double y);
    static double init_angle(double y, double s);
    static bool reachable(double x, double y, double s);

public:
    static double predict_angle(double x, double y, double s);
};


#endif //ROBOTBASE_PRED_ALGRSM_H
