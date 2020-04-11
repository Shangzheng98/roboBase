//
// Created by eric on 4/10/20.
//

#ifndef ROBOTBASE_PRED_ALGRSM_H
#define ROBOTBASE_PRED_ALGRSM_H
#include <math.h>
#include <iostream>
using namespace std;
#define r 1.225
#define Cd 0.47
#define A 0.001
#define g 9.8
#define m 0.039/9.0
#define Vt sqrt((2*m*g)/(Cd*r*A))
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
