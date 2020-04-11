//
// Created by eric on 4/10/20.
//

#include "pred_algrsm.h"


double pred_algrsm::horizontal_time(double u_0, double x) {
    return (Vt*Vt*exp(x*g/(Vt*Vt))-Vt*Vt)/(g*u_0);
}
double pred_algrsm::vertical_time(double v_0, double y) {float e = exp(2*y*g/(Vt*Vt));
    float v = sqrt((v_0*v_0+Vt*Vt-e*Vt*Vt)/e);
    float t = (atan(v_0/Vt)-atan(v/Vt))*Vt/g;

    return t;
}
double pred_algrsm::init_angle(double y,double s){
    float e = exp(2*y*g/(Vt*Vt));
    float v_0 = sqrt(Vt*Vt*e-Vt*Vt);
    return asin(v_0/s);
}

bool pred_algrsm::reachable(double x, double y, double s) {
    double e = exp(2*y*g/(Vt*Vt));
    double v_0 = sqrt(Vt*Vt*e-Vt*Vt);
    double t = atan(v_0/Vt)*Vt/g;
    if(s<v_0) return false;
    double u_0 = sqrt(s*s-v_0*v_0);
    double expected_x = Vt*Vt*log((Vt*Vt+g*u_0*t)/(Vt*Vt))/g;
    if (expected_x<x)return false;

    return true;
}

 double pred_algrsm::predict_angle(double x, double y, double s){
    // x: horizontal distance
    // y: vertical distance
    // s: initial speed
    if(!reachable(x,y,s)) return -1;

    double theta_min = init_angle(y,s);
    cout<<theta_min<<endl;
    double theta_max = theta_min + 7*PI/180.0;
    double temp = (theta_max+theta_min)/2.0;
    double d_t = vertical_time(s*sin(temp),y)-horizontal_time(s*cos(temp),x);
    while(abs(d_t)>0.0001){
        /// get new theta
        if(d_t > 0){
            theta_min = temp;
        }else{
            theta_max = temp;
        }
        temp = (theta_max+theta_min)/2.0;
        cout<<d_t<<"   "<<theta_min*180/PI <<"   "<<theta_max*180/PI <<endl;
        d_t = vertical_time(s*sin(temp),y)-horizontal_time(s*cos(temp),x);

    }
    return temp;
}