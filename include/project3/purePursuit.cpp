#include <project3/purePursuit.h>

// |t - sin(t)| < 0.001 when |t| <  0.181812
#define STD_V 0.45  // max speed is 1.2 m/s
#define STD_W 0.45  //rp max angular speed is 5.235988 rad/sec (=300 degree/sec)

purePursuit::purePursuit(){}

control purePursuit::get_control(point x_robot, point x_goal){
    /* TO DO
     *
     * implement purepursuit algorithm
     *
    */
//    double rel_x = x_goal.x - x_robot.x;
//    double rel_y = x_goal.y - x_robot.y;
//    double gamma = 2.0 * (-rel_x*sin(x_robot.th) + rel_y*cos(x_robot.th)) / l_sqr;
    double rel_x = (x_goal.x - x_robot.x)*cos(x_robot.th) + (x_goal.y - x_robot.y)*sin(x_robot.th);
    double rel_y = -(x_goal.x - x_robot.x)*sin(x_robot.th) + (x_goal.y - x_robot.y)*cos(x_robot.th);
    double l_sqr = pow(rel_x,2) + pow(rel_y,2);
    if( ((rel_y>0)-(rel_y<0))*rel_y < 0.2 && rel_x > 0){   // in this case, it's better to go straight
        ctrl.v = l_sqr > 2.0? STD_V: (l_sqr*0.25 + 0.5)*STD_V;
        ctrl.w = 0.0;
        return ctrl;
    }
    double rel_th = atan(rel_y/rel_x);
    if(rel_x < 0) rel_y > 0? rel_th += M_PI: rel_th -= M_PI;
//    rel_th -= x_robot.th;
//    if(rel_th > M_PI) rel_th -= 2*M_PI;
//    if(rel_th < -M_PI) rel_th += 2*M_PI;
    if(((rel_th>0)-(rel_th<0))*rel_th > M_PI/2.5 ){
        ctrl.v *= ctrl.v > 0.10? 0.50: 0.0;
        ctrl.w = (-(rel_th > 0) + (rel_th < 0)) * STD_W;// -0.50*rel_th;
        return ctrl;
    } else {
//        double l_sqr = pow(rel_x,2) + pow(rel_y,2);
        ctrl.v = l_sqr > 2.50? STD_V*0.85: (l_sqr*0.2 + 0.5)*0.85*STD_V;
        ctrl.w = -ctrl.v * 2.0 * rel_y / l_sqr;     // -gamma * STD_V
    }
    return ctrl;
}
