#include <kw_arm/my_ik_solver.h>
#include "ros/ros.h"
#include <math.h>


my_ik_solver::my_ik_solver(double LengthOfL0, double LengthOfL1, double LengthOfL2, double LengthOfL3)
{
    this->L0 = LengthOfL0;
    this->L1 = LengthOfL1;
    this->L2 = LengthOfL2;
    this->L3 = LengthOfL3;
    ROS_INFO("MY_IK_SOLVER created successfully!\n");
}

int my_ik_solver::Solve(double xx, double yy, double zz, double Pitch_r)
{
    this->PITCH = Pitch_r;
    this->Joint0Angles[0] = - atan2(yy, xx);
    this->Joint0Angles[1] = - atan2(yy, xx);

    double X_r = sqrt(xx * xx + yy * yy) - this->L3 * cos(Pitch_r);
    double Y_r = zz - this->L0 + sin(Pitch_r);

    double x1, y1, x2, y2;
    if ((L1 + L2) < sqrt(X_r * X_r + Y_r * Y_r))
    {
        return 0;
    }
    else  
    {
        double a = ((X_r * X_r + Y_r * Y_r + L1 * L1 - L2 * L2) / (2 * X_r));
        double b = Y_r / X_r;
        double delta = 4 * a * a * b * b - 4 * (b * b + 1) * (a * a - L1 * L1);
        if (delta < 0)                //find no solution
        {
            this->NumberOfSolution = 0;
            return 0;
        }
        else if (abs(delta) < 1e-6)   //find one solutino
        {
            y1 = (a * b) / (b * b + 1);
            x1 = a - b * y1;
            this->Joint1Angles[0] = PI / 2 - atan2(y1, x1);
            this->Joint2Angles[0] = - this->Joint1Angles[0] - atan2(Y_r - y1, X_r - x1) + PI / 2;
            /*
            if ((sin(this->Joint1Angles[0]) * L1 +  sin(this->Joint1Angles[0] + this->Joint2Angles[0]) * L2) - X_r > 0.1)
            {
                this->Joint2Angles[0] = this->Joint2Angles[0] + 2 * this->Joint1Angles[0];
                this->Joint1Angles[0] =  - this->Joint1Angles[0];
            }*/
            this->NumberOfSolution =1;
            return 1;
        }
        else                          //find two solutions
        {
            y1 = (2 * a * b + sqrt(delta)) / (2 * (b * b + 1));
            x1 = a - b * y1;

            y2 = (2 * a * b - sqrt(delta)) / (2 * (b * b + 1));
            x2 = a - b * y2;

            this->Joint1Angles[0] = PI / 2 - atan2(y1, x1);
            this->Joint2Angles[0] = - this->Joint1Angles[0] - atan2(Y_r - y1, X_r - x1) + PI / 2;

            this->Joint1Angles[1] = PI / 2 - atan2(y2, x2);
            this->Joint2Angles[1] = - this->Joint1Angles[1] - atan2(Y_r - y2, X_r - x2) + PI / 2;
            // ROS_INFO("%lf", this->Joint0Angles[0]);
            // ROS_INFO("%lf", this->Joint1Angles[0]);
            // ROS_INFO("%lf", this->Joint2Angles[0]);
            this->NumberOfSolution = 2;
            return 2;
        }
    }
}

double my_ik_solver::GetJoint0Angles(int n)
 {
     if (NumberOfSolution == 0) return 0;
     if (n >= 2 || n < 0) return 0;
     return this->Joint0Angles[n];
 }

double my_ik_solver::GetJoint1Angles(int n)
 {
     if (NumberOfSolution == 0) return 0;
     if (n >= 2 || n < 0) return 0;
     return this->Joint1Angles[n];
 }

double my_ik_solver::GetJoint2Angles(int n)
 {
     if (NumberOfSolution == 0) return 0;
     if (n >= 2 || n < 0) return 0;
     return this->Joint2Angles[n];
 }

 double my_ik_solver::GetJoint3Angles(int n)
 {
     if (NumberOfSolution == 0) return 0;
     if (n >= 2 || n < 0) return 0;
     this->Joint3Angles[0] = PI / 2 - this->Joint1Angles[0] - this->Joint2Angles[0] - PITCH;
     return this->Joint3Angles[n];
 }
