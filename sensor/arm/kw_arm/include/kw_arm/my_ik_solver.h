#ifndef _MY_IK_SOLVER_
#define _MY_IK_SOLVER_

class my_ik_solver
{
    public:
        constexpr static const double PI = 3.141592653589793;
        my_ik_solver(double LengthOfL0, double LengthOfL1, double LengthOfL2, double LengthOfL3);
        ~my_ik_solver(){};
        int Solve(double X_r, double Y_r, double Z_r, double Pitch_r);
        double GetJoint0Angles(int n);
        double GetJoint1Angles(int n);
        double GetJoint2Angles(int n);
        double GetJoint3Angles(int n);

    private:
        int NumberOfSolution;
        double Joint0Angles[2];
        double Joint1Angles[2];
        double Joint2Angles[2];
        double Joint3Angles[2];
        double L0, L1, L2, L3, PITCH;
};

#endif
