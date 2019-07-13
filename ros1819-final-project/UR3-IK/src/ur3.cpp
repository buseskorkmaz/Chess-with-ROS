#include <ur3.h>
#include <boost/math/special_functions/sign.hpp>
#include <iostream>


//    <xacro:property name="d1" value="0.089159" />
//    <xacro:property name="a2" value="-0.42500" />
//    <xacro:property name="a3" value="-0.39225" />
//    <xacro:property name="d4" value="0.10915" />
//    <xacro:property name="d5" value="0.09465" />
//    <xacro:property name="d6" value="0.0823" />

//Modified DH parameters - UR5
//static const double alpha[6] = {0., M_PI_2, 0., 0., -M_PI_2, -M_PI_2};
//static const double a[6] = {0., 0., 0.425, 0.39225, 0., 0.};
//static const double d[6] = {0.089159, 0., 0., 0.10915, 0.09465, 0.0823};
//static const double offset[6] = {-M_PI, M_PI, 0., 0., M_PI, 0.};
////static const double offset[6] = {-0.75*M_PI, M_PI, 0., 0., M_PI, 0.};




//    <!-- Kinematic model UR3-->
//    <!-- Properties from urcontrol.conf -->
//    <xacro:property name="d1" value="0.1519" />
//    <xacro:property name="a2" value="-0.24365" />
//    <xacro:property name="a3" value="-0.21325" />
//    <xacro:property name="d4" value="0.11235" />
//    <xacro:property name="d5" value="0.08535" />
//    <xacro:property name="d6" value="0.0819" />

//Modified DH parameters - UR3
static const double alpha[6] = {0., M_PI_2, 0., 0., -M_PI_2, -M_PI_2};
static const double a[6] = {0., 0., 0.24365, 0.21325, 0., 0.};
static const double d[6] = {0.1519, 0., 0., 0.11235, 0.08535, 0.0819};
static const double offset[6] = {-M_PI, M_PI, 0., 0., M_PI, 0.};


using namespace std;

//A UR3 configuration, that can be valid or not
struct Solution {
    Solution() : valid(false) {}
    bool valid;
    UR3::Configuration theta;
};


//Returns false if x is not in the interval [xmin-tol, xmax+tol]
//Otherwise, returns true and x is adjuesteb to be in the interval [xmin, xmax]
bool in_interval(double &x, const double xmin, const double xmax, const double tol = 1e-6) {
    //If x is in the interval
    if ((xmin-tol <= x) && (x <= xmax+tol)) {
        //Adjust x if necessary
        if (x < xmin) {
            x = xmin;
        } else if (xmax < x) {
            x = xmax;
        }

        return true;
    } else {
        return false;
    }
}


//Returns theta in the [-pi, pi] interval
double saturate(double theta) {
    if (in_interval(theta,-M_PI,M_PI)) {
        return theta;
    } else {
        return theta - boost::math::sign(theta)*2.*M_PI*floor(fabs(theta)/(2.*M_PI) + 0.5);
    }
}


//Adjusts the specified joint value to be the nearest one next to the
//reference, in the physically achievable interval
void adjust(double &theta, const double theta_ref) {
    double tmp_theta;

    theta = saturate(theta);

    //Calculate the other physically achievable joint value
    if (in_interval(theta,0.,0.)) {//If theta is aproximately equal to zero
        tmp_theta = theta + boost::math::sign(theta_ref)*2.*M_PI;
    } else {
        tmp_theta = theta - boost::math::sign(theta)*2.*M_PI;
    }

    //Update joint value if necessary
    if (fabs(theta_ref-theta) > fabs(theta_ref-tmp_theta)) {
        theta = tmp_theta;
    }

    //theta = theta_ref if |theta-theta_ref| < TOL
    in_interval(theta,theta_ref,theta_ref);
}


//Solves the UR3 inverse kinematics for the 3rd and 4th joints
//and decides if the solution is valid
//joint value will be the nearest one next to the reference
void find_theta34(const Eigen::AffineCompact3d &transform, Solution *solution,
                  const UR3::Configuration &theta_min,
                  const UR3::Configuration &theta_max,
                  const double s5, const double c5, const double s6, const double c6,
                  const double  x, const double  y) {
    const double theta2 = solution[0].theta[1]+offset[1];
    const double s2 = sin(theta2);
    const double c2 = cos(theta2);

    const double alpha = atan2(y-a[2]*s2,x-a[2]*c2);

    double theta3 = alpha - theta2 - offset[2];
    adjust(theta3,0.5*(theta_min[2]+theta_max[2]));
    if (in_interval(theta3,theta_min[2],theta_max[2])) {
        solution[0].theta[2] = theta3;

        const double nz = transform(2,0);

        const double oz = transform(2,1);

        const double az = transform(2,2);

        const double beta = atan2(c5*(c6*nz-s6*oz)-s5*az,-nz*s6-oz*c6);

        theta3 += offset[2];

        double theta4 = beta - theta2 - theta3 - offset[3];
        adjust(theta4,0.5*(theta_min[3]+theta_max[3]));
        if (in_interval(theta4,theta_min[3],theta_max[3])) {
            solution[0].theta[3] = theta4;

            solution[0].valid = true;
        }
        else{// else theta4 is out of range
            cout<<"theta4 is out of range"<<endl;
        }
    }
    else{// else theta3 is out of range
        cout<<"theta3 is out of range"<<endl;
    }
}


//Solves the UR3 inverse kinematics for the 6th joint and the two possible values of the 2nd joint
//and calls the function of the next joints to be found
//joint value will be the nearest one next to the reference
void find_theta62(const Eigen::AffineCompact3d &transform, Solution *solution,
                  const UR3::Configuration &theta_ref,
                  const UR3::Configuration &theta_min,
                  const UR3::Configuration &theta_max,
                  const double s1, const double c1, const double tol = 1e-6) {
    const double nx = transform(0,0);
    const double ny = transform(1,0);

    const double ox = transform(0,1);
    const double oy = transform(1,1);

    const double theta5 = solution[0].theta[4]+offset[4];
    const double s5 = sin(theta5);

    double theta6;
    if (fabs(s5) < tol) {
        theta6 = theta_ref[5];
    } else {
        theta6 = atan2((s1*ox-c1*oy)/s5,(-s1*nx+c1*ny)/s5) - offset[5];
        adjust(theta6,0.5*(theta_min[5]+theta_max[5]));
    }
    if (in_interval(theta6,theta_min[5],theta_max[5])) {
        for (unsigned int i = 0; i < 2; ++i) {
            solution[i].theta[5] = theta6;
        }

        theta6 += offset[5];
        const double s6 = sin(theta6) ;
        const double c6 = cos(theta6);

        const double nz = transform(2,0);

        const double oz = transform(2,1);

        const double ax = transform(0,2);
        const double ay = transform(1,2);
        const double az = transform(2,2);

        const double px = transform(0,3);
        const double py = transform(1,3);
        const double pz = transform(2,3);

        const double x = s1*(d[4]*(s6*ny+c6*oy)-ay*d[5]+py)+c1*(d[4]*(s6*nx+c6*ox)-ax*d[5]+px);
        const double y = d[4]*(nz*s6+oz*c6)-az*d[5]-d[0]+pz;

        double c_beta = (a[3]*a[3] - a[2]*a[2] - x*x - y*y) / (2.*a[2]*hypot(x,y));

        if (in_interval(c_beta,-1.,1.)) {
            const double c5 = cos(theta5);

            const double alpha = atan2(-y,-x);
            const double beta = acos(c_beta);

            double theta2;

            theta2 = alpha + beta - offset[1];
            adjust(theta2,0.5*(theta_min[1]+theta_max[1]));
            if (in_interval(theta2,theta_min[1],theta_max[1])) {
                solution[0].theta[1] = theta2;

                find_theta34(transform,&solution[0],theta_min,theta_max,s5,c5,s6,c6,x,y);
            }
            else{// else theta2 is out of range
                cout<<"theta2 is out of range"<<endl;
            }

            theta2 = alpha - beta - offset[1];
            adjust(theta2,0.5*(theta_min[1]+theta_max[1]));
            if (in_interval(theta2,theta_min[1],theta_max[1])) {
                solution[1].theta[1] = theta2;

                find_theta34(transform,&solution[1],theta_min,theta_max,s5,c5,s6,c6,x,y);
            }
            else{// else theta2 is out of range
                cout<<"theta2 is out of range"<<endl;
            }
        }
        else{// else theta2 has no solution
            cout<<"theta2 has no solution"<<endl;
        }
    }
    else{// else theta6 is out of range
        cout<<"theta6 is out of range"<<endl;
    }
}


//Solves the UR3 inverse kinematics for the two possible values of the 5th joint
//and calls the function of the next joints to be found
//joint value will be the nearest one next to the reference
void find_theta5(const Eigen::AffineCompact3d &transform, Solution *solution,
                 const UR3::Configuration &theta_ref,
                 const UR3::Configuration &theta_min,
                 const UR3::Configuration &theta_max) {
    const double px = transform(0,3);
    const double py = transform(1,3);

    const double theta1 = solution[0].theta[0]+offset[0];
    const double s1 = sin(theta1);
    const double c1 = cos(theta1);

    double c_alpha = (-px*s1+py*c1+d[3]) / d[5];

    if (in_interval(c_alpha,-1.,1.)) {
        const double alpha = acos(c_alpha);

        double theta5;

        theta5 = alpha - offset[4];
        adjust(theta5,0.5*(theta_min[4]+theta_max[4]));
        if (in_interval(theta5,theta_min[4],theta_max[4])) {
            for (unsigned int i = 0; i < 2; ++i) {
                solution[i].theta[4] = theta5;
            }

            find_theta62(transform,&solution[0],theta_ref,theta_min,theta_max,s1,c1);
        }
        else{// else theta5 is out of range
            cout<<"theta5 is out of range"<<endl;
        }

        theta5 = -alpha - offset[4];
        adjust(theta5,0.5*(theta_min[4]+theta_max[4]));
        if (in_interval(theta5,theta_min[4],theta_max[4])) {
            for (unsigned int i = 2; i < 4; ++i) {
                solution[i].theta[4] = theta5;
            }

            find_theta62(transform,&solution[2],theta_ref,theta_min,theta_max,s1,c1);
        }
        else{// else theta5 is out of range
            cout<<"theta5 is out of range"<<endl;
        }
    }
    else{// else theta5 has no solution
            cout<<"theta5 has no solution"<<endl;
    }
}


//Solves the UR3 inverse kinematics for the two possible values of the 1st joint
//and calls the function of the next joints to be found
//joint value will be the nearest one next to the reference
void find_theta1(const Eigen::AffineCompact3d &transform, Solution *solution,
                 const UR3::Configuration &theta_ref,
                 const UR3::Configuration &theta_min,
                 const UR3::Configuration &theta_max) {
    const double ax = transform(0,2);
    const double ay = transform(1,2);

    const double px = transform(0,3);
    const double py = transform(1,3);

    const double c_alpha = px - d[5]*ax;
    const double s_alpha = py - d[5]*ay;

    double c_beta = d[3] / hypot(c_alpha,s_alpha);

    if (in_interval(c_beta,-1., 1.)) {
        const double alpha = atan2(s_alpha,c_alpha);
        const double beta = acos(c_beta);

        double theta1;

        theta1 = alpha + beta + M_PI_2 - offset[0];
        adjust(theta1,0.5*(theta_min[0]+theta_max[0]));

        if (in_interval(theta1,theta_min[0],theta_max[0])) {
            for (unsigned int i = 0; i < 4; ++i) {
                solution[i].theta[0] = theta1;
            }

            find_theta5(transform,&solution[0],theta_ref,theta_min,theta_max);
        }
        else{// else theta1 is out of range
            cout<<"theta1 is out of range"<<endl;
        }

        theta1 = alpha - beta + M_PI_2 - offset[0];
        adjust(theta1,0.5*(theta_min[0]+theta_max[0]));

        if (in_interval(theta1,theta_min[0],theta_max[0])) {
            for (unsigned int i = 4; i < 8; ++i) {
                solution[i].theta[0] = theta1;
            }

            find_theta5(transform,&solution[4],theta_ref,theta_min,theta_max);
        }
        else{// else theta1 is out of range
            cout<<"theta1 is out of range"<<endl;
        }
    }
    else{// else theta1 has no solution
            cout<<"theta1 has no solution"<<endl;
    }
}


//Solves the UR3 inverse kinematics to place the TCP in the given pose
//Returns all possible valid solutions
bool UR3::solveIK(const Eigen::AffineCompact3d &transform,
                  std::vector<UR3::Configuration> &theta,
                  const UR3::Configuration &theta_ref,
                  const UR3::Configuration &theta_min,
                  const UR3::Configuration &theta_max) {
    Solution solution[8];
    find_theta1(transform,solution,theta_ref,theta_min,theta_max);
    theta.clear();
    for (unsigned int i = 0; i < 8; ++i) {
        if (solution[i].valid) theta.push_back(solution[i].theta);
    }
    return theta.size() > 0;
}
