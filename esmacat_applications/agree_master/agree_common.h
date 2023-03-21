#ifndef AGREE_COMMON
#define AGREE_COMMON

#include <eigen3/Eigen/Geometry>


class Frame
{
    public:
        //MEMBERS
        Eigen::Matrix4d pose;           //complete pose (4x4)
        Eigen::Matrix3d orientation;    //orientation matrix (3x3)
        Eigen::Vector3d position;       //position vector (3x1)

        //METHODS
        Frame();                                    //default constructor of Frame
        Frame(Eigen::Matrix3d, Eigen::Vector3d);    //constructor of Frame (rotation matrix, translation vector)
        Frame(Eigen::Matrix4d);                     //constructor of Frame (transformation matrix)
        Eigen::Vector3d get_ang_vel(const char);          //get angular velocity of a specific axis
        Frame inv();                                //take the inverse
        Frame operator*(Frame);                     //overloaded multiplication operator of Frame: frame*frame = frame
        Eigen::Vector3d operator*(Eigen::Vector3d); //overloaded multiplication operator of Frame: frame*position = position

};


//transforms vectors of any size from degrees to radians
Eigen::VectorXd deg2rad(Eigen::VectorXd&);

//transforms scalars from degrees to radians
double deg2rad(const double);

//transforms scalars from degrees to radians
double rad2deg(const double);

//returns a rotation matrix (3x3), given an angle and an axis ('x','y' or 'z')
Eigen::Matrix3d rotation(const double, const char);

//returns a translation vector (3x1), given a displacement and an axis ('x','y' or 'z')
Eigen::Vector3d translation(const double, const char);

//returns a translation vector (3x1), given a displacement in 'x','y' and 'z'
Eigen::Vector3d translation(const double, const double, const double);

//returns a homogeneous transformation matrix (4x4), given an angle, rotation axis, displacement and and translation axis ('x','y' or 'z')
Frame htm(const double, const char, const double, const char);

//transforms a vector in a skew matrix (for cross product)
Eigen::MatrixXd vec2skew(Eigen::Vector3d);

//cross product of two vectors - To delete. Already defined in Eigen
//Eigen::Vector3d cross_product(Eigen::Vector3d a, Eigen::Vector3d b);

float const Nm_to_mNm = 1000;
float const deg_to_rad = M_PI/180;
float const rad_to_deg = 180/M_PI;


#endif // AGREE_COMMON

