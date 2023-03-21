# include "agree_common.h"


//Default constructor of Frame
Frame::Frame()
{
    this->orientation = Eigen::Matrix3d::Identity();
    this->position = Eigen::Vector3d::Zero();
    this->pose = Eigen::Matrix4d::Identity();
}

//Constructor of Frame (rotation matrix, translation vector)
Frame::Frame(Eigen::Matrix3d r, Eigen::Vector3d p)
{
    this->orientation = r;
    this->position = p;
    this->pose = Eigen::Matrix4d::Identity();
    this->pose.block<3,3>(0,0) = r;
    this->pose.block<3,1>(0,3) = p;
}

//Constructor of Frame (transformation matrix)
Frame::Frame(Eigen::Matrix4d t)
{
    this->pose = t;
    this->orientation = this->pose.block<3,3>(0,0);
    this->position = this->pose.block<3,1>(0,3);
}

//Get angular velocity of a specific axis
Eigen::Vector3d Frame::get_ang_vel(const char axis)
{
    Eigen::Vector3d ret;
            ret = Eigen::Vector3d::Zero();

    switch(axis)
    {
        case 'x':
            ret = this->orientation.block<3,1>(0,0);
            break;
        case 'y':
            ret = this->orientation.block<3,1>(0,1);
            break;
        case 'z':
            ret = this->orientation.block<3,1>(0,2);
            break;
        default:
            break;
    }

    return ret;
}

//Take the inverse
Frame Frame::inv()
{
    Frame ret;

    ret.pose = this->pose.inverse();
    ret.orientation = ret.pose.block<3,3>(0,0);
    ret.position = ret.pose.block<3,1>(0,3);

    return ret;
}

//Overloaded multiplication operator of Frame: frame*frame = frame
Frame Frame::operator*(Frame h)
{
    Frame ret;
    ret.pose = this->pose*h.pose;
    ret.orientation = ret.pose.block<3,3>(0,0);
    ret.position = ret.pose.block<3,1>(0,3);

    return ret;
}

//Overloaded multiplication operator of Frame: frame*position = position
Eigen::Vector3d Frame::operator*(Eigen::Vector3d p)
{
    Eigen::Vector4d p_augmented, p_result;
    p_augmented.head<3>() = p;
    p_augmented(3) = 1;
    p_result = this->pose*p_augmented;

    return p_result.head<3>();
}

//Transforms vectors of any size from degrees to radians
Eigen::VectorXd deg2rad(Eigen::VectorXd& angle)
{
    return angle*0.0174532925;
}


//Transforms scalars from degrees to radians
double deg2rad(const double angle)
{
    return angle*0.0174532925;
}

//Transforms scalars from radians to degrees
double rad2deg(const double angle)
{
    return angle*180/M_PI;
}

//Returns a rotation matrix (3x3), given an angle and an axis ('x','y' or 'z')
Eigen::Matrix3d rotation(const double angle, const char axis)
{
    Eigen::Matrix3d ret;
    ret << Eigen::Matrix3d::Identity();

    switch(axis)
    {
        case 'x':
            ret << 1, 0, 0,
                   0, cos(angle), -sin(angle),
                   0, sin(angle), cos(angle);
            break;
        case 'y':
            ret << cos(angle), 0, sin(angle),
                   0, 1, 0,
                   -sin(angle), 0, cos(angle);
            break;
        case 'z':
            ret << cos(angle), -sin(angle), 0,
                   sin(angle), cos(angle), 0,
                   0, 0, 1;
            break;
    }

    return ret;
}

//Returns a translation vector (3x1), given a displacement and an axis ('x','y' or 'z')
Eigen::Vector3d translation(const double d, const char axis)
{
    Eigen::Vector3d ret;
    ret << Eigen::Vector3d::Zero();

    switch(axis)
    {
        case 'x':
            ret(0) = d;
            break;
        case 'y':
            ret(1) = d;
            break;
        case 'z':
            ret(2) = d;
            break;
    }

    return ret;
}

//Returns a translation vector (3x1), given a displacement in 'x','y' and 'z'
Eigen::Vector3d translation(const double x, const double y, const double z)
{
    Eigen::Vector3d ret;
    ret << x, y, z;

    return ret;
}

//Returns a homogeneous transformation matrix (4x4), given an angle, rotation axis, displacement and and translation axis ('x','y' or 'z')
Frame htm(const double angle, const char axis_r, const double d, const char axis_t)
{
    Frame ret;
    ret = Frame(rotation(angle,axis_r),translation(d,axis_t));
    return ret;
}


//Transforms a vector in a skew matrix (for cross product)
Eigen::MatrixXd vec2skew(const Eigen::Vector3d v)
{
    Eigen::Matrix3d ret;
    ret << 0, -v(2), v(1),
           v(2), 0, -v(0),
           -v(1), v(0), 0;

    return ret;
}

// //Returns the cross product of two vectors - To delete. Already defined in Eigen
//Eigen::Vector3d cross_product(Eigen::Vector3d a, Eigen::Vector3d b){
//    Eigen::Vector3d ret;
//    ret = vec2skew(a)*b;
//    return ret;
//}
