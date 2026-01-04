#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix;

int main(){
    MatrixXd m(2,2);
    m(0,0) = 3;
    m(0,1) = 1;
    m(1,0) = 0;
    m(1,1) = m(0,0) + m(1,0);
    std::cout<<m<<std::endl;

    MatrixXd n = MatrixXd::Random(3,3);
    MatrixXd const_matrix = MatrixXd::Constant(3,3,1.2);
    n = (n + const_matrix) * 50; // matrix addition
    VectorXd v(3); // Vector is a column matrix
    v<<1,2,4; // initialize at run time
    std::cout<<"n = "<<n<<std::endl;
    std::cout<<"const_matrix = "<<const_matrix<<std::endl;
    std::cout<<"vector = "<<v<<std::endl;
    std::cout << "n * v =" << std::endl << n * v << std::endl;

    // Identity, null matrix initialization
    MatrixXd identity_matrix = MatrixXd::Identity(3,3);
    MatrixXd null_matrix = MatrixXd::Zero(3,3);
    std::cout<<"Identity matrix = "<<identity_matrix<<std::endl;
    std::cout<<"Null Matrix = "<<null_matrix<<std::endl;

    Matrix<double,3,3> matrix_1{{1,2,4},{4,3,6},{4,2.5,4}}; // same as MatrixXd(3,3), dynamic size
    std::cout<<"Matrix 1 = "<<matrix_1<<std::endl;
    Eigen::RowVectorXd row_vector{{1,9,2}};
    // Eigen::RowVectorXd row_vector{1,2,4} gets an error
    std::cout<<"Row vector = "<<row_vector<<std::endl;
    
    // get the row and columns
    std::cout<<"Rows = "<<row_vector.rows()<<std::endl;
    std::cout<<"Cols = "<<row_vector.cols()<<std::endl;
    std::cout<<"Size = "<<row_vector.size()<<std::endl;
    std::cout<<"Column 0 of matrix = "<<matrix_1.col(0)<<std::endl;
    std::cout<<"Row 0 of matrix = "<<matrix_1.row(0)<<std::endl;

    Eigen::Vector3d fixed_size_vector{{0,0,0}}; // Vector of fixed size
    std::cout<<"Fixed vector : "<<fixed_size_vector<<std::endl;
    std::cout<<"Fixed vector size : "<<fixed_size_vector.size()<<std::endl;
    
    Eigen::Matrix4d fixed_size_matrix; // Matrix of fixed size
    std::cout<<"Fixed Matrix : "<<fixed_size_matrix<<std::endl;
    std::cout<<"Fixed Matrix size : "<<fixed_size_matrix.size()<<std::endl;

    // operators with scalar should have the same type as the scalar
    // type as the matrix, matrix binary overloads will require the 
    // operands to have the same dimensions

    // scalar operands with a matrix : *,/,*=,/=
    std::cout<<"Matrix = "<<fixed_size_matrix<<std::endl;
    std::cout<<"Transpose = "<<fixed_size_matrix.transpose()<<std::endl;
    std::cout<<"Conjugate = "<<fixed_size_matrix.conjugate()<<std::endl;
    std::cout<<"Adjoint = "<<fixed_size_matrix.adjoint()<<std::endl;

    // do not do this a = a.transpose

    // vector operations
    Eigen::Vector3d vector1(1,2,4);
    Eigen::Vector3d vector2(6,3,8);

    std::cout<<"Dot product = "<<vector1.dot(vector2)<<std::endl;
    std::cout<<"Cross product = "<<vector1.cross(vector2)<<std::endl;

    // arithmetic reductions
    fixed_size_matrix = MatrixXd::Random(4,4);
    std::cout<<"Matrix sum = "<<fixed_size_matrix.sum()<<std::endl;
    std::cout<<"Product = "<<fixed_size_matrix.prod()<<std::endl;
    std::cout<<"Mean = "<<fixed_size_matrix.mean()<<std::endl;
    std::cout<<"Minimum Coefficient = "<<fixed_size_matrix.minCoeff()<<std::endl;
    std::cout<<"Maximum Coefficient = "<<fixed_size_matrix.maxCoeff()<<std::endl;
    std::cout<<"Trace = "<<fixed_size_matrix.trace()<<std::endl;
    std::cout<<"Diagonal = "<<fixed_size_matrix.diagonal()<<std::endl;
    std::cout<<"Norm = "<<fixed_size_matrix.norm()<<" Squared norm = "<<fixed_size_matrix.squaredNorm()<<std::endl;
    std::cout<<"Lp norm of 1 "<<fixed_size_matrix.lpNorm<1>()<<std::endl;

    // Geometry
    // Angle axis
    Eigen::Vector3d axis{1,1,1};
    axis/=axis.norm();
    std::cout<<"Axis : "<<axis<<std::endl;
    Eigen::AngleAxisd rotation = Eigen::AngleAxisd(1.5,axis);
    std::cout<<"Rotation angle = "<<rotation.angle()<<std::endl;
    std::cout<<"Rotation axis = "<<rotation.axis()<<std::endl;
    std::cout<<"Rotation matrix = "<<rotation.toRotationMatrix()<<std::endl;
    std::cout<<"Rotation inverse angle = "<<rotation.inverse().angle()<<std::endl;
    std::cout<<"Rotation inverse axis = "<<rotation.inverse().axis()<<std::endl;

    // Quaternion
    Eigen::Quaternion<double> rotation_q(rotation); // quaternion from angle axis
    std::cout<<"Quaternion w = "<<rotation_q.w()<<std::endl;
    std::cout<<"Quaternion x = "<<rotation_q.x()<<std::endl;
    std::cout<<"Quaternion y = "<<rotation_q.y()<<std::endl;
    std::cout<<"Quaternion z = "<<rotation_q.z()<<std::endl;
    std::cout<<"Quaternion w = "<<rotation_q.coeffs()[3]<<std::endl;
    rotation_q.coeffs()[3] = 0.0; // reset to test the normalization
    rotation_q.normalize(); // normalize the initialized quaternion
    std::cout<<"Quaternion w = "<<rotation_q.w()<<std::endl;
    std::cout<<"Quaternion x = "<<rotation_q.x()<<std::endl;
    std::cout<<"Quaternion y = "<<rotation_q.y()<<std::endl;
    std::cout<<"Quaternion z = "<<rotation_q.z()<<std::endl;

    // Rotation matrix
    Eigen::MatrixXd rotation_m(3,3);
    rotation_m = rotation_q.toRotationMatrix();
    std::cout<<"Rotation matrix = "<<rotation_m<<std::endl;
    Eigen::Matrix3d rotation2_m(rotation_m);
    std::cout<<"Euler angles of the rotation matrix = "<<rotation2_m.eulerAngles(2,1,0)<<std::endl;

    // Euler rotations
    Eigen::AngleAxis<double> yaw(M_PI/2, Eigen::Vector3d::UnitX());
    Eigen::AngleAxis<double> pitch(M_PI/2, Eigen::Vector3d::UnitY());
    Eigen::AngleAxis<double> roll(M_PI/2, Eigen::Vector3d::UnitX());
    Eigen::Matrix3d euler_rotation(yaw*pitch*roll);
    std::cout<<"Rotation matrix from euler angles (pi/2, pi/2, pi/2) = "<<euler_rotation<<std::endl;

    // transformation object
    Eigen::Transform<double, 3, Eigen::TransformTraits::Affine> t(euler_rotation);
    std::cout<<"Tranformation object constructed from the same rotation = "<<t.matrix()<<std::endl;
    std::cout<<"Inverse of t = "<<t.inverse().data()<<std::endl;
    std::cout<<"Rotation of t = "<<t.rotation()<<std::endl;
    std::cout<<"Translation of t = "<<t.translation()<<std::endl;
    t.translation()[0] = 1.0; // edit the matrix
    std::cout<<"Translation of t = "<<t.translation()<<std::endl;

    return 0;
}