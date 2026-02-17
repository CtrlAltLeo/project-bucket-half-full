/* 
This program is a POC that will find the difference in rotation and translation between
two images in order to create a 3D map of a shape
*/
#include "Eigen/Dense"
#include "Eigen/SVD"
#include <iostream>

#define F 2.5
#define Cx 0
#define Cy 0

int main(){

    // Inverted Translation Matrix
    float K[3][3] = 
        {{1/F, 0, -Cx/F},
         {0, 1/F, -Cy/F},
         {0, 0, 1  }};

    // Vanilla 2D X Coordinates
    int rawX[2][8] = 
        {{213, 525, 216, 520, 102, 371, 381, 302}, 
        {138, 485, 143, 483, 251, 539, 291, 353}};

    // Vanilla 2D Y Coordinates
     int rawY[2][8] = 
        {{129, 120, 407, 370, 112, 102, 251, 118}, 
        {106, 116, 369, 391, 74, 81, 240, 94}};

    // Convert to Camera Coordinates

    float Camera[2][8][3]; // Format: [Camera][Point][Handle]

    for (int i = 0; i < 2; i++){
        for (int j = 0; j < 8; j++){
            Camera[i][j][0] = (rawX[i][j] - Cx) / F;
            Camera[i][j][1] = (rawY[i][j] - Cy) / F;
            Camera[i][j][2] = 1;
        }
    }

    // Now compute the Essential Matrix where AE = 0

    Eigen::MatrixXd A(8, 9);

    for (int i = 0; i < 8; i++){
        A(i, 0) = Camera[0][i][0] * Camera[1][i][0];
        A(i, 1) = Camera[1][i][0] * Camera[0][i][1];
        A(i, 2) = Camera[1][i][0];
        A(i, 3) = Camera[1][i][1] * Camera[0][i][0];
        A(i, 4) = Camera[1][i][1] * Camera[0][i][1];
        A(i, 5) = Camera[1][i][1];
        A(i, 6) = Camera[0][i][0];
        A(i, 7) = Camera[0][i][1];
        A(i, 8) = 1;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    Eigen::VectorXd e = svd.matrixV().col(8);

    Eigen::Matrix3d E;
    E << e(0), e(1), e(2), e(3), e(4), e(5), e(6), e(7), e(8);

    Eigen::JacobiSVD<Eigen::Matrix3d> svdE(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svdE.matrixU();
    Eigen::Matrix3d V = svdE.matrixV();
    Eigen::Vector3d S = svdE.singularValues();

    Eigen::Matrix3d correctedS = Eigen::Matrix3d::Zero();
    correctedS(0,0) = 1;
    correctedS(1,1) = 1;

    E = U * correctedS * V.transpose();

    Eigen::Matrix3d W;
    W << 0, -1, 0, 1, 0, 0, 0, 0, 1;

    Eigen::Matrix3d R1 = U * W * V.transpose();
    Eigen::Matrix3d R2 = U * W.transpose() * V.transpose();

    Eigen::Vector3d t1 = U.col(2);
    Eigen::Vector3d t2 = -U.col(2);

    // Build Camera Matrixes

    Eigen::Matrix<double, 3, 4> C1;
    C1 << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0;
    
    std::vector<Eigen::Matrix3d> rVals = {R1, R1, R2, R2};
    std::vector<Eigen::Vector3d> tVals = {t1, t2, t1, t2};

    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    int bestCount = -1;

    for (int i = 0; i < 4; i++){
        Eigen::Matrix3d R_test = rVals[i];
        Eigen::Vector3d t_test = tVals[i];

        if (R_test.determinant() < 0){
            R_test = -R_test;
        }

        Eigen::Matrix<double, 3, 4> C2_test;
        C2_test.block<3,3>(0,0) = R_test;
        C2_test.col(3) = t_test;

        int positiveDepthCount = 0;

        for (int j = 0; j < 8; j++){
            Eigen::Matrix4d Atri;

            Atri.row(0) = Camera[0][j][0] * C1.row(2) - C1.row(0);
            Atri.row(1) = Camera[0][j][1] * C1.row(2) - C1.row(1);
            Atri.row(2) = Camera[1][j][0] * C2_test.row(2) - C2_test.row(0);
            Atri.row(3) = Camera[1][j][1] * C2_test.row(2) - C2_test.row(1);

            // Solve AX = 0
            Eigen::JacobiSVD<Eigen::Matrix4d> svdTri(Atri, Eigen::ComputeFullV);
            Eigen::Vector4d X = svdTri.matrixV().col(3);

            X /= X(3);

            double C1z = X(2);

            Eigen::Vector3d X3 = X.head<3>();
            Eigen::Vector3d X_cam2 = R_test * X3 + t_test;
            double C2_testz = X_cam2(2);

            if (C1z > 0 && C2_testz > 0){
                positiveDepthCount++;
            }
        }
        if(positiveDepthCount > bestCount){
            bestCount = positiveDepthCount;
            R = R_test;
            t = t_test;
        }
    }

    // Triangulate Correct Position

    Eigen::Matrix<double, 3, 4> C2;
    C2.block<3,3>(0,0) = R;
    C2.col(3) = t;

    std::cout << "3D Points\n";
    Eigen::Matrix4d Atri;

    for (int i = 0; i < 8; i++){

        Atri.row(0) = Camera[0][i][0] * C1.row(2) - C1.row(0);
        Atri.row(1) = Camera[0][i][1] * C1.row(2) - C1.row(1);
        Atri.row(2) = Camera[1][i][0] * C2.row(2) - C2.row(0);
        Atri.row(3) = Camera[1][i][1] * C2.row(2) - C2.row(1);

        Eigen::JacobiSVD<Eigen::Matrix4d> svdTri(Atri, Eigen::ComputeFullV);
        Eigen::Vector4d X = svdTri.matrixV().col(3);

        X /= X(3);

        // Print Triangulated Points
        std::cout << "Point " << i << ": " << X(0) << ",\t" << X(1) << ",\t" << X(2) << std::endl;

    }
    
    return 0;
}