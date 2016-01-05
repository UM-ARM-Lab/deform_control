#pragma once

#include "colab_cloth_defines.h"

#include <string>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include "simulation/simplescene.h"

#define UNUSED(x) (void)(x)

inline std::string PrettyPrint(const btVector3& vector_to_print, const bool add_delimiters = false, const std::string& separator = "")
{
    UNUSED(add_delimiters);
    UNUSED(separator);
    return "btVector3: <x: " + std::to_string(vector_to_print.x()) + " y: " + std::to_string(vector_to_print.y()) + " z: " + std::to_string(vector_to_print.z()) + ">";
}

inline std::string PrettyPrint(const btQuaternion& quaternion_to_print, const bool add_delimiters = false, const std::string& separator = "")
{
    UNUSED(add_delimiters);
    UNUSED(separator);
    return "btQuaternion <x: " + std::to_string(quaternion_to_print.x()) + " y: " + std::to_string(quaternion_to_print.y()) + " z: " + std::to_string(quaternion_to_print.z()) + " w: " + std::to_string(quaternion_to_print.w()) + ">";
}

inline std::string PrettyPrint(const btTransform& transform_to_print, const bool add_delimiters = false, const std::string& separator = "")
{
    UNUSED(add_delimiters);
    UNUSED(separator);
    btVector3 vector_to_print = transform_to_print.getOrigin();
    btQuaternion quaternion_to_print(transform_to_print.getRotation());
    return "btTransform <x: " + std::to_string(vector_to_print.x()) + " y: " + std::to_string(vector_to_print.y()) + " z: " + std::to_string(vector_to_print.z()) + ">, <x: " + std::to_string(quaternion_to_print.x()) + " y: " + std::to_string(quaternion_to_print.y()) + " z: " + std::to_string(quaternion_to_print.z()) + " w: " + std::to_string(quaternion_to_print.w()) + ">";
}

inline Eigen::Affine3d toEigenAffine3d( const btTransform& bt )
{
    const btVector3& bt_origin = bt.getOrigin();
    const btQuaternion& bt_rot = bt.getRotation();
    const Eigen::Translation3d trans( bt_origin.getX(), bt_origin.getY(), bt_origin.getZ() );
    const Eigen::Quaterniond rot( bt_rot.getW(), bt_rot.getX(), bt_rot.getY(), bt_rot.getZ() );

    return trans*rot;
}

inline void nodeArrayToNodePosVector(const btAlignedObjectArray<btSoftBody::Node> &m_nodes, std::vector<btVector3> &nodeposvec)
{
    nodeposvec.resize(m_nodes.size());
    for(int i = 0; i < m_nodes.size(); i++)
    {
        nodeposvec[i] = m_nodes[i].m_x;
    }
}

inline Eigen::MatrixXd pinv(const Eigen::MatrixXd &a)
{
    // see : http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse#The_general_case_and_the_SVD_method

    if ( a.rows()<a.cols() )
    {
        cout << "pinv error!" << endl;
        return Eigen::MatrixXd();
    }

    // SVD
    Eigen::JacobiSVD< Eigen::MatrixXd> svdA(a,Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::MatrixXd vSingular = svdA.singularValues();

    // Build a diagonal matrix with the Inverted Singular values
    // The pseudo inverted singular matrix is easy to compute :
    // is formed by replacing every nonzero entry by its reciprocal (inversing).
    Eigen::MatrixXd vPseudoInvertedSingular(svdA.matrixV().cols(),1);

    for (int iRow =0; iRow<vSingular.rows(); iRow++)
    {
        if ( fabs(vSingular(iRow))<=1e-10 ) // Todo : Put epsilon in parameter
        {
            vPseudoInvertedSingular(iRow,0)=0.;
        }
        else
        {
            vPseudoInvertedSingular(iRow,0)=1./vSingular(iRow);
        }
    }

    // A little optimization here
    Eigen::MatrixXd mAdjointU = svdA.matrixU().adjoint().block(0,0,vSingular.rows(),svdA.matrixU().adjoint().cols());

    // Pseudo-Inversion : V * S * U'
    return (svdA.matrixV() *  vPseudoInvertedSingular.asDiagonal()) * mAdjointU  ;

}

inline float box_muller(float m, float s)	/* normal random variate generator */
{				        /* mean m, standard deviation s */
        float x1, x2, w, y1;
        static float y2;
        static int use_last = 0;

        if (use_last)		        /* use value from previous call */
        {
                y1 = y2;
                use_last = 0;
        }
        else
        {
                do {
                        x1 = 2.0 * (float)rand()/(float)RAND_MAX - 1.0;
                        x2 = 2.0 * (float)rand()/(float)RAND_MAX - 1.0;
                        w = x1 * x1 + x2 * x2;
                } while ( w >= 1.0 );

                w = sqrt( (-2.0 * log( w ) ) / w );
                y1 = x1 * w;
                y2 = x2 * w;
                use_last = 1;
        }

        return( m + y1 * s );
}

inline int getExtremalPoint(std::vector<btVector3> &pnt_vec, btMatrix3x3 projection_matrix, int first_dim_minmax, int second_dim_minmax, int third_dim_minmax)
{
    //first_dim_maxmin = 0 if min, first_dim_maxmin = 1 if max, first_dim_maxmin = -1 don't care
    btVector3 extremal_pnt = btVector3(pow(-1,first_dim_minmax)*1000,pow(-1,second_dim_minmax)*1000,pow(-1,third_dim_minmax)*1000);
    btVector3 projected_pnt;
    int extremal_ind = -1;
    bool bDimOK_1,bDimOK_2,bDimOK_3;
    for(size_t i = 0; i < pnt_vec.size();i++)
    {
        projected_pnt = projection_matrix * pnt_vec[i];

        bDimOK_1 = bDimOK_2 = bDimOK_3 = false;

        if(first_dim_minmax == -1)
            bDimOK_1 = true;
        if(second_dim_minmax == -1)
            bDimOK_2 = true;
        if(third_dim_minmax == -1)
            bDimOK_3 = true;

        if(projected_pnt[0] >= extremal_pnt[0] && first_dim_minmax == 1)
            bDimOK_1 = true;

        if(projected_pnt[0] <= extremal_pnt[0] && !first_dim_minmax)
            bDimOK_1 = true;

        if(projected_pnt[1] >= extremal_pnt[1] && second_dim_minmax == 1)
            bDimOK_2 = true;

        if(projected_pnt[1] <= extremal_pnt[1] && !second_dim_minmax )
            bDimOK_2 = true;

        if(projected_pnt[2] >= extremal_pnt[2] && third_dim_minmax == 1)
            bDimOK_3 = true;

        if(projected_pnt[2] <= extremal_pnt[2] && !third_dim_minmax)
            bDimOK_3 = true;


        if(bDimOK_1 && bDimOK_2 && bDimOK_3)
        {
            extremal_pnt = projected_pnt;
            extremal_ind = i;
        }
    }
    return extremal_ind;

}
