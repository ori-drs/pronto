/*
 * Copyright (c) 2015-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * Authors: Marco Frigerio, Michele Focchi, Marco Camurri
 *
 * This file is part of pronto_quadruped_commons, a library for
 * algebra, kinematics and dynamics for quadruped robots.
 * This library is a fork of iit_commons.
 * For more information see:
 * https://github.com/iit-DLSLab/iit_commons
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#pragma once

#include <Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXR;

namespace pronto {
namespace commons {

template<typename DerivedA, typename DerivedB>
/**
 * @brief allclose Compare dense vectors (matrices) using both relative and
 * absolute tolerances. Equivalent of all close in python If the following
 * equation is element-wise True, then allclose returns True.
 *
 * absolute(a - b) <= (atol + rtol * absolute(b))
 *
 * The tolerance values are positive, typically very small numbers.
 *
 * The relative difference (rtol * abs(b)) and the absolute difference atol are
 * added together to compare against the absolute difference between a and b.
 *
 * @param a
 * @param b
 * @param rtol
 * @param atol
 * @return Returns True if two arrays are element-wise equal within a tolerance.
 */
bool allclose(const Eigen::DenseBase<DerivedA>& a,
              const Eigen::DenseBase<DerivedB>& b,
              const typename DerivedA::RealScalar& rtol = Eigen::NumTraits<typename DerivedA::RealScalar>::dummy_precision(),
              const typename DerivedA::RealScalar& atol = Eigen::NumTraits<typename DerivedA::RealScalar>::epsilon())
{
    return ((a.derived() - b.derived()).array().abs()
            <= (atol + rtol * b.derived().array().abs())).all();
}

template <typename Scalar, int R, int C>
/**
 * @brief setFinite removes all inf and nans and replaces them with zeros
 * @param matrix
 */
void setFinite(Eigen::Matrix<Scalar, R, C>& matrix){
    matrix = (matrix.unaryExpr([](Scalar v) { return std::isfinite(v)? v : 0; }));
}

template <typename Scalar, int R, int C>
/**
 * @brief setFinite removes all inf and nans and replaces them with the
 * values from the second argument
 * @param m1 a matrix with potentially inf or nans
 * @param m2 a matrix from which to replace with values if the corresponding ones
 * from m1 are inf or nans
 * @warning no checks are made if m2 has nans or inf itself
 */
void setFinite(Eigen::Matrix<Scalar, R, C>& m1, const Eigen::Matrix<Scalar, R, C>& m2){
    m1 = (m1.binaryExpr(m2, [](Scalar v1, Scalar v2) { return std::isfinite(v1)? v1 : v2; }));
}


template <typename MxIN, typename MxOut> inline
/**
 * @brief psdInv Computes the Moore-Penrose pseudo inverse of a matrix using
 * Singular Value Decomposition, as described
 * <a href="https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse">here</a>
 * @param[in] a
 * @param[out] result
 * @param[in] tolerance
 * @remark the second parameter is not really const.
 * It gets modified (it contains the result)
 * see http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
 */
void psdInv(const Eigen::MatrixBase<MxIN>& a,
            const Eigen::MatrixBase<MxOut>& result,
            const double& tolerance)
{
    Eigen::JacobiSVD<MatrixXR> svd;

    svd.compute(a, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // Build a diagonal matrix with the Inverted Singular values
    // The pseudo inverted singular matrix is easy to compute :
    // is formed by replacing every nonzero entry by its reciprocal (inversing).
    Eigen::Matrix<double, Eigen::Dynamic, 1> singularValuesVector (svd.matrixV().cols(), 1); // size of E has same size as columns of V

    singularValuesVector = (svd.singularValues().array() > tolerance).select(svd.singularValues().array().inverse(), 0);

    // Pseudo-Inversion : V * S * U'
    const_cast< Eigen::MatrixBase<MxOut>& >(result) = svd.matrixV() *  singularValuesVector.asDiagonal() * svd.matrixU().transpose();
}

template <typename MxIN, typename MxOut>
 /**
 * @brief psdInv same as psdInv(const Eigen::MatrixBase<MxIN>& a, const Eigen::MatrixBase<MxOut>& result,const double& tolerance)
 * but with the tolerance of:
 * \f[
 * t = \varepsilon \cdot \max(m,n) \cdot \max(\Sigma)
 * \f]
 * where \f$\varepsilon\f$ is the machine epsilon.
 * @param[in] a matrix to be inverted
 * @param[out] result pseudoinverted matrix
 * @remark the second parameter is not really const.
 * It gets modified (it contains the result)
 * see http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
 * @sa psdInv(const Eigen::MatrixBase<MxIN>& a, const Eigen::MatrixBase<MxOut>& result, const double& tolerance)
 */
inline void psdInv(const Eigen::MatrixBase<MxIN>& a,
                   const Eigen::MatrixBase<MxOut>& result)
 {

    double epsilon = Eigen::NumTraits<double>::epsilon();

    Eigen::JacobiSVD<MatrixXR> svd;
    svd.compute(a, Eigen::ComputeThinU | Eigen::ComputeThinV);

    double tolerance = epsilon * std::max((int)a.cols(), (int)a.rows()) *
                       svd.singularValues().array().abs().maxCoeff();

    // Build a diagonal matrix with the Inverted Singular values
    // The pseudo inverted singular matrix is easy to compute :
    // is formed by replacing every nonzero entry by its reciprocal (inversing).
    Eigen::Matrix<double, Eigen::Dynamic, 1> singularValuesVector (svd.matrixV().cols(), 1); // size of E has same size as columns of V

    singularValuesVector = (svd.singularValues().array() > tolerance).select(svd.singularValues().array().inverse(), 0);

    // Pseudo-Inversion : V * S * U'
    const_cast< Eigen::MatrixBase<MxOut>& >(result) = svd.matrixV() *  singularValuesVector.asDiagonal() * svd.matrixU().transpose();
}



/**
 * @brief same as psdInv(const Eigen::MatrixBase<MxIN>& a, const Eigen::MatrixBase<MxOut>& result,const double& tolerance)
 *  with returned output, with input and output of type MatrixXR
 * @param in[in] a matrix of type MatrixXR, which is an Eigen matrix of variable
 * size and double values
 * @param tolerance[in] is the smallest eigen-value you tolerate for the SVD.
 * In practice, if the eigen-value is smaller than tolerance, it is considered
 * as zero.
 * @return the pseudo-inverse of \p in
 */
 inline MatrixXR psdInv(const MatrixXR & in,
                        const double& tolerance)
{

     MatrixXR output;
     psdInv(in,output,tolerance);
     return output;
}

 /**
  * @brief same as psdInv(const Eigen::MatrixBase<MxIN>& a,
  *                              const Eigen::MatrixBase<MxOut>& result)
  *  with returned output, with input and output of type MatrixXR
  * @param in[in]
  * @return the pseudoinverse of in
  */
 inline MatrixXR psdInv(const MatrixXR& in) {
     MatrixXR output;
     psdInv(in,output);
     return output;
 }

/**
 * @brief psdInvW weighted version of psdInv(const MatrixXR & in, const double& tolerance)
 * @param[in] in
 * @param[in] W weighting matrix used during the SVD process
 * @param[in] tolerance
 * @return the pseudo inverse if the function succeeds, an empty matrix otherwise
 */
inline MatrixXR psdInvW(const MatrixXR & in,
                        const MatrixXR & W,
                        const double& tolerance)
{
    MatrixXR out(in.cols(), in.rows());
    out.setZero();

    if (W.rows() != W.cols())
    {
        std::cerr << "ERROR: weighting matrix is not square!! returning not weighted psdinv" << std::endl;
        return psdInv(in, tolerance);
    }

    if (in.cols() >= in.rows())  //FAT matrix
    {
        if  (W.rows() != in.cols())
        {
            std::cerr << "ERROR: weighting matrix has the wrong size!! returning not weighted psdinv " << std::endl;
            return psdInv(in, tolerance);
        }
        //fat  A#w = W^-1*A^T (A W^-1 A^T)^{#} this works also for rank deficient matrix
        out = W.inverse() * in.transpose() * psdInv(in * W.inverse() * in.transpose(), tolerance);
    }  else { //SKINNY matrix
        if  (W.rows() != in.rows())
        {
            std::cerr << "ERROR: weighting matrix has the wrong size!! returning not weighted psdinv" << std::endl;
            return psdInv(in, tolerance);
        }
        //skinny A#w =  (A^T W^-1 A)^{#}A^T*W^-1
        out = psdInv(in.transpose() * W.inverse() * in, tolerance) * in.transpose() * W.inverse();
    }
    return out;
}

/**
 * @brief skew_sim computes the skew symmetric matrix (a.k.a. the cross-product
 * matrix) \f$[\mathbf{v}]_\times\f$  of the vector
 *  \f$\mathbf{v} = (v_1,v_2,v_3)\f$:
 * \f[
 * [\mathbf{v}]_\times= \begin{bmatrix}
 *                       0   & -v_3 & v_2 \\
 *                       v_3 &  0   & v_1 \\
 *                      -v_2 &  v_1 & 0
 *                      \end{bmatrix}
 * \f]
 * @param[in] v vector to compute the skew symmetic matrix from
 * @return the skew symmetric matrix of v
 */
Eigen::Matrix3d inline  skew_sim(const Eigen::Vector3d& v) {

    Eigen::Matrix3d S = Eigen::Matrix3d::Zero();

    S(0,1) = -v(2);
    S(0,2) =  v(1);
    S(1,2) = -v(0);

    S(1,0) =  v(2);
    S(2,0) = -v(1);
    S(2,1) =  v(0);

    return S;
}

/**
 * @brief computes the vector \f$\mathbf{v}\f$ from its cross-product matrix
 * \f$[\mathbf{v}]_\times\f$
 * @param R cross-product matrix (or skew-symmetric matrix) \f$[\mathbf{v}]_\times\f$
 * @return the vector \f$\mathbf{v}\f$
 * @sa skew_sim()
 */
Eigen::Vector3d inline  skew_simToVec(const Eigen::Matrix3d& R) {

    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    if (R.rows() != 3){
        std::cerr << "ERROR: skew matrix must be 3 x 3! " << std::endl;
    } else {
        v(0) = 0.5 *  (R(2,1) - R(1,2));
        v(1) = 0.5 *  (R(0,2) - R(2,0));
        v(2) = 0.5 *  (R(1,0) - R(0,1));
    }
    return v;
}

/**
 * @brief computes the QR decomposition of a matrix \f$A\f$ into an
 * orthonormal matrix \f$Q\f$ and a upper triangular matrix \f$R\f$, such that
 * \f$ A = QR\f$.
 * @param[in] A
 * @param[out] Q
 * @param[out] R
 * @param[in] tolerance
 */
inline void computeQR(const MatrixXR & A,
                      MatrixXR & Q,
                      MatrixXR & R,
                      const double& tolerance)
{
    // FIXME why tolerance is not used??

    //compute the QR decomposition of the Jacobian
    //Q = U ; R = EPS*V^T
    Eigen::JacobiSVD<MatrixXR> svd;

    svd.compute(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Q.resize(A.rows(),A.rows());
    Q = svd.matrixU();

    R.resize(A.cols(),A.cols());

    MatrixXR S(A.cols(),A.cols());
    S = svd.singularValues().segment(0,A.cols()).asDiagonal();

    R = S * svd.matrixV().transpose();
}

} // namespace commons
} // namespace pronto
