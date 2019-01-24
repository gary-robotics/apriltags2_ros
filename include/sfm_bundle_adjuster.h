#ifndef bundle_adjust_h_
#define bundle_adjust_h_

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace boost::math;

// This file is adapted from `libmv_bundle_adjuster.cc`
namespace bundle_adjustment
{   
    // The intrinsics need to get combined into a single parameter block;
    // use these enums to index instead of numeric constants.
    enum {
        OFFSET_FOCAL_LENGTH_X = 0,
        OFFSET_FOCAL_LENGTH_Y,
        OFFSET_PRINCIPAL_POINT_X,
        OFFSET_PRINCIPAL_POINT_Y,
        OFFSET_K1,
        OFFSET_K2,
        OFFSET_K3,
        OFFSET_P1,
        OFFSET_P2,
    };
    
    template <typename T>
    inline void apply_radio_distortion_camera_intrinsics(const T &focal_length_x,
                                                         const T &focal_length_y,
                                                         const T &principal_point_x,
                                                         const T &principal_point_y,
                                                         const T &k1,
                                                         const T &k2,
                                                         const T &k3,
                                                         const T &p1,
                                                         const T &p2,
                                                         const T &normalized_x,
                                                         const T &normalized_y,
                                                         T *image_x,
                                                         T *image_y)
    {
        T x = normalized_x;
        T y = normalized_y;
        
        // apply distortion to the normalized points to get (xd, yd)
        T r2 = x*x + y*y;
        T r4 = r2 * r2;
        T r6 = r4 * r2;
        T r_coeff = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
        T xd = x * r_coeff + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        T yd = y * r_coeff + 2.0 * p2 * x * y + p1 * (r2 + 2.0 * y * y);
        
        // apply focal length and principal point to get the final image coordinates
        *image_x = focal_length_x * xd + principal_point_x;
        *image_y = focal_length_y * yd + principal_point_y;
    }
    
    struct Open3DCVReprojectionError
    {
        // Open3DCVReprojectionError(const double observed_x, const double observed_y)
        //     : observed_x_(observed_x), observed_y_(observed_y) {}
        
        Open3DCVReprojectionError()
           {}
        template<typename T>
        bool operator()(const T* const intrinsics,
                        const T* const extrinsics,
                        const T* const point,
                        const T* const img,
                        T* residules) const
        {
            const T& focal_length_x       = intrinsics[OFFSET_FOCAL_LENGTH_X];
            const T& focal_length_y       = intrinsics[OFFSET_FOCAL_LENGTH_Y];
            const T& principal_point_x  = intrinsics[OFFSET_PRINCIPAL_POINT_X];
            const T& principal_point_y  = intrinsics[OFFSET_PRINCIPAL_POINT_Y];
            const T& k1                 = intrinsics[OFFSET_K1];
            const T& k2                 = intrinsics[OFFSET_K2];
            const T& k3                 = intrinsics[OFFSET_K3];
            const T& p1                 = intrinsics[OFFSET_P1];
            const T& p2                 = intrinsics[OFFSET_P2];
            
            // compute projective coordinates: x = RX + t.
            // extrinsics[0, 1, 2]: axis-angle
            // extrinsics[3, 4, 5]: translation
            T x[3];
            ceres::AngleAxisRotatePoint(extrinsics, point, x);
            x[0] += extrinsics[3];
            x[1] += extrinsics[4];
            x[2] += extrinsics[5];
            
            // compute normalized coordinates
            T xn = x[0] / x[2];
            T yn = x[1] / x[2];
            
            T predicted_x, predicted_y;
            
            // apply distortion to the normalized points to get (xd, yd)
            // do something for zero distortion
            apply_radio_distortion_camera_intrinsics(focal_length_x,
                                                     focal_length_y,
                                                     principal_point_x,
                                                     principal_point_y,
                                                     k1, k2, k3,
                                                     p1, p2,
                                                     xn, yn,
                                                     &predicted_x,
                                                     &predicted_y);
            
            // residules[0] = predicted_x - T(observed_x_);
            // residules[1] = predicted_y - T(observed_y_);
            residules[0] = predicted_x - img[0];
            residules[1] = predicted_y - img[1];
            return true;
        }
        
        // Factory to hide the construction of the CostFunction object from the client code
        static ceres::CostFunction* create( )
        {
            return (new ceres::AutoDiffCostFunction<Open3DCVReprojectionError, 2, 9, 6, 3, 2>(
                        new Open3DCVReprojectionError( )));
        }
        // static ceres::CostFunction* create(const double observed_x,
        //                                    const double observed_y)
        // {
        //     return (new ceres::AutoDiffCostFunction<Open3DCVReprojectionError, 2, 9, 6, 3>(
        //                 new Open3DCVReprojectionError(observed_x, observed_y)));
        // }
        // double observed_x_;
        // double observed_y_;
    };

    struct PlanePoseSimilarityError
    {
        PlanePoseSimilarityError()
        {}
        template<typename T>
        bool operator()(const T* const intrinsics_pre,
                        const T* const extrinsics_pre,
                        const T* const img_pre,
                        const T* const intrinsics_cur,
                        const T* const extrinsics_cur,
                        const T* const img_cur,
                        T* residules) const
        {
            const T& focal_length_x_pre       = intrinsics_pre[OFFSET_FOCAL_LENGTH_X];
            const T& focal_length_y_pre       = intrinsics_pre[OFFSET_FOCAL_LENGTH_Y];
            const T& principal_point_x_pre  = intrinsics_pre[OFFSET_PRINCIPAL_POINT_X];
            const T& principal_point_y_pre  = intrinsics_pre[OFFSET_PRINCIPAL_POINT_Y];
            const T& k1_pre                 = intrinsics_pre[OFFSET_K1];
            const T& k2_pre                 = intrinsics_pre[OFFSET_K2];
            const T& k3_pre                 = intrinsics_pre[OFFSET_K3];
            const T& p1_pre                 = intrinsics_pre[OFFSET_P1];
            const T& p2_pre                 = intrinsics_pre[OFFSET_P2];

            const T& focal_length_x_cur     = intrinsics_cur[OFFSET_FOCAL_LENGTH_X];
            const T& focal_length_y_cur     = intrinsics_cur[OFFSET_FOCAL_LENGTH_Y];
            const T& principal_point_x_cur  = intrinsics_cur[OFFSET_PRINCIPAL_POINT_X];
            const T& principal_point_y_cur  = intrinsics_cur[OFFSET_PRINCIPAL_POINT_Y];
            const T& k1_cur                 = intrinsics_cur[OFFSET_K1];
            const T& k2_cur                 = intrinsics_cur[OFFSET_K2];
            const T& k3_cur                 = intrinsics_cur[OFFSET_K3];
            const T& p1_cur                 = intrinsics_cur[OFFSET_P1];
            const T& p2_cur                 = intrinsics_cur[OFFSET_P2];

            std::cout << "Into PlanePoseSimilarityError !" << std::endl;
            Eigen::Matrix<T, 3, 3> camera_matrix_pre, camera_matrix_cur;
            camera_matrix_pre(0,0) = focal_length_x_pre;
            camera_matrix_pre(0,1) = T(0);
            camera_matrix_pre(0,2) = principal_point_x_pre;
            camera_matrix_pre(1,0) = T(0);
            camera_matrix_pre(1,1) = focal_length_y_pre;
            camera_matrix_pre(1,2) = principal_point_y_pre;
            camera_matrix_pre(2,0) = T(0);
            camera_matrix_pre(2,1) = T(0);
            camera_matrix_pre(2,2) = T(1);

            camera_matrix_cur(0,0) = focal_length_x_cur;
            camera_matrix_cur(0,1) = T(0);
            camera_matrix_cur(0,2) = principal_point_x_cur;
            camera_matrix_cur(1,0) = T(0);
            camera_matrix_cur(1,1) = focal_length_y_cur;
            camera_matrix_cur(1,2) = principal_point_y_cur;
            camera_matrix_cur(2,0) = T(0);
            camera_matrix_cur(2,1) = T(0);
            camera_matrix_cur(2,2) = T(1);

            Eigen::Matrix<T, 3, 1> rvec_pre, tvec_pre;
            rvec_pre(0,0) = extrinsics_pre[0];
            rvec_pre(1,0) = extrinsics_pre[1];
            rvec_pre(2,0) = extrinsics_pre[2];
            tvec_pre(0,0) = extrinsics_pre[3];
            tvec_pre(1,0) = extrinsics_pre[4];
            tvec_pre(2,0) = extrinsics_pre[5];

            T norm = rvec_pre.norm();
            rvec_pre = rvec_pre / norm;
            Eigen::AngleAxis<T> angleaxis1(norm, rvec_pre);
            Eigen::Quaternion<T> q1(angleaxis1);
            Eigen::Matrix<T, 3, 3> rotation_pre = q1.toRotationMatrix();
            Eigen::Matrix<T, 3, 1> tvec_pre_inv = -rotation_pre.inverse() * tvec_pre;

            Eigen::Matrix<T, 3, 1> img_pt_pre;
            img_pt_pre(0,0) = img_pre[0];
            img_pt_pre(1,0) = img_pre[1];
            img_pt_pre(2,0) = img_pre[2];
            Eigen::Matrix<T, 3, 1> wl_pt_pre;

            std::cout << "computer pre point at world coordinate!" << std::endl;
            wl_pt_pre = rotation_pre.inverse()*(camera_matrix_pre.inverse()*img_pt_pre) + tvec_pre_inv;
            std::cout << "computer pre point at world coordinate ended!" << std::endl;

            Eigen::Matrix<T, 3, 1> rvec_cur, tvec_cur;
            rvec_cur(0,0) = extrinsics_cur[0];
            rvec_cur(1,0) = extrinsics_cur[1];
            rvec_cur(2,0) = extrinsics_cur[2];
            tvec_cur(0,0) = extrinsics_cur[3];
            tvec_cur(1,0) = extrinsics_cur[4];
            tvec_cur(2,0) = extrinsics_cur[5];
            norm = rvec_cur.norm();
            rvec_cur = rvec_cur / norm;
            Eigen::AngleAxis<T> angleaxis2(norm, rvec_cur);
            Eigen::Quaternion<T> q2(angleaxis2);
            Eigen::Matrix<T, 3, 3> rotation_cur = q2.toRotationMatrix();
            Eigen::Matrix<T, 3, 1> tvec_cur_inv = -rotation_cur.inverse() * tvec_cur;

            Eigen::Matrix<T, 3, 1> img_pt_cur;
            img_pt_cur(0,0) = img_cur[0];
            img_pt_cur(1,0) = img_cur[1];
            img_pt_cur(2,0) = img_cur[2];
            Eigen::Matrix<T, 3, 1> wl_pt_cur;
            wl_pt_cur = rotation_cur.inverse()*(camera_matrix_cur.inverse()*img_pt_cur) + tvec_cur_inv;

            residules[0] = wl_pt_cur(0, 0) - wl_pt_pre(0, 0);
            residules[1] = wl_pt_cur(1, 0) - wl_pt_pre(1, 0);
            residules[2] = wl_pt_cur(2, 0) - wl_pt_pre(2, 0);

            return true;
        }
        
        // Factory to hide the construction of the CostFunction object from the client code
        static ceres::CostFunction* create()
        {
            return (new ceres::AutoDiffCostFunction<PlanePoseSimilarityError, 3, 9, 6, 3, 9, 6, 3>(
                        new PlanePoseSimilarityError( )));
        }
    };

    struct PoseSimilarityError
    {
        PoseSimilarityError(const double distance_z) : distance_z_ ( distance_z )
        {}
        template<typename T>
        bool operator()(const T* const intrinsics_pre,
                        const T* const extrinsics_pre,
                        const T* const point_pre,
                        const T* const img_pre,
                        const T* const intrinsics_cur,
                        const T* const extrinsics_cur,
                        const T* const point_cur,
                        const T* const img_cur,
                        const T* const homography,
                        T* residules) const
        {
            const T& focal_length_x_pre       = intrinsics_pre[OFFSET_FOCAL_LENGTH_X];
            const T& focal_length_y_pre       = intrinsics_pre[OFFSET_FOCAL_LENGTH_Y];
            const T& principal_point_x_pre  = intrinsics_pre[OFFSET_PRINCIPAL_POINT_X];
            const T& principal_point_y_pre  = intrinsics_pre[OFFSET_PRINCIPAL_POINT_Y];
            const T& k1_pre                 = intrinsics_pre[OFFSET_K1];
            const T& k2_pre                 = intrinsics_pre[OFFSET_K2];
            const T& k3_pre                 = intrinsics_pre[OFFSET_K3];
            const T& p1_pre                 = intrinsics_pre[OFFSET_P1];
            const T& p2_pre                 = intrinsics_pre[OFFSET_P2];

            const T& focal_length_x_cur     = intrinsics_cur[OFFSET_FOCAL_LENGTH_X];
            const T& focal_length_y_cur     = intrinsics_cur[OFFSET_FOCAL_LENGTH_Y];
            const T& principal_point_x_cur  = intrinsics_cur[OFFSET_PRINCIPAL_POINT_X];
            const T& principal_point_y_cur  = intrinsics_cur[OFFSET_PRINCIPAL_POINT_Y];
            const T& k1_cur                 = intrinsics_cur[OFFSET_K1];
            const T& k2_cur                 = intrinsics_cur[OFFSET_K2];
            const T& k3_cur                 = intrinsics_cur[OFFSET_K3];
            const T& p1_cur                 = intrinsics_cur[OFFSET_P1];
            const T& p2_cur                 = intrinsics_cur[OFFSET_P2];
            
            // compute projective coordinates: x = RX + t.
            // extrinsics[0, 1, 2]: axis-angle
            // extrinsics[3, 4, 5]: translation
            T x[3];
            ceres::AngleAxisRotatePoint(extrinsics_pre, point_pre, x);
            x[0] += extrinsics_pre[3];
            x[1] += extrinsics_pre[4];
            x[2] += extrinsics_pre[5];
            
            // compute normalized coordinates
            T xn = x[0] / x[2];
            T yn = x[1] / x[2];
            
            T predicted_x_pre, predicted_y_pre;
            
            // apply distortion to the normalized points to get (xd, yd)
            // do something for zero distortion
            apply_radio_distortion_camera_intrinsics(focal_length_x_pre,
                                                     focal_length_y_pre,
                                                     principal_point_x_pre,
                                                     principal_point_y_pre,
                                                     k1_pre, k2_pre, k3_pre,
                                                     p1_pre, p2_pre,
                                                     xn, yn,
                                                     &predicted_x_pre,
                                                     &predicted_y_pre);
            
            residules[0] = predicted_x_pre - img_pre[0];
            residules[1] = predicted_y_pre - img_pre[1];
            // std::cout << "residules: " << residules[0] << "," << residules[1] << std::endl;

            ceres::AngleAxisRotatePoint(extrinsics_cur, point_cur, x);
            x[0] += extrinsics_cur[3];
            x[1] += extrinsics_cur[4];
            x[2] += extrinsics_cur[5];
            
            // compute normalized coordinates
            xn = x[0] / x[2];
            yn = x[1] / x[2];
            
            // apply distortion to the normalized points to get (xd, yd)
            // do something for zero distortion

            T predicted_x_cur, predicted_y_cur;

            apply_radio_distortion_camera_intrinsics(focal_length_x_cur,
                                                     focal_length_y_cur,
                                                     principal_point_x_cur,
                                                     principal_point_y_cur,
                                                     k1_cur, k2_cur, k3_cur,
                                                     p1_cur, p2_cur,
                                                     xn, yn,
                                                     &predicted_x_cur,
                                                     &predicted_y_cur);

            residules[2] = predicted_x_cur - img_cur[0];
            residules[3] = predicted_y_cur - img_cur[1];

            Eigen::Matrix<T,3,1> rvec1, tvec1;
            rvec1(0,0) = extrinsics_pre[0];
            rvec1(1,0) = extrinsics_pre[1];
            rvec1(2,0) = extrinsics_pre[2];
            tvec1(0,0) = extrinsics_pre[3];
            tvec1(1,0) = extrinsics_pre[4];
            tvec1(2,0) = extrinsics_pre[5];
            T norm = rvec1.norm();
            rvec1 = rvec1 / norm;
            Eigen::AngleAxis<T> angleaxis1(norm, rvec1); 
            Eigen::Quaternion<T> q1(angleaxis1);
            Eigen::Matrix<T, 3, 3> rotation1 = q1.toRotationMatrix();
            Eigen::Matrix<T, 3, 1> tvec1_inv = -rotation1.inverse() * tvec1;

            Eigen::Matrix<T, 3, 1> rvec2, tvec2;
            rvec2(0,0) = extrinsics_cur[0];
            rvec2(1,0) = extrinsics_cur[1];
            rvec2(2,0) = extrinsics_cur[2];
            tvec2(0,0) = extrinsics_cur[3];
            tvec2(1,0) = extrinsics_cur[4];
            tvec2(2,0) = extrinsics_cur[5];
            norm = rvec2.norm();
            rvec2 = rvec2 / norm;
            Eigen::AngleAxis<T> angleaxis2(norm, rvec2); 
            Eigen::Quaternion<T> q2(angleaxis2);
            Eigen::Matrix<T, 3, 3> rotation2 = q2.toRotationMatrix();
            Eigen::Matrix<T, 3, 1> tvec2_inv = -rotation2.inverse() * tvec2;
            residules[4] = tvec1_inv(0, 1) - tvec2_inv(0, 1);
            residules[5] = tvec1_inv(0, 2) - tvec2_inv(0, 2);

            T predicted_homo_x_cur, predicted_homo_y_cur;
            predicted_homo_x_cur = homography[0] * predicted_x_pre + homography[1] * predicted_y_pre + homography[2];
            predicted_homo_y_cur = homography[3] * predicted_x_pre + homography[4] * predicted_y_pre + homography[5];
            
            residules[6] = predicted_homo_x_cur - predicted_x_cur;
            residules[7] = predicted_homo_y_cur - predicted_y_cur;

            // residules[8] = q1.angularDistance(q2) * T(100);
            // residules[9] = tvec1_inv(0, 2) - T(distance_z_);
            // residules[10] = tvec2_inv(0, 2) - T(distance_z_);
            return true;
        }
        
        // Factory to hide the construction of the CostFunction object from the client code
        static ceres::CostFunction* create( const double distance_z )
        {
            return (new ceres::AutoDiffCostFunction<PoseSimilarityError, 8, 9, 6, 3, 2, 9, 6, 3, 2, 9>(
                        new PoseSimilarityError( distance_z )));
        }

        double distance_z_;
    };
    
    void Open3DCVBundleAdjustment(std::vector<std::vector<double> >& extrinsics,
                                  std::vector<std::vector<double> >& intrinsics,
                                  std::vector<std::vector<double> >& pts3d,
                                  std::vector<std::vector<double> >& impts2d,
                                  int frame_num,
                                  const int bundle_intrinsics)
    {
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);
        
        // construct the problem
        bool is_camera_locked = true;

//        if( pts3d.size() == impts2d.size() == (frame_num * 4) == extrinsics.size() == intrinsics.size()){
//            std::cout << "size same!" << std::endl;;
//        }
//        else{
//            std::cout << "size not same!" << std::endl;
//            return;
//        }

        for (int frame_iter = 0; frame_iter < frame_num; ++frame_iter)
        {
            for (int m = 0; m < 4; ++m)
            {
                // double x = impt2d[0];
                // double y = impt2d[1];
                // ceres::CostFunction *cost_function = Open3DCVReprojectionError::create(x, y);
                // problem.AddResidualBlock(cost_function, NULL, &intrinsics[frame_iter][0], &extrinsics[frame_iter][0], &pts3d[m + frame_iter*4][0]);

                ceres::CostFunction *cost_function = Open3DCVReprojectionError::create( );
                problem.AddResidualBlock(cost_function, NULL, &intrinsics[frame_iter][0], &extrinsics[frame_iter][0], 
                                        &pts3d[m + frame_iter * 4][0], &impts2d[m + frame_iter * 4][0]);
                
                if (!is_camera_locked)
                {
                    problem.SetParameterBlockConstant(&extrinsics[frame_iter][0]);
                    is_camera_locked = true;
                }
            }
        }

        // set part of parameters constant
        if (bundle_intrinsics == 0)
        {
            for (std::size_t i = 0; i < intrinsics.size(); ++i)
                problem.SetParameterBlockConstant(&intrinsics[i][0]);
        }
        else
        {
            std::vector<int> constant_intrinsics;

#define MAYBE_SET_CONSTANT(bundle_enum, offset) \
            if (!(bundle_intrinsics & bundle_enum)) { \
                constant_intrinsics.push_back(offset); \
            }
#undef MAYBE_SET_CONSTANT

            // always set K3 constant, it's not used at the moment
            constant_intrinsics.push_back(OFFSET_FOCAL_LENGTH_X);
            constant_intrinsics.push_back(OFFSET_FOCAL_LENGTH_Y);
            constant_intrinsics.push_back(OFFSET_PRINCIPAL_POINT_X);
            constant_intrinsics.push_back(OFFSET_PRINCIPAL_POINT_Y);
            constant_intrinsics.push_back(OFFSET_K1);
            constant_intrinsics.push_back(OFFSET_K2);
            constant_intrinsics.push_back(OFFSET_K3);
            constant_intrinsics.push_back(OFFSET_P1);
            constant_intrinsics.push_back(OFFSET_P2);
            ceres::SubsetParameterization *subset_intrinsics_parameterizaiton =
                new ceres::SubsetParameterization(9, constant_intrinsics);

            for (std::size_t i = 0; i < intrinsics.size(); ++i)
            {
                problem.SetParameterization(&intrinsics[i][0], subset_intrinsics_parameterizaiton);
            }
        }


        for (std::size_t i = 0; i < pts3d.size(); ++i)
        {
            problem.SetParameterBlockConstant(&pts3d[i][0]);
        }

        // for (std::size_t i = 0; i < impts2d.size(); ++i)
        // {
        //     problem.SetParameterUpperBound(&impts2d[i][0], 0, impts2d[i][0]+0.5);
        //     problem.SetParameterLowerBound(&impts2d[i][0], 0, impts2d[i][0]-0.5);
        //     problem.SetParameterUpperBound(&impts2d[i][0], 1, impts2d[i][1]+0.2);
        //     problem.SetParameterLowerBound(&impts2d[i][0], 1, impts2d[i][1]-0.2);
        // }

        // extrinsics[0, 1, 2]: axis-angle
        // extrinsics[3, 4, 5]: translation
        std::vector<int> constant_extrinsics;
        constant_extrinsics.push_back(0);
        constant_extrinsics.push_back(1);
        constant_extrinsics.push_back(2);
        //constant_extrinsics.push_back(5);
        ceres::SubsetParameterization *subset_extrinsics_parameterizaiton =
                new ceres::SubsetParameterization(6, constant_extrinsics);
        for (std::size_t i = 0; i < extrinsics.size(); ++i)
        {
            problem.SetParameterization(&extrinsics[i][0], subset_extrinsics_parameterizaiton);
        }

        // configure the solver
        ceres::Solver::Options options;
        options.use_nonmonotonic_steps = true;
        options.preconditioner_type = ceres::SCHUR_JACOBI;
        options.linear_solver_type = ceres::ITERATIVE_SCHUR; //DENSE_SCHUR;
        options.use_inner_iterations = true;
        options.max_num_iterations = 100;
        options.minimizer_progress_to_stdout = false;

        // solve
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        // std::cout << summary.FullReport() << std::endl; //BriefReport
        // std::cout << "optimised rotation and translation:";
        // for (int i = 0; i < 6; i++){
        //     std::cout << extrinsics[frame_num-1][i];
        //     std::cout << ",";
        // }
        // std::cout << std::endl;
    }

    void PlanePoseEstimation(std::vector<std::vector<double> >& extrinsics, std::vector<std::vector<double> >& intrinsics,
                             std::vector<std::vector<double> >& matched_impts2d_ref, std::vector<std::vector<double> >& matched_impts2d_curr)
    {

        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);

        // construct the problem
        bool is_camera_locked = true;

        int pt_num = matched_impts2d_ref.size();
        std::cout << "pt num: " << pt_num << std::endl;
        assert(matched_impts2d_ref.size() == matched_impts2d_curr.size());

        for (int m = 0; m < 4; ++m)
        {
            std::cout << "pre_point: " << matched_impts2d_ref[m][0] << "," << matched_impts2d_ref[m][1] << "," << matched_impts2d_ref[m][2] << std::endl;
            std::cout << "cur_point: " << matched_impts2d_curr[m][0] << "," << matched_impts2d_curr[m][1] << "," << matched_impts2d_curr[m][2] << std::endl;
            ceres::CostFunction *cost_function = PlanePoseSimilarityError::create();
            problem.AddResidualBlock(cost_function, NULL, &intrinsics[0][0], &extrinsics[0][0], &matched_impts2d_ref[m][0], 
                                     &intrinsics[1][0], &extrinsics[1][0], &matched_impts2d_curr[m][0]);
        }

        // set part of parameters constant
        for (std::size_t i = 0; i < intrinsics.size(); ++i)
        {
            problem.SetParameterBlockConstant(&intrinsics[i][0]);
        }

        for (std::size_t i = 0; i < 4; ++i)
        {
            problem.SetParameterBlockConstant(&matched_impts2d_ref[i][0]);
        }

        for (std::size_t i = 0; i < 4; ++i)
        {
            problem.SetParameterBlockConstant(&matched_impts2d_curr[i][0]);
        }

        std::cout << "finished adding residualblock !" << std::endl;

        // configure the solver
        ceres::Solver::Options options;
        options.use_nonmonotonic_steps = true;
        options.preconditioner_type = ceres::SCHUR_JACOBI;
        options.linear_solver_type = ceres::ITERATIVE_SCHUR; //DENSE_SCHUR;
        options.use_inner_iterations = true;
        options.max_num_iterations = 200;
        options.minimizer_progress_to_stdout = false;

        std::cout << "beging solver !" << std::endl;
        // solve
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << std::endl; //BriefReport
    }

    void PoseEstimation(std::vector<std::vector<double> >& extrinsics,
                                  std::vector<std::vector<double> >& intrinsics,
                                  std::vector<std::vector<double> >& pts3d,
                                  std::vector<std::vector<double> >& impts2d,
                                  std::vector<double >& homography,
                                  double distance_z)
    {
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);
        
        // construct the problem
        bool is_camera_locked = true;

        for (int m = 0; m < 4; ++m)
        {
            ceres::CostFunction *cost_function = PoseSimilarityError::create( distance_z );
            problem.AddResidualBlock(cost_function, NULL, &intrinsics[0][0], &extrinsics[0][0],
                                     &pts3d[m][0], &impts2d[m][0], &intrinsics[1][0], &extrinsics[1][0],
                                     &pts3d[m+4][0], &impts2d[m+4][0], &homography[0]);
        }

        // set part of parameters constant
        for (std::size_t i = 0; i < intrinsics.size(); ++i)
        {
            problem.SetParameterBlockConstant(&intrinsics[i][0]);
        }

        for (std::size_t i = 0; i < pts3d.size(); ++i)
        {
            problem.SetParameterBlockConstant(&pts3d[i][0]);
        }

        // for (std::size_t i = 0; i < impts2d.size(); ++i)
        // {
        //     problem.SetParameterBlockConstant(&impts2d[i][0]);
        // }

        problem.SetParameterBlockConstant(&homography[0]);

        // // extrinsics[0, 1, 2]: axis-angle
        // // extrinsics[3, 4, 5]: translation
        // std::vector<int> constant_extrinsics;
        // constant_extrinsics.push_back(0);
        // constant_extrinsics.push_back(1);
        // constant_extrinsics.push_back(2);
        // //constant_extrinsics.push_back(5);
        // ceres::SubsetParameterization *subset_extrinsics_parameterizaiton =
        //         new ceres::SubsetParameterization(6, constant_extrinsics);
        // for (std::size_t i = 0; i < extrinsics.size(); ++i)
        // {
        //     problem.SetParameterization(&extrinsics[i][0], subset_extrinsics_parameterizaiton);
        // }

        // configure the solver
        ceres::Solver::Options options;
        options.use_nonmonotonic_steps = true;
        options.preconditioner_type = ceres::SCHUR_JACOBI;
        options.linear_solver_type = ceres::ITERATIVE_SCHUR; //DENSE_SCHUR;
        options.use_inner_iterations = true;
        options.max_num_iterations = 200;
        options.minimizer_progress_to_stdout = false;

        // solve
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.FullReport() << std::endl; //BriefReport
    }

    void distanceAverage(std::vector<std::vector<double>> &extrinsics,
                          cv::Vec3d &disaverage)
    {
        disaverage(0) = 0.0;
        disaverage(1) = 0.0;
        disaverage(2) = 0.0;
        int i = 0;
        for (i = 0; i < extrinsics.size(); ++i)
        {
            cv::Vec3d rvec;
            Eigen::Vector3d translation;
            rvec(0) = extrinsics[i][0];
            rvec(1) = extrinsics[i][1];
            rvec(2) = extrinsics[i][2];
            translation(0) = extrinsics[i][3];
            translation(1) = extrinsics[i][4];
            translation(2) = extrinsics[i][5];
            cv::Matx33d R;
            cv::Rodrigues(rvec, R);

            Eigen::Matrix3d wRo;
            wRo << R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2);
            Eigen::Matrix3d rot_inv = wRo.transpose();

            Eigen::Vector3d translation_inv = -1. * rot_inv * translation;
            disaverage(0) = disaverage(0) + translation_inv(0);
            disaverage(1) = disaverage(1) + translation_inv(1);
            disaverage(2) = disaverage(2) + translation_inv(2);
        }
        if (i == 0) return;
        disaverage(0) = disaverage(0) / i;
        disaverage(1) = disaverage(1) / i;
        disaverage(2) = disaverage(2) / i;
        return;
    }

    void QuaternionAverage(std::vector<std::vector<double>> &extrinsics,
                           double weights,
                           cv::Vec3d &rvec_avg)
    {
        double wSum = 0.0;
        cv::Mat M(4, 4, CV_64FC1, 0.0);

        for (size_t i = 0; i < extrinsics.size(); ++i)
        {
            cv::Vec3d rvec_avg1;
            rvec_avg1(0) = extrinsics[i][0];
            rvec_avg1(1) = extrinsics[i][1];
            rvec_avg1(2) = extrinsics[i][2];
            cv::Matx33d R1;
            cv::Rodrigues(rvec_avg1, R1);
            Eigen::Matrix3d wRo1;
            wRo1 << R1(0, 0), R1(0, 1), R1(0, 2), R1(1, 0), R1(1, 1), R1(1, 2), R1(2, 0), R1(2, 1), R1(2, 2);
            Eigen::Quaterniond q1(wRo1);

            cv::Mat q1_vec(1, 4, CV_64FC1), q1_vec_T(4, 1, CV_64FC1);
            q1_vec.at<double>(0, 0) = q1.x();
            q1_vec.at<double>(0, 1) = q1.y();
            q1_vec.at<double>(0, 2) = q1.z();
            q1_vec.at<double>(0, 3) = q1.w();

            q1_vec_T.at<double>(0, 0) = q1.x();
            q1_vec_T.at<double>(1, 0) = q1.y();
            q1_vec_T.at<double>(2, 0) = q1.z();
            q1_vec_T.at<double>(3, 0) = q1.w();

            cv::Mat qq = weights * (q1_vec_T * q1_vec);
            M = M + qq;
            wSum = wSum + weights;
        }

        M = (1.0 / wSum) * M;
        cv::Mat eValuesMat;
        cv::Mat eVectorsMat;
        cv::eigen(M, eValuesMat, eVectorsMat);

        Eigen::Quaterniond quaternion_avg(eVectorsMat.at<double>(0, 3), eVectorsMat.at<double>(0, 0), eVectorsMat.at<double>(0, 1), eVectorsMat.at<double>(0, 2));
        Eigen::Matrix3d wRo_avg;
        wRo_avg = quaternion_avg.toRotationMatrix();

        // std::cout << "averaged quaternion: " << quaternion_avg.w() << "," << quaternion_avg.x() << "," << quaternion_avg.y() << "," << quaternion_avg.z() << std::endl;
        // std::ofstream os;
        // os.open("/home/xd/quaternion_avg.txt", std::ios::out | std::ios::app);
        // os << quaternion_avg.w() << "," << quaternion_avg.x() << "," << quaternion_avg.y() << "," << quaternion_avg.z() << std::endl;

        cv::Matx33d R_avg;
        R_avg(0, 0) = wRo_avg(0, 0);
        R_avg(0, 1) = wRo_avg(0, 1);
        R_avg(0, 2) = wRo_avg(0, 2);
        R_avg(1, 0) = wRo_avg(1, 0);
        R_avg(1, 1) = wRo_avg(1, 1);
        R_avg(1, 2) = wRo_avg(1, 2);
        R_avg(2, 0) = wRo_avg(2, 0);
        R_avg(2, 1) = wRo_avg(2, 1);
        R_avg(2, 2) = wRo_avg(2, 2);

        cv::Rodrigues(R_avg, rvec_avg);
    }

    bool QuaternionFilter(std::vector<std::vector<double>> &extrinsics, cv::Vec3d& rvec_avg, double accept_frame_num, double angular_threshold)
    {
        int num = extrinsics.size();
        if (num <= 0) {
            return false;
        }

        ////////in radian
        std::vector<int> impulse(num, 0);
        impulse[0] = 0;

        for (int i = 0; i < extrinsics.size()-1; ++i)
        {
            cv::Vec3d rvec1;    // tvec1, tvec_inv1;
            rvec1(0) = extrinsics[i][0];
            rvec1(1) = extrinsics[i][1];
            rvec1(2) = extrinsics[i][2];
            // tvec1(0) = extrinsics[i][3];
            // tvec1(1) = extrinsics[i][4];
            // tvec1(2) = extrinsics[i][5];
            cv::Matx33d R1; // R1_inv;
            cv::Rodrigues(rvec1, R1);
            // R1_inv = R1.t();
            // tvec_inv1 = -R1_inv * tvec1;
            Eigen::Matrix3d wRo1;
            wRo1 << R1(0, 0), R1(0, 1), R1(0, 2), R1(1, 0), R1(1, 1), R1(1, 2), R1(2, 0), R1(2, 1), R1(2, 2);
            Eigen::Quaterniond q1(wRo1);

            cv::Vec3d rvec2;    //, tvec2, tvec_inv2;
            rvec2(0) = extrinsics[i+1][0];
            rvec2(1) = extrinsics[i+1][1];
            rvec2(2) = extrinsics[i+1][2];
            // tvec2(0) = extrinsics[i+1][3];
            // tvec2(1) = extrinsics[i+1][4];
            // tvec2(2) = extrinsics[i+1][5];
            cv::Rodrigues(rvec2, R1);
            // R1_inv = R1.t();
            // tvec_inv2 = -R1_inv * tvec2;
            Eigen::Matrix3d wRo2;
            wRo2 << R1(0, 0), R1(0, 1), R1(0, 2), R1(1, 0), R1(1, 1), R1(1, 2), R1(2, 0), R1(2, 1), R1(2, 2);
            Eigen::Quaterniond q2(wRo2);

            double angular = q1.angularDistance(q2);
            // double dis_z = tvec1(2) - tvec2(2);
            impulse[i+1] = impulse[i];
            if (std::fabs(angular) > angular_threshold) // || fabs(dis_z) > DISTANCE_THRESHOLD)
            {
                impulse[i+1] = 1 - impulse[i+1];
            }
        }
        
        for(int i = impulse.size() - 1; i >= 1; i--)
        {
            int length = 0;
            int j;
            for(j = i-1; j >= 0; j--)
            {
                if(impulse[j] == impulse[i])
                {
                    length ++;
                    continue;
                }
                else
                {
                   break;
                }
            }
            std::cout << "stationary rotation angular length: " << length << std::endl;
            if (length >= accept_frame_num)
            {
                double wSum = 0.0;
                cv::Mat M(4, 4, CV_64FC1, 0.0);
                double weights = 1.0/(length+1);

                assert((j+1) < extrinsics.size());
                assert(i < extrinsics.size());
                for (int iter = j + 1; iter <= i; ++iter)
                {
                    cv::Vec3d rvec_avg1;
                    rvec_avg1(0) = extrinsics[iter][0];
                    rvec_avg1(1) = extrinsics[iter][1];
                    rvec_avg1(2) = extrinsics[iter][2];

                    cv::Matx33d R1;
                    cv::Rodrigues(rvec_avg1, R1);

                    Eigen::Matrix3d wRo1;
                    wRo1 << R1(0, 0), R1(0, 1), R1(0, 2), R1(1, 0), R1(1, 1), R1(1, 2), R1(2, 0), R1(2, 1), R1(2, 2);
                    Eigen::Quaterniond q1(wRo1);

                    cv::Mat q1_vec(1, 4, CV_64FC1), q1_vec_T(4, 1, CV_64FC1);
                    q1_vec.at<double>(0, 0) = q1.x();
                    q1_vec.at<double>(0, 1) = q1.y();
                    q1_vec.at<double>(0, 2) = q1.z();
                    q1_vec.at<double>(0, 3) = q1.w();

                    q1_vec_T.at<double>(0, 0) = q1.x();
                    q1_vec_T.at<double>(1, 0) = q1.y();
                    q1_vec_T.at<double>(2, 0) = q1.z();
                    q1_vec_T.at<double>(3, 0) = q1.w();

                    cv::Mat qq = weights * (q1_vec_T * q1_vec);
                    
                    M = M + qq;
                    wSum = wSum + weights;
                }
                
                M = (1.0 / wSum) * M;
                cv::Mat eValuesMat;
                cv::Mat eVectorsMat;
                if(cv::eigen(M, eValuesMat, eVectorsMat)){
                    Eigen::Quaterniond quaternion_avg(eVectorsMat.at<double>(0, 3), eVectorsMat.at<double>(0, 0), eVectorsMat.at<double>(0, 1), eVectorsMat.at<double>(0, 2));
                    Eigen::Matrix3d wRo_avg = quaternion_avg.toRotationMatrix();
                    cv::Matx33d R_avg;
                    R_avg(0, 0) = wRo_avg(0, 0);
                    R_avg(0, 1) = wRo_avg(0, 1);
                    R_avg(0, 2) = wRo_avg(0, 2);
                    R_avg(1, 0) = wRo_avg(1, 0);
                    R_avg(1, 1) = wRo_avg(1, 1);
                    R_avg(1, 2) = wRo_avg(1, 2);
                    R_avg(2, 0) = wRo_avg(2, 0);
                    R_avg(2, 1) = wRo_avg(2, 1);
                    R_avg(2, 2) = wRo_avg(2, 2);
                    cv::Rodrigues(R_avg, rvec_avg);
                    return true;
                }
            }
        }
        return false;  
    }
}

#endif
