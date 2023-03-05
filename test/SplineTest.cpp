// STL
#include <iostream>
#include <fstream>
// gtest
#include <gtest/gtest.h>
// Local
#include "csv.h"
#include "PoseSpline/QuaternionSpline.hpp"
#include "PoseSpline/QuaternionSplineUtility.hpp"
#include "PoseSpline/PoseSpline.hpp"
#include "PoseSpline/VectorSpaceSpline.hpp"
#include "PoseSpline/Time.hpp"

struct StampedPose
{
    uint64_t timestamp_;
    Eigen::Vector3d t_;    // position
    Eigen::Quaterniond q_; // angular
    Eigen::Vector3d v_;    // velocity
};

struct StampedImu
{
    uint64_t timestamp_;
    Eigen::Vector3d accel_; // acceleration
    Eigen::Vector3d gyro_;  // gyroscope
};

// class TestSample: position & imu data import
class TestSample
{
public:
    // load status e.g. gt pose
    void readStates(const std::string &states_file)
    {
        // initialize a csv reader, 11 corresponding to the colomn number
        io::CSVReader<11> in(states_file);
        // standard format to parse a csv
        in.read_header(io::ignore_extra_column, "#timestamp",
                       "p_RS_R_x [m]", "p_RS_R_y [m]", "p_RS_R_z [m]",
                       "q_RS_w []", "q_RS_x []", "q_RS_y []", "q_RS_z []",
                       "v_RS_R_x [m s^-1]", "v_RS_R_y [m s^-1]", "v_RS_R_z [m s^-1]");
        // corresponding to the ingored header line
        int64_t timestamp;
        // position
        double p_RS_R_x, p_RS_R_y, p_RS_R_z;
        // angular
        double q_RS_w, q_RS_x, q_RS_y, q_RS_z;
        // velocity
        double v_RS_R_x, v_RS_R_y, v_RS_R_z;
        int cnt = 0;
        // load each line regularly
        states_vec_.clear();
        while (in.read_row(timestamp,
                           p_RS_R_x, p_RS_R_y, p_RS_R_z,
                           q_RS_w, q_RS_x, q_RS_y, q_RS_z,
                           v_RS_R_x, v_RS_R_y, v_RS_R_z))
        {
            // instantiate pose
            StampedPose pose;
            pose.timestamp_ = timestamp;
            pose.t_ = Eigen::Vector3d(p_RS_R_x, p_RS_R_y, p_RS_R_z);
            pose.v_ = Eigen::Vector3d(v_RS_R_x, v_RS_R_y, v_RS_R_z);
            pose.q_ = Eigen::Quaterniond(q_RS_w, q_RS_x, q_RS_y, q_RS_z);
            states_vec_.push_back(pose);
            cnt++;
        }
        std::cout << "Load states: " << states_vec_.size() << std::endl;
    }
    // load imu info, format is inherited from euroc dataset
    void readImu(const std::string &IMU_file)
    {
        // initialize a csv reader
        io::CSVReader<7> in(IMU_file);
        // standard format to parse a csv
        in.read_header(io::ignore_extra_column, "#timestamp [ns]",
                       "w_RS_S_x [rad s^-1]", "w_RS_S_y [rad s^-1]", "w_RS_S_z [rad s^-1]",
                       "a_RS_S_x [m s^-2]", "a_RS_S_y [m s^-2]", "a_RS_S_z [m s^-2]");
        // corresponding to the ingored header line
        int64_t timestamp;
        // acceleration
        double w_RS_S_x, w_RS_S_y, w_RS_S_z;
        // gyroscope
        double a_RS_S_x, a_RS_S_y, a_RS_S_z;
        int cnt = 0;
        // load each line regularly
        imu_vec_.clear();
        while (in.read_row(timestamp,
                           w_RS_S_x, w_RS_S_y, w_RS_S_z,
                           a_RS_S_x, a_RS_S_y, a_RS_S_z))
        {
            // instantiate imu
            StampedImu imu;
            imu.timestamp_ = timestamp;
            imu.accel_ = Eigen::Vector3d(a_RS_S_x, a_RS_S_y, a_RS_S_z);
            imu.gyro_ = Eigen::Vector3d(w_RS_S_x, w_RS_S_y, w_RS_S_z);
            imu_vec_.push_back(imu);
            cnt++;
        }
        std::cout << "Load imu: " << imu_vec_.size() << std::endl;
    }
    std::vector<StampedPose> states_vec_;
    std::vector<StampedImu> imu_vec_;
};

TEST(Spline, poseSplineInitialization)
{
    // from MH_01_easy/mav0
    std::string pose_file = "../state_groundtruth_estimate0/data.csv";
    std::string imu_meas_file = "../imu0/data.csv";
    // load data
    TestSample testSample;
    testSample.readStates(pose_file);
    testSample.readImu(imu_meas_file);
    // initialize a PoseSpline, with 0.1 as the time interval
    PoseSpline poseSpline(0.1);
    // vector as spline knot variables
    // 3 for position, while 6 for acceleration and gyroscope
    VectorSpaceSpline<3> vectorSpaceSpline(0.1);
    VectorSpaceSpline<6> vectorSpaceSpline6(0.1);
    // predefined data container
    // samples, positionSamples and position6Samples are samples to be fed into spline 
    // to estimate the control point coefficients
    std::vector<std::pair<double, Pose<double>>> samples, queryMeas;
    std::vector<std::pair<double, Eigen::Vector3d>> queryVelocityMeas;
    std::vector<std::pair<double, Eigen::Vector3d>> positionSamples;
    std::vector<std::pair<double, Eigen::Matrix<double, 6, 1>>> position6Samples;
    //
    int start = 0;
    int end = testSample.states_vec_.size() / 5;
    for (uint i = start; i < end; i++)
    {
        // fetch a single pose point
        StampedPose stampedPose = testSample.states_vec_.at(i);
        // coordinate transform
        Eigen::Quaterniond QuatHamilton(stampedPose.q_);
        Eigen::Matrix3d R = QuatHamilton.toRotationMatrix();
        Quaternion QuatJPL = rotMatToQuat(R);
        // construct a pose, necessary data structure convert
        Pose<double> pose(stampedPose.t_, QuatJPL);
        queryMeas.push_back(std::pair<double, Pose<double>>(Time(stampedPose.timestamp_).toSec(), pose));
        queryVelocityMeas.push_back(std::pair<double, Eigen::Vector3d>(Time(stampedPose.timestamp_).toSec(), stampedPose.v_));
        // convert to n second
        // std::cout << Time(stampedPose.timestamp_).toNSec() << std::endl;
        // downsample the input sample measurements
        if (i % 5 == 0)
        {
            samples.push_back(std::pair<double, Pose<double>>(Time(stampedPose.timestamp_).toSec(), pose));
            positionSamples.push_back(std::pair<double, Eigen::Vector3d>(Time(stampedPose.timestamp_).toSec(), stampedPose.t_));
            Eigen::VectorXd meas(6);
            meas << stampedPose.t_, stampedPose.t_;
            position6Samples.push_back(std::pair<double, Eigen::Matrix<double, 6, 1>>(Time(stampedPose.timestamp_).toSec(), meas));
        }
    }
    // initialize the spline by samples(sequences of time,pose)
    poseSpline.initialPoseSpline(samples);
    vectorSpaceSpline.initialSpline(positionSamples);
    vectorSpaceSpline6.initialSpline(position6Samples);
    // Test: pose spline evalPoseSpline
    for (auto pair : queryMeas)
    {
        if (poseSpline.isTsEvaluable(pair.first))
        {// check if located in time interval
            // get a interpolated pose by timestamp and spline model
            // to check if there are artifacts caused by spline fitting
            Pose<double> query = poseSpline.evalPoseSpline(pair.first);
            // pair.second is gt
            // std::cout <<"Gt:    "<<pair.second.r().transpose() << " " << pair.second.q().transpose()<<std::endl;
            // std::cout <<"Query: "<<query.r().transpose()<<" "<< query.q().transpose()<< std::endl << std::endl;
            // check translation and rotation
            GTEST_ASSERT_LT((pair.second.r() - query.r()).norm(), 5e-2);
            GTEST_ASSERT_LT((pair.second.q() - query.q()).norm(), 5e-2);
            // check the translation vector and measure translation vector are equal with in a certain threshold
            Eigen::Vector3d queryPosition = vectorSpaceSpline.evaluateSpline(pair.first);
            GTEST_ASSERT_LT((pair.second.r() - queryPosition).norm(), 5e-2);
            // check 6-D vector
            Eigen::Matrix<double, 6, 1> queryPosition6 = vectorSpaceSpline6.evaluateSpline(pair.first);
            GTEST_ASSERT_LT((pair.second.r() - queryPosition6.head<3>()).norm(), 5e-2);
            GTEST_ASSERT_LT((pair.second.r() - queryPosition6.tail<3>()).norm(), 5e-2);
        }
    }
    // Check PoseSpline linear velocity
    for (auto pair : queryVelocityMeas)
    {
        if (poseSpline.isTsEvaluable(pair.first))
        {
            Eigen::Vector3d query = poseSpline.evalLinearVelocity(pair.first);
            // std::cout <<"Gt:    "<< pair.second.transpose()<<std::endl;
            // std::cout <<"Query: "<< query.transpose()<< std::endl << std::endl;
            // ofs_debug << pair.second.transpose() << " " << query.transpose() << std::endl;
            GTEST_ASSERT_LT((pair.second - query).norm(), 0.1);
        }
    }
    // seems that no accelerator check exist?
    // std::ofstream ofs_debug("/home/pang/debug.txt");
    const Eigen::Vector3d G(0.0, 0.0, 9.81);
    for (uint i = start; i < end; i++)
    {
        StampedImu stampedImu = testSample.imu_vec_.at(i);
        auto ts = Time(stampedImu.timestamp_).toSec();
        if (poseSpline.isTsEvaluable(ts))
        {
            Eigen::Vector3d evalAccel = poseSpline.evalLinearAccelerator(ts, G);
            // Note: the accelerator is noisy,
            // std::cout <<"Gt:    "<< stampedImu.accel_.transpose()<<std::endl;
            // std::cout <<"Query: "<< evalAccel.transpose()<< std::endl << std::endl;
            // ofs_debug << stampedImu.accel_.transpose() << " " << evalAccel.transpose() << std::endl;
        }
    }
    // seems that no gyro check exist?
    for (uint i = start; i < end; i++)
    {
        StampedImu stampedImu = testSample.imu_vec_.at(i);
        auto ts = Time(stampedImu.timestamp_).toSec();
        if (poseSpline.isTsEvaluable(ts))
        {
            Eigen::Vector3d query = poseSpline.evalOmega(ts);
            // std::cout <<"Gt:    "<< stampedImu.gyro_.transpose()<<std::endl;
            // std::cout <<"Query: "<< query.transpose()<< std::endl << std::endl;
            // ofs_debug << stampedImu.gyro_.transpose() << " " << query.transpose() << std::endl;
            // std::cout << i << "/" << end << std::endl;
        }
    }
    // ofs_debug.close();
}

TEST(Spline, quaternionSplineInitialization)
{
    std::string pose_file = "/home/pang/disk/dataset/euroc/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv";
    std::string imu_meas_file = "/home/pang/disk/dataset/euroc/MH_01_easy/mav0/imu0/data.csv";

    TestSample testSample;
    testSample.readStates(pose_file);
    testSample.readImu(imu_meas_file);

    int start = 0;
    int end = testSample.states_vec_.size() / 5;

    QuaternionSpline qspline(0.1);
    std::vector<std::pair<double, Quaternion>> samples, queryMeas;

    for (uint i = start; i < end; i++)
    {
        StampedPose stampedPose = testSample.states_vec_.at(i);

        Eigen::Quaterniond QuatHamilton(stampedPose.q_);
        Eigen::Matrix3d R = QuatHamilton.toRotationMatrix();
        Quaternion QuatJPL = rotMatToQuat(R);

        queryMeas.push_back(std::pair<double, Quaternion>(Time(stampedPose.timestamp_).toSec(), QuatJPL));

        if (i % 5 == 0)
        {
            samples.push_back(std::pair<double, Quaternion>(Time(stampedPose.timestamp_).toSec(), QuatJPL));
        }
    }

    qspline.initialQuaternionSpline(samples);

    for (auto pair : queryMeas)
    {
        if (qspline.isTsEvaluable(pair.first))
        {
            auto query = qspline.evalQuatSpline(pair.first);
            GTEST_ASSERT_LT((pair.second - query).norm(), 5e-2);
        }
    }
    //    std::ofstream ofs_debug("/home/pang/debug.txt");
    for (uint i = start; i < end; i++)
    {
        StampedImu stampedImu = testSample.imu_vec_.at(i);
        auto ts = Time(stampedImu.timestamp_).toSec();
        if (qspline.isTsEvaluable(ts))
        {
            Eigen::Vector3d query = qspline.evalOmega(ts);
            //            std::cout <<"Gt:    "<< stampedImu.gyro_.transpose()<<std::endl;
            //            std::cout <<"Query: "<< query.transpose()<< std::endl << std::endl;
            //            ofs_debug << stampedImu.gyro_.transpose() << " " << query.transpose() << std::endl;
            //            std::cout << i << "/" << end << std::endl;
        }
    }
    //        ofs_debug.close();
}