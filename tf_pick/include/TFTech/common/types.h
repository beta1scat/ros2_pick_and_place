#ifndef TFTECH_TYPES_H
#define TFTECH_TYPES_H

#include <Eigen/Dense>
#include <exception>
#include <string>

namespace TFTech {
    struct RigidTransform;

    typedef Eigen::Matrix<double, 6, 1> RotationVectorPose;
    typedef Eigen::Matrix<double, 6, 1> XYZEulerPose;
    typedef Eigen::Matrix<double, 6, 1> XZYEulerPose;
    typedef Eigen::Matrix<double, 6, 1> YXZEulerPose;
    typedef Eigen::Matrix<double, 6, 1> YZXEulerPose;
    typedef Eigen::Matrix<double, 6, 1> ZXYEulerPose;
    typedef Eigen::Matrix<double, 6, 1> ZYXEulerPose;
    typedef Eigen::Matrix<double, 6, 1> ZYZEulerPose;
    typedef Eigen::Matrix<double, 6, 1> XYZFixedPose;
    typedef Eigen::Matrix<double, 6, 1> JointPose;
    typedef RigidTransform CartesianPose;

    /**
     * @brief RigidTransform
     *  此处的rigid transformation 专指 proper rigid transformation，即不包含reflection的 rigid transformation
     *  结构体内的两个成员变量都不是Fixed-size vectorizable Eigen objects，
     *  所以无需进行特殊处理，也可以正常与STL容器一起使用。 commented by lihk11 
     **/
    struct RigidTransform {
        Eigen::AngleAxisd rotation;
        Eigen::Translation3d translation;

        static RigidTransform fromMatrix(const Eigen::Matrix4d &other);
        static RigidTransform fromRT(const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation);
        static RigidTransform fromRT(const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translation);
        static RigidTransform fromRT(const Eigen::AngleAxisd &rotation, const Eigen::Vector3d &translation);
        static RigidTransform fromXYZEulerPose(const XYZEulerPose &pose);
        static RigidTransform fromXZYEulerPose(const XZYEulerPose &pose);
        static RigidTransform fromYZXEulerPose(const YZXEulerPose &pose);
        static RigidTransform fromYXZEulerPose(const YXZEulerPose &pose);
        static RigidTransform fromZYXEulerPose(const ZYXEulerPose &pose);
        static RigidTransform fromZXYEulerPose(const ZXYEulerPose &pose);
        static RigidTransform fromZYZEulerPose(const ZYZEulerPose &pose);
        static RigidTransform fromXYZFixedPose(const XYZFixedPose &pose);
        static RigidTransform fromAffine(const Eigen::Affine3d &other);
        static RigidTransform fromRotationVectorPose(const RotationVectorPose &pose);

        Eigen::Vector3d getRotationVector() const;
        Eigen::Vector3d getTranslationVector() const;
        Eigen::Matrix4d getMatrix() const;
        Eigen::Affine3d getAffine() const;
        Eigen::Isometry3d getIsometry() const;
        Eigen::Quaterniond getQuaternion() const;
        XYZEulerPose getXYZEulerPose() const;
        XZYEulerPose getXZYEulerPose() const;
        YZXEulerPose getYZXEulerPose() const;
        YXZEulerPose getYXZEulerPose() const;
        ZYXEulerPose getZYXEulerPose() const;
        ZXYEulerPose getZXYEulerPose() const;
        ZYZEulerPose getZYZEulerPose() const;
        XYZFixedPose getXYZFixedPose() const;
        RotationVectorPose getRotationVectorPose() const;
    };

}  // namespace TFTech

#endif  // TFTECH_TYPES_H