#include <iostream>
#include <string>

#include <HalconCpp.h>

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_array.hpp"


#define BOOL_STR(b) (b?"true":"false")


const std::string basePath = "/root/";
const std::string childPath = "Downloads/";
const std::string modelName = "cube_5cm";
const std::string modelPath = basePath + childPath + modelName + ".ply";
HalconCpp::HTuple nullGenParamName, nullGenParamValue, status;


std::unique_ptr<HalconCpp::HObjectModel3D> toHalconObject3D(const sensor_msgs::msg::PointCloud2& source) {
    int offset_x, offset_y, offset_z;
    offset_x = offset_y = offset_z = 0;
    for (unsigned int i = 0; i < source.fields.size(); i++) {
        sensor_msgs::msg::PointField field = source.fields[i];
        if (field.name == "x") {
            offset_x = field.offset;
        }
        if (field.name == "y") {
            offset_y = field.offset;
        }
        if (field.name == "z") {
            offset_z = field.offset;
        }
    }
    HalconCpp::HTuple x_coords = HalconCpp::HTuple::TupleGenConst(
        (int)(source.width * source.height), 0);
    HalconCpp::HTuple y_coords = HalconCpp::HTuple::TupleGenConst(
        (int)(source.width * source.height), 0);
    HalconCpp::HTuple z_coords = HalconCpp::HTuple::TupleGenConst(
        (int)(source.width * source.height), 0);
    for (int i = 0; i < x_coords.Length(); i++) {
        x_coords[i] = *(float*)&source.data[(i * source.point_step) + offset_x];
        if(isnan(float(x_coords[i]))){
            x_coords[i] = 0.0;
        }
        y_coords[i] = *(float*)&source.data[(i * source.point_step) + offset_y];
        if(isnan(float(y_coords[i]))) {
            y_coords[i] = 0.0;
        }
        z_coords[i] = *(float*)&source.data[(i * source.point_step) + offset_z];
        if(isnan(float(z_coords[i]))) {
            z_coords[i] = 0.0;
        }
    }
    return std::unique_ptr<HalconCpp::HObjectModel3D>(new HalconCpp::HObjectModel3D(x_coords, y_coords, z_coords));
}

void surfaceMatchingHalcon(const std::string modelFile, const HalconCpp::HTuple& sceneModel3D,
                           geometry_msgs::msg::PoseArray& poseList, int maxResultsNum) {
    // HalconCpp::HTuple nullGenParamName, nullGenParamValue, status;
    HalconCpp::HTuple objectModel3D, objectModel3DNormals;
    ReadObjectModel3d(modelFile.c_str(), "m", nullGenParamName, nullGenParamValue, &objectModel3D, &status);
    SurfaceNormalsObjectModel3d(objectModel3D, "mls", nullGenParamName, nullGenParamValue, &objectModel3DNormals);
    HalconCpp::HTuple relSamplingDistance = 0.05;
    HalconCpp::HTuple keyPointFraction = 0.2;
    HalconCpp::HTuple minScore = 0.1;
    HalconCpp::HTuple surfaceModelID;
    CreateSurfaceModel(objectModel3DNormals, relSamplingDistance, nullGenParamName, nullGenParamValue, &surfaceModelID);
    HalconCpp::HTuple sceneModelROI;
    HalconCpp::HTuple Attrib;
    Attrib.Append("point_coord_z");
    HalconCpp::HTuple minValue = 0.1;
    HalconCpp::HTuple maxValue = 1 - 0.002;
    SelectPointsObjectModel3d(sceneModel3D, Attrib, minValue, maxValue, &sceneModelROI);
    // WriteObjectModel3d(sceneModelROI, "ply", "/root/Downloads/scene_halcon_roi.ply", nullGenParamName, nullGenParamValue); /* Save failed */
    HalconCpp::HTuple sceneModel3DNormals;
    SurfaceNormalsObjectModel3d(sceneModel3D, "mls", nullGenParamName, nullGenParamValue, &sceneModel3DNormals);
    HalconCpp::HTuple returnResultHandle, pose, score, surfaceMatchingResultID;
    HalconCpp::HTuple sfmGenParamName, sfmGenParamValue;
    sfmGenParamName.Append("num_matches");
    sfmGenParamValue.Append(maxResultsNum);
    FindSurfaceModel(surfaceModelID, sceneModel3DNormals, relSamplingDistance, keyPointFraction, minScore, "false",
                     sfmGenParamName, sfmGenParamValue, &pose, &score, &surfaceMatchingResultID);
    std::cout << pose.Length() << std::endl;
    std::cout << score.Length() << std::endl;
    HalconCpp::HTuple poseQuat;
    PoseToQuat(pose, &poseQuat);
    std::cout << poseQuat.Length() << std::endl;
    for(int i = 0; i < score.Length(); i++) {
        geometry_msgs::msg::Pose tmpPose;
        tmpPose.position.x = pose[0 + i*7];
        tmpPose.position.y = pose[1 + i*7];
        tmpPose.position.z = pose[2 + i*7];
        tmpPose.orientation.w = poseQuat[0 + i*4];
        tmpPose.orientation.x = poseQuat[1 + i*4];
        tmpPose.orientation.y = poseQuat[2 + i*4];
        tmpPose.orientation.z = poseQuat[3 + i*4];
        poseList.poses.push_back(tmpPose);
    }
}

/* ROS */
class MinimalSubscriber : public rclcpp::Node {
public:
    MinimalSubscriber(int maxResultsNum)
    : Node("minimal_subscriber")
    {
        this->subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/RGBD_camera/points", 1, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));

        this->publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("pick_poses", 1);
        this->MaxResultsNum = maxResultsNum;
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->header);
        // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->fields);
        RCLCPP_INFO(this->get_logger(), "height: '%d'", msg->height);
        RCLCPP_INFO(this->get_logger(), "width: '%d'", msg->width);
        RCLCPP_INFO(this->get_logger(), "is_bigendian: '%s'", BOOL_STR(msg->is_bigendian));
        RCLCPP_INFO(this->get_logger(), "point_step: '%d'", msg->point_step);
        RCLCPP_INFO(this->get_logger(), "row_step: '%d'", msg->row_step);
        RCLCPP_INFO(this->get_logger(), "is_dense: '%s'", BOOL_STR(msg->is_dense));
        /* Convert sensor_msgs::msg::PointCloud2 to Halcon HObjectModel3D */
        std::unique_ptr<HalconCpp::HObjectModel3D> hModel3DPtr = toHalconObject3D(*msg);
        HalconCpp::HTuple hModelTuple(*hModel3DPtr);
        // Debug object construct
        // WriteObjectModel3d(hModelTuple, "ply", "/root/Downloads/scene_halcon.ply", nullGenParamName, nullGenParamValue);
        geometry_msgs::msg::PoseArray poseList;
        try {
            surfaceMatchingHalcon(modelPath, hModelTuple, poseList, MaxResultsNum);
        } catch(HalconCpp::HException &except) {
            std::cout << except.ErrorMessage() << std::endl;
        }
        for (int i = 0; i < poseList.poses.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "pose %d in camera:", i);
            RCLCPP_INFO(this->get_logger(), "  + position:");
            RCLCPP_INFO(this->get_logger(), "    - x = %f", poseList.poses[i].position.x);
            RCLCPP_INFO(this->get_logger(), "    - y = %f", poseList.poses[i].position.y);
            RCLCPP_INFO(this->get_logger(), "    - z = %f", poseList.poses[i].position.z);
            RCLCPP_INFO(this->get_logger(), "  + orientation:");
            RCLCPP_INFO(this->get_logger(), "    - x = %f", poseList.poses[i].orientation.x);
            RCLCPP_INFO(this->get_logger(), "    - y = %f", poseList.poses[i].orientation.y);
            RCLCPP_INFO(this->get_logger(), "    - z = %f", poseList.poses[i].orientation.z);
            RCLCPP_INFO(this->get_logger(), "    - w = %f", poseList.poses[i].orientation.w);
        }
        publisher_->publish(poseList);
        rclcpp::shutdown();
    }

    int MaxResultsNum;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    for (int i = 0; i<argc; i++)
	{
		std::cout << "Parameter index =" << i << " ";
		std::cout << "Parameter value =" << argv[i] << std::endl;
	}
    int num = 1;
    if (argc <= 1) {
        std::cout<< "Please input max match results num" << std::endl;
    } else {
        num = atoi(argv[1]);
    }
    std::cout<< "Max match results is: " << num << std::endl;
    rclcpp::spin(std::make_shared<MinimalSubscriber>(num));
    rclcpp::shutdown();
    return 0;
    // try{
    //     std::string sceneFile = "/root/Downloads/scene.ply";
    //     HalconCpp::HTuple sceneModel3D;
    //     ReadObjectModel3d(sceneFile.c_str(), "m", nullGenParamName, nullGenParamValue, &sceneModel3D, &status);
    //     geometry_msgs::msg::PoseArray poseList;
    //     surfaceMatchingHalcon(modelPath, sceneModel3D, poseList);
    // }
    // catch(HalconCpp::HException &except){
    //     std::cout << except.ErrorMessage() << std::endl;
    // }
}