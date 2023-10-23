#ifndef TFTECH_PCU_FEATURE_NORMAL_H
#define TFTECH_PCU_FEATURE_NORMAL_H

#include <opencv2/core/core.hpp>
#include "common/pcl_shorthand.h"

namespace TFTech {
    enum Orientation {
        PLUS_X = 0,           //!< N.x always positive
        MINUS_X = 1,          //!< N.x always negative
        PLUS_Y = 2,           //!< N.y always positive
        MINUS_Y = 3,          //!< N.y always negative
        PLUS_Z = 4,           //!< N.z always positive
        MINUS_Z = 5,          //!< N.z always negative
        PLUS_BARYCENTER = 6,  //!< Normals always opposite to the cloud barycenter
        MINUS_BARYCENTER = 7, //!< Normals always towards the cloud barycenter
        PLUS_ORIGIN = 8,      //!< Normals always opposite to the origin
        MINUS_ORIGIN = 9,     //!< Normals always towards the origin
        PREVIOUS = 10,        //!< Re-use previous normal (if any)
        UNDEFINED = 255       //!< Undefined (no orientation is required)
    };

    enum LOCAL_MODEL_TYPES {
        NO_MODEL			= 0,	//!< No local model
        CC_LS				= 1,	//!< Least Square best fitting plane
        CC_TRI				= 2,	//!< 2.5D Delaunay triangulation
        CC_QUADRIC			= 3	    //!< 2.5D quadric function
    };

    PointCloudNT::Ptr estimateNormals(const PointCloudT::Ptr &cloud, 
                                      LOCAL_MODEL_TYPES localModel=LOCAL_MODEL_TYPES::CC_LS, 
                                      Orientation preferredOrientation=Orientation::MINUS_Z);

    cv::Mat estimateNormals(const cv::Mat &pointMat, 
                            LOCAL_MODEL_TYPES localModel=LOCAL_MODEL_TYPES::CC_LS, 
                            Orientation preferredOrientation=Orientation::MINUS_Z);

}  // namespace TFTech

#endif  // TFTECH_PCU_FEATURE_NORMAL_H