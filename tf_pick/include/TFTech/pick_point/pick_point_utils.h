
#ifndef TFTECH_EPIC_PICK_POINT_UTIL_H
#define TFTECH_EPIC_PICK_POINT_UTIL_H

#include "TFTech/common/json/nlohmann_json.hpp"
#include "pick_point_object.h"
#include "types.h"

namespace TFTech {

std::vector<PickPointObject> loadPickPoints(const nlohmann::json& jsonArray);
std::vector<std::shared_ptr<PickStrategy>> loadPickStrategies(const nlohmann::json& json);

}  // namespace TFTech

#endif