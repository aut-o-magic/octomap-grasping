#include "octomap_grasping/OcTreeGraspQuality.hpp"

namespace octomap
{

OcTreeGraspQuality::OcTreeGraspQuality(double resolution) : OccupancyOcTreeBase<OcTreeNodeGraspQuality>(resolution)
{
    ocTreeGraspQualityMemberInit.ensureLinking();
}

void OcTreeGraspQuality::updateNodeLogOdds(OcTreeNodeGraspQuality* node, const float& update) const {
OccupancyOcTreeBase<OcTreeNodeGraspQuality>::updateNodeLogOdds(node, update);
// TODO Add stuff
}

OcTreeGraspQuality::~OcTreeGraspQuality()
{
}

OcTreeGraspQuality::StaticMemberInitializer OcTreeGraspQuality::ocTreeGraspQualityMemberInit;

}  // namespace octomap_grasping
