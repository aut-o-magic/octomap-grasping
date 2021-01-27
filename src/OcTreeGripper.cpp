#include "octomap_grasping/OcTreeGripper.hpp"

namespace octomap
{

OcTreeGripper::OcTreeGripper(double resolution) : OccupancyOcTreeBase<OcTreeNodeGripper>(resolution)
{
    OcTreeGripperMemberInit.ensureLinking();
}

void OcTreeGripper::updateNodeLogOdds(OcTreeNodeGripper* node, const float& update) const {
OccupancyOcTreeBase<OcTreeNodeGripper>::updateNodeLogOdds(node, update);
// TODO Add stuff
}


OcTreeGripper::StaticMemberInitializer OcTreeGripper::OcTreeGripperMemberInit;

}  // namespace octomap_grasping
