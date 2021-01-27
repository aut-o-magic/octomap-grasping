#ifndef OCTOMAP_GRASPING__OCTREEGRIPPER_HPP_
#define OCTOMAP_GRASPING__OCTREEGRIPPER_HPP_

#include "octomap_grasping/visibility_control.h"
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>

namespace octomap
{
  class OcTreeNodeGripper : public OcTreeNode
  {
    public:
    OcTreeNodeGripper() : OcTreeNode() {}       // TODO add extra stuff

    OcTreeNodeGripper(const OcTreeNodeGripper& rhs) : OcTreeNode(rhs) {}  // TODO add extra stuff

    bool operator=(const OcTreeNodeGripper& rhs)
    {
      return (value = rhs.value); // TODO add extra stuff
    }

    bool operator==(const OcTreeNodeGripper& rhs) const
    {
      return (rhs.value == value); // && ...  // TODO add extra stuff
    }

    void copyData(const OcTreeNodeGripper& from)
    {
      OcTreeNode::copyData(from);
      // TODO add extra stuff
    }

    // update occupancy of inner nodes
    inline void updateOccupancyChildren() {
      this->setLogOdds(this->getMaxChildLogOdds());  // conservative
      // TODO add extra stuff
    }

    virtual ~OcTreeNodeGripper() {};

    protected:
      // TODO add extra stuff
  };

  class OcTreeGripper : public OccupancyOcTreeBase <OcTreeNodeGripper>
  {
    public:
    OcTreeGripper(double resolution);

    OcTreeGripper* create() const {return new OcTreeGripper(resolution);}

    std::string getTreeType() const {return "OcTreeGripper";}

    virtual void updateNodeLogOdds(OcTreeNodeGripper* node, const float& update) const;

    virtual ~OcTreeGripper() {};
    
    protected:
    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once. You need this as a
     * static member in any derived octree class in order to read .ot
     * files through the AbstractOcTree factory. You should also call
     * ensureLinking() once from the constructor.
     */
    class StaticMemberInitializer{
    public:
      StaticMemberInitializer() {
        OcTreeGripper* tree = new OcTreeGripper(0.1);
        tree->clearKeyRays();
        AbstractOcTree::registerTreeType(tree);
      }

      /**
      * Dummy function to ensure that MSVC does not drop the
      * StaticMemberInitializer, causing this tree failing to register.
      * Needs to be called from the constructor of this octree.
      */
      void ensureLinking() {};
    };
    /// to ensure static initialization (only once)
    static StaticMemberInitializer OcTreeGripperMemberInit;
  };

}  // namespace octomap

#endif  // OCTOMAP_GRASPING__OcTreeGripperMAP_HPP_


