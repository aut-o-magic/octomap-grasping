#ifndef OCTOMAP_GRASPING__OCTREEGRASPQUALITYMAP_HPP_
#define OCTOMAP_GRASPING__OCTREEGRASPQUALITYMAP_HPP_

#include "octomap_grasping/visibility_control.h"
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>

namespace octomap
{
  class OcTreeNodeGraspQuality : public OcTreeNode
  {
    public:
    OcTreeNodeGraspQuality() : OcTreeNode() {}       // TODO add extra stuff

    OcTreeNodeGraspQuality(const OcTreeNodeGraspQuality& rhs) : OcTreeNode(rhs) {}  // TODO add extra stuff

    bool operator=(const OcTreeNodeGraspQuality& rhs)
    {
      return (value = rhs.value); // TODO add extra stuff
    }

    bool operator==(const OcTreeNodeGraspQuality& rhs) const
    {
      return (rhs.value == value); // && ...  // TODO add extra stuff
    }

    void copyData(const OcTreeNodeGraspQuality& from)
    {
      OcTreeNode::copyData(from);
      // TODO add extra stuff
    }

    // update occupancy of inner nodes
    inline void updateOccupancyChildren() {
      this->setLogOdds(this->getMaxChildLogOdds());  // conservative
      // TODO add extra stuff
    }

    virtual ~OcTreeNodeGraspQuality() {};

    protected:
      // TODO add extra stuff
  };

  class OcTreeGraspQuality : public OccupancyOcTreeBase <OcTreeNodeGraspQuality>
  {
    public:
    OcTreeGraspQuality(double resolution);

    OcTreeGraspQuality* create() const {return new OcTreeGraspQuality(resolution);}

    std::string getTreeType() const {return "OcTreeGraspQuality";}

    virtual void updateNodeLogOdds(OcTreeNodeGraspQuality* node, const float& update) const;

    virtual ~OcTreeGraspQuality() {};
    
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
        OcTreeGraspQuality* tree = new OcTreeGraspQuality(0.1);
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
    static StaticMemberInitializer ocTreeGraspQualityMemberInit;
  };

}  // namespace octomap

#endif  // OCTOMAP_GRASPING__OCTREEGRASPQUALITYMAP_HPP_


