#ifndef OCTOMAP_GRASPING__OCTREEGRIPPER_HPP_
#define OCTOMAP_GRASPING__OCTREEGRIPPER_HPP_

#include "octomap_grasping/visibility_control.h"
#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <eigen3/Eigen/Dense>

namespace octomap
{
  class OcTreeNodeGripper : public OcTreeNode
  {
    public:
    friend class OcTreeGripper; // needs access to node children with protected status

    OcTreeNodeGripper() : OcTreeNode(), is_grasping_surface{false} {}

    OcTreeNodeGripper(const OcTreeNodeGripper& rhs) : OcTreeNode(rhs), is_grasping_surface{rhs.is_grasping_surface} {}

    /* UNNECESSARY
    bool operator=(const OcTreeNodeGripper& rhs)
    {
      is_grasping_surface = rhs.is_grasping_surface;
      return (value = rhs.value);
    }*/

    bool operator==(const OcTreeNodeGripper& rhs) const
    {
      return (rhs.value == value && rhs.is_grasping_surface == is_grasping_surface);
    }

    void copyData(const OcTreeNodeGripper& from)
    {
      OcTreeNode::copyData(from);
      this->is_grasping_surface = from.isGraspingSurface();
    }

    inline bool isGraspingSurface() const {return is_grasping_surface;}

    inline void setIsGraspingSurface(bool grasping_surface_flag) {this->is_grasping_surface = grasping_surface_flag;}

    void updateIsGraspingSurfaceChildren();

    bool getAverageChildIsGraspingSurface() const;

    virtual ~OcTreeNodeGripper() {};

    // file I/O
    std::istream& readData(std::istream &s);
    std::ostream& writeData(std::ostream &s) const;

    protected:
      bool is_grasping_surface; // Flag that dictates is the voxel is part of the grasping surface of the gripper or it is an obstacle
  };

  class OcTreeGripper : public OccupancyOcTreeBase <OcTreeNodeGripper>
  {
    public:
    OcTreeGripper(double resolution);

    OcTreeGripper* create() const {return new OcTreeGripper(resolution);}

    std::string getTreeType() const {return "OcTreeGripper";}

    ColorOcTree toColorOcTree() const;

    // Custom conversion function with color coding for grasp quality
    operator ColorOcTree() const;

    /**
     * Prunes a node when it is collapsible. This overloaded
     * version only considers the node occupancy for pruning,
     * different grasp qualities of child nodes are ignored.
     * @return true if pruning was successful
     */
    virtual bool pruneNode(OcTreeNodeGripper* node);

    virtual bool isNodeCollapsible(const OcTreeNodeGripper* node) const;

    OcTreeNodeGripper* setNodeIsGraspingSurface(const OcTreeKey& key, bool grasping_surface_flag);

    OcTreeNodeGripper* setNodeIsGraspingSurface(const point3d& octo_point3d, bool grasping_surface_flag)
    {
      OcTreeKey key;
      if (!this->coordToKeyChecked(octo_point3d, key)) return NULL;
      return setNodeIsGraspingSurface(key, grasping_surface_flag);
    }

    OcTreeNodeGripper* setNodeIsGraspingSurface(float x, float y, float z, bool grasping_surface_flag)
    {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
      return setNodeIsGraspingSurface(key, grasping_surface_flag);
    }

    //TODO integrate/averageNodeIsGraspingSurface functions... (line 143-164 ColorOcTree file https://github.com/OctoMap/octomap/blob/ros2/octomap/include/octomap/ColorOcTree.h)

    // update inner nodes, sets grasp quality to average child grasp quality
    void updateInnerOccupancy();

    virtual ~OcTreeGripper() {};
    
    protected:
    void updateInnerOccupancyRecurs(OcTreeNodeGripper* node, unsigned int depth);

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

  //! user friendly output in format
  std::ostream& operator<<(std::ostream& out, bool const& grasping_surface_flag); // TODO Implement

}  // namespace octomap

#endif  // OCTOMAP_GRASPING__OcTreeGripper_HPP_
