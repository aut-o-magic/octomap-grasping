#ifndef OCTOMAP_GRASPING__OCTREEGRIPPER_HPP_
#define OCTOMAP_GRASPING__OCTREEGRIPPER_HPP_

#include "octomap_grasping/visibility_control.h"
#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <eigen3/Eigen/Dense>
#include <octomap/OcTree.h>

namespace octomap
{
  class OcTreeGripperNode : public OcTreeNode
  {
    public:
    friend class OcTreeGripper; // needs access to node children with protected status

    OcTreeGripperNode() : OcTreeNode(), is_grasping_surface{false} {}

    OcTreeGripperNode(const OcTreeGripperNode& rhs) : OcTreeNode(rhs), is_grasping_surface{rhs.is_grasping_surface} {}

    /* UNNECESSARY
    bool operator=(const OcTreeGripperNode& rhs)
    {
      is_grasping_surface = rhs.is_grasping_surface;
      return (value = rhs.value);
    }*/

    bool operator==(const OcTreeGripperNode& rhs) const
    {
      return (rhs.value == value && rhs.is_grasping_surface == is_grasping_surface);
    }

    void copyData(const OcTreeGripperNode& from)
    {
      OcTreeNode::copyData(from);
      this->is_grasping_surface = from.isGraspingSurface();
    }

    inline bool isGraspingSurface() const {return this->is_grasping_surface;}

    inline void setIsGraspingSurface(bool grasping_surface_flag) {this->is_grasping_surface = grasping_surface_flag;}

    void updateIsGraspingSurfaceChildren();

    bool getAverageChildIsGraspingSurface() const;

    virtual ~OcTreeGripperNode() {};

    // file I/O
    std::istream& readData(std::istream &s);
    std::ostream& writeData(std::ostream &s) const;

    protected:
      bool is_grasping_surface; // Flag that dictates is the voxel is part of the grasping surface of the gripper or it is an obstacle
  };

  class OcTreeGripper : public OccupancyOcTreeBase <OcTreeGripperNode>
  {
    public:
    OcTreeGripper(double resolution);

    /**
     * Reads an OcTree from an object tree file
     * @param _filename Filename
     */
    OcTreeGripper(std::string _filename);

    OcTreeGripper* create() const {return new OcTreeGripper(resolution);}

    std::string getTreeType() const {return "OcTreeGripper";}

    // Copy assignment operator
    OcTreeGripper& operator=(const OcTreeGripper& rhs)
    {
      // Guard self assignment
      if (this == &rhs)
        return *this;

      this->setResolution(rhs.getResolution());
      this->root = rhs.root;
      this->graspable_voxels = rhs.graspable_voxels;
      this->grasping_normal = rhs.grasping_normal;
      return *this;
    }


    // Custom conversion function with color coding for grasp quality
    operator ColorOcTree() const;

    // Custom import function from generic OcTree
    void importOcTree(const OcTree*);

    /**
     * Prunes a node when it is collapsible. This overloaded
     * version only considers the node occupancy for pruning,
     * different grasp qualities of child nodes are ignored.
     * @return true if pruning was successful
     */
    virtual bool pruneNode(OcTreeGripperNode* node);

    virtual bool isNodeCollapsible(const OcTreeGripperNode* node) const;

    OcTreeGripperNode* setNodeIsGraspingSurface(const OcTreeKey& key, bool grasping_surface_flag);

    OcTreeGripperNode* setNodeIsGraspingSurface(const point3d& octo_point3d, bool grasping_surface_flag)
    {
      OcTreeKey key;
      if (!this->coordToKeyChecked(octo_point3d, key)) return NULL;
      return setNodeIsGraspingSurface(key, grasping_surface_flag);
    }

    OcTreeGripperNode* setNodeIsGraspingSurface(float x, float y, float z, bool grasping_surface_flag)
    {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
      return setNodeIsGraspingSurface(key, grasping_surface_flag);
    }

    //TODO integrate/averageNodeIsGraspingSurface functions... (line 143-164 ColorOcTree file https://github.com/OctoMap/octomap/blob/ros2/octomap/include/octomap/ColorOcTree.h)

    // update inner nodes, sets grasp quality to average child grasp quality
    void updateInnerOccupancy();

    /**
     * Update the object attribute storing the number of graspable voxels at max depth of the tree
     */
    void updateNumGraspableVoxels();

    /**
     * Get number of graspable voxels at max depth of the tree
     * @returns Graspable voxels count in tree
     */
    const unsigned long& getNumGraspableVoxels() const;

    /**
     * Get pointing vector towards object to grasp in gripper reference frame
     * @returns Pointing vector
     */
    inline const point3d getGraspingNormal() const {return grasping_normal;};

    /**
     * Set pointing vector towards object to grasp in gripper reference frame
     * @param __grasping_normal Pointing vector
     */
    inline void setGraspingNormal(const point3d& __grasping_normal) {this->grasping_normal = __grasping_normal;};

    /**
     * Translates the tree origin according to the input parameter (origin - translation)
     * @param translation Linear translation vector to new origin
     */
    void setOrigin(const octomap::point3d& translation);

    virtual ~OcTreeGripper() {};
    
    protected:
    void updateInnerOccupancyRecurs(OcTreeGripperNode* node, unsigned int depth);

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

    // stores graspable voxel count for grasp quality score normalisation
    unsigned long graspable_voxels;
    point3d grasping_normal; // Pointing vector towards object to grasp in gripper reference frame
  };

  /**
   * User friendly stream output
   * * red-green format
   * TODO Still needs to be implemented
   */  
  std::ostream& operator<<(std::ostream& out, bool const& grasping_surface_flag);

}  // namespace octomap

#endif  // OCTOMAP_GRASPING__OcTreeGripper_HPP_
