#ifndef OCTOMAP_GRASPING__OCTREEGRASPQUALITY_HPP_
#define OCTOMAP_GRASPING__OCTREEGRASPQUALITY_HPP_

#include "octomap_grasping/visibility_control.h"
#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <eigen3/Eigen/Dense>
#include <octomap/OcTree.h>

#define ORIENTATION_STEPS 18 // discretisation of planar gripper orientation for grasp planning 

namespace octomap
{
  class OcTreeGraspQualityNode : public OcTreeNode
  {
    public:
    friend class OcTreeGraspQuality; // needs access to node children with protected status

    // Class for storing grasp quality information of the voxel
    class GraspQuality
    {
      public:
      GraspQuality() : normal{1,0,0}, angle_quality{}
      {
        angle_quality.row(0).setLinSpaced(0, M_PI_2);
      }
      GraspQuality(Eigen::Vector3f _normal, Eigen::Matrix<float, 2, ORIENTATION_STEPS> _angle_quality) : normal{_normal}, angle_quality{_angle_quality} {}
      /* UNNECESSARY
      inline void operator= (const GraspQuality& other)
      {
        normal = other.normal;
        angle_quality = other.angle_quality;
      }*/
      inline bool operator== (const GraspQuality& other) const
      {
        return (normal == other.normal && angle_quality == other.angle_quality);
      }
      inline bool operator!= (const GraspQuality& other) const
      {
        return (normal != other.normal && angle_quality != other.angle_quality);
      }

      Eigen::Vector3f normal;
      Eigen::Matrix<float, 2, ORIENTATION_STEPS> angle_quality; // orientation of the gripper and numeric grasp quality
    };

    OcTreeGraspQualityNode() : OcTreeNode(), grasp_quality{} {}

    OcTreeGraspQualityNode(const OcTreeGraspQualityNode& rhs) : OcTreeNode(rhs), grasp_quality{rhs.grasp_quality} {}

    /* UNNECESSARY
    bool operator=(const OcTreeGraspQualityNode& rhs)
    {
      grasp_quality = rhs.grasp_quality;
      return (value = rhs.value);
    }*/

    bool operator==(const OcTreeGraspQualityNode& rhs) const
    {
      return (rhs.value == value && rhs.grasp_quality == grasp_quality);
    }

    void copyData(const OcTreeGraspQualityNode& from)
    {
      OcTreeNode::copyData(from);
      this->grasp_quality = from.getGraspQuality();
    }

    inline GraspQuality getGraspQuality() const {return grasp_quality;}

    inline void setGraspQuality(GraspQuality gq) {this->grasp_quality = gq;}

    inline void setGraspQuality(Eigen::Vector3f _normal, Eigen::Matrix<float, 2, ORIENTATION_STEPS> _angle_quality)
    {
      this->grasp_quality = GraspQuality(_normal, _angle_quality);
    }

    void updateGraspQualityChildren();

    OcTreeGraspQualityNode::GraspQuality getAverageChildGraspQuality() const;

    /**
     * @brief Check if GraspQuality properties of voxel have been populated with real data
     * @returns Applicability state of GraspQuality data
     */
    inline bool isGraspQualitySet() const
    {
      return grasp_quality.angle_quality.row(1).isZero();
    }

    virtual ~OcTreeGraspQualityNode() {};

    // file I/O
    std::istream& readData(std::istream &s);
    std::ostream& writeData(std::ostream &s) const;

    protected:
      GraspQuality grasp_quality;
  };

  class OcTreeGraspQuality : public OccupancyOcTreeBase <OcTreeGraspQualityNode>
  {
    public:
    OcTreeGraspQuality(double resolution);

    OcTreeGraspQuality* create() const {return new OcTreeGraspQuality(resolution);}

    std::string getTreeType() const {return "OcTreeGraspQuality";}

    // Copy assignment operator
    OcTreeGraspQuality& operator=(const OcTreeGraspQuality& rhs)
    {
      // Guard self assignment
      if (this == &rhs)
        return *this;

      *this = rhs;
      return *this;
    }

    /**
     * Custom conversion function with color coding for grasp quality
     * ? Set this to PRIVATE and only expose the conversion operator, or entirely refactor into only the operator existing
     */
    ColorOcTree toColorOcTree() const;

    operator ColorOcTree() const;

    void importOcTree(const OcTree&);

    /**
     * Prunes a node when it is collapsible. This overloaded
     * version only considers the node occupancy for pruning,
     * different grasp qualities of child nodes are ignored.
     * @return true if pruning was successful
     */
    virtual bool pruneNode(OcTreeGraspQualityNode* node);

    virtual bool isNodeCollapsible(const OcTreeGraspQualityNode* node) const;

    OcTreeGraspQualityNode* setNodeGraspQuality(const OcTreeKey& key, Eigen::Vector3f& _normal, Eigen::Matrix<float, 2, ORIENTATION_STEPS>& _angle_quality);

    OcTreeGraspQualityNode* setNodeGraspQuality(const point3d& octo_point3d, Eigen::Vector3f& _normal, Eigen::Matrix<float, 2, ORIENTATION_STEPS>& _angle_quality)
    {
      OcTreeKey key;
      if (!this->coordToKeyChecked(octo_point3d, key)) return NULL;
      return setNodeGraspQuality(key,_normal, _angle_quality);
    }

    OcTreeGraspQualityNode* setNodeGraspQuality(float x, float y, float z, Eigen::Vector3f& _normal, Eigen::Matrix<float, 2, ORIENTATION_STEPS>& _angle_quality)
    {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
      return setNodeGraspQuality(key,_normal, _angle_quality);
    }

    //TODO integrate/averageNodeGraspQuality functions... (line 143-164 ColorOcTree file https://github.com/OctoMap/octomap/blob/ros2/octomap/include/octomap/ColorOcTree.h)

    // update inner nodes, sets grasp quality to average child grasp quality
    void updateInnerOccupancy();

    // uses gnuplot to plot a histogram in EPS format
    void writeGraspQualityHistogram(std::string filename);

    virtual ~OcTreeGraspQuality() {};
    
    protected:
    void updateInnerOccupancyRecurs(OcTreeGraspQualityNode* node, unsigned int depth);

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

  /**
   * User friendly stream output
   * * RGB Format
   * ! BROKEN
   */  
  std::ostream& operator<<(std::ostream& out, OcTreeGraspQualityNode::GraspQuality const& gq);

}  // namespace octomap

#endif  // OCTOMAP_GRASPING__OCTREEGRASPQUALITYMAP_HPP_
