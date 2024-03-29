#ifndef OCTOMAP_GRASPING__OCTREEGRASPQUALITY_HPP_
#define OCTOMAP_GRASPING__OCTREEGRASPQUALITY_HPP_

#include "octomap_grasping/visibility_control.h"
#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <eigen3/Eigen/Dense>
#include <octomap/OcTree.h>

#define ORIENTATION_STEPS 4 // discretisation of planar gripper orientation for grasp planning 

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
      GraspQuality() : angle_quality{}
      {
        angle_quality.row(0).setLinSpaced(0, M_PI_2);
        angle_quality.row(1).setZero();
      }
      GraspQuality(Eigen::Matrix<float, 2, ORIENTATION_STEPS> _angle_quality) : angle_quality{_angle_quality} {}
      /* UNNECESSARY
      inline void operator= (const GraspQuality& other)
      {
        angle_quality = other.angle_quality;
      }*/
      inline bool operator== (const GraspQuality& other) const
      {
        return (angle_quality == other.angle_quality);
      }
      inline bool operator!= (const GraspQuality& other) const
      {
        return (angle_quality != other.angle_quality);
      }

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

    inline void setGraspQuality(Eigen::Matrix<float, 2, ORIENTATION_STEPS> _angle_quality)
    {
      this->grasp_quality = GraspQuality(_angle_quality);
    }

    void updateGraspQualityChildren();

    OcTreeGraspQualityNode::GraspQuality getAverageChildGraspQuality() const;

    /**
     * @brief Check if GraspQuality properties of voxel have been populated with real data
     * @returns Applicability state of GraspQuality data
     */
    inline bool isGraspQualitySet() const
    {
      return !grasp_quality.angle_quality.row(1).isZero(); // if all grasp qualities are zero (which is the value they are initialised to), most certainly the quality is not set
    }

    operator ColorOcTreeNode () const;

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

    /**
     * Reads an OcTree from an object tree file
     * @param _filename Filename
     */
    OcTreeGraspQuality(std::string _filename);

    OcTreeGraspQuality* create() const {return new OcTreeGraspQuality(resolution);}

    std::string getTreeType() const {return "OcTreeGraspQuality";}

    // Copy assignment operator
    OcTreeGraspQuality& operator=(const OcTreeGraspQuality& rhs)
    {
      // Guard self assignment
      if (this == &rhs)
        return *this;

      this->setResolution(rhs.getResolution());
      this->root = rhs.root;
      return *this;
    }

    operator ColorOcTree() const;

    void importOcTree(const OcTree *);

    /**
     * Prunes a node when it is collapsible. This overloaded
     * version only considers the node occupancy for pruning,
     * different grasp qualities of child nodes are ignored.
     * @return true if pruning was successful
     */
    virtual bool pruneNode(OcTreeGraspQualityNode* node);

    virtual bool isNodeCollapsible(const OcTreeGraspQualityNode* node) const;

    OcTreeGraspQualityNode* setNodeGraspQuality(const OcTreeKey& key, Eigen::Matrix<float, 2, ORIENTATION_STEPS>& _angle_quality);

    OcTreeGraspQualityNode* setNodeGraspQuality(const point3d& octo_point3d, Eigen::Matrix<float, 2, ORIENTATION_STEPS>& _angle_quality)
    {
      OcTreeKey key;
      if (!this->coordToKeyChecked(octo_point3d, key)) return NULL;
      return setNodeGraspQuality(key, _angle_quality);
    }

    OcTreeGraspQualityNode* setNodeGraspQuality(float x, float y, float z, Eigen::Matrix<float, 2, ORIENTATION_STEPS>& _angle_quality)
    {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
      return setNodeGraspQuality(key, _angle_quality);
    }

    /**
     * Calculates the direction from the node in the provided coordinates to the center of volume (CoV) of the occupied nodes in the neighborhood. This method will only give either 0 or 1 surface normals, in comparison to getNormals() which can give multiple candidates
     * @param coords Point coordinates for which to calculate the surface normal
     * @param normals Surface normals collection to which the calculated normal will be written
     * @param depth Depth of the neighborhood to map in each direction. A depth of 1 will use a cube of 3x3x3 nodes to calculate the CoV, depth of 2 5x5x5, etc
     * @returns True for a succesful call, false if point requested is in unknown space
     */
    bool getNormal(const point3d& coords, std::vector<point3d>& normals, const unsigned int depth=1U) const;

        /**
     * Calculates the direction from the node to the center of volume (CoV) of the occupied nodes in the neighborhood. This method will only give either 0 or 1 surface normals, in comparison to getNormals() which can give multiple candidates
     * @param key Node key for which to calculate the surface normal
     * @param normals Surface normals collection to which the calculated normal will be written
     * @param depth Depth of the neighborhood to map in each direction. A depth of 1 will use a cube of 3x3x3 nodes to calculate the CoV, depth of 2 5x5x5, etc
     * @returns True for a succesful call, false if point requested is in unknown space
     */
    bool getNormal(const OcTreeKey& key, std::vector<point3d>& normals, const unsigned int depth=1U) const;

    //TODO integrate/averageNodeGraspQuality functions... (line 143-164 ColorOcTree file https://github.com/OctoMap/octomap/blob/ros2/octomap/include/octomap/ColorOcTree.h)

    // update inner nodes, sets grasp quality to average child grasp quality
    void updateInnerOccupancy();

    // uses gnuplot to plot a histogram in EPS format
    void writeGraspQualityHistogram(std::string filename) const;

    virtual ~OcTreeGraspQuality() {};
    
    protected:
    point3d_collection getOccupiedNeighbors(const point3d& coords, const unsigned int depth) const;

    point3d_collection getOccupiedNeighbors(const OcTreeKey& center_key, const unsigned int depth) const;
    
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
