#include "octomap_grasping/OcTreeGripper.hpp"

namespace octomap
{
    // node implementation -----------------------------
    std::ostream& OcTreeGripperNode::writeData(std::ostream &s) const
    {
        s.write((const char*) &value, sizeof(value)); // occupancy
        s.write((const char*) &is_grasping_surface, sizeof(is_grasping_surface)); // grasping surface flag
        return s;
    }

    std::istream& OcTreeGripperNode::readData(std::istream &s)
    {
        s.read((char*) &value, sizeof(value)); // occupancy
        s.read((char*) &is_grasping_surface, sizeof(is_grasping_surface)); // grasping surface flag
        return s;
    }

    void OcTreeGripperNode::updateIsGraspingSurfaceChildren() {
        this->is_grasping_surface = getAverageChildIsGraspingSurface();
    }

    bool OcTreeGripperNode::getAverageChildIsGraspingSurface() const {
        int c = 0;
        if (children != NULL){
            for (int i=0; i<8; i++) {
                OcTreeGripperNode* child = static_cast<OcTreeGripperNode*>(children[i]);
                if (child != NULL) 
                {
                    if (child->isGraspingSurface()) ++c;
                }
            }
        }
        if (c >= 4) return true; // is half or more are graspable, set as graspable
        else return false;
    }


    // tree implementation --------------------------

    OcTreeGripper::OcTreeGripper(double resolution) : OccupancyOcTreeBase<OcTreeGripperNode>(resolution), graspable_voxels{0}, grasping_normal{0,1,0}
    {
        OcTreeGripperMemberInit.ensureLinking();
    }

    OcTreeGripper::OcTreeGripper(std::string _filename) : OccupancyOcTreeBase<OcTreeGripperNode>(0.1), graspable_voxels{0} // tree resolution will be set according to tree file
    {
        OcTreeGripperMemberInit.ensureLinking();
        AbstractOcTree* tree = AbstractOcTree::read(_filename);
        OcTreeGripper* grtree = dynamic_cast<OcTreeGripper*>(tree);
        this->setResolution(grtree->getResolution());
        this->root = grtree->getRoot(); // this will recursively copy all nodes
        this->graspable_voxels = grtree->graspable_voxels;
        this->grasping_normal = grtree->grasping_normal;
        //delete tree;
    }
    
    // Type casting to ColorOcTree with 255 green set as grasping surface and 255 red as non-grasping surface
    OcTreeGripper::operator ColorOcTree () const
    {
        ColorOcTree tree{this->getResolution()};
        if (this->root) // if not NULL
        {    
            // Copy tree occupancy contents and convert GQ to color scale
            for(OcTreeGripper::iterator it = this->begin(), end=this->end(); it!= end; ++it)
            {
                // cannot use node key as it is only valid for the previous node
                point3d node_point = it.getCoordinate();
                ColorOcTreeNode* n = tree.updateNode(node_point, it->getLogOdds()+(((float)it->isGraspingSurface())*1e-4F));
                // Pruning ignores color, therefore slightly alter the log odds update to avoid pruning different node types
                if (it->isGraspingSurface()) n->setColor(0, 255, 0); // GREEN
                else n->setColor(255, 0, 0); // RED
            }
            tree.updateInnerOccupancy();
        } else {
            std::cerr << "[OcTreeGraspQuality->ColorOcTree operator] root node of original tree is NULL" << std::endl;
        }
        return tree;
    }
    
    void OcTreeGripper::importOcTree(const OcTree *octree_in)
    {
        unsigned int max_depth{0};
        unsigned int min_depth{16};

        this->clear(); // must delete all data as resetting resolution would void metric scale
        this->setResolution(octree_in->getResolution());
        // Copy tree occupancy contents
        for(OcTree::leaf_iterator it = octree_in->begin_leafs(), end=octree_in->end_leafs(); it!= end; ++it)
        {
            // cannot use node key as it is only valid for the previous node
            point3d node_point = it.getCoordinate();

            unsigned int depth_node{it.getDepth()};
            if (depth_node > max_depth) max_depth = depth_node;
            if (depth_node < min_depth) min_depth = depth_node;

            this->updateNode(node_point, it->getLogOdds(), true);
        }
        this->updateInnerOccupancy();
        if (max_depth != min_depth) std::cerr << "[importOcTree] Warning: tree leaf depth is not constant (min = " << min_depth << ", max = " << max_depth << "), pruned tree?" << std::endl;
    }

    OcTreeGripperNode* OcTreeGripper::setNodeIsGraspingSurface(const OcTreeKey& key, bool grasping_surface_flag)
    {
        OcTreeGripperNode* n = search(key);
        if (!n)
        {
            n->setIsGraspingSurface(grasping_surface_flag);
        }
        return n;
    }

    bool OcTreeGripper::pruneNode(OcTreeGripperNode* node)
    {
        if (!isNodeCollapsible(node)) return false;

        // set value to children's values (all assumed equal)
        node->copyData(*(getNodeChild(node, 0)));

        node->setIsGraspingSurface(node->getAverageChildIsGraspingSurface());

        // delete children
        for (unsigned int i=0;i<8;i++) 
        {
            deleteNodeChild(node, i);
        }
        delete[] node->children;
        node->children = NULL;

        return true;
    }

    bool OcTreeGripper::isNodeCollapsible(const OcTreeGripperNode* node) const
    {
        // all children must exist, must not have children of
        // their own and have the same occupancy probability
        if (!nodeChildExists(node, 0))
        return false;

        const OcTreeGripperNode* firstChild = getNodeChild(node, 0);
        if (nodeHasChildren(firstChild))
        return false;

        for (unsigned int i = 1; i<8; i++) {
            // compare nodes using the defined == operator for the node class
            if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) || !(getNodeChild(node, i)->operator==(*firstChild)))
                return false;
        }

        return true;
    }

    void OcTreeGripper::updateInnerOccupancy() {
        this->updateInnerOccupancyRecurs(this->root, 0);
    }

    void OcTreeGripper::updateNumGraspableVoxels()
    {
        if (!this->isChangeDetectionEnabled())
        {
            this->enableChangeDetection(true); // enable change detection
        }
        this->expand();

        if (this->numChangesDetected() || !this->graspable_voxels) // if changes detected or graspable_voxels     uninitialised (zero)
        {
            this->graspable_voxels = 0; // reset
            for(OcTreeGripper::leaf_iterator it = this->begin_leafs(), end=this->end_leafs(); it!= end; ++it)
            {
                if (it->isGraspingSurface()) this->graspable_voxels++;
            }
            this->resetChangeDetection();
        }
    }

    const unsigned long& OcTreeGripper::getNumGraspableVoxels() const
    {
        if (this->numChangesDetected() || !this->graspable_voxels) // if changes detected or graspable_voxels uninitialised (zero)
        {
            std::string reason{};
            if (this->numChangesDetected()) reason += "Changes detected. ";
            if (!this->graspable_voxels) reason += "Empty attribute. ";
            std::cerr << "[OcTreeGripper::getNumGraspableVoxels] Warning: " << reason << "Value may be outdated" << std::endl;
        }
        return graspable_voxels;
    }

    void OcTreeGripper::setOrigin(const octomap::point3d& translation)
    {
        this->expand();

        // new octree
        octomap::OcTreeGripper* new_tree = new octomap::OcTreeGripper(this->getResolution());

        // match translation to resolution steps
        const float res{(float)new_tree->getResolution()};
        const float x{std::round(translation.x()/res)*res};
        const float y{std::round(translation.y()/res)*res};
        const float z{std::round(translation.z()/res)*res};
        const octomap::point3d matched_translation{x,y,z};

        // iterate over current tree and copy transformed nodes to new tree
        for (octomap::OcTreeGripper::leaf_iterator it = this->begin_leafs(), end=this->end_leafs(); it!= end; ++it)
        {
            octomap::point3d coord{it.getCoordinate() - matched_translation};
            octomap::OcTreeGripperNode* n = new_tree->updateNode(coord, it->getLogOdds(), true);
            n->setIsGraspingSurface(it->isGraspingSurface());
        }
        new_tree->updateInnerOccupancy();
        this->root = new_tree->getRoot(); // recursively copy all nodes to *this
    }

    void OcTreeGripper::updateInnerOccupancyRecurs(OcTreeGripperNode* node, unsigned int depth) {
        // only recurse and update for inner nodes:
        if (nodeHasChildren(node)){
            // return early for last level:
            if (depth < this->tree_depth){
                for (unsigned int i=0; i<8; i++) {
                    if (nodeChildExists(node, i)) {
                        updateInnerOccupancyRecurs(getNodeChild(node, i), depth+1);
                    }
                }
            }
            node->updateOccupancyChildren();
            node->updateIsGraspingSurfaceChildren();
        }
    }

    OcTreeGripper::StaticMemberInitializer OcTreeGripper::OcTreeGripperMemberInit;

}  // namespace octomap_grasping
