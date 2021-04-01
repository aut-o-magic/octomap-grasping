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

    OcTreeGripper::OcTreeGripper(double resolution) : OccupancyOcTreeBase<OcTreeGripperNode>(resolution), pointing_to_target_surface_normal{0,0,1}
    {
        OcTreeGripperMemberInit.ensureLinking();
    }
    
    // Type casting to ColorOcTree with 255 green set as grasping surface and 255 red as non-grasping surface
    OcTreeGripper::operator ColorOcTree () const
    {
        ColorOcTree tree{this->getResolution()};
        if (this->root) // if not NULL
        {    
            // temp tree as modification (tree expansion) is needed for type casting operation
            OcTreeGripper* temp_tree{new OcTreeGripper(this->getResolution())}; // calling delete on the raw pointer leads to seg fault, I suspect something weird with octomap's library implementation?
            temp_tree->root = this->getRoot(); // this will recursively copy all children
            temp_tree->expand();

            // Copy tree occupancy contents and convert GQ to color scale
            for(OcTreeGripper::leaf_iterator it = temp_tree->begin_leafs(), end=temp_tree->end_leafs(); it!= end; ++it)
            {
                // cannot use node key as it is only valid for the previous node
                point3d node_point = it.getCoordinate();
                ColorOcTreeNode* n = tree.updateNode(node_point, true); // nodes auto-prune

                if (it->isGraspingSurface()) n->setColor(0, 255, 0); // GREEN
                else n->setColor(255, 0, 0); // RED
            }
            tree.updateInnerOccupancy();
        } else {
            std::cerr << "[OcTreeGraspQuality->ColorOcTree operator] root node of original tree is NULL" << std::endl;
        }
        return tree;
    }
    
    void OcTreeGripper::importOcTree(OcTree octree_in)
    {
        // ! debug variables, no need to disable as their overhead is negligible
        unsigned int max_depth{0};
        unsigned int min_depth{16};
 
        octree_in.expand(); // expand all nodes to have all leafs at the highest depth

        this->clear(); // must delete all data as resetting resolution would void metric scale
        this->setResolution(octree_in.getResolution());
        // Copy tree occupancy contents
        for(OcTree::leaf_iterator it = octree_in.begin_leafs(), end=octree_in.end_leafs(); it!= end; ++it)
        {
            // cannot use node key as it is only valid for the previous node
            point3d node_point = it.getCoordinate();

            unsigned int depth_node{it.getDepth()};
            if (depth_node > max_depth) max_depth = depth_node;
            if (depth_node < min_depth) min_depth = depth_node;

            this->updateNode(node_point, true);
        }
        //std::cout << "max_depth=" << max_depth << std::endl << "min_depth=" << min_depth <<std::endl; // ! debug print
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
            // compare nodes only using their occupancy, ignoring color for pruning
            if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) || !(getNodeChild(node, i)->getValue() == firstChild->getValue()))
                return false;
        }

        return true;
    }

    void OcTreeGripper::updateInnerOccupancy() {
        this->updateInnerOccupancyRecurs(this->root, 0);
    }

    void OcTreeGripper::setGripperOrientation(Eigen::Vector3f& vector)
    {
        this->pointing_to_target_surface_normal = vector;
    }

    Eigen::Vector3f OcTreeGripper::getGripperOrientation() const
    {
        return this->pointing_to_target_surface_normal;
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
