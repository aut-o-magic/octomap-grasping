#include "octomap_grasping/OcTreeGripper.hpp"

namespace octomap
{
    // node implementation -----------------------------
    std::ostream& OcTreeNodeGripper::writeData(std::ostream &s) const
    {
        s.write((const char*) &value, sizeof(value)); // occupancy
        s.write((const char*) &is_grasping_surface, sizeof(is_grasping_surface)); // grasping surface flag
        return s;
    }

    std::istream& OcTreeNodeGripper::readData(std::istream &s)
    {
        s.read((char*) &value, sizeof(value)); // occupancy
        s.read((char*) &is_grasping_surface, sizeof(is_grasping_surface)); // grasping surface flag
        return s;
    }

    void OcTreeNodeGripper::updateIsGraspingSurfaceChildren() {
        this->is_grasping_surface = getAverageChildIsGraspingSurface();
    }

    bool OcTreeNodeGripper::getAverageChildIsGraspingSurface() const {
        int c = 0;
        if (children != NULL){
            for (int i=0; i<8; i++) {
                OcTreeNodeGripper* child = static_cast<OcTreeNodeGripper*>(children[i]);
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

    OcTreeGripper::OcTreeGripper(double resolution) : OccupancyOcTreeBase<OcTreeNodeGripper>(resolution)
    {
        OcTreeGripperMemberInit.ensureLinking();
    }

    ColorOcTree OcTreeGripper::toColorOcTree() const
    {
        ColorOcTree color_tree{this->getResolution()};

        // Copy tree occupancy contents and convert grasping surface flag to Red/Green
        for(OcTreeGripper::leaf_iterator it = this->begin_leafs(), end=this->end_leafs(); it!= end; ++it)
        {
            // cannot use node key as it is only valid for the previous node
            point3d node_point = it.getCoordinate();
            color_tree.updateNode(node_point, true, true);

            if (it->isGraspingSurface()) color_tree.setNodeColor(node_point.x(), node_point.y(), node_point.z(), 0, 255, 0);
            else color_tree.setNodeColor(node_point.x(), node_point.y(), node_point.z(), 255, 0, 0);
        }
        color_tree.updateInnerOccupancy();        
        //color_tree.writeColorHistogram("colorhistogram");
        return color_tree;
    }
    
    OcTreeGripper::operator ColorOcTree () const
    {
        return this->toColorOcTree();
    }

    OcTreeNodeGripper* OcTreeGripper::setNodeIsGraspingSurface(const OcTreeKey& key, bool grasping_surface_flag)
    {
        OcTreeNodeGripper* n = search(key);
        if (n!=0)
        {
            n->setIsGraspingSurface(grasping_surface_flag);
        }
        return n;
    }

    bool OcTreeGripper::pruneNode(OcTreeNodeGripper* node)
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

    bool OcTreeGripper::isNodeCollapsible(const OcTreeNodeGripper* node) const
    {
        // all children must exist, must not have children of
        // their own and have the same occupancy probability
        if (!nodeChildExists(node, 0))
        return false;

        const OcTreeNodeGripper* firstChild = getNodeChild(node, 0);
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

    void OcTreeGripper::updateInnerOccupancyRecurs(OcTreeNodeGripper* node, unsigned int depth) {
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
