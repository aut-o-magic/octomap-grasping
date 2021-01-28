#include "octomap_grasping/OcTreeGraspQuality.hpp"

namespace octomap
{
    // node implementation -----------------------------
    std::ostream& OcTreeNodeGraspQuality::writeData(std::ostream &s) const
    {
        s.write((const char*) &value, sizeof(value)); // occupancy
        s.write((const char*) &grasp_quality, sizeof(grasp_quality)); // grasp quality
        return s;
    }

    std::istream& OcTreeNodeGraspQuality::readData(std::istream &s)
    {
        s.read((char*) &value, sizeof(value)); // occupancy
        s.read((char*) &grasp_quality, sizeof(grasp_quality)); // grasp quality
        return s;
    }

    void OcTreeNodeGraspQuality::updateGraspQualityChildren() {
        grasp_quality = getAverageChildGraspQuality();
    }

    OcTreeNodeGraspQuality::GraspQuality OcTreeNodeGraspQuality::getAverageChildGraspQuality() const {
        Eigen::Vector3f normal;
        Eigen::Matrix<float, 2, ORIENTATION_STEPS> angle_quality; // orientation of the gripper and numeric grasp quality
        int c = 0;
        
        if (children != NULL){
            for (int i=0; i<8; i++) {
                OcTreeNodeGraspQuality* child = static_cast<OcTreeNodeGraspQuality*>(children[i]);

                if (child != NULL && child->isGraspQualitySet()) {
                    normal += child->getGraspQuality().normal;
                    angle_quality += child->getGraspQuality().angle_quality;
                    ++c;
                }
            }
        }

        if (c > 0) {
            normal /= c;
            angle_quality /= c;
            return OcTreeNodeGraspQuality::GraspQuality(normal, angle_quality);
        }
        else { // no child had a set grasp quality
            return OcTreeNodeGraspQuality::GraspQuality();
        }
    }


    // tree implementation --------------------------

    OcTreeGraspQuality::OcTreeGraspQuality(double resolution) : OccupancyOcTreeBase<OcTreeNodeGraspQuality>(resolution)
    {
        ocTreeGraspQualityMemberInit.ensureLinking();
    }

    ColorOcTree& OcTreeGraspQuality::toColorOcTree() const
    {
        ColorOcTree color_tree{this->getResolution()};

        // Copy tree occupancy contents and convert GQ to color scale
        for(OcTreeGraspQuality::tree_iterator it = this->begin_tree(), end=this->end_tree(); it!= end; ++it)
        {
            color_tree.updateNode(it.getKey(), true, true);

            // convert GQ to Red-Green color scale
            float max_gq = it->getGraspQuality().angle_quality.row(1).maxCoeff();
            uint16_t rg = max_gq*512;
            uint8_t r = std::max(255-rg,0);
            uint8_t g = std::max(rg-256,0);
            color_tree.setNodeColor(it.getKey(), r, g, 0);
        }
        color_tree.updateInnerOccupancy();
        return color_tree;
    }
    
    OcTreeGraspQuality::operator ColorOcTree () const
    {
        return this->toColorOcTree();
    }

    OcTreeNodeGraspQuality* OcTreeGraspQuality::setNodeGraspQuality(const OcTreeKey& key, Eigen::Vector3f& _normal, Eigen::Matrix<float, 2, ORIENTATION_STEPS>& _angle_quality)
    {
        OcTreeNodeGraspQuality* n = search(key);
        if (n!=0)
        {
            n->setGraspQuality(_normal, _angle_quality);
        }
        return n;
    }

    bool OcTreeGraspQuality::pruneNode(OcTreeNodeGraspQuality* node)
    {
        if (!isNodeCollapsible(node)) return false;

        // set value to children's values (all assumed equal)
        node->copyData(*(getNodeChild(node, 0)));

        if (node->isGraspQualitySet())
            node->setGraspQuality(node->getAverageChildGraspQuality());

        // delete children
        for (unsigned int i=0;i<8;i++) 
        {
            deleteNodeChild(node, i);
        }
        delete[] node->children;
        node->children = NULL;

        return true;
    }

    bool OcTreeGraspQuality::isNodeCollapsible(const OcTreeNodeGraspQuality* node) const
    {
        // all children must exist, must not have children of
        // their own and have the same occupancy probability
        if (!nodeChildExists(node, 0))
        return false;

        const OcTreeNodeGraspQuality* firstChild = getNodeChild(node, 0);
        if (nodeHasChildren(firstChild))
        return false;

        for (unsigned int i = 1; i<8; i++) {
            // compare nodes only using their occupancy, ignoring color for pruning
            if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) || !(getNodeChild(node, i)->getValue() == firstChild->getValue()))
                return false;
        }

        return true;
    }

    void OcTreeGraspQuality::writeGraspQualityHistogram(std::string filename)
    {
        #ifdef _MSC_VER
            fprintf(stderr, "The color histogram uses gnuplot, this is not supported under windows.\n");
        #else
            // build RGB histogram
            std::vector<int> histogram_r (256,0);
            std::vector<int> histogram_g (256,0);
            std::vector<int> histogram_b (256,0);
            ColorOcTree color_tree{this->toColorOcTree()};
            for(ColorOcTree::tree_iterator it = color_tree.begin_tree(),
                end=color_tree.end_tree(); it!= end; ++it) {
            if (!it.isLeaf() || !color_tree.isNodeOccupied(*it)) continue;
            ColorOcTreeNode::Color& c = it->getColor();
            ++histogram_r[c.r];
            ++histogram_g[c.g];
            ++histogram_b[c.b];
            }
            // plot data
            FILE *gui = popen("gnuplot ", "w");
            fprintf(gui, "set term postscript eps enhanced color\n");
            fprintf(gui, "set output \"%s\"\n", filename.c_str());
            fprintf(gui, "plot [-1:256] ");
            fprintf(gui,"'-' w filledcurve lt 1 lc 1 tit \"r\",");
            fprintf(gui, "'-' w filledcurve lt 1 lc 2 tit \"g\",");
            fprintf(gui, "'-' w filledcurve lt 1 lc 3 tit \"b\",");
            fprintf(gui, "'-' w l lt 1 lc 1 tit \"\",");
            fprintf(gui, "'-' w l lt 1 lc 2 tit \"\",");
            fprintf(gui, "'-' w l lt 1 lc 3 tit \"\"\n");

            for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_r[i]);
            fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
            for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_g[i]);
            fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
            for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_b[i]);
            fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
            for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_r[i]);
            fprintf(gui, "e\n");
            for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_g[i]);
            fprintf(gui, "e\n");
            for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_b[i]);
            fprintf(gui, "e\n");
            fflush(gui);
        #endif
    }

    void OcTreeGraspQuality::updateInnerOccupancy() {
        this->updateInnerOccupancyRecurs(this->root, 0);
    }

    void OcTreeGraspQuality::updateInnerOccupancyRecurs(OcTreeNodeGraspQuality* node, unsigned int depth) {
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
            node->updateGraspQualityChildren();
        }
    }

    OcTreeGraspQuality::StaticMemberInitializer OcTreeGraspQuality::ocTreeGraspQualityMemberInit;

}  // namespace octomap_grasping
