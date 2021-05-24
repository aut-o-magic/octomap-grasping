#include "octomap_grasping/OcTreeGraspQuality.hpp"

namespace octomap
{
    // node implementation -----------------------------
    std::ostream& OcTreeGraspQualityNode::writeData(std::ostream &s) const
    {
        s.write((const char*) &value, sizeof(value)); // occupancy
        s.write((const char*) &grasp_quality, sizeof(grasp_quality)); // grasp quality
        return s;
    }

    std::istream& OcTreeGraspQualityNode::readData(std::istream &s)
    {
        s.read((char*) &value, sizeof(value)); // occupancy
        s.read((char*) &grasp_quality, sizeof(grasp_quality)); // grasp quality
        return s;
    }

    void OcTreeGraspQualityNode::updateGraspQualityChildren() {
        this->grasp_quality = getAverageChildGraspQuality();
    }

    OcTreeGraspQualityNode::GraspQuality OcTreeGraspQualityNode::getAverageChildGraspQuality() const {
        Eigen::Matrix<float, 2, ORIENTATION_STEPS> angle_quality; // orientation of the gripper and numeric grasp quality
        int c = 0;
        
        if (children){ // if not NULL
            for (int i=0; i<8; i++) {
                OcTreeGraspQualityNode* child = static_cast<OcTreeGraspQualityNode*>(children[i]);

                if (child && child->isGraspQualitySet()) { // if child not NULL && grasp quality is meaningful
                    angle_quality += child->getGraspQuality().angle_quality;
                    ++c;
                }
            }
        }

        if (c > 0) {
            angle_quality /= c;
            return OcTreeGraspQualityNode::GraspQuality{angle_quality};
        }
        else { // no child had a set grasp quality
            return OcTreeGraspQualityNode::GraspQuality{};
        }
    }

    // ! Not used nor tested
    OcTreeGraspQualityNode::operator ColorOcTreeNode() const
    {
        ColorOcTreeNode color_node{};
        // convert GQ to Red-Green color scale
        float max_gq = this->getGraspQuality().angle_quality.row(1).maxCoeff();
        uint16_t rg = max_gq*512;
        uint8_t r = std::max(255-rg,0);
        uint8_t g = std::max(rg-256,0);
        color_node.setColor(r, g, 0);
        color_node.setLogOdds(this->getLogOdds());

        return color_node;
    }

    // tree implementation --------------------------

    OcTreeGraspQuality::OcTreeGraspQuality(double resolution) : OccupancyOcTreeBase<OcTreeGraspQualityNode>(resolution)
    {
        ocTreeGraspQualityMemberInit.ensureLinking();
    }

    OcTreeGraspQuality::OcTreeGraspQuality(std::string _filename) : OccupancyOcTreeBase<OcTreeGraspQualityNode>(0.1) // resolution will be set according to tree file
    {
        ocTreeGraspQualityMemberInit.ensureLinking();
        AbstractOcTree* tree = AbstractOcTree::read(_filename);
        OcTreeGraspQuality* gqtree = dynamic_cast<OcTreeGraspQuality*>(tree);
        this->setResolution(gqtree->getResolution());
        this->root = gqtree->getRoot(); // this will recursively copy all nodes
        //delete tree; // octomap delays with pointer management on the backend?
    }
    
    // Type casting to ColorOcTree with 255 green set as perfect grasp quality and 255 red as zero grasp quality. GQ in range [0,1]
    OcTreeGraspQuality::operator ColorOcTree () const
    {
        ColorOcTree tree{this->getResolution()};
        if (this->root) // if not NULL
        {    
            // temp tree as modification (tree expansion) is needed for type casting operation
            OcTreeGraspQuality* temp_tree = new OcTreeGraspQuality(this->getResolution());
            temp_tree->root = this->getRoot(); // this will recursively copy all children
            temp_tree->expand();
            
            // Copy tree occupancy contents and convert GQ to color scale
            for(OcTreeGraspQuality::leaf_iterator it = temp_tree->begin_leafs(), end=temp_tree->end_leafs(); it!= end; ++it)
            {
                // cannot use node key as it is only valid for the previous node
                point3d node_point = it.getCoordinate();
                ColorOcTreeNode* n = tree.updateNode(node_point, it->getLogOdds(), true);

                // convert GQ to Red-(Yellow)-Green color scale
                float max_gq = it->getGraspQuality().angle_quality.row(1).maxCoeff();
                uint16_t rg = max_gq*512;
                uint8_t r = std::min(std::max(512-rg,0),255); // Color channels bound between [0,255]
                uint8_t g = std::min(std::max(rg-51,0),255); // ? Yellow level shifted to ~10% grasp quality, provides much clearer/nicer visualisation
                n->setColor(r, g, 0);
            }
            tree.updateInnerOccupancy();
        } else {
            std::cerr << "[OcTreeGraspQuality->ColorOcTree operator] root node of original tree is NULL" << std::endl;
        }
        return tree;
    }

    void OcTreeGraspQuality::importOcTree(const OcTree *octree_in)
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

            OcTreeGraspQualityNode* n = this->updateNode(node_point, true, true);
            if (n->isGraspQualitySet())
            {
                std::cerr << "[importOcTree] ERROR: grasp quality should not be set on newly created octree" << std::endl;
            }
        }
        this->updateInnerOccupancy();
        //std::cout << "max_depth=" << max_depth << std::endl << "min_depth=" << min_depth <<std::endl; // debug print
    }

    OcTreeGraspQualityNode* OcTreeGraspQuality::setNodeGraspQuality(const OcTreeKey& key, Eigen::Matrix<float, 2, ORIENTATION_STEPS>& _angle_quality)
    {
        OcTreeGraspQualityNode* n = search(key);
        if (n) // if not null
        {
            n->setGraspQuality(_angle_quality);
        }
        return n;
    }

    bool OcTreeGraspQuality::pruneNode(OcTreeGraspQualityNode* node)
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

    bool OcTreeGraspQuality::isNodeCollapsible(const OcTreeGraspQualityNode* node) const
    {
        // all children must exist, must not have children of
        // their own and have the same occupancy probability
        if (!nodeChildExists(node, 0))
        return false;

        const OcTreeGraspQualityNode* firstChild = getNodeChild(node, 0);
        if (nodeHasChildren(firstChild))
        return false;

        for (unsigned int i = 1; i<8; i++) {
            // compare nodes using the defined == operator for the node class
            if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) || !(getNodeChild(node, i)->operator==(*firstChild)))
                return false;
        }

        return true;
    }
    // ! Not working well
    void OcTreeGraspQuality::writeGraspQualityHistogram(std::string filename) const
    {
        #ifdef _MSC_VER
            fprintf(stderr, "The color histogram uses gnuplot, this is not supported under windows.\n");
        #else
            // build RGB histogram
            std::vector<int> histogram_r (256,0);
            std::vector<int> histogram_g (256,0);
            std::vector<int> histogram_b (256,0);
            ColorOcTree color_tree{(ColorOcTree)*this};
            for(ColorOcTree::leaf_iterator it = color_tree.begin_leafs(),
                end=color_tree.end_leafs(); it!= end; ++it) {
            if (!color_tree.isNodeOccupied(*it)) continue;
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

    point3d_collection OcTreeGraspQuality::getOccupiedNeighbors(const point3d& coords, const unsigned int depth=1U) const
    {
        OcTreeKey center_key;
        if (!this->coordToKeyChecked(coords, this->getTreeDepth(), center_key)) 
        {
            std::cerr << "[getOccupiedNeighbors] Error: Point requested not within octree" << std::endl;
            return point3d_collection{};
        }
        return getOccupiedNeighbors(center_key, depth);
    }

    point3d_collection OcTreeGraspQuality::getOccupiedNeighbors(const OcTreeKey& center_key, const unsigned int depth=1U) const
    {
        point3d_collection neighbors;
        OcTreeKey k;
        for (k[0] = center_key[0]-depth; k[0] <= center_key[0]+depth; ++k[0]){
            for (k[1] = center_key[1]-depth; k[1] <= center_key[1]+depth; ++k[1]){
                for (k[2] = center_key[2]-depth; k[2] <= center_key[2]+depth; ++k[2]){
                    OcTreeGraspQualityNode* node = this->search(k, this->getTreeDepth());
                    if (node && this->isNodeOccupied(*node))
                    {
                        neighbors.push_back(this->keyToCoord(k));
                    }
                }
            }
        }
        if (neighbors.size() > std::pow(1.0F+((float)depth)*2.0F, 3.0F)) std::cerr << "[getOccupiedNeighbors] Error: Neighbors collection larger than theoretical maximum (size=" << neighbors.size() << ", max=" << std::pow(1.0F+((float)depth)*2.0F, 3.0F) << ")" << std::endl;
        //std::cout << "size=" << neighbors.size() << ", max=" << std::pow(1.0F+((float)depth)*2.0F, 3.0F) << ")" << std::endl;
        return neighbors;
    }

    bool OcTreeGraspQuality::getNormal(const point3d& coords, std::vector<point3d>& normals, const unsigned int depth) const
    {
        return getNormal(this->coordToKey(coords, this->getTreeDepth()), normals, depth);
    }

    bool OcTreeGraspQuality::getNormal(const OcTreeKey& key, std::vector<point3d>& normals, const unsigned int depth) const
    {
        if (!this->search(key)) return false;
        const point3d_collection neighbors = getOccupiedNeighbors(key, depth);
        if (neighbors.size() == std::pow(1.0F+((float)depth)*2.0F, 3.0F) || neighbors.empty()) // if completely full or empty neighbors region, it's not part of a surface and there's no normal. Early return
        {
            return true;
        }

        point3d cov{0.0F, 0.0F, 0.0F}; // center of volume

        // calculate CoV
        for (point3d_collection::const_iterator p = neighbors.cbegin(); p != neighbors.cend(); ++p)
        {
            cov += *p;
        }
        cov /= neighbors.size();

        // cast vector from center coords to CoV of region, which is the normal
        point3d normal{(cov-this->keyToCoord(key)).normalized()};
        if (normal.norm() < 0.99) // if the CoV is the same as the selected point, because of symmetric neighbors
        {
            // use the original (generic) marching cubes algorithm
            getNormals(this->keyToCoord(key), normals, false);
        }
        else normals.push_back(normal);

        return true;
    }

    void OcTreeGraspQuality::updateInnerOccupancy() {
        this->updateInnerOccupancyRecurs(this->root, 0);
    }

    void OcTreeGraspQuality::updateInnerOccupancyRecurs(OcTreeGraspQualityNode* node, unsigned int depth) {
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

}  // namespace octomap
