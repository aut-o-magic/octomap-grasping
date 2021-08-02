# octomap-grasping
[Octomap](https://octomap.github.io/) derived classes for grasp planning operations. Include useful utilities such as importers from base octree objects, type casting to a visualisable tree (color-coded), and others.

It consists of two classes:
- `OcTreeGraspQuality`: Class for target objects. Nodes contain a grasp quality array with its numeric score and associated orientation 
- `OcTreeGripper`: Class for grippers. Nodes can be tagged as _grasping surfaces_ or not, which dictate whether a contact with a voxel of the target object there is desirable or not.
