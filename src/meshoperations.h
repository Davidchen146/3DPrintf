#ifndef MESHOPERATIONS_H
#define MESHOPERATIONS_H

#include "src/mesh.h"
#include "Eigen/StdVector"

#include <igl/exact_geodesic.h>
#include <Eigen/Core>

#include <unordered_map>
#include <limits>
#include <numbers>

using namespace Eigen;
using namespace std;

class MeshOperations
{

public:
    MeshOperations(Mesh m);

    // Computes global shortest path between faces using geodesic distance
    void preprocess();
    void geodesicDistance();

    // Computes local angular distance between pairs of adjacent faces
    void angularDistance();

    // Distances between arbitrary faces
    void visualize(std::vector<std::unordered_set<int>>& coloringGroups);

    void weightedDistance();
    void calculateAvgDistances();
    void makeAdjacency();
    double getGeodesicDistance(int i, int j);
    double getWeightedDistance(int i, int j);
    Eigen::VectorXd dijkstra(int start);

    // Sets parameters for various operations
    void setPreprocessingParameters(double geodesic_weight = 0.1,
                                    double convex_coeff = 0.05,
                                    double concave_coeff = 1);
    void setOversegmentationParameters(int num_seed_faces = 0,
                                       double proportion_seed_faces = 0.1,
                                       double bounding_box_coeff = 0.01,
                                       int num_iterations = 3,
                                       bool seeds_only = false);
    void setInitialSegmentationParameters(int num_random_dir_samples = 512,
                                          double printer_tolerance_angle = 55,
                                          double ambient_occlusion_supports_alpha = 0.5,
                                          double ambient_occlusion_smoothing_alpha = 0.5,
                                          double smoothing_width_t = 0.3);

    // Oversegmentation: returns list of lists of faces
    // Each list of faces represents a connected patch (to be merged and assigned a printing direction)
    void generateOversegmentation(std::vector<std::unordered_set<int>> &patches);

    // Initial segmentation: returns list of lists of faces and printing directions
    // Takes in an initial list of lists representing patches from oversegmentation
    // Each element in the output list represents a printable component
    // The other list stores the corresponding printing direction (the ith component is printed in direction i)
    void generateInitialSegmentation(const std::vector<std::unordered_set<int>> &patches,
                                     std::vector<std::unordered_set<int>> &printable_components,
                                     std::vector<Eigen::Vector3f> &printing_directions);

private:
    Mesh _mesh;
    std::vector<Eigen::Vector3f> _vertices;
    std::vector<Eigen::Vector3i> _faces;
  
    int numVertices;
    int numFaces;
    // Also the number of faces, I think?
    int _n;
  
    std::vector<std::vector<bool>> _adjacency;

    MatrixXf _V;
    MatrixXi _F;
    Eigen::MatrixXd _geodesicDistances;
    Eigen::MatrixXd _angularDistances;
    double _avgGeodesic;
    double _avgAngular;

    // Subroutines used for Phase 1 (Oversegmentation)
    // Initial seed computation
    void sampleRandomFaces(std::unordered_set<int> &faces, int n);
    void generateInitialSeeds(std::unordered_set<int> &seeds);

    // Iterative steps to generate oversegmentation
    void generatePatches(const std::unordered_set<int> &seeds, std::vector<std::unordered_set<int>> &patches);
    void recenterSeeds(const std::vector<std::unordered_set<int>> &patches, std::unordered_set<int> &new_seeds);

    // Subroutines used for Phase 2 (Initial Segmentation)
    // Sample random directions
    void sampleRandomDirections(std::vector<Eigen::Vector3f> &directions);
    // Determine if a face should be supported
    bool isFaceSupported(const int face, const Eigen::Vector3f &direction, const std::vector<std::unordered_set<int>> &patches);
    bool isFaceOverhanging(const int face, const Eigen::Vector3f &direction);
    bool isEdgeOverhanging(const std::pair<int, int> &edge, const Eigen::Vector3f &direction);
    bool isVertexOverhanging(const int vertex, const Eigen::Vector3f &direction);
    bool isFaceFooted(const int face, const Eigen::Vector3f &direction, const std::vector<std::unordered_set<int>> &patches);
    // Compute support coefficient for a face in direction
    double computeSupportCoefficient(const int face, const Eigen::Vector3f &direction,
                                     const std::vector<std::unordered_set<int>> &patches);
    // Compute smoothing coefficient between two sets of faces
    double computeSmoothingCoefficient(const std::vector<std::unordered_set<int>> &patch_one,
                                       const std::vector<std::unordered_set<int>> &patch_two);
    // Interface to ILP
    void assignPrintingDirections(const std::vector<std::vector<int>> &patches,
                                  const std::vector<Eigen::Vector3f> &printing_directions,
                                  std::vector<Eigen::Vector3f> &patch_printing_directions);
    // Assign results of the ILP to something we can return out
    void generatePrintableComponents(const std::vector<std::vector<int>> &patches,
                                     std::vector<unordered_set<int>> &printable_components,
                                     const std::vector<Eigen::Vector3f> &patch_printing_directions,
                                     std::vector<Eigen::Vector3f> &component_printing_directions);

    // Other general subroutines
/*-------------------------------------------------------------------------------------------------*/
    // Normals
    Eigen::Vector3f getFaceNormal(const int &face);
    Eigen::Vector3f getEdgeNormal(const std::pair<int, int> &edge);
    Eigen::Vector3f getVertexNormal(const int &vertex);

    // Ambient Occlusion
    // Could cache this as a preprocessing step?
    double getFaceAmbientOcclusion(const int &face);
    double getEdgeAmbientOcclusion(const std::pair<int, int> &edge);

    // For random direction generation
    Eigen::Vector3f generateRandomVector();

    // For determining intersections with other faces
    // Should use BVH, some other structure, or there might be something in libigl/VCGlib we can use
    // If using BVH, may need to make BVH initialization a preprocessing step
    int getIntersection(const Eigen::Vector3f &ray_position, const Eigen::Vector3f ray_direction);

    // Gets intersection of edges (or faces, TBD) between two patches
    void getBoundaryEdges(const std::unordered_set<int> &patch_one, const std::unordered_set<int> &patch_two);

    // Basic utility functions for faces
    // TODO: Change these functions to do lookups to values stored in the face struct
    Eigen::Vector3f getCentroid(const int &face);
    double getArea(const int &face);

    // Distances to sets of points
    std::pair<double, int> getMinGeodesicDistanceToSet(const int &face, const std::unordered_set<int> &faces, bool include_self = false);
    std::pair<double, int> getMinWeightedDistanceToSet(const int &face, const std::unordered_set<int> &faces, bool include_self = false);
    std::pair<double, int> getMaxGeodesicDistanceToSet(const int &face, const std::unordered_set<int> &faces);
    std::pair<double, int> getMaxWeightedDistanceToSet(const int &face, const std::unordered_set<int> &faces);
    double getTotalGeodesicDistanceToSet(const int &face, const std::unordered_set<int> &faces);
    double getTotalWeightedDistanceToSet(const int &face, const std::unordered_set<int> &faces);


    double bbd; // bounding box diagonal
    Eigen::MatrixXd _weightedDistances;

    // Configurable parameters
    // Preprocessing parameters
    double _geodesic_distance_weight;
    double _convex_coeff;
    double _concave_coeff;

    // Oversegmentation parameters
    int _num_seed_faces;
    double _proportion_seed_faces;
    double _oversegmentation_bounding_box_coeff;
    int _num_oversegmentation_iterations;
    bool _seeds_only;

    // Initial Segmentation parameters
    int _num_random_dir_samples;
    double _printer_tolerance_angle; // Measured in degrees
    double _ambient_occlusion_supports_alpha;
    double _ambient_occlusion_smoothing_alpha;
    double _smoothing_width_t;
};

#endif // MESHOPERATIONS_H
