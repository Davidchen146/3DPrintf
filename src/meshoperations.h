#ifndef MESHOPERATIONS_H
#define MESHOPERATIONS_H

#include "src/mesh.h"
#include "Eigen/StdVector"
#include "ortools/linear_solver/linear_solver.h"

#include <igl/exact_geodesic.h>
#include <Eigen/Core>
#include <igl/embree/ambient_occlusion.h>
#include <igl/embree/EmbreeIntersector.h>
#include <igl/Hit.h>

#include <unordered_map>
#include <limits>
#include <numbers>

// TODO: Reorganize this file into multiple subclasses for each phases along with common util functions
// TODO: Mesh class, preprocessing, oversegmentation, initial segmentation, refined segmentation, and fabrication should all have different classes
// TODO: Get rid of the namespace; that's lazy
using namespace Eigen;
using namespace std;
using namespace operations_research;

// Custom hash function for Vector3i
struct Vector3iHash {
    size_t operator()(const Eigen::Vector3i& v) const {
        // Compute hash based on elements
        return std::hash<int>()(v[0]) ^ std::hash<int>()(v[1]) ^ std::hash<int>()(v[2]);
    }
};

// Custom equality comparator for Vector3i
struct Vector3iEqual {
    bool operator()(const Eigen::Vector3i& lhs, const Eigen::Vector3i& rhs) const {
        // Sort the elements of both vectors
        Eigen::Vector3i sorted_lhs = lhs;
        Eigen::Vector3i sorted_rhs = rhs;
        std::sort(sorted_lhs.begin(), sorted_lhs.end());
        std::sort(sorted_rhs.begin(), sorted_rhs.end());

        // Compare sorted vectors for equality
        return sorted_lhs == sorted_rhs;
    }
};

class MeshOperations
{

public:
    MeshOperations(Mesh m);

    // Data preprocessing
    void preprocessData();
    void preprocessDistances();
    void preprocessRaytracer();
    void preprocessZeroCostFaces();
    void preprocessSolver();

    // Computes global shortest path between faces using geodesic distance
    void geodesicDistance();

    // Computes local angular distance between pairs of adjacent faces
    void angularDistance();

    // Distances between arbitrary faces
    void weightedDistance();
    void calculateAvgDistances();
    void makeAdjacency();
    double getGeodesicDistance(int i, int j);
    double getWeightedDistance(int i, int j);
    double getAngularDistance(int i, int j);
    Eigen::VectorXd dijkstra(int start);

    // Sets parameters for various operations
    void setPreprocessingParameters(double geodesic_weight = 0.1,
                                    double convex_coeff = 0.05,
                                    double concave_coeff = 1,
                                    bool use_zero_cost_faces = false);
    void setOversegmentationParameters(int num_seed_faces = 0,
                                       double proportion_seed_faces = 0.1,
                                       double bounding_box_coeff = 0.01,
                                       int num_iterations = 3,
                                       bool seeds_only = false,
                                       bool skip_visualization = false);
    void setInitialSegmentationParameters(int num_random_dir_samples = 512,
                                          double printer_tolerance_angle = 55,
                                          double ambient_occlusion_supports_alpha = 0.5,
                                          double ambient_occlusion_smoothing_alpha = 0.5,
                                          double smoothing_width_t = 0.3,
                                          int ambient_occlusion_samples = 500,
                                          int footing_samples = 1,
                                          bool axis_only = false,
                                          bool skip_visualization = false);
    void setRefinedSegmentationParameters(double e_fuzzy = 0.02,
                                          double ambient_occlusion_lambda = 4,
                                          bool skip_visualization = false);

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

    // Visualization routines
    void visualize(const std::vector<std::unordered_set<int>>& coloringGroups);
    void visualizePrintableComponents(const std::vector<std::unordered_set<int>> &printable_components, const std::vector<Eigen::Vector3f> &printing_directions);

    // Debug options for visualization
    void generateRefinedSegmentation(std::vector<std::unordered_set<int>> &printable_components,
                                     std::vector<Eigen::Vector3f> &printing_directions,
                                     std::vector<std::unordered_set<int>> &fuzzyRegions);
    void visualizeFaceAO();
    void visualizeEdgeAO();
    void visualizeAngularDistance();
    void visualizeWeightedDistance();
    void visualizeSupportCosts(const Eigen::Vector3f &printing_direction);
    void visualizeSmoothingCosts(const std::vector<std::unordered_set<int>>& patches);
    void visualizePrintableVolumes(const std::vector<std::unordered_set<int>> &printable_components,
                                   const std::vector<Eigen::Vector3f> &printing_directions,
                                   const std::vector<std::vector<Eigen::Vector4i>> &printable_volumes);
    // void visualizePrintableVolume(std::vector<Eigen::Vector3i> &surface_faces, int groupNum, Eigen::Vector3f printing_direction);

    // Random direction visualization
    Eigen::Vector3f generateRandomVector();

    // Fabrication step
    void tetrahedralizeMesh();
    Eigen::Vector3d computeTetCentroid(Eigen::Vector4i &tetrahedron);
    void getComponentBoundingBox(const std::unordered_set<int> &component, Eigen::Vector3d& min, Eigen::Vector3d& max);
    void partitionVolume(const std::vector<std::unordered_set<int>> &printable_components,
                         std::vector<std::vector<Eigen::Vector4i>> &printable_volumes);
    void updateFaceMap(std::unordered_map<Vector3i, Vector4i, Vector3iHash, Vector3iEqual>& faceMap, Vector3i face, Vector4i tet);
    Vector3i orderVertices(Vector3i& face, Vector4i& tet);
    void extractSurface(const std::vector<Eigen::Vector4i> &volume, std::vector<Eigen::Vector3i> &surface_faces);

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

    // initialized via call to tetrahedralizeMesh()
    MatrixXd _TV;
    MatrixXi _TT;
    MatrixXi _TF;

    Eigen::MatrixXd _geodesicDistances;
    Eigen::MatrixXd _angularDistances;
    double _avgGeodesic;
    double _avgAngular;

    // for visualization routines
    // mostly so that the colors of the patches from initial segmentation step
    // will match the colors of the patches from fabrication
    std::unordered_map<int, int> faceToGroup;
    std::unordered_map<int, Vector3d> groupToColor;

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
    bool isFaceOverhanging(const int face, const Eigen::Vector3f &direction);
    bool isEdgeOverhanging(const std::pair<int, int> &edge, const Eigen::Vector3f &direction);
    bool isVertexOverhanging(const int vertex, const Eigen::Vector3f &direction);
    bool isFaceFooted(const int face, const Eigen::Vector3f &direction, const std::vector<std::unordered_set<int>> &patches);
    void populateSupportMatrix(const std::vector<std::unordered_set<int>> &patches,
                               std::vector<Eigen::Vector3f> &directions);
    // For a supported face, find its footing faces (if any)
    void findFootingFaces(const int face, const Eigen::Vector3f &direction, std::vector<int> &footing_faces);
    void populateSmoothingMatrix(const std::vector<std::unordered_set<int>> &patches);
    // Compute support coefficient for a face in direction
    double computeSupportCoefficient(const int &face);
    // Compute smoothing coefficient between two sets of faces
    double computeSmoothingCoefficient(const std::unordered_set<int> &patch_one,
                                       const std::unordered_set<int> &patch_two);
    // Assign results of the ILP to something we can return out
    void generatePrintableComponents(const std::vector<std::unordered_set<int>> &patches,
                                     std::vector<unordered_set<int>> &printable_components,
                                     const std::vector<std::vector<const MPVariable*>> &solutions,
                                     const std::vector<Eigen::Vector3f> &patch_printing_directions,
                                     std::vector<Eigen::Vector3f> &component_printing_directions);

    // Phase 3 (Refined Segmentation)
    void generateFuzzyRegions(std::vector<std::unordered_set<int>> &printable_components,
                              std::vector<Eigen::Vector3f> &printing_directions,
                              std::vector<FuzzyNode*> &nodes);
    // Using results generateFuzzyRegions, make graphs connecting regions
    void makeFuzzyGraph(std::vector<FuzzyNode*> &nodes);
    // combine nodes into connected fuzzy regions
    void combineFuzzyRegions(std::vector<FuzzyNode*> &nodes,
                             std::vector<std::unordered_set<int>> &fuzzyRegions,
                             std::vector<std::unordered_set<int>> &fuzzyRegionDirections);
    double computeRefinedCoefficient(const int &face_one, const int &face_two);
    void initializeFuzzyRegionCoefficients(const std::unordered_set<int> &fuzzy_region, std::unordered_map<std::pair<int, int>, double, PairHash> &adjacent_face_coefficients);
    void solveFuzzyRegion(std::vector<std::unordered_set<int>> &printable_components,
                          const std::unordered_set<int> &fuzzy_region,
                          const unordered_set<int> &fuzzy_region_directions,
                          const std::unordered_map<std::pair<int, int>, double, PairHash> &adjacent_face_coefficients);
    void addRefinedFaceVariable(const int &face,
                                const std::unordered_set<int> &fuzzy_region,
                                const std::vector<std::unordered_set<int>> &printable_components,
                                std::unordered_map<int, int> &variable_to_direction,
                                std::unordered_map<int, int> &face_to_variable,
                                std::vector<std::vector<const MPVariable*>> &variables);
    void updatePrintableComponents(const int &face,
                                   const int &face_variable,
                                   const std::unordered_set<int> &fuzzy_region,
                                   std::vector<std::unordered_set<int>> &printable_components,
                                   std::unordered_map<int, int> &variable_to_direction,
                                   std::vector<std::vector<const MPVariable*>> &variables);


    // Other general subroutines
/*-------------------------------------------------------------------------------------------------*/
    // Normals
    Eigen::Vector3f getFaceNormal(const int &face);
    Eigen::Vector3f getEdgeNormal(const std::pair<int, int> &edge);
    Eigen::Vector3f getVertexNormal(const int &vertex);

    // Random sampling
    Eigen::Vector3f sampleRandomPoint(const int &face);

    // For determining intersections with other faces
    // Should use BVH, some other structure, or there might be something in libigl/VCGlib we can use
    // If using BVH, may need to make BVH initialization a preprocessing step
    int getIntersection(const Eigen::Vector3f &ray_position, const Eigen::Vector3f &ray_direction);
    int getIntersectionWithDistance(const Eigen::Vector3f &ray_position, const Eigen::Vector3f &ray_direction, float &distance);
    // helper for getPatchBoundary
    void updateBoundarySet(const std::pair<int, int> edge, std::unordered_set<std::pair<int, int>, PairHash>& boundary_set);
    // helper for getPairwiseFuzzyRegion
    void updateFuzzyRegion(std::unordered_set<int> &fuzzyRegion, std::unordered_set<int> &boundaryFaces, int f);
    // given a patch, populate the set which contains the edges of the boundary
    void getPatchBoundary(const std::unordered_set<int>& patch, std::unordered_set<std::pair<int, int>, PairHash>& patch_boundary);
    // Gets intersection of edges between two patches
    void getBoundaryEdges(const std::unordered_set<int> &patch_one,
                          const std::unordered_set<int> &patch_two,
                          std::unordered_set<std::pair<int, int>, PairHash> &boundaryEdges);
    // Gets the set of faces adjacent to the boundary edge between two patches
    void getBoundaryFaces(const std::unordered_set<int> &patch_one, const std::unordered_set<int> &patch_two, std::unordered_set<int> &boundaryFaces);
    // Gets the fuzzy region between two patches
    void getPairwiseFuzzyRegion(const std::unordered_set<int> &patch_one,
                                const std::unordered_set<int> &patch_two,
                                std::unordered_set<int> &fuzzyRegion);

    // Basic utility functions for faces
    Eigen::Vector3f getCentroid(const int &face);
    double getArea(const int &face);

    // Ambient occlusion
    double getEdgeAO(const std::pair<int, int> &edge);
    double getFaceAO(const int &face);

    // Visualization
    Eigen::Vector3d mapValueToColor(double value, double max_value);
    Eigen::Vector3d lerp(const Eigen::Vector3d& color1, const Eigen::Vector3d& color2, double t);

    // Distances to sets of points
    std::pair<double, int> getMinGeodesicDistanceToSet(const int &face, const std::unordered_set<int> &faces, bool include_self = false);
    std::pair<double, int> getMinWeightedDistanceToSet(const int &face, const std::unordered_set<int> &faces, bool include_self = false);
    std::pair<double, int> getMaxGeodesicDistanceToSet(const int &face, const std::unordered_set<int> &faces);
    std::pair<double, int> getMaxWeightedDistanceToSet(const int &face, const std::unordered_set<int> &faces);
    double getTotalGeodesicDistanceToSet(const int &face, const std::unordered_set<int> &faces);
    double getTotalWeightedDistanceToSet(const int &face, const std::unordered_set<int> &faces);

    // Support for ILP when doing initial segmentation
    void addSupportCosts(std::vector<std::vector<const MPVariable*>> &variables, const std::vector<std::unordered_set<int>> &patches);
    void addSmoothingCosts(std::vector<std::vector<const MPVariable*>> &variables);

    // Generic ILP Subroutines
    const MPVariable* addVariable(const double &coefficient, const double &min_val = 0.0, const double &max_val = 1.0, const std::string &name = "");
    const MPVariable* addXORVariable(const MPVariable* var_1, const MPVariable* var_2, const double &coefficient, const std::string &name = "");
    void addConstraint(const std::vector<const MPVariable*> &variables, const std::vector<double> &coefficients, const double &min_val = 0.0, const double &max_val = 1.0, const std::string &name = "");
    void clearSolver();

    double bbd; // bounding box diagonal
    Eigen::MatrixXd _weightedDistances;

    // Configurable parameters
    // Preprocessing parameters
    double _geodesic_distance_weight;
    double _convex_coeff;
    double _concave_coeff;
    bool _use_zero_cost_faces;

    // Oversegmentation parameters
    int _num_seed_faces;
    double _proportion_seed_faces;
    double _oversegmentation_bounding_box_coeff;
    int _num_oversegmentation_iterations;
    bool _visualize_seeds;
    bool _oversegmentation_skip_visualization;

    // Initial Segmentation parameters
    int _num_random_dir_samples;
    double _printer_tolerance_angle; // Measured in RADIANS (specified as degrees)
    double _ambient_occlusion_supports_alpha;
    double _ambient_occlusion_smoothing_alpha;
    double _smoothing_width_t;
    Eigen::MatrixXd _supportCoefficients;
    int _ambient_occlusion_samples;
    int _footing_samples;
    bool _axis_only;
    std::unordered_set<int> _zero_cost_faces;
    bool _initial_skip_visualization;

    // Refined Segmentation parameters
    // TODO: Add them
    double _e_fuzzy;
    double _ambient_occlusion_lambda;
    bool _refined_skip_visualization;

    // ILP solver used for phases 2 and 3 (should be cleared before using in phase 2)
    operations_research::MPSolver* _solver;

    // Fields for raytracing
    igl::embree::EmbreeIntersector _intersector;

    // A very small number
    double epsilon = 0.0001;
    std::unordered_map<std::pair<int, int>, double, PairHash> _smoothingCoefficients;
};

#endif // MESHOPERATIONS_H
