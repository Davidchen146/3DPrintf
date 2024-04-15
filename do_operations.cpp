#include "do_operations.h"

// 3D Printf operations:
// Preprocess:
    // "Preprocess/angular_distance_convex": coefficient for angular distance with convex angles
    // "Preprocess/angular_distance_concave": coefficient for angular distance with concave angles
    // "Preprocess/geodesic_distance_weight": proportion of weighted distance is geodesic (instead of angular) distance
// Oversegmentation:
    // "Oversegmentation/num_seed_faces": number of seed faces to randomly sample
    // "Oversegmentation/proportion_seed_faces": proportion of faces to sample as seed faces (num_seed_faces will take precedence)
    // "Oversegmentation/e_patch": controls initial seed termination; when distance from seeds is less than this * bounding box diagonal, terminate
    // "Oversegmentation/num_iterations": number of iterations to recenter and regrow seeds; 0 means patches will be regrown, but will not recenter
    // "Oversegmentation/seeds_only": only returns the center seed faces of patches
// Initial:
    // TODO: Implement!
    // Extension: specify faces to hardcode cost values for support (or a region of faces)
// Refined:
    // TODO: Implement!
// Fabricate:
    // TODO: Implement!
    // Extension: solid/hollow shell objects and if they should have internal connectors

// File to wrap around parsing command line options and executing operations
void doPreprocess(QSettings *settings, Mesh *m, MeshOperations *m_o) {
    // Load parameters
    double angular_distance_convex = settings->value("Preprocess/angular_distance_convex").toDouble();
    double angular_distance_concave = settings->value("Preprocess/angular_distance_concave").toDouble();
    double geodesic_dist_coeff = settings->value("Preprocess/geodesic_distance_weight").toDouble();

    // Pass params to object mesh operations and do preprocessing
    m_o->setPreprocessingParameters(geodesic_dist_coeff, angular_distance_convex, angular_distance_concave);
    m_o->preprocess();
}

// May need to modify this to pass around a labels vector
// Shouldn't be difficult so I leave that to later
void doOversegmentation(QSettings *settings, Mesh *m, MeshOperations *m_o, std::vector<std::vector<int>> &patches) {
    // Load parameters
    int num_seed_faces = settings->value("Oversegmentation/num_seed_faces").toInt();
    double proportion_seed_faces = settings->value("Oversegmentation/proportion_seed_faces").toDouble();
    double e_patch = settings->value("Oversegmentation/e_patch").toDouble();
    int num_iterations = settings->value("Oversegmentation/num_iterations").toInt();
    bool seeds_only = settings->value("Oversegmentation/seeds_only").toBool();

    // Pass params to object mesh operations and do preprocessing
    m_o->setOversegmentationParameters(num_seed_faces, proportion_seed_faces, e_patch, num_iterations, seeds_only);
    m_o->generateOversegmentation(patches);
    m_o->visualize(patches);
}

void doInitialSegmentation(QSettings *settings, Mesh *m, MeshOperations *m_o) {
    std::cerr << "Error: This phase hasn't been implemented yet" << std::endl;
}

void doRefinedSegmentation(QSettings *settings, Mesh *m, MeshOperations *m_o) {
    std::cerr << "Error: This phase hasn't been implemented yet" << std::endl;
}

void doFabrication(QSettings *settings, Mesh *m, MeshOperations *m_o) {
    std::cerr << "Error: This phase hasn't been implemented yet" << std::endl;
}
