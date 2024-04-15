#include <QCoreApplication>
#include <QCommandLineParser>
#include <QtCore>

#include <iostream>
#include <chrono>

#include "mesh.h"
#include "meshoperations.h"
#include "do_operations.h"

int main(int argc, char *argv[])
{
    srand(static_cast<unsigned>(time(0)));

    QCoreApplication a(argc, argv);
    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addPositionalArgument("config",  "Path of the config (.ini) file.");
    parser.process(a);

    // Check for invalid argument count
    const QStringList args = parser.positionalArguments();
    if (args.size() < 1) {
        std::cerr << "Not enough arguments. Please provide a path to a config file (.ini) as a command-line argument." << std::endl;
        a.exit(1);
        return 1;
    }

    // Mesh operations from the Mesh project are supported as well as the new 3D printing operations
    // TODO: Add the remaining mesh operations (remeshing could help, but Ed's implementation is bugged/needs work)
    //       There is an issue with determining when to collapse/split edges
    // Valid operations:
    // Mesh
        // Subdivide: loop subdivision
        // Simplify: quadric error mesh simplification
        // Remesh: isotropic remeshing

    // 3D printf
        // Preprocess: performs mesh loading and preprocessing (shortest paths algorithms)
        // Oversegmentation: phase 1 of paper; also requires preprocessing
        // Inital: phase 2 of paper, requires previous phases
        // Refined: phase 3 of paper, requires previous phases
        // Fabricate: phase 4 of paper, requires previous phases

    // Parameters for each option:

    // General/All:
        // "Mesh/meshfile": input mesh file
        // "Mesh/output": path to output file or output path (directory) if using 3D printing pipeline
        // "Method/method": mesh operation to perform

    // Mesh operations:
    // Subdivide:
        // "Subdivide/num_iterations": number of iterations for loop subdivision
    // Simplify:
        // "Simplify/target_faces": number of target faces in final mesh
        // "Simplify/faces_to_remove": number of faces to remove from mesh (target_faces will take precedence)
    // Remesh:
        // "Remesh/num_iterations": number of iterations for isotropic remeshing
        // "Remesh/smoothing_weight": tangential smoothing weight for remeshing

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

    // Parse common inputs
    QSettings settings( args[0], QSettings::IniFormat );
    QString infile  = settings.value("Mesh/meshfile").toString();
    QString outfile = settings.value("Mesh/output").toString();
    QString method  = settings.value("Method/method").toString();

    // Load
    Mesh m;
    m.loadFromFile(infile.toStdString());
    m.preProcess();
    // TODO: Make this take a reference to an existing mesh instead to avoid copying?
    MeshOperations m_o(m);

    // Start timing
    auto t0 = std::chrono::high_resolution_clock::now();

    // Determine operation
    // Control flow flag to determine if doing 3Dprintf or mesh operations
    bool is_mesh_operation = method == "subdivide" || method == "simplify" || method == "remesh";
    bool is_3d_print_operation = method == "preprocess" || method == "oversegmentation" || method == "initial" || method == "refined" || method == "fabricate";

    // Mesh project operations
    if (method == "subdivide") {
        // Load parameters
        int num_iterations = settings.value("Subdivide/num_iterations").toInt();
        std::cerr << "Error: Mesh loop subdivision operation not yet supported" << std::endl;
    }
    else if (method == "simplify") {
        // Load parameters
        int target_faces = settings.value("Simplify/target_faces").toInt();
        int faces_to_remove = settings.value("Simplify/faces_to_remove").toInt();

        // Determine how many faces to remove
        if (target_faces != 0) {
            int num_mesh_faces = m.getFaceSet().size();
            faces_to_remove = std::max(num_mesh_faces - target_faces, 0);
        }
        m.simplify(faces_to_remove);
    }
    else if (method == "remesh") {
        // Load parameters
        std::cerr << "Error: Mesh isotropic remeshing operation not yet supported" << std::endl;
    }

    // Operations for 3Dprintf
    else if (method == "preprocess") {
        doPreprocess(&settings, &m, &m_o);
    }
    else if (method == "oversegmentation") {
        std::vector<std::vector<int>> patches;
        doPreprocess(&settings, &m, &m_o);
        doOversegmentation(&settings, &m, &m_o, patches);
    }
    else if (method == "initial") {
        std::cerr << "Error: 3D printing initial segmentation operation not yet supported" << std::endl;
    }
    else if (method == "refined") {
        std::cerr << "Error: 3D printing refined segmentation operation not yet supported" << std::endl;
    }
    else if (method == "fabricate") {
        std::cerr << "Error: 3D printing fabrication operation not yet supported" << std::endl;
    }
    else {
        std::cerr << "Error: Unknown method \"" << method.toUtf8().constData() << "\"" << std::endl;
    }

    // Finish timing
    auto t1 = std::chrono::high_resolution_clock::now();
    auto duration = duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    std::cout << "Execution took " << duration << " milliseconds." << std::endl;

    // Save
    if (outfile != "") {
        if (is_mesh_operation) {
            m.saveToFile(outfile.toStdString());
        }

        if (is_3d_print_operation) {
            std::cerr << "Error: Saving outputs in 3D print mode not yet supported" << std::endl;
        }
    }

    a.exit();
}
