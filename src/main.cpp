#include <QCoreApplication>
#include <QCommandLineParser>
#include <QtCore>

#include <iostream>
#include <chrono>

#include "src/mesh.h"
#include "src/meshoperations.h"

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
        // "General/meshfile": input mesh file
        // "General/output": path to output file or output path (directory) if using 3D printing pipeline
        // "General/method": mesh operation to perform

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
        // "Preprocess/use_zero_cost_faces": allows for specifying faces to have 0 support cost as a preprocessing step to initial segmentation
    // Oversegmentation:
        // "Oversegmentation/num_seed_faces": number of seed faces to randomly sample
        // "Oversegmentation/proportion_seed_faces": proportion of faces to sample as seed faces (num_seed_faces will take precedence)
        // "Oversegmentation/e_patch": controls initial seed termination; when distance from seeds is less than this * bounding box diagonal, terminate
        // "Oversegmentation/num_iterations": number of iterations to recenter and regrow seeds; 0 means patches will be regrown, but will not recenter
        // "Oversegmentation/visualize_seeds": provides a visual of the seed faces before the patches are grown from seeds
        // "Oversegmentation/skip_visualization": skips visualization for oversegmentation
    // Initial:
        // "Intital/num_random_dir_samples": number of directions to randomly sample
        // "Initial/printer_tolerance_angle": how far faces can deviate from the printing direction without requiring supports (in degrees)
        // "Initial/ambient_occlusion_supports_alpha": alpha coefficient for ambient occlusion computations for support cost
        // "Initial/ambient_occlusion_smoothing_alpha": alpha coefficient for ambient occlusion computations for smoothing cost
        // "Initial/smoothing_width_t": t coefficient for smoothing cost (measures size of cut)
        // "Initial/ambient_occlusion_samples": number of samples to cast for ambient occlusion; more samples is more accurate but takes longer
        // "Initial/footing_samples": number of samples to cast for footing faces
        // "Initial/axis_only": only use the 6 cardinal printing directions
        // "Initial/skip_visualization": skips visualization for initial segmentation
    // Refined:
        // "Refined/e_fuzzy": coefficient controlling how large the fuzzy region is in relation to bounding box
        // "Refined/ambient_occlusion_lambda": determines how important occlusion is for reshaping cuts; larger values are more important
        // "Refined/skip_visualization": skips visualization of refined segmentation
    // Fabricate:
        // "Fabricate/t_quality": Quality parameter for tetgen. Smaller values (close to 1.0) generate better tetrahedra
        // "Fabricate/t_volume": Volume parametter for tetgen. Smaller values mean smaller, more fine-grained tetrahedra
        // "Fabricate/smoothing_iterations": Number of iterations for Laplacian smoothing of interface surfaces
        // "Fabricate/smoothing_weight": Controls smoothing weight for each iteration of Laplacian smoothing of interface surfaces
        // "Fabricate/num_rays": Number of rays cast for tetrahedra assignment to printable components
        // "Fabricate/inset_amount": Coefficient controlling how far large the inset surface is offset
        // "Fabricate/inset_resolution": Resolution of inset surface
        // "Fabricate/solid_components": If components should not be hollowed out and instead be solid pieces
        // "Fabricate/skip_visualization": skips visualization of fabrication step
    // Debug:
        // "Debug/function": function to debug. Should be one of:
            // "ao_face": ambient occlusion for each face; red values are visible, green are not as visible, and blue are occluded
            // "ao_edge": ambient occlusion for each edge; red values are visible, green are not as visible, and blue are occluded
            // "support_costs": costs for supports in a specified printing direction; red values are heavy support costs, green are not as heavy, blue are light
            // "smoothing_costs": costs for smoothing a patch. Costs are averaged by patch edge length boundary, red are high costs, blue are lower costs
            // "angular_distance": average angular distance from a face to its neighbors; red are high distances, blue are lower
            // "weighted_distance": average weighted distance a face to its neighbors; red are high distances, blue are lower
            // "remesh_intersection": remesh a mesh to resolve self-intersections. Saves output mesh to outfile
        // "Debug/debug_x": X coordinate for printing direction to determine support costs
        // "Debug/debug_y": Y coordinate for printing direction to determine support costs
        // "Debug/debug_z": Z coordinate for printing direction to determine support costs

    // Parse common inputs
    std::cout << "Loading config " << args[0].toStdString() << std::endl;
    QSettings settings(args[0], QSettings::IniFormat);
    QString infile  = settings.value("Global/meshfile").toString();
    QString outfile = settings.value("Global/outfile").toString();
    QString method  = settings.value("Global/method").toString();

    // Load
    Mesh m;
    m.loadFromFile(infile.toStdString());
    m.preProcess();
    m.validate();
    // TODO: Make this take a reference to an existing mesh instead to avoid copying?
    MeshOperations m_o(m);

    // Start timing
    auto t0 = std::chrono::high_resolution_clock::now();

    // Determine operation
    // Control flow flag to determine if doing 3Dprintf or mesh operations
    bool is_mesh_operation = method == "subdivide" || method == "simplify" || method == "remesh";
    bool is_3d_print_operation = method == "preprocess" || method == "oversegmentation" || method == "initial" || method == "refined" || method == "fabricate" || method == "debug";

    // Mesh project operations
    // TODO: Add this as a preprocessing pipeline option
    if (is_mesh_operation) {
        // Load mesh parameters here
        // Subdivision
        int subdivide_num_iterations = settings.value("Subdivide/num_iterations").toInt();
        // Simplification
        int target_faces = settings.value("Simplify/target_faces").toInt();
        int faces_to_remove = settings.value("Simplify/faces_to_remove").toInt();
        // Remeshing
        int remesh_num_iterations = settings.value("Remesh/num_iterations").toInt();
        double smoothing_weight = settings.value("Remesh/smoothing_weight").toDouble();

        // Case on method
        if (method == "subdivide") {
            for (int i = 0; i < subdivide_num_iterations; i++) {
                m.loopSubdivide();
            }
            m.validate();
            m.convert();
        }
        else if (method == "simplify") {
            // Determine how many faces to remove
            if (target_faces != 0) {
                int num_mesh_faces = m.getFaceMap().size();
                faces_to_remove = std::max(num_mesh_faces - target_faces, 0);
            }
            m.simplify(faces_to_remove);
            m.validate();
            m.convert();
        }
        else if (method == "remesh") {
            std::cerr << "Error: Mesh isotropic remeshing operation not yet supported" << std::endl;
        }
    }

    // Operations for 3Dprintf
    else if (is_3d_print_operation) {
        // Load 3D printing operations here
        // Preprocessing
        double angular_distance_convex = settings.value("Preprocess/angular_distance_convex").toDouble();
        double angular_distance_concave = settings.value("Preprocess/angular_distance_concave").toDouble();
        double geodesic_dist_coeff = settings.value("Preprocess/geodesic_distance_weight").toDouble();
        bool use_zero_cost_faces = settings.value("Preprocess/use_zero_cost_faces").toBool();

        // Oversegmentation
        int num_seed_faces = settings.value("Oversegmentation/num_seed_faces").toInt();
        double proportion_seed_faces = settings.value("Oversegmentation/proportion_seed_faces").toDouble();
        double e_patch = settings.value("Oversegmentation/e_patch").toDouble();
        int num_iterations = settings.value("Oversegmentation/num_iterations").toInt();
        bool visualize_seeds = settings.value("Oversegmentation/visualize_seeds").toBool();
        bool oversegmentation_skip_visual = settings.value("Oversegmentation/skip_visualization").toBool();

        // Initial Segmentation
        int num_random_dir_samples = settings.value("Initial/num_random_dir_samples").toInt();
        double printer_tolerance_angle = settings.value("Initial/printer_tolerance_angle").toDouble(); // In degrees
        double ambient_occlusion_supports_alpha = settings.value("Initial/ambient_occlusion_supports_alpha").toDouble();
        double ambient_occlusion_smoothing_alpha = settings.value("Initial/ambient_occlusion_smoothing_alpha").toDouble();
        double smoothing_width_t = settings.value("Initial/smoothing_width_t").toDouble();
        int ambient_occlusion_samples = settings.value("Initial/ambient_occlusion_samples").toInt();
        int footing_samples = settings.value("Initial/footing_samples").toInt();
        bool axis_only = settings.value("Initial/axis_only").toBool();
        bool initial_skip_visual = settings.value("Initial/skip_visualization").toBool();

        // Refined segmentation
        double e_fuzzy = settings.value("Refined/e_fuzzy").toDouble();
        double ambient_occlusion_lambda = settings.value("Refined/ambient_occlusion_lambda").toDouble();
        bool refined_skip_visual = settings.value("Refined/skip_visualization").toBool();

        // Fabricate
        double t_quality = settings.value("Fabricate/t_quality").toDouble();
        double t_volume = settings.value("Fabricate/t_volume").toDouble();
        int smoothing_iterations = settings.value("Fabricate/smoothing_iterations").toInt();
        double smoothing_weight = settings.value("Fabricate/smoothing_weight").toDouble();
        int num_random_rays = settings.value("Fabricate/num_rays").toInt();
        double inset_amount = settings.value("Fabricate/inset_amount").toDouble();
        int inset_resolution = settings.value("Fabricate/inset_resolution").toInt();
        bool solid_components = settings.value("Fabricate/solid_components").toBool();
        bool fabricate_skip_visual = settings.value("Fabricate/skip_visualization").toBool();

        // Debug
        std::string debug_mode = settings.value("Debug/mode").toString().toStdString();
        double debug_x = settings.value("Debug/debug_x").toDouble();
        double debug_y = settings.value("Debug/debug_y").toDouble();
        double debug_z = settings.value("Debug/debug_z").toDouble();

        // Case on the method
        if (method == "preprocess") {
            m_o.setPreprocessingParameters(geodesic_dist_coeff, angular_distance_convex, angular_distance_concave, use_zero_cost_faces);
            m_o.preprocessData();
            m_o.preprocessDistances();
            m_o.preprocessRaytracer();
            m_o.preprocessZeroCostFaces();
        }
        else if (method == "oversegmentation") {
            m_o.setPreprocessingParameters(geodesic_dist_coeff, angular_distance_convex, angular_distance_concave, use_zero_cost_faces);
            m_o.preprocessData();
            m_o.preprocessDistances();
            m_o.preprocessRaytracer();            

            // This vec will hold the labelings
            std::vector<std::unordered_set<int>> patches;
            m_o.setOversegmentationParameters(num_seed_faces, proportion_seed_faces, e_patch, num_iterations, visualize_seeds, oversegmentation_skip_visual);
            m_o.generateOversegmentation(patches);
            if (!oversegmentation_skip_visual) {
                m_o.visualize(patches);
            }
        }
        else if (method == "initial") {
            m_o.setPreprocessingParameters(geodesic_dist_coeff, angular_distance_convex, angular_distance_concave, use_zero_cost_faces);
            m_o.preprocessData();
            m_o.preprocessDistances();
            m_o.preprocessRaytracer();
            m_o.preprocessZeroCostFaces();
            m_o.preprocessSolver();

            // This vec will hold the labelings
            std::vector<std::unordered_set<int>> patches;
            m_o.setOversegmentationParameters(num_seed_faces, proportion_seed_faces, e_patch, num_iterations, visualize_seeds, oversegmentation_skip_visual);
            m_o.generateOversegmentation(patches);
            if (!oversegmentation_skip_visual) {
                m_o.visualize(patches);
            }

            // The printable components
            std::vector<std::unordered_set<int>> printable_components;
            // Printing directions for each component
            std::vector<Eigen::Vector3f> printing_directions;
            m_o.setInitialSegmentationParameters(num_random_dir_samples, printer_tolerance_angle, ambient_occlusion_supports_alpha, ambient_occlusion_smoothing_alpha, smoothing_width_t, ambient_occlusion_samples, footing_samples, axis_only, initial_skip_visual);
            m_o.generateInitialSegmentation(patches, printable_components, printing_directions);
            // Visualize printing components, then each printable components with supported faces highlighted in red
            if (!initial_skip_visual) {
                m_o.visualize(printable_components);
                m_o.visualizePrintableComponents(printable_components, printing_directions);
            }
        }
        else if (method == "refined") {
            m_o.setPreprocessingParameters(geodesic_dist_coeff, angular_distance_convex, angular_distance_concave, use_zero_cost_faces);
            m_o.preprocessData();
            m_o.preprocessDistances();
            m_o.preprocessRaytracer();
            m_o.preprocessZeroCostFaces();
            m_o.preprocessSolver();

            // This vec will hold the labelings
            std::vector<std::unordered_set<int>> patches;
            m_o.setOversegmentationParameters(num_seed_faces, proportion_seed_faces, e_patch, num_iterations, visualize_seeds, oversegmentation_skip_visual);
            m_o.generateOversegmentation(patches);
            if (!oversegmentation_skip_visual) {
                m_o.visualize(patches);
            }

            // The printable components
            std::vector<std::unordered_set<int>> printable_components;
            // Printing directions for each component
            std::vector<Eigen::Vector3f> printing_directions;
            m_o.setInitialSegmentationParameters(num_random_dir_samples, printer_tolerance_angle, ambient_occlusion_supports_alpha, ambient_occlusion_smoothing_alpha, smoothing_width_t, ambient_occlusion_samples, footing_samples, axis_only, initial_skip_visual);
            m_o.generateInitialSegmentation(patches, printable_components, printing_directions);
            // Visualize printing components, then each printable components with supported faces highlighted in red
            if (!initial_skip_visual) {
                m_o.visualize(printable_components);
                m_o.visualizePrintableComponents(printable_components, printing_directions);
            }

            std::vector<std::unordered_set<int>> fuzzyRegions;
            m_o.setRefinedSegmentationParameters(e_fuzzy, ambient_occlusion_lambda, refined_skip_visual);
            m_o.generateRefinedSegmentation(printable_components, printing_directions, fuzzyRegions);
            // Visualize with the changes
            if (!refined_skip_visual) {
                m_o.visualize(printable_components);
                m_o.visualizePrintableComponents(printable_components, printing_directions);
            }
        }
        else if (method == "fabricate") {
            m_o.setPreprocessingParameters(geodesic_dist_coeff, angular_distance_convex, angular_distance_concave, use_zero_cost_faces);
            m_o.preprocessData();
            m_o.preprocessDistances();
            m_o.preprocessRaytracer();
            m_o.preprocessZeroCostFaces();
            m_o.preprocessSolver();

            // This vec will hold the labelings
            std::vector<std::unordered_set<int>> patches;
            m_o.setOversegmentationParameters(num_seed_faces, proportion_seed_faces, e_patch, num_iterations, visualize_seeds, oversegmentation_skip_visual);
            m_o.generateOversegmentation(patches);
            if (!oversegmentation_skip_visual) {
                m_o.visualize(patches);
            }

            // The printable components
            std::vector<std::unordered_set<int>> printable_components;
            // Printing directions for each component
            std::vector<Eigen::Vector3f> printing_directions;
            m_o.setInitialSegmentationParameters(num_random_dir_samples, printer_tolerance_angle, ambient_occlusion_supports_alpha, ambient_occlusion_smoothing_alpha, smoothing_width_t, ambient_occlusion_samples, footing_samples, axis_only, initial_skip_visual);
            m_o.generateInitialSegmentation(patches, printable_components, printing_directions);
            // Visualize printing components, then each printable components with supported faces highlighted in red
            if (!initial_skip_visual) {
                m_o.visualize(printable_components);
                m_o.visualizePrintableComponents(printable_components, printing_directions);
            }

            std::vector<std::unordered_set<int>> fuzzyRegions;
            m_o.setRefinedSegmentationParameters(e_fuzzy, ambient_occlusion_lambda, refined_skip_visual);
            m_o.generateRefinedSegmentation(printable_components, printing_directions, fuzzyRegions);
            // Visualize with the changes
            if (!refined_skip_visual) {
                m_o.visualize(printable_components);
                m_o.visualizePrintableComponents(printable_components, printing_directions);
            }

            m_o.setFabricateParameters(t_quality, t_volume, smoothing_iterations, smoothing_weight, num_random_rays, inset_amount, inset_resolution, solid_components, fabricate_skip_visual);
            m_o.tetrahedralizeMesh(); // this is effectively a preprocessing step for fabrication
            std::vector<std::vector<Eigen::Vector4i>> printable_volumes;
            m_o.partitionVolume(printable_components, printable_volumes);
            m_o.pruneVolume(printable_volumes);
            if (!fabricate_skip_visual) {
                m_o.visualizePrintableVolumes(printable_components, printing_directions, printable_volumes);
            }

            // Send to file
            std::vector<Eigen::MatrixXf> printable_vertices;
            std::vector<Eigen::MatrixXi> printable_faces;
            m_o.boolOpsApply(printable_volumes, printable_vertices, printable_faces);
            m_o.outputComponentsToFile(printable_vertices, printable_faces, printing_directions, outfile.toStdString());
        }
        else if (method == "debug") {
            // Case on the debug option
            if (debug_mode == "ao_face") {
                // Setup
                m_o.setPreprocessingParameters(geodesic_dist_coeff, angular_distance_convex, angular_distance_concave, use_zero_cost_faces);
                m_o.preprocessData();
                m_o.setInitialSegmentationParameters(num_random_dir_samples, printer_tolerance_angle, ambient_occlusion_supports_alpha, ambient_occlusion_smoothing_alpha, smoothing_width_t, ambient_occlusion_samples, footing_samples, axis_only);

                // Visualize!
                m_o.visualizeFaceAO();
            } else if (debug_mode == "ao_edge") {
                // Setup
                m_o.setPreprocessingParameters(geodesic_dist_coeff, angular_distance_convex, angular_distance_concave, use_zero_cost_faces);
                m_o.preprocessData();
                m_o.setInitialSegmentationParameters(num_random_dir_samples, printer_tolerance_angle, ambient_occlusion_supports_alpha, ambient_occlusion_smoothing_alpha, smoothing_width_t, ambient_occlusion_samples, footing_samples, axis_only);

                // Visualize!
                m_o.visualizeEdgeAO();
            } else if (debug_mode == "support_costs") {
                // Setup
                m_o.setPreprocessingParameters(geodesic_dist_coeff, angular_distance_convex, angular_distance_concave, use_zero_cost_faces);
                m_o.preprocessData();
                m_o.preprocessZeroCostFaces();
                m_o.setInitialSegmentationParameters(num_random_dir_samples, printer_tolerance_angle, ambient_occlusion_supports_alpha, ambient_occlusion_smoothing_alpha, smoothing_width_t, ambient_occlusion_samples, footing_samples, axis_only);
                m_o.preprocessRaytracer();

                // Determine visualization direction
                Eigen::Vector3f direction(debug_x, debug_y, debug_z);
                direction.normalize();
                if (direction.norm() == 0) {
                    direction = m_o.generateRandomVector();
                    direction.normalize();
                }

                // Visualize!
                std::cout << "Visualizing support costs in direction (" << direction(0) << ", " << direction(1) << ", " << direction(2) << ")" << std::endl;
                m_o.visualizeSupportCosts(direction);

            } else if (debug_mode == "smoothing_costs"){
                m_o.setPreprocessingParameters(geodesic_dist_coeff, angular_distance_convex, angular_distance_concave, use_zero_cost_faces);
                m_o.preprocessData();
                m_o.preprocessDistances();
                m_o.preprocessRaytracer();
                std::vector<std::unordered_set<int>> patches;
                m_o.setOversegmentationParameters(num_seed_faces, proportion_seed_faces, e_patch, num_iterations, visualize_seeds, oversegmentation_skip_visual);
                m_o.generateOversegmentation(patches);
                m_o.visualize(patches);
                m_o.setInitialSegmentationParameters(num_random_dir_samples, printer_tolerance_angle, ambient_occlusion_supports_alpha, ambient_occlusion_smoothing_alpha, smoothing_width_t, ambient_occlusion_samples, footing_samples, axis_only, initial_skip_visual);
                m_o.visualizeSmoothingCosts(patches);
            }
            else if (debug_mode == "angular_distance") {
                // Setup
                m_o.setPreprocessingParameters(geodesic_dist_coeff, angular_distance_convex, angular_distance_concave, use_zero_cost_faces);
                m_o.preprocessData();
                m_o.preprocessDistances();

                // View distances
                m_o.visualizeAngularDistance();
            }

            else if (debug_mode == "weighted_distance") {
                // Setup
                m_o.setPreprocessingParameters(geodesic_dist_coeff, angular_distance_convex, angular_distance_concave, use_zero_cost_faces);
                m_o.preprocessData();
                m_o.preprocessDistances();

                // View distances
                m_o.visualizeWeightedDistance();

            } else if (debug_mode == "remesh_intersection") {
                // NOTE: make sure to specify outfile
                m.remeshSelfIntersections();
                m.saveToFile(outfile.toStdString());
                return 0;
            } else {
                std::cout << "Error: Unknown debug mode \"" << debug_mode << "\"" << std::endl;
            }
        }
    }
    else {
        std::cerr << "Error: Unknown method \"" << method.toUtf8().constData() << "\"" << std::endl;
    }

    // Finish timing
    auto t1 = std::chrono::high_resolution_clock::now();
    auto duration = duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    std::cout << "Execution took " << duration << " milliseconds." << std::endl;

    a.exit();
}
