#include <QCoreApplication>
#include <QCommandLineParser>
#include <QtCore>

#include <iostream>
#include <chrono>

#include "src/mesh.h"
#include "src/meshoperations.h"

#include <vcglib/vcg/complex/complex.h>
#include<vcglib/vcg/complex/algorithms/create/platonic.h>

class MyEdge;
class MyFace;
class MyVertex;
struct MyUsedTypes : public vcg::UsedTypes<	vcg::Use<MyVertex>::AsVertexType, vcg::Use<MyFace>::AsFaceType>{};

class MyVertex  : public vcg::Vertex< MyUsedTypes, vcg::vertex::Coord3f, vcg::vertex::BitFlags  >{};
class MyFace    : public vcg::Face  < MyUsedTypes, vcg::face::VertexRef,vcg::face::FFAdj, vcg::face::Mark, vcg::face::BitFlags > {};
class MyMesh : public vcg::tri::TriMesh< std::vector<MyVertex>, std::vector<MyFace > >{};

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

    MyMesh mesh;

    //generate a mesh
    vcg::tri::Icosahedron(mesh);

    //update the face-face topology
    vcg::tri::UpdateTopology<MyMesh>::FaceFace(mesh);

    // Now for each face the FFp() FFi() members are correctly initialized

    if(vcg::face::IsBorder(mesh.face[0],0)) printf("Edge 0 of face 0 is a border\n");
    else printf("Edge 0 of face 0 is NOT a border\n"); // always this path!

    vcg::face::FFDetach<MyFace>(mesh.face[0],0);  // Detach the face [0] from the mesh
    vcg::face::FFDetach<MyFace>(mesh.face[0],1);
    vcg::face::FFDetach<MyFace>(mesh.face[0],2);

    if(vcg::face::IsBorder(mesh.face[0],0)) printf("Edge 0 of face 0 is a border\n"); // always this path!
    else printf("Edge 0 of face 0 is NOT a border\n");

    vcg::tri::Allocator<MyMesh>::DeleteFace(mesh,mesh.face[0]);

    // declare an iterator on the mesh
    vcg::face::Pos<MyMesh::FaceType> he, hei;

    UnMarkAll(mesh);

    // Now a simple search and trace of all the borders of the mesh
    int BorderEdgeNum=0;
    int HoleNum=0;
    for(MyMesh::FaceIterator fi=mesh.face.begin();fi!=mesh.face.end();++fi) if(!(*fi).IsD())
        {
            for(int j=0;j<3;j++)
            {
                if ( vcg::face::IsBorder(*fi,j) && !vcg::tri::IsMarked(mesh,&*fi))
                {
                    vcg::tri::Mark(mesh,&*fi);
                    hei.Set(&*fi,j,fi->V(j));
                    he=hei;
                    do
                    {
                        BorderEdgeNum++;
                        he.NextB(); // next pos along a border
                        vcg::tri::Mark(mesh,he.f);
                    }
                    while (he.f!=hei.f);
                    HoleNum++;
                }
            }
        }

    printf("Mesh has %i holes and %i border edges\n",HoleNum,BorderEdgeNum);
    return 0;

    // Parse common inputs
    std::cout << "Loading config " << args[0].toStdString() << std::endl;
    QSettings settings(args[0], QSettings::IniFormat);
    QString infile  = settings.value("Global/meshfile").toString();
    QString outfile = settings.value("Global/output").toString();
    QString method  = settings.value("Global/method").toString();

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

        // Case on the method
        if (method == "subdivide") {
            std::cerr << "Error: Mesh loop subdivision operation not yet supported" << std::endl;
        }
        else if (method == "simplify") {
            // Determine how many faces to remove
            if (target_faces != 0) {
                int num_mesh_faces = m.getFaceSet().size();
                faces_to_remove = std::max(num_mesh_faces - target_faces, 0);
            }
            m.simplify(faces_to_remove);
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
        // Oversegmentation
        int num_seed_faces = settings.value("Oversegmentation/num_seed_faces").toInt();
        double proportion_seed_faces = settings.value("Oversegmentation/proportion_seed_faces").toDouble();
        double e_patch = settings.value("Oversegmentation/e_patch").toDouble();
        int num_iterations = settings.value("Oversegmentation/num_iterations").toInt();
        bool seeds_only = settings.value("Oversegmentation/seeds_only").toBool();

        // Case on the method
        if (method == "preprocess") {
            m_o.setPreprocessingParameters(geodesic_dist_coeff, angular_distance_convex, angular_distance_concave);
            m_o.preprocess();
        }
        else if (method == "oversegmentation") {
            m_o.setPreprocessingParameters(geodesic_dist_coeff, angular_distance_convex, angular_distance_concave);
            m_o.preprocess();

            // This vec will hold the labelings
            std::vector<std::vector<int>> patches;
            m_o.setOversegmentationParameters(num_seed_faces, proportion_seed_faces, e_patch, num_iterations, seeds_only);
            m_o.generateOversegmentation(patches);
            m_o.visualize(patches);
        }
        else if (method == "initial") {
            std::cerr << "Error: This phase hasn't been implemented yet" << std::endl;
        }
        else if (method == "refined") {
            std::cerr << "Error: This phase hasn't been implemented yet" << std::endl;
        }
        else if (method == "fabricate") {
            std::cerr << "Error: This phase hasn't been implemented yet" << std::endl;
        }
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
