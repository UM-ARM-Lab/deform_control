#include "simulation/stl_object.h"
#include <fstream>
#include <arc_utilities/arc_exceptions.hpp>
#include <ros/ros.h>

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include "utils/config.h"
#include "simulation/convexdecomp.h"
#include <bullet_helpers/bullet_pretty_print.hpp>

// Code comes from the following sources (accessed 2019/09/04):
// https://github.com/rll/bulletsim/blob/master/src/simulation/openravesupport.cpp
// https://github.com/bulletphysics/bullet3/blob/master/examples/Importers/ImportSTLDemo/LoadMeshFromSTL.h

static inline void PrintVertex(const float vec[3])
{
    std::cout << vec[0] << " " << vec[1] << " " << vec[2];
}

static inline void PrintVertex(const aiVector3D& vec)
{
    std::cout << vec.x << " " << vec.y << " " << vec.z;
}

static inline void CheckEquality(const std::vector<btVector3>& vec1, const std::vector<btVector3>& vec2)
{
    if (vec1.size() != vec2.size())
    {
        std::cerr << "Different length vectors: " << vec1.size() << "    " << vec2.size() << std::endl;
        return;
    }
    if (vec1.size() % 3 != 0)
    {
        std::cerr << "number of vectors not divisible by 3: " << vec1.size() << std::endl;
    }

    for (int i = 0; i < vec1.size(); i += 3)
    {
        const btVector3 ia = vec1[i + 0];
        const btVector3 ib = vec1[i + 1];
        const btVector3 ic = vec1[i + 2];

        int match = -1;
        for (int j = 0; i < vec2.size(); j += 3)
        {
            const btVector3 ja = vec2[j + 0];
            const btVector3 jb = vec2[j + 1];
            const btVector3 jc = vec2[j + 2];

            if (ia == ja && ib == jb && ic == jc)
            {
                match = j;
                break;
            }
        }
        if (match == -1)
        {
            std::cerr << "No match found for vec1 idx " << i << std::endl;
        }
        else
        {
            const btVector3 ja = vec2[match + 0];
            const btVector3 jb = vec2[match + 1];
            const btVector3 jc = vec2[match + 2];
            std::cout << "Vec1: " << PrettyPrint::PrettyPrint(ia, false, "") << "    "
                                  << PrettyPrint::PrettyPrint(ib, false, "") << "    "
                                  << PrettyPrint::PrettyPrint(ic, false, "") << std::endl
                      << "Vec2: " << PrettyPrint::PrettyPrint(ja, false, "") << "    "
                                  << PrettyPrint::PrettyPrint(jb, false, "") << "    "
                                  << PrettyPrint::PrettyPrint(jc, false, "") << std::endl
                      << std::endl;
        }
    }

    std::cerr << "Done checking equality" << std::endl;
}

struct MySTLTriangle
{
    float normal[3];
    float vertex0[3];
    float vertex1[3];
    float vertex2[3];
};

static inline btVector3 toBtVector3(const float vec[3])
{
    return btVector3(vec[0], vec[1], vec[2]) * METERS;
}

static inline btVector3 toBtVector3(const aiVector3D& vec)
{
    return btVector3(vec.x, vec.y, vec.z) * METERS;
}


static std::vector<uint8_t> ReadFile(const std::string& filename)
{
    // Open in concatenate mode so that tellg() is at the end of the file
    std::ifstream input_file(filename, std::ios::binary | std::ios::in | std::ios::ate);
    if (!input_file.is_open())
    {
        throw_arc_exception(std::runtime_error, "Couldn't open file " + filename);
    }
    const std::streamsize size = input_file.tellg();
    input_file.seekg(0, std::ios::beg);
    std::vector<uint8_t> file_buffer((size_t)size);
    if (!(input_file.read(reinterpret_cast<char*>(file_buffer.data()), size)))
    {
        throw_arc_exception(std::runtime_error, "Unable to read entire contents of file");
    }
    return file_buffer;
}


StlObject::StlObject(const std::vector<boost::shared_ptr<btCollisionShape>> subshapes,
                     const boost::shared_ptr<btCompoundShape> compound_shape,
                     const btScalar mass,
                     const btTransform& initial_transform,
                     const bool is_kinematic)
    : BulletObject(mass, compound_shape.get(), initial_transform, is_kinematic)
    , subshapes_(subshapes)
    , compound_shape_(compound_shape)
    , mass_(mass)
{
    if (is_kinematic && mass != 0)
    {
        throw_arc_exception(std::invalid_argument, "Kinematic objects should have zero mass");
    }

    // Margins go on the subshape, not the overall shape?
    compound_shape->setMargin(0); //margin: subshape. seems to result in padding convex shape AND increases collision dist on top of that
}

StlObject::Ptr StlObject::MakeStlObject(const std::string &filename,
                                        const btScalar mass,
                                        const btTransform& initial_transform,
                                        const bool is_kinematic)
{
    // Create an instance of the Importer class
    Assimp::Importer importer;
    // And have it read the given file with some example postprocessing -
    // the importer owns the data pointed to by the returned object
    const aiScene* scene = importer.ReadFile(filename,
        aiProcess_Triangulate            |
        aiProcess_JoinIdenticalVertices  |
        aiProcess_SortByPType);

    if (!scene)
    {
        ROS_FATAL_STREAM("Unable to parse " << filename << ": " << importer.GetErrorString());
        throw_arc_exception(std::runtime_error, importer.GetErrorString());
    }

    assert(scene->HasMeshes());
    ROS_INFO_STREAM("Number of meshes: " << scene->mNumMeshes);
    if (scene->mNumMeshes != 1)
    {
        ROS_FATAL("More than one mesh; this is unhandled.");
        throw_arc_exception(std::invalid_argument, "More than 1 mesh in STL file");
    }
    const aiMesh* mesh = scene->mMeshes[0];
    assert(mesh->HasFaces());
    ROS_INFO_STREAM("Num faces: " << mesh->mNumFaces);

    std::vector<btVector3> assimp_parsed_faces(mesh->mNumFaces * 3);

    const auto margin = 0.0;
    ConvexDecomp decomp(margin);
    for (size_t i = 0; i < mesh->mNumVertices; ++i)
    {
        const aiVector3D& vertex = mesh->mVertices[i];
        decomp.addPoint(toBtVector3(vertex));
    }
    for (size_t i = 0; i < mesh->mNumFaces; ++i)
    {
        const aiFace& face = mesh->mFaces[i];
        assert(face.mNumIndices == 3);
        decomp.addTriangle(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
//        std::cout << "0: "; PrintVertex(mesh->mVertices[face.mIndices[0]]); std::cout << std::endl;
//        std::cout << "1: "; PrintVertex(mesh->mVertices[face.mIndices[1]]); std::cout << std::endl;
//        std::cout << "2: "; PrintVertex(mesh->mVertices[face.mIndices[2]]); std::cout << std::endl << std::endl;

        assimp_parsed_faces[3*i + 0] = toBtVector3(mesh->mVertices[face.mIndices[0]]);
        assimp_parsed_faces[3*i + 1] = toBtVector3(mesh->mVertices[face.mIndices[1]]);
        assimp_parsed_faces[3*i + 2] = toBtVector3(mesh->mVertices[face.mIndices[2]]);
    }

//    std::string tmp;
//    std::cout << "Pausing before dumping other data ";
//    std::cin >> tmp;


    const auto data = ReadFile(filename);
    if (data.size() < 81)
    {
        ROS_ERROR_STREAM("Misformatted or otherwise incorrect STL file. Number of bytes: " << data.size());
        throw_arc_exception(std::runtime_error, "Unable to parse STL file");
    }

    const auto num_triangles = *reinterpret_cast<const int*>(&data[80]);

    // Sanity check the number of triangles
    if (num_triangles < 1)
    {
        ROS_ERROR_STREAM("Empty STL file load requested, num triangles: " << num_triangles);
        throw_arc_exception(std::runtime_error, "Unable to parse STL file");
    }
    else
    {
        ROS_INFO_STREAM("Loading mesh with " << num_triangles << " triangles");
    }

    // Sanity check the file size
    const auto expected_binary_file_size = num_triangles * 50 + 84;
    if (expected_binary_file_size != data.size())
    {
        ROS_ERROR_STREAM("Expected " << expected_binary_file_size << " bytes. Got " << data.size());
        throw_arc_exception(std::runtime_error, "Unable to parse STL file");
    }
    else
    {
        ROS_INFO_STREAM("Filesize matches expected size: " << data.size());
    }

    auto trimesh = boost::make_shared<btTriangleMesh>();
    std::vector<btVector3> manual_parsed_faces(num_triangles * 3);
    for (int i = 0; i < num_triangles; ++i)
    {
        const auto curr_triangle = reinterpret_cast<const MySTLTriangle*>(&data[84 + i * 50]);
        trimesh->addTriangle(toBtVector3(curr_triangle->vertex0),
                             toBtVector3(curr_triangle->vertex1),
                             toBtVector3(curr_triangle->vertex2),
                             true);

//        std::cout << "0: "; PrintVertex(curr_triangle->vertex0); std::cout << std::endl;
//        std::cout << "1: "; PrintVertex(curr_triangle->vertex0); std::cout << std::endl;
//        std::cout << "2: "; PrintVertex(curr_triangle->vertex0); std::cout << std::endl << std::endl;
        // TODO: should I be doing anything with the normals?

        manual_parsed_faces[3*i + 0] = toBtVector3(curr_triangle->vertex0);
        manual_parsed_faces[3*i + 1] = toBtVector3(curr_triangle->vertex1);
        manual_parsed_faces[3*i + 2] = toBtVector3(curr_triangle->vertex2);
    }

#if false
    const auto collision_shape = boost::make_shared<btBvhTriangleMeshShape>(trimesh.get(), true);
    return boost::make_shared<StlObject>(trimesh, collision_shape, mass, initial_transform, is_kinematic);
#endif

    CheckEquality(assimp_parsed_faces, manual_parsed_faces);

    ROS_INFO("Running convex decomposition");
    // Both of these need to be stored by someone (i.e.; the final StlObject)
    std::vector<boost::shared_ptr<btCollisionShape>> subshapes;
    boost::shared_ptr<btCompoundShape> compound = decomp.run(subshapes);
    return boost::make_shared<StlObject>(subshapes, compound, mass, initial_transform, is_kinematic);
}

EnvironmentObject::Ptr StlObject::copy(Fork &f) const
{
    Ptr o(new StlObject(*this));
    internalCopy(o, f);
    return o;
}
