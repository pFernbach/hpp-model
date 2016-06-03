// Copyright (c) 2016, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-model.
// hpp-model is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-model is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-model. If not, see <http://www.gnu.org/licenses/>.

# include <assimp/Importer.hpp>      // C++ importer interface
# include <assimp/scene.h>           // Output data structure
# include <assimp/postprocess.h>     // Post processing flags

# include <hpp/fcl/BV/OBBRSS.h>
# include <hpp/fcl/BVH/BVH_model.h>

typedef fcl::BVHModel< fcl::OBBRSS > PolyhedronType;
typedef boost::shared_ptr <PolyhedronType> PolyhedronPtrType;

bool buildMesh (const aiScene* scene,
    const aiNode* node,
    std::vector<unsigned>& subMeshIndexes,
    std::vector<fcl::Vec3f>& vertices,
    std::vector<fcl::Triangle>& triangles,
    const PolyhedronPtrType& mesh)
{
  if (!node) return false;
  bool success = true;

  aiMatrix4x4 transform = node->mTransformation;
  aiNode *pnode = node->mParent;
  while (pnode)
  {
    // Don't convert to y-up orientation, which is what the root node in
    // Assimp does
    if (pnode->mParent != NULL)
      transform = pnode->mTransformation * transform;
    pnode = pnode->mParent;
  }

  for (uint32_t i = 0; i < node->mNumMeshes; i++) {
    aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];

    unsigned oldNbPoints = mesh->num_vertices;
    unsigned oldNbTriangles = mesh->num_tris;

    // Add the vertices
    for (uint32_t j = 0; j < input_mesh->mNumVertices; j++) {
      aiVector3D p = input_mesh->mVertices[j];
      p *= transform;
      vertices.push_back (fcl::Vec3f (p.x, p.y, p.z));
    }

    // add the indices
    for (uint32_t j = 0; j < input_mesh->mNumFaces; j++) {
      aiFace& face = input_mesh->mFaces[j];
      if (face.mNumIndices != 3) {
        std::cout << "Mesh has a face with "
          << face.mNumIndices << " vertices. This is not supported\n";
        std::cout << "Node name is: " << node->mName.C_Str() << "\n";
        std::cout << "Mesh index: " << i << "\n";
        std::cout << "Face index: " << j << "\n";
        success = false;
      }
      triangles.push_back (fcl::Triangle
          (oldNbPoints + face.mIndices[0],
           oldNbPoints + face.mIndices[1],
           oldNbPoints + face.mIndices[2]));
    }

    // Save submesh triangles indexes interval.
    if (subMeshIndexes.size () == 0)
      subMeshIndexes.push_back (0);

    subMeshIndexes.push_back (oldNbTriangles + input_mesh->mNumFaces);
  }

  for (uint32_t i=0; i < node->mNumChildren; ++i) {
    if (!buildMesh(scene, node->mChildren[i], subMeshIndexes, vertices, triangles, mesh))
      success = false;
  }
  return success;
}

bool meshFromAssimpScene (const std::string& name,
    const aiScene* scene,
    const PolyhedronPtrType& mesh)
{
  if (!scene->HasMeshes()) {
    throw std::runtime_error (std::string ("No meshes found in file ")+ name);
  }

  std::vector<unsigned> subMeshIndexes;
  int res = mesh->beginModel ();
  if (res != fcl::BVH_OK) {
    std::ostringstream error;
    error << "fcl BVHReturnCode = " << res;
    throw std::runtime_error (error.str ());
  }
  std::vector<fcl::Vec3f> vertices;
  std::vector<fcl::Triangle> triangles;
  bool success = buildMesh (scene, scene->mRootNode, subMeshIndexes, vertices, triangles, mesh);
  if (success)
    mesh->addSubModel (vertices, triangles);
  else
    std::cout << "Not calling addSubModel because the file was not parsed properly." << std::endl;
  mesh->endModel ();
  return success;
}

bool loadPolyhedronFromResource (
    const std::string& filename, const PolyhedronPtrType& polyhedron)
{
  Assimp::Importer importer;
  // set list of ignored parameters (parameters used for rendering)
  importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
      aiComponent_TANGENTS_AND_BITANGENTS|
      aiComponent_COLORS |
      aiComponent_BONEWEIGHTS |
      aiComponent_ANIMATIONS |
      aiComponent_LIGHTS |
      aiComponent_CAMERAS|
      aiComponent_TEXTURES |
      aiComponent_TEXCOORDS |
      aiComponent_MATERIALS |
      aiComponent_NORMALS
      );
  const aiScene* scene = importer.ReadFile(filename,
      aiProcess_SortByPType | 
      aiProcess_Triangulate |
      aiProcess_RemoveComponent |
      aiProcess_JoinIdenticalVertices);
  if (!scene) {
    throw std::runtime_error (std::string ("Could not load resource ") + filename);
  }

  return meshFromAssimpScene (filename, scene, polyhedron);
}

void usage(const char* prog) {
  std::cout << prog << " filename ... [filename]" << std::endl;
}

int main(int argc, char* argv[])
{
  if (argc <= 1) {
    usage (argv[0]);
    return EXIT_FAILURE;
  }
  for (int i = 1; i < argc; ++i) {
    PolyhedronPtrType polyhedron (new PolyhedronType());
    const std::string filename = argv[i];
    std::cout << "--> Checking " << filename << "..." << std::endl;
    if (!loadPolyhedronFromResource(filename, polyhedron))
      std::cout << "Something seems wrong" << std::endl;
    std::cout << "--> Done with " << filename << "." << std::endl;
  }
  return EXIT_SUCCESS;
}
