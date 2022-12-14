#pragma once
#include "base.h"
#include "shader.h"

#include <Expblas/ftensor.h>
#include <string>
#include <vector>

#define MAX_BONE_INFLUENCE 4

namespace Visual {
struct Vertex {
  // position
  Expblas::vec3f Position;
  // normal
  Expblas::vec3f Normal;
  // texCoords
  Expblas::vec2f TexCoords;
  // tangent
  Expblas::vec3f Tangent;
  // bitangent
  Expblas::vec3f Bitangent;
  // bone indexes which will influence this vertex
  int m_BoneIDs[MAX_BONE_INFLUENCE];
  // weights from each bone
  float m_Weights[MAX_BONE_INFLUENCE];
};

struct Texture {
  unsigned int id;
  std::string type;
  std::string path;
};

class Mesh {
public:
  // mesh Data
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;
  std::vector<Texture> textures;
  unsigned int VAO;

  // constructor
  Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices,
       std::vector<Texture> textures = {});

  // Mesh(Mesh &&mesh);

  Mesh() {}

  // render the mesh
  void Draw(Shader &shader);

  void setGeometry(const std::vector<Vertex> &vertices,
                   const std::vector<unsigned int> &indices);

  // initializes all the buffer objects/arrays
  void setupMesh();

private:
  // render data
  unsigned int VBO, EBO;
};
} // namespace Visual