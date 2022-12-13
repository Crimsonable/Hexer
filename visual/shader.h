#pragma once
#include <Expblas/ftensor.h>
#include <fstream>
#include <glad/glad.h>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>

namespace Visual {
class Shader {
public:
  unsigned int ID;

  Shader() {}
  // constructor generates the shader on the fly
  // ------------------------------------------------------------------------
  Shader(const char *vertexPath, const char *fragmentPath,
         const char *geometryPath = nullptr);
  // activate the shader
  // ------------------------------------------------------------------------
  void use();
  // utility uniform functions
  // ------------------------------------------------------------------------
  void setBool(const std::string &name, bool value) const;
  // ------------------------------------------------------------------------
  void setInt(const std::string &name, int value) const;
  // ------------------------------------------------------------------------
  void setFloat(const std::string &name, float value) const;
  // ------------------------------------------------------------------------
  void setVec2(const std::string &name, const Expblas::vec2f &value) const;

  void setVec2(const std::string &name, float x, float y) const;
  // ------------------------------------------------------------------------
  void setVec3(const std::string &name, const Expblas::vec3f &value) const;

  void setVec3(const std::string &name, float x, float y, float z) const;
  // ------------------------------------------------------------------------
  void setVec4(const std::string &name, const Expblas::vec4f &value) const;

  void setVec4(const std::string &name, float x, float y, float z, float w);
  // ------------------------------------------------------------------------
  void setMat2(const std::string &name, const Expblas::mat2f &mat) const;
  // ------------------------------------------------------------------------
  void setMat3(const std::string &name, const Expblas::mat3f &mat) const;
  // ------------------------------------------------------------------------
  void setMat4(const std::string &name, const Expblas::mat4f &mat) const;

private:
  // utility function for checking shader compilation/linking errors.
  // ------------------------------------------------------------------------
  void checkCompileErrors(GLuint shader, std::string type);
};

extern std::unordered_map<std::string, Shader> gShaderMap;
} // namespace Visual