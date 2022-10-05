#include "base.h"
#include <string>
#include "graph.h"

namespace Hexer {


class MeshIO : public GraphVertex {
 public:
  bool readFromFile(const std::string& path);
  bool writeToFile(const std::string& path);
};
}  // namespace Hexer