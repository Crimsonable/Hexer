#include "base.h"
#include <unordered_map>
#include <vector>

namespace Hexer {
struct GraphEdge;
class GraphVertex;
using Edge_list = std::vector<GraphEdge *>;

class GlobalID {
public:
  static GlobalID *getInstance();

  static ID getID();

private:
  GlobalID() = default;

  static ID _globe_counter;
};

class GraphSlot {
public:
  GraphSlot(ID _id, GraphVertex *_node = nullptr);

  ID getID() const;
  void setNode(GraphVertex *_node);

protected:
  GraphVertex *node;
  ID id;
};

class GraphVertex {
public:
  GraphVertex(ID _id);

  Edge_list operator()(Edge_list &&inputs);

  void addInput(GraphEdge *edge);
  void addOutput(GraphEdge *edge);

  ID getInput(int idx);
  ID getOutput(int idx);
  ID getID() const;

  Edge_list &getInputs();
  Edge_list &getOutputs();

private:
  virtual Edge_list eval() = 0;

protected:
  Edge_list inwards;
  Edge_list outwards;
  ID id;
};

struct GraphEdge {
  GraphEdge(ID id) : id_(id) {}

  ID tail_vertex;
  ID head_vertex;
  ID data_id;
  ID id_;
};

class Graph {
public:
  Graph();

  void insertVertex(GraphVertex *node);

  GraphEdge *connetVertex(GraphVertex *head, GraphVertex *tail, ID data_id);

private:
  std::unordered_map<ID, GraphVertex *> nodes;
  std::unordered_map<ID, GraphEdge *> edges;

  uint nodes_count, edge_count;
};

} // namespace Hexer