#include <vector>
#include <unordered_map>
#include "base.h"

namespace Hexer {
struct GraphEdge;
class GraphVertex;

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

  void addInput(GraphSlot *input);
  void addOutput(GraphSlot *output);
  GraphSlot *getInput(int idx);
  GraphSlot *getOutput(int idx);
  std::vector<GraphSlot *> &getInputs();
  std::vector<GraphSlot *> &getOutputs();

  std::vector<ID> retriveInward();
  std::vector<ID> retriveOutward();

  ID getID() const;
  GraphEdge *&firstInEdge();
  GraphEdge *&firstOutEdge();

protected:
  std::vector<GraphSlot *> inputs;
  std::vector<GraphSlot *> outputs;

  GraphEdge *firstIn = nullptr;
  GraphEdge *firstOut = nullptr;
  ID id;
};

struct GraphEdge {
  ID tail_vertex;
  ID head_vertex;
  GraphEdge *head_link = nullptr;
  GraphEdge *tail_link = nullptr;

  GraphSlot *startSlot = nullptr, *endSlot = nullptr;
};

class Graph {
public:
  Graph();

  void insertVertex(GraphVertex *node);

  void connetVertex(GraphVertex *head, GraphSlot *start_slot, GraphVertex *tail,
                    GraphSlot *end_slot);

private:
  std::unordered_map<ID, GraphVertex *> nodes;
  std::unordered_map<ID, GraphSlot *> slots;

  uint nodes_count, edge_count;
};

} // namespace Hexer