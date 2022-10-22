#include "graph.h"

size_t Hexer::GlobalID::_globe_counter = 0;

typename Hexer::GlobalID::GlobalID *Hexer::GlobalID::getInstance() {
  static GlobalID instance;
  return &instance;
}

Hexer::ID Hexer::GlobalID::getID() {
  _globe_counter++;
  return _globe_counter;
}

Hexer::GraphVertex::GraphVertex(ID _id) : id(_id) {}

inline Hexer::ID Hexer::GraphVertex::getID() const { return id; }

inline void Hexer::GraphVertex::addInput(GraphEdge *edge) {
  edge->tail_vertex = this->id;
  inwards.push_back(edge);
}

inline void Hexer::GraphVertex::addOutput(GraphEdge *edge) {
  edge->head_vertex = this->id;
  outwards.push_back(edge);
}

inline Hexer::ID Hexer::GraphVertex::getInput(int idx) {
  return inwards[idx]->data_id;
}

inline Hexer::ID Hexer::GraphVertex::getOutput(int idx) {
  return outwards[idx]->data_id;
}

inline Hexer::Edge_list &Hexer::GraphVertex::getInputs() { return inwards; }

inline Hexer::Edge_list &Hexer::GraphVertex::getOutputs() { return outwards; }

Hexer::Graph::Graph() : nodes_count(0), edge_count(0) {}

void Hexer::Graph::insertVertex(GraphVertex *node) {
  nodes[node->getID()] = node;
}

Hexer::Edge_list Hexer::GraphVertex::operator()(Edge_list &&inputs) {
  //inwards = std::move(inputs);
  return eval();
}

Hexer::GraphEdge *Hexer::Graph::connetVertex(GraphVertex *head,
                                             GraphVertex *tail, ID data_id) {
  auto new_edge = new GraphEdge(GlobalID::getInstance()->getID());
  new_edge->data_id = data_id;
  head->addOutput(new_edge);
  tail->addInput(new_edge);
  return new_edge;
}

Hexer::GraphSlot::GraphSlot(ID _id, GraphVertex *_node)
    : id(_id), node(_node) {}

inline Hexer::ID Hexer::GraphSlot::getID() const { return id; }

inline void Hexer::GraphSlot::setNode(GraphVertex *_node) { _node = node; }
