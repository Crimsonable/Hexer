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

void Hexer::GraphVertex::addInput(GraphSlot *input) {
  input->setNode(this);
  inputs.push_back(input);
}

void Hexer::GraphVertex::addOutput(GraphSlot *output) {
  output->setNode(this);
  outputs.push_back(output);
}

inline Hexer::GraphSlot *Hexer::GraphVertex::getInput(int idx) {
  return inputs[idx];
}

inline Hexer::GraphSlot *Hexer::GraphVertex::getOutput(int idx) {
  return outputs[idx];
}

inline std::vector<Hexer::GraphSlot *> &Hexer::GraphVertex::getInputs() {
  return inputs;
}

inline std::vector<Hexer::GraphSlot *> &Hexer::GraphVertex::getOutputs() {
  return outputs;
}

Hexer::Graph::Graph() : nodes_count(0), edge_count(0) {}

void Hexer::Graph::insertVertex(GraphVertex *node) {
  nodes[node->getID()] = node;
  for (auto &&it : node->getInputs())
    slots[it->getID()] = it;
  for (auto &&it : node->getOutputs())
    slots[it->getID()] = it;
}

void Hexer::Graph::connetVertex(GraphVertex *head, GraphSlot *start_slot,
                                GraphVertex *tail, GraphSlot *end_slot) {
  auto insert_edge = new GraphEdge;
  insert_edge->head_vertex = head->getID();
  insert_edge->tail_vertex = tail->getID();
  insert_edge->startSlot = start_slot;
  insert_edge->endSlot = end_slot;
  if (!head->firstOutEdge())
    head->firstOutEdge() = insert_edge;
  else {
    auto temp_edge = head->firstOutEdge()->head_link;
    while (temp_edge)
      temp_edge = temp_edge->head_link;
    temp_edge = insert_edge;
  }

  if (!tail->firstInEdge())
    tail->firstInEdge() = insert_edge;
  else {
    auto temp_edge = tail->firstInEdge()->tail_link;
    while (temp_edge)
      temp_edge = temp_edge->tail_link;
    temp_edge = insert_edge;
  }
}

std::vector<Hexer::ID> Hexer::GraphVertex::retriveInward() {
  std::vector<ID> res;
  auto start_edge = firstIn;
  while (start_edge) {
    res.push_back(start_edge->tail_vertex);
    start_edge = start_edge->head_link;
  }
  return res;
}

std::vector<Hexer::ID> Hexer::GraphVertex::retriveOutward() {
  std::vector<ID> res;
  auto start_edge = firstOut;
  while (start_edge) {
    res.push_back(start_edge->head_vertex);
    start_edge = start_edge->tail_link;
  }
  return res;
}

inline Hexer::ID Hexer::GraphVertex::getID() const { return id; }

inline Hexer::GraphEdge *&Hexer::GraphVertex::firstInEdge() { return firstIn; }

inline Hexer::GraphEdge *&Hexer::GraphVertex::firstOutEdge() {
  return firstOut;
}

Hexer::GraphSlot::GraphSlot(ID _id, GraphVertex *_node)
    : id(_id), node(_node) {}

inline Hexer::ID Hexer::GraphSlot::getID() const { return id; }

inline void Hexer::GraphSlot::setNode(GraphVertex *_node) { _node = node; }
