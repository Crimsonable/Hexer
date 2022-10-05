#include <iostream>
#include <OpenMesh/Core/IO/BinaryHelper.hh>

#include "Hexer.h"
#include "core/op.h"

using namespace std;

int main() {
  Hexer::Graph graph;
  auto ID_generator = Hexer::GlobalID::getInstance();

  Hexer::GraphVertex *node1 = new Hexer::GraphVertex(ID_generator->getID());
  Hexer::GraphVertex *node2 = new Hexer::GraphVertex(ID_generator->getID());
  auto node1_slot_output = new Hexer::GraphSlot(ID_generator->getID());
  auto node2_slot_input = new Hexer::GraphSlot(ID_generator->getID());
  node1->addOutput(node1_slot_output);
  node2->addInput(node2_slot_input);

  graph.insertVertex(node1);
  graph.insertVertex(node2);
  graph.connetVertex(node1, node1_slot_output, node2, node2_slot_input);

  auto node1_outward = node1->retriveOutward();
  auto node2_inwad = node2->retriveInward();

  std::cout << "node 1 outward: ";
  for (auto &&it : node1_outward)
    std::cout << it << " ";
  std::cout << std::endl;
  std::cout << "node 2 inward: ";
  for (auto &&it : node2_inwad)
    std::cout << it << " ";

  system("pause");
  return 1;
}
