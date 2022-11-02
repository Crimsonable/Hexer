//#include <OpenMesh/Core/IO/BinaryHelper.hh>
#include <iostream>

#include "Hexer.h"
//#include "core/op.h"
//#include "core/ops.h"
#include "core/expr.h"
#include <vector>

using namespace std;

struct filt {
  std::pair<Hexer::uint, Hexer::uint> range() { return {1, 3}; }
};

int main() {
  std::vector<int> a{1, 2, 3, 4};
  Hexer::Stream<Hexer::ExecutePolicy::Hexer_Inplace, decltype(a), filt>(a,
                                                                        filt())
      .eval([](int &i) { i += 1; });

  system("pause");
  return 1;
}
