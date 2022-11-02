#include "base.h"

#include <algorithm>
#include <type_traits>

namespace Hexer {

enum class ExecutePolicy { Hexer_Inplace, Hexer_Outplace };

template <typename Derived> class Expr {};

template <ExecutePolicy Policy, typename Container, typename Filter>
class Stream;

template <typename Container, typename Filter>
class Stream<ExecutePolicy::Hexer_Inplace, Container, Filter>
    : public Expr<Stream<ExecutePolicy::Hexer_Inplace, Container, Filter>> {
  using Self = typename Stream<ExecutePolicy::Hexer_Inplace, Container, Filter>;

public:
  Stream(Container &data, Filter* filter) : _filter(filter), _data(data) {}

  template <typename Op>
  Stream<ExecutePolicy::Hexer_Inplace, Container, void> eval(Op &&op) {
    std::pair<uint, uint> range = filter.range();
    std::for_each(_data.begin + range.first, _data.begin + range.second,
                  std::forward<Op>(op));
    return Stream<ExecutePolicy::Hexer_Outplace, Container, void>(_data,
                                                                  nullptr);
  }

  auto inplace() { return *this; }

  auto outplace() {
    return Stream<ExecutePolicy::Hexer_Outplace, Container, Filter>(_data,
                                                                    _filter);
  }

  template <typename New_Filter>
  Stream<ExecutePolicy::Hexer_Inplace, Container, New_Filter>
  filter(New_Filter &&filter) {
    return Stream<ExecutePolicy::Hexer_Inplace, Container, New_Filter>(
        _data, std::forward<New_Filter>(filter));
  }

private:
  Filter *_filter;
  Container &_data;
};

} // namespace Hexer