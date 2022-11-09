
namespace Hexer {

template <typename... Paras> class OpBase {};
template <typename T> class UnaryOp : public OpBase<T> {};

template <typename T> class NoneOp : public UnaryOp<T> {};

template <typename Derived, typename DataType> class Expr {
public:
  Derived *cast_to() { return static_cast<Derived *>(this); }
};

template <typename Op, typename DataType>
class Container : public Expr<Container<Op, DataType>> {
  using Self = Container<Op, DataType>;

public:
  Container(DataType *data) : _data(data) {}

  template <typename FilterOp> SLFilter<FilterOp, Self> filter_sl() {
    return SLFilter<FilterOp, Self>(*this);
  }

  DataType *_data;
};

template <typename Op, typename PreExp>
class SLOperation : public Expr<SLOperation<Op, PreExp>> {
public:
  SLOperation(PreExp &&exp) : _exp(exp) {}

private:
  PreExp _exp;
};

template <typename FilterOp, typename PreExp>
class SLFilter : public Expr<SLFilter<FilterOp, PreExp>> {
public:
  SLFilter(PreExp &&exp) : _exp(exp) {}

private:
  PreExp _exp;
};
//template <typename Saver> struct Executor;

template <typename Saver> struct Executor {
  template <typename Container, typename DataType, typename Exp,typename... Args>
  static inline void run(TensorBase<Container, DataType, device> *dst,
                         const ExpBase<Exp, DataType, exp_type> &exp,
                         Args &&...args) {
    auto dst_shape = ShapeCheck::Check(dst->derived_to());
    auto exp_shape = ShapeCheck::Check(exp.derived_to());

    if (dst_shape == exp_shape) {
      CPUEngine<Saver, TensorBase<Container, DataType, device>, Exp,
                exp_type>::dispatch(dst, exp, std::forward<Args>(args)...);
    }
  }
};

template <typename Saver, typename DataType> struct ExpEngine {
  template <typename Dst, typename Exp, typename... Args>
  inline static void eval(Dst *dst, const Expr<Exp, DataType> &exp,
                          Args &&...args) {
    Executor<Saver>::run(dst, exp, std::forward<Args>(args)...);
  }
};

void test() {
  int *data = nullptr;
  auto ct = Container<NoneOp<int>, int>(data);
}
} // namespace Hexer