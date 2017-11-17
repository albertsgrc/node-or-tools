#ifndef NODE_OR_TOOLS_MATRIX_F83F49233E85_H
#define NODE_OR_TOOLS_MATRIX_F83F49233E85_H

#include <cstdint>
#include <vector>

using namespace std;

template <typename T> class Matrix {
  static_assert(std::is_arithmetic<T>::value, "Matrix<T> requires T to be integral or floating point");

public:
  using Value = T;

  Matrix() = default;
  Matrix(std::int32_t n_) : n{n_} {
    if (n < 0)
      throw std::runtime_error{"Negative dimension"};

    data.resize(n * n);
  }


  std::int32_t dim() const { return n; }
  std::int32_t size() const { return dim() * dim(); }

  static int digits(int n) {
    int c = 0;
    while (n >= 10) { n /= 10; ++c; }
    return c + 1;
  }

  static int abs(int x)  {
    if (x < 0) return -x;
    return x;
  }



    static string pad(const string& x, int total, char c = ' ') {
      return x + std::string(total - x.length(), c);
    }

  void print(ostream& s) const {
    int maxLength = 1;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            int length = std::to_string(this->at(i, j)).length();
            if (maxLength < length) {
                maxLength = length;
            }
        }
    }

    int indexLength = std::to_string(n - 1).length();
    if (indexLength > maxLength) maxLength = indexLength;

    // Espais abans header
    s << std::string(indexLength, ' ') << "   ";

    // Header
    for (int i = 0; i < n; ++i)
        s << pad(to_string(i), maxLength) << ' ';

    s << endl << endl;

    for (int i = 0; i < n; ++i) {
        s << pad(to_string(i), indexLength) << "   ";
        for (int j = 0; j < n; ++j) {
            s << pad(to_string(this->at(i, j)), maxLength) << ' ';
        }
        s << endl << endl;
    }
  }

  T& at(std::int32_t x, std::int32_t y) { return data.at(y * n + x); }
  const T& at(std::int32_t x, std::int32_t y) const { return data.at(y * n + x); }

private:
  std::int32_t n;
  std::vector<T> data;
};

#endif
