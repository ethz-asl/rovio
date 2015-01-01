/*
 * Copyright (C) 2014 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef BINARY_SERIALIZATION_H_
#define BINARY_SERIALIZATION_H_

#include <fstream>  // NOLINT
#include <iostream>  // NOLINT
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#ifndef CHECK_NOTNULL
#define CHECK_NOTNULL(x)  do {\
  if (!x) { \
    std::cout << "ERROR: CHECK(" << #x << ") Failed. " \
      " x = " << x << " " << std::endl << __FILE__ << ":" << __LINE__ \
      << std::endl; \
    assert(x); /* Stack trace in debug.*/\
    exit(1); \
  } \
  } while(0)
#endif  // CHECK_NOTNULL

#ifndef CHECK_EQ
#define CHECK_EQ(x, y) do {\
  if (!(x == y)) { \
    std::cout << "ERROR: CHECK_EQ(" << #x << ", " << #y << ") Failed. " \
      " " << x << " vs. " << y <<  " " << std::endl << __FILE__ << ":" \
      << __LINE__ << std::endl; \
    assert(x == y); /* Stack trace in debug.*/\
    exit(1); \
  } \
  } while(0)
#endif  // CHECK_EQ

template<class TYPE>
void Serialize(
    const TYPE& value, std::ostream* out,
    typename std::enable_if<std::is_integral<TYPE>::value>::type* = 0) {
  CHECK_NOTNULL(out);
  out->write(reinterpret_cast<const char*>(&value), sizeof(value));
}

template<class TYPE>
void Deserialize(TYPE* value, std::istream* in,
                 typename std::enable_if<std::is_integral<TYPE>::value>::type* =
                     0) {
  CHECK_NOTNULL(value);
  CHECK_NOTNULL(in);
  in->read(reinterpret_cast<char*>(value), sizeof(*value));
  CHECK_EQ(in->gcount(), static_cast<std::streamsize>(sizeof(*value)));
}

template<class TYPE>
void Serialize(
    const TYPE& value, std::ostream* out,
    typename std::enable_if<std::is_floating_point<TYPE>::value>::type* = 0) {
  CHECK_NOTNULL(out);
  out->write(reinterpret_cast<const char*>(&value), sizeof(value));
}

template<class TYPE>
void Deserialize(
    TYPE* value, std::istream* in,
    typename std::enable_if<std::is_floating_point<TYPE>::value>::type* = 0) {
  CHECK_NOTNULL(value);
  CHECK_NOTNULL(in);
  in->read(reinterpret_cast<char*>(value), sizeof(*value));
  CHECK_EQ(in->gcount(), static_cast<std::streamsize>(sizeof(*value)));
}

inline void Serialize(const char* memory_start, unsigned int memory_block_length_bytes,
                      std::ostream* out) {
  CHECK_NOTNULL(memory_start);
  CHECK_NOTNULL(out);
  out->write(memory_start, memory_block_length_bytes);
}

inline void Deserialize(char* memory_start, unsigned int memory_block_length_bytes,
                        std::istream* in) {
  CHECK_NOTNULL(memory_start);
  CHECK_NOTNULL(in);
  in->read(memory_start, memory_block_length_bytes);
  CHECK_EQ(in->gcount(),
           static_cast<std::streamsize>(memory_block_length_bytes));
}

template<typename T>
void Serialize(const std::vector<T>& value, std::ostream* out) {
  CHECK_NOTNULL(out);
  uint32_t length = value.size();
  Serialize(length, out);
  for (const T& entry : value) {
    Serialize(entry, out);
  }
}

template<typename T>
void Deserialize(std::vector<T>* value, std::istream* in) {
  CHECK_NOTNULL(value);
  CHECK_NOTNULL(in);
  value->clear();
  uint32_t length;
  Deserialize(&length, in);
  value->resize(length);
  for (T& entry : *value) {
    Deserialize(&entry, in);
  }
}

template<class Scalar, int Rows, int Cols, int C, int D, int E>
void Serialize(const Eigen::Matrix<Scalar, Rows, Cols, C, D, E>& M,
               std::ostream* out) {
  CHECK_NOTNULL(out);
  // Not using index type because this is different between x86-64 and arm.
  int rows = M.rows();
  int cols = M.cols();

  Serialize(rows, out);
  Serialize(cols, out);
  Serialize(reinterpret_cast<const char*>(M.data()),
            sizeof(Scalar) * rows * cols, out);
}

template<class Scalar, int Rows, int Cols, int C, int D, int E>
void Deserialize(Eigen::Matrix<Scalar, Rows, Cols, C, D, E>* M,
                 std::istream* in) {
  CHECK_NOTNULL(M);
  CHECK_NOTNULL(in);
  // Not using index type because this is different between x86-64 and arm.
  int rows, cols;
  Deserialize(&rows, in);
  Deserialize(&cols, in);

  CHECK_EQ(rows, Rows);
  CHECK_EQ(cols, Cols);

  Deserialize(reinterpret_cast<char*>(M->data()), sizeof(Scalar) * rows * cols,
              in);
}

template<class Scalar, int Cols, int C, int D, int E>
void Deserialize(Eigen::Matrix<Scalar, Eigen::Dynamic, Cols, C, D, E>* M,
                std::istream* in) {
  CHECK_NOTNULL(M);
  CHECK_NOTNULL(in);
  // Not using index type because this is different between x86-64 and arm.
  int rows, cols;
  Deserialize(&rows, in);
  Deserialize(&cols, in);

  CHECK_EQ(cols, Cols);
  M->resize(rows, Eigen::NoChange);

  Deserialize(reinterpret_cast<char*>(M->data()), sizeof(Scalar) * rows * cols,
              in);
}

template<class Scalar, int Rows, int C, int D, int E>
void Deserialize(Eigen::Matrix<Scalar, Rows, Eigen::Dynamic, C, D, E>* M,
                std::istream* in) {
  CHECK_NOTNULL(M);
  CHECK_NOTNULL(in);
  // Not using index type because this is different between x86-64 and arm.
  int rows, cols;
  Deserialize(&rows, in);
  Deserialize(&cols, in);

  CHECK_EQ(rows, Rows);
  M->resize(Eigen::NoChange, cols);

  Deserialize(reinterpret_cast<char*>(M->data()), sizeof(Scalar) * rows * cols,
              in);
}

template<class Scalar, int C, int D, int E>
void Deserialize(Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic,
                 C, D, E>* M, std::istream* in) {
  CHECK_NOTNULL(M);
  CHECK_NOTNULL(in);
  // Not using index type because this is different between x86-64 and arm.
  int rows, cols;
  Deserialize(&rows, in);
  Deserialize(&cols, in);

  M->resize(rows, cols);

  Deserialize(reinterpret_cast<char*>(M->data()), sizeof(Scalar) * rows * cols,
              in);
}

#endif  // BINARY_SERIALIZATION_H_
