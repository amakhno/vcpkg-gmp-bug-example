#pragma once

#ifdef _WIN32
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT
#endif

#include <Eigen/Core>

using namespace Eigen;

template <class T>
Eigen::Matrix<T, Dynamic, Dynamic, ColMajor> convert_matrix(const T* data, int cols, int rows)
{
    Matrix<T,Dynamic,Dynamic,RowMajor> M(cols, rows);
    std::memcpy(M.data(), data, rows * cols * sizeof(T));
    return M;
}

template <class T>
void convert_matrix(const Eigen::Matrix<T, Dynamic, Dynamic, ColMajor>& matrix, T* dest)
{
    Matrix<T,Dynamic,Dynamic,RowMajor> M = matrix;
    std::memcpy(dest, M.data(), M.rows() * M.cols() * sizeof(T));
}