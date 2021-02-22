#pragma once
#include <iostream>

using namespace Eigen;

typedef Array3f Color; // Like a Vector3f, but allows component-wise multiplication

// Some convenience procedures: These are all provided by Eigen, but
// require two nested constructor calls to get the final result, with
// the intermediate being some form of a generic Transform<...> type.
Quaternionf angleAxis(const float& angle, const Vector3f& axis);
Matrix4f toMat4(const Quaternionf& q);
Matrix4f rotate(const float& angle, const Vector3f& axis);
Matrix4f scale(const Vector3f& v);
Matrix4f translate(const Vector3f& v);

// Eigen provided no perspective projection, so we build our own, paterned off glFrustum.
Matrix4f frustum(float const& left,    float const& right,
                 float const& bottom,  float const& top, 
                 float const& nearVal, float const& farVal);

// Debug Print() making use of Eigen's nice formatting class.
template <typename T> void Print(const std::string& s, const T& m)
{ 
    IOFormat Fmt(3, DontAlignCols, ", ", "  |  ", "", "", ": [", "]\n");
    std::cout << s << ": " << m.format(Fmt); 
}

template <> void Print(const std::string& s, const float& f);
template <> void Print(const std::string& s, const double& f);
template <> void Print(const std::string& s, const Quaternionf& f);
