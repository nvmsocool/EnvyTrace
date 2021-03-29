#include "Material.h"

float Material::D(Eigen::Vector3f h, Eigen::Vector3f N)
{
  float a_2 = alpha * alpha;
  float h_N = h.dot(N);
  if (h_N == 0 || std::abs(h_N) > 1.f)
    return 0;
  float tan_theta = std::sqrt(1 - h_N * h_N) / h_N;
  float denom = (PI * std::pow(h_N, 4) * std::pow(a_2 + tan_theta * tan_theta, 2));
  if (denom == 0)
    return 1.f;
  return Charictaristic(h_N) * a_2 / denom;
}

Eigen::Vector3f Material::F(float d)
{
  return Ks + (Eigen::Vector3f::Ones() - Ks) * std::pow((1 - d), 5);
}

float Material::G(Eigen::Vector3f w_i, Eigen::Vector3f w_o, Eigen::Vector3f h, Eigen::Vector3f N)
{
  return G_1(w_i, h, N) * G_1(w_o, h, N);
}

#include <iostream>;

float Material::G_1(Eigen::Vector3f w, Eigen::Vector3f h, Eigen::Vector3f N)
{
  float w_N = w.dot(N);
  if (abs(w_N) > 1.f) w_N = abs(w_N) / w_N;
  float tan_theta = std::sqrt(1 - w_N * w_N) / w_N;
  if (tan_theta == 0.0f)
    return 1.0f;
  if (w_N == 0)
    return 0.f;
  float g = Charictaristic(w.dot(h) / w_N) * 2 / (1 + std::sqrt(1 + alpha * alpha * tan_theta * tan_theta));
  return g;
}
