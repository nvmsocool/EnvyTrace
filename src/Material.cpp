#include "Material.h"
#include "imgui.h"

float Material::D(Eigen::Vector3f h, Eigen::Vector3f N)
{
  float a_2 = alpha * alpha;
  float h_N = h.dot(N);
  if (h_N == 0 || std::abs(h_N) > 1.f)
    return 0;
  float tan_theta = std::sqrt(1 - h_N * h_N) / h_N;
  float denom = (pi * (float)(std::pow(h_N, 4) * std::pow(a_2 + tan_theta * tan_theta, 2)));
  if (denom < 0.001f)
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

float Material::G_1(Eigen::Vector3f w, Eigen::Vector3f h, Eigen::Vector3f N)
{
  float w_N = w.dot(N);
  if (abs(w_N) > 1.f)
    w_N = abs(w_N) / w_N;
  float tan_theta = std::sqrt(1 - w_N * w_N) / w_N;
  if (tan_theta == 0.0f)
    return 1.0f;
  if (w_N == 0)
    return 0.f;
  float g = Charictaristic(w.dot(h) / w_N) * 2 / (1 + std::sqrt(1 + alpha * alpha * tan_theta * tan_theta));
  return g;
}

std::string Material::Serialize()
{
  std::string ret = "brdf ";
  for (size_t i = 0; i < 3; i++)
  {
    ret += std::to_string(Kd[i]) + " ";
  }
  for (size_t i = 0; i < 3; i++)
  {
    ret += std::to_string(Ks[i]) + " ";
  }
  ret += std::to_string(alpha) + " ";
  ret += std::to_string(specularity) + " ";
  return ret;
}

bool Light::RenderGUI(size_t shape_num)
{
  bool something_changed = false;

  if (ImGui::CollapsingHeader("light material"))
  {
    ImGui::Indent(10.f);
    something_changed |= ImGui::DragFloat3((std::string("light val##") + std::to_string(shape_num)).data(), light_value.data(), 0.1f, 0, 1000000, "%.1f");
    something_changed |= Material::RenderGUI(shape_num);
    ImGui::Unindent(10.f);
  }

  return something_changed;
}

std::string Light::Serialize()
{
  std::string ret = "light ";
  for (size_t i = 0; i < 3; i++)
  {
    ret += std::to_string(light_value[i]) + " ";
  }
  return ret;
}

bool Material::RenderGUI(size_t shape_num)
{
  bool something_changed = false;

  if (ImGui::CollapsingHeader("material"))
  {
    ImGui::Indent(10.f);
    something_changed |= ImGui::SliderFloat3((std::string("Kd##") + std::to_string(shape_num)).data(), Kd.data(), 0, 1, "%.2f");
    something_changed |= ImGui::SliderFloat3((std::string("Ks##") + std::to_string(shape_num)).data(), Ks.data(), 0, 1, "%.2f");

    something_changed |= ImGui::DragFloat((std::string("alpha##") + std::to_string(shape_num)).data(), &alpha, 0.01f, 0, 10000, "%.2f");
    something_changed |= ImGui::SliderFloat((std::string("specularity##") + std::to_string(shape_num)).data(), &specularity, 0, 1, "%.2f");
    ImGui::Unindent(10.f);
  }

  return something_changed;
}