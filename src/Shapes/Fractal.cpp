#include "Fractal.h"
#include "..\material.h"

static const Eigen::Vector3f norm_step_x(1.f, 0.0f, 0.0f);
static const Eigen::Vector3f norm_step_y(0.0f, 1.f, 0.0f);
static const Eigen::Vector3f norm_step_z(0.0f, 0.0f, 1.f);

void Fractal::Intersect(const Ray &in, Intersection &i)
{
  // plan: step along hte ray, starting at the ray point, with the step size of DE.
  // if it keeps getting bigger then return no intersection
  // else if its below a threshold, return an intersection

  float dist = 0;
  int colorsteps = 0;
  int totalColorSteps = max_iteration;

  for (int steps = 0; steps < max_iteration; steps++)
  {
    Eigen::Vector3f p = in.origin + dist * in.direction;
    float estimate = DE_Generic(p);
    dist += estimate;
    if (dist > originErrorMargin && estimate < min_distance)
    {
      //intersection, huzzah
      i.t = dist;
      i.P = in.eval(i.t);
      i.object = this;

      if (flat_color)
      {
        i.Kd = material->Kd;
      }
      else
      {
        Eigen::Vector3f it_based, fold_based;
        if (color_it_fold_ratio > 0)
        {
          float f = (static_cast<float>(colorsteps) / static_cast<float>(max_iteration)) * color_it_scale + color_it_add;
          while (f > 1)
            f -= 1;
          it_based = ColorFromFloat(f);
        }

        if (color_it_fold_ratio < 1)
        {
          fold_based = FoldBased(i.P);
        }

        i.Kd = color_it_fold_ratio * it_based + (1.f - color_it_fold_ratio) * fold_based;

        if (color_intensity < 0)
          i.Kd *= (1 + color_intensity);
        else
          i.Kd += (Eigen::Vector3f::Ones() - i.Kd) * color_intensity;
      }

      //norm needs to be estimated
      Eigen::Vector3f step_back = in.origin + (dist - min_distance) * in.direction;
      float step_size = DE_Generic(step_back);
      i.N = Eigen::Vector3f(
          DE_Generic(step_back + step_size * norm_step_x) - DE_Generic(step_back - step_size * norm_step_x),
          DE_Generic(step_back + step_size * norm_step_y) - DE_Generic(step_back - step_size * norm_step_y),
          DE_Generic(step_back + step_size * norm_step_z) - DE_Generic(step_back - step_size * norm_step_z))
                .normalized();

      return;
    }
    colorsteps += 1;
  }
}

bool Fractal::RenderGUI(size_t n)
{
  bool something_changed = false;

  ImGui::Text("Marching");
  ImGui::Indent(5.f);
  something_changed |= ImGui::InputInt((std::string("max_iteration##") + std::to_string(n)).data(), &max_iteration);
  something_changed |= ImGui::DragFloat((std::string("min_distance##") + std::to_string(n)).data(), &min_distance, 0.00001f, 0.0f, 1000.0f, "%.5f");
  something_changed |= ImGui::InputInt((std::string("num_subdivisions##") + std::to_string(n)).data(), &num_subdivisions);
  ImGui::Unindent(5.f);

  ImGui::Separator();

  ImGui::Text("Transform");
  ImGui::Indent(5.f);
  if (ImGui::DragFloat3((std::string("center##") + std::to_string(n)).data(), Center.data(), 0.01f, -10000, 10000, "%.2f"))
  {
    something_changed = true;
    this->Position = Center;
  }
  something_changed |= ImGui::DragFloat((std::string("scale##") + std::to_string(n)).data(), &Scale, 0.001f, 0, 100000, "%.3f");
  if (ImGui::DragFloat3((std::string("rotation##") + std::to_string(n)).data(), &rot_eulers[0], 0.1f, -180, 180, "%.1f"))
  {
    something_changed = true;
    rot = EulerToQuat(rot_eulers);
    rot_inv = rot.inverse();
  }
  ImGui::Unindent(5.f);

  ImGui::Separator();
  ImGui::Text("Color");
  ImGui::Indent(5.f);
  something_changed |= ImGui::Checkbox((std::string("flat_color##") + std::to_string(n)).data(), &flat_color);
  if (!flat_color)
  {
    something_changed |= ImGui::SliderFloat((std::string("ratio##") + std::to_string(n)).data(), &color_it_fold_ratio, 0.f, 1.f);
    something_changed |= ImGui::SliderFloat((std::string("intensity##") + std::to_string(n)).data(), &color_intensity, -1.f, 1.f);
    if (color_it_fold_ratio > 0)
    {
      something_changed |= ImGui::SliderFloat((std::string("it_add##") + std::to_string(n)).data(), &color_it_add, 0.f, 1.f);
      something_changed |= ImGui::DragFloat((std::string("it_scale##") + std::to_string(n)).data(), &color_it_scale, 0.1f, 0.f, 1000.f, "%.1f");
    }
  }
  ImGui::Unindent(5.f);

  ImGui::Separator();
  ImGui::Text("Actions:");
  ImGui::Indent(5.f);

  for (size_t i = 0; i < CombinedActions.size(); i++)
  {

    ImGui::Separator();
    const char *items[] = {
      "FOLD",
      "ROTATION",
      "SCALE",
      "TRANSLATE",
      "MODULO",
      "POWER",
      "C_ITERATION"
    };

    if (ImGui::Combo((std::string("type##") + std::to_string(n) + std::to_string(i)).data(), &CombinedActions[i].action_type, items, IM_ARRAYSIZE(items), 4))
    {
      something_changed = true;
      switch (CombinedActions[i].action_type)
      {
      case Fractal::ACTION_TYPE::FOLD:
        CombinedActions[i].DisplayOp = Eigen::Vector3f(1, 0, 0);
        CombinedActions[i].VecOp = Eigen::Vector3f(1, 0, 0);
        CombinedActions[i].VecOp2 = Eigen::Vector3f(2, 0, 0);
        break;
      case Fractal::ACTION_TYPE::ROTATION:
        CombinedActions[i].DisplayOp = Eigen::Vector3f(0, 0, 0);
        CombinedActions[i].QuatOp = EulerToQuat(CombinedActions[i].DisplayOp);
        break;
      case Fractal::ACTION_TYPE::SCALE:
        CombinedActions[i].VecOp = Eigen::Vector3f(1, 1, 1);
        break;
      case Fractal::ACTION_TYPE::TRANSLATE:
        CombinedActions[i].VecOp = Eigen::Vector3f(0, 0, 0);
        break;
      case Fractal::ACTION_TYPE::MODULO:
        CombinedActions[i].VecOp = Eigen::Vector3f(100, 100, 100);
        break;
      case Fractal::ACTION_TYPE::POWER:
        CombinedActions[i].VecOp = Eigen::Vector3f(1, 1, 1);
        break;
      case Fractal::ACTION_TYPE::C_ITERATION:
        CombinedActions[i].IntOp = 0;
        break;
      }
      CombinedActions[i].VecOp = CombinedActions[i].DisplayOp;
    }

    switch (CombinedActions[i].action_type)
    {
    case Fractal::ACTION_TYPE::FOLD:
      if (ImGui::DragFloat3((std::string("fold_plane##") + std::to_string(n) + std::to_string(i)).data(), CombinedActions[i].DisplayOp.data(), 0.001f, -1.f, 1.f, "%.3f"))
      {
        something_changed = true;
        CombinedActions[i].VecOp = CombinedActions[i].DisplayOp.normalized();
        CombinedActions[i].VecOp2 = 2 * CombinedActions[i].DisplayOp.normalized();
      }
      if (ImGui::DragFloat3(("fold_color##" + std::to_string(n) + std::to_string(i)).data(), CombinedActions[i].Color.data(), 0.1f, 0.f, 1.f, "%.1f"))
      {
        something_changed = true;
      }
      break;
    case Fractal::ACTION_TYPE::ROTATION:
      if (ImGui::DragFloat3((std::string("rotation##") + std::to_string(n) + std::to_string(i)).data(), CombinedActions[i].DisplayOp.data(), 0.01f, -180, 180.f, "%.2f"))
      {
        something_changed = true;
        CombinedActions[i].QuatOp = EulerToQuat(CombinedActions[i].DisplayOp);
      }
      break;
    case Fractal::ACTION_TYPE::SCALE:
      something_changed |= ImGui::DragFloat3((std::string("scale##") + std::to_string(n) + std::to_string(i)).data(), CombinedActions[i].VecOp.data(), 0.001f, -1000, 1000, "%.3f");
      break;
    case Fractal::ACTION_TYPE::TRANSLATE:
      something_changed |= ImGui::DragFloat3((std::string("translation##") + std::to_string(n) + std::to_string(i)).data(), CombinedActions[i].VecOp.data(), 0.001f, -1000, 1000, "%.3f");
      break;
    case Fractal::ACTION_TYPE::MODULO:
      something_changed |= ImGui::DragFloat3((std::string("modulo##") + std::to_string(n) + std::to_string(i)).data(), CombinedActions[i].VecOp.data(), 0.001f, -1000, 1000, "%.3f");
      break;
    case Fractal::ACTION_TYPE::POWER:
      something_changed |= ImGui::DragFloat3((std::string("power##") + std::to_string(n) + std::to_string(i)).data(), CombinedActions[i].VecOp.data(), 0.001f, -1000, 1000, "%.3f");
      break;
    case Fractal::ACTION_TYPE::C_ITERATION:
      something_changed |= ImGui::DragInt((std::string("c_iteration##") + std::to_string(n) + std::to_string(i)).data(), &CombinedActions[i].IntOp, 1, 0, 10000, "%.3f");
      break;
    }
    if (ImGui::Button((std::string("+##") + std::to_string(n) + std::to_string(i)).data()))
    {
      something_changed = true;
      //insert new operation
      CombinedActions.insert(CombinedActions.begin() + i, CombinedActions[i]);
      if (CombinedActions[i].action_type == 0)
      {
        GenColors();
      }
    }

    ImGui::SameLine();
    if (ImGui::Button((std::string("X##") + std::to_string(n) + std::to_string(i)).data()))
    {
      something_changed = true;
      //delete this operation
      CombinedActions.erase(CombinedActions.begin() + i);
      if (CombinedActions[i].action_type == 0)
      {
        GenColors();
      }
    }
  }

  ImGui::Unindent(5.f);
  return something_changed;
}

std::string Fractal::Serialize()
{
  std::string ret = "fractal ";
  /*
    # fractal parameters: pos scale rotation step_iterations num_subdivisions min_distance {options}
  # options will be read and applied in order
  # options: fold 0 (normal vector), rotate 1 (euler angles), scale 2 (scale factors), translate 3(translation amt)
  fractal 0 0 0     2     0 0 0     100 11 0.0001   0  1 0 0    0  0 1 0    0  0 0 1    0  1 -1 0
  */

  for (size_t i = 0; i < 3; i++)
  {
    ret += std::to_string(Position[i]) + " ";
  }
  ret += std::to_string(Scale) + " ";
  for (size_t i = 0; i < 3; i++)
  {
    ret += std::to_string(rot_eulers[i]) + " ";
  }
  ret += std::to_string(max_iteration) + " ";
  ret += std::to_string(num_subdivisions) + " ";
  ret += std::to_string(min_distance) + " ";

  //actions
  for (auto a : CombinedActions)
  {
    ret += std::to_string(a.action_type) + " ";
    if (a.action_type == ACTION_TYPE::C_ITERATION)
    {
      ret += std::to_string(a.IntOp) + " ";
    }
    else
    {
      Eigen::Vector3f to_write;
      switch (a.action_type)
      {
      case ACTION_TYPE::FOLD:
        to_write = a.DisplayOp;
        break;
      case ACTION_TYPE::ROTATION:
        to_write = a.DisplayOp;
        break;
      case ACTION_TYPE::SCALE:
        to_write = a.VecOp;
        break;
      case ACTION_TYPE::TRANSLATE:
        to_write = a.VecOp;
        break;
      case ACTION_TYPE::MODULO:
        to_write = a.VecOp;
        break;
      case ACTION_TYPE::POWER:
        to_write = a.VecOp;
        break;
      }
      for (size_t i = 0; i < 3; i++)
      {
        ret += std::to_string(to_write[i]) + " ";
      }
    }
  }
  return ret;
}

float Fractal::DE_Sphere(Eigen::Vector3f p)
{
  Eigen::Vector3f floored(std::fmod(p.x(), 1.f) - 0.5f, std::fmod(p.y(), 1.f) - 0.5f, p.z());

  return floored.norm() - Scale;
}

const float bailout_dist = 1000;


float Fractal::DE_Generic(Eigen::Vector3f _z)
{
  Eigen::Vector3f z = rot_inv._transformVector(_z - Center);

  float r = z.squaredNorm();
  int trace_num;
  bool skip = false;
  for (trace_num = 0; trace_num < num_subdivisions && r < bailout_dist; trace_num++)
  {
    for (size_t i = 0; i < CombinedActions.size(); i++)
    {
      if (skip)
      {
        skip = false;
        continue;
      }

      switch (CombinedActions[i].action_type)
      {
      case ACTION_TYPE::FOLD: // fold
        Action_Fold(z, i);
        break;
      case ACTION_TYPE::ROTATION: //rotation
        Action_Rotate(z, i);
        break;
      case ACTION_TYPE::SCALE: //scale
        Action_Scale(z, i);
        break;
      case ACTION_TYPE::TRANSLATE: //translation
        Action_Translate(z, i);
        break;
      case ACTION_TYPE::MODULO: //translation
        Action_Modulo(z, i);
        break;
      case ACTION_TYPE::POWER: //translation
        Action_Power(z, i);
        break;
      case ACTION_TYPE::C_ITERATION:
        skip = CombinedActions[i].IntOp < 1 || trace_num > CombinedActions[i].IntOp;
        break;
      }
    }

    r = z.squaredNorm();
  }

  return (std::sqrt(r) - 2.f) * (float)std::pow(Scale, -trace_num);
}

Eigen::Vector3f Fractal::FoldBased(Eigen::Vector3f _z)
{
  Eigen::Vector3f c(1.0, 1.0, 1.0);
  Eigen::Vector3f z = rot_inv._transformVector(_z) - Center;
  float r = z.squaredNorm();
  int i = 0;
  float w = 1;
  for (i = 0; i < num_subdivisions && r < bailout_dist; i++)
  {
    for (size_t action_num = 0; action_num < CombinedActions.size(); action_num++)
    {
      switch (CombinedActions[action_num].action_type)
      {
      case ACTION_TYPE::FOLD: // fold
        if (Action_Fold_Color(z, action_num))
          c = (1 - w) * c + w * CombinedActions[action_num].Color;
        break;
      case ACTION_TYPE::ROTATION: //rotation
        Action_Rotate(z, action_num);
        break;
      case ACTION_TYPE::SCALE: //scale
        Action_Scale(z, action_num);
        break;
      case ACTION_TYPE::TRANSLATE: //translation
        Action_Translate(z, action_num);
        break;
      case ACTION_TYPE::MODULO: //translation
        Action_Modulo(z, action_num);
        break;
      case ACTION_TYPE::POWER: //translation
        Action_Power(z, action_num);
        break;
      }
    }

    w *= 0.5f;
    r = z.squaredNorm();
  }

  return c;
}

bool Fractal::Action_Fold_Color(Eigen::Vector3f &p, size_t fold_index)
{
  float dot = p.dot(CombinedActions[fold_index].VecOp);
  if (dot < 0)
  {
    p -= dot * CombinedActions[fold_index].VecOp2;
    return true;
  }
  return false;
}

void Fractal::Action_Fold(Eigen::Vector3f &p, size_t fold_index)
{
  float dot = p.dot(CombinedActions[fold_index].VecOp);
  if (dot < 0)
    p -= dot * CombinedActions[fold_index].VecOp2;
}

void Fractal::Action_Rotate(Eigen::Vector3f &p, size_t rot_index)
{
  p = CombinedActions[rot_index].QuatOp._transformVector(p);
}

void Fractal::Action_Scale(Eigen::Vector3f &p, size_t scale_index)
{
  p = p.cwiseProduct(CombinedActions[scale_index].VecOp);
}

void Fractal::Action_Translate(Eigen::Vector3f &p, size_t trans_index)
{
  p = p + CombinedActions[trans_index].VecOp;
}

float SanityMod(float x, float m)
{
  float ret = x;
  if (m == 0)
    return ret;
  float sign = x > 0 ? 1.f : -1.f;
  while (abs(ret) > m)
  {
    ret -= sign * m;
  }
  return ret;
}


void Fractal::Action_Modulo(Eigen::Vector3f &p, size_t index)
{
  for (size_t i = 0; i < 3; i++)
    p[i] = SanityMod(p[i], CombinedActions[index].VecOp[i]);
}

void Fractal::Action_Power(Eigen::Vector3f &p, size_t index)
{
  for (size_t i = 0; i < 3; i++)
  {
    if (p[i] < 0)
      p[i] = -std::pow(-p[i], CombinedActions[index].VecOp[i]);
    else
      p[i] = std::pow(p[i], CombinedActions[index].VecOp[i]);
  }
}

int Fractal::NumFolds()
{
  int ret = 0;
  for (auto a : CombinedActions)
    if (a.action_type == 0)
      ret++;
  return ret;
}

void Fractal::GenColors()
{
  int num_folds = NumFolds();
  int current_color = 0;
  for (size_t i = 0; i < CombinedActions.size(); i++)
  {
    if (CombinedActions[i].action_type == 0)
    {
      float val = static_cast<float>(current_color) / static_cast<float>(num_folds);
      CombinedActions[i].Color = ColorFromFloat(val);
      current_color++;
      CombinedActions[i].VecOp2 = 2 * CombinedActions[i].VecOp;
    }
  }
}

Eigen::Vector3f Fractal::ColorFromFloat(float val)
{
  float r = std::sin(pi_2 * (val)) * 0.5f + 0.5f;
  float g = std::sin(pi_2 * (val + 0.33333333f)) * 0.5f + 0.5f;
  float b = std::sin(pi_2 * (val + 0.66666666f)) * 0.5f + 0.5f;
  Eigen::Vector3f ret(r, g, b);
  return ret;
}