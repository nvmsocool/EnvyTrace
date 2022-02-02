#include "Shape.h"
#include "..\material.h"

bool Shape::RenderGenericGUI(size_t shape_num)
{
  bool something_changed = false;
  if (ImGui::CollapsingHeader(name.data()))
  {
    ImGui::Indent(10.f);
    something_changed |= material->RenderGUI(shape_num);
    something_changed |= ImGui::DragFloat((std::string("origin_margin##") + std::to_string(shape_num)).data(), &originErrorMargin, 0.00001f, 0, 10000, "%.5f");
    something_changed |= this->RenderGUI(shape_num);
    ImGui::Unindent(10.f);
  }
  return something_changed;
}
