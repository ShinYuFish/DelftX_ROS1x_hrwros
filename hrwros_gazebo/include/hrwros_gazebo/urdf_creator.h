#ifndef GILBRETH_GAZEBO_URDF_CREATOR_H
#define GILBRETH_GAZEBO_URDF_CREATOR_H

#include <tinyxml.h>

namespace hrwros
{
namespace simulation
{

struct InertiaTag
{
  InertiaTag(const std::string& ixx = "1",
             const std::string& iyy = "1",
             const std::string& izz = "1",
             const std::string& ixy = "0",
             const std::string& iyz = "0",
             const std::string& ixz = "0")
    : self(new TiXmlElement ("inertia"))
  {
    self->SetAttribute("ixx", ixx);
    self->SetAttribute("iyy", iyy);
    self->SetAttribute("izz", izz);
    self->SetAttribute("ixy", ixy);
    self->SetAttribute("iyz", iyz);
    self->SetAttribute("ixz", ixz);
  }

  TiXmlElement* self;
};

struct OriginTag
{
  OriginTag(const std::string& xyz = "0 0 0",
            const std::string& rpy = "0 0 0")
    : self(new TiXmlElement ("origin"))
  {
    self->SetAttribute("xyz", xyz);
    self->SetAttribute("rpy", rpy);
  }

  TiXmlElement* self;
};

struct GeometryTag
{
  GeometryTag(const std::string& mesh_filename)
    : self(new TiXmlElement ("geometry"))
    , mesh(new TiXmlElement ("mesh"))
  {
    mesh->SetAttribute("filename", mesh_filename);
    self->LinkEndChild(mesh);
  }

  TiXmlElement* self;
  TiXmlElement* mesh;
};

struct MaterialTag
{
  MaterialTag(const std::string& color_name = "medium_gray",
              const std::string& color_string = "0.5 0.5 0.5 1.0")
    : self(new TiXmlElement ("material"))
    , color(new TiXmlElement ("color"))
  {
    color->SetAttribute("rgba", color_string);
    self->SetAttribute("name", color_name);
    self->LinkEndChild(color);
  }

  MaterialTag(const std::string& color_name)
    : self(new TiXmlElement ("material"))
  {
    self->SetAttribute("name", color_name);
  }

  TiXmlElement* self;
  TiXmlElement* color;
};

struct VisualTag
{
  VisualTag(const GeometryTag& geometry_tag,
            const MaterialTag& material_tag = MaterialTag(),
            const OriginTag& origin_tag = OriginTag())
    : self(new TiXmlElement ("visual"))
  {
    self->LinkEndChild(origin_tag.self);
    self->LinkEndChild(geometry_tag.self);
    self->LinkEndChild(material_tag.self);
  }

  TiXmlElement* self;
};

struct CollisionTag
{
  CollisionTag(const GeometryTag& geometry_tag,
               const MaterialTag& material_tag = MaterialTag(),
               const OriginTag& origin_tag = OriginTag())
    : self(new TiXmlElement ("collision"))
  {
    self->LinkEndChild(origin_tag.self);
    self->LinkEndChild(geometry_tag.self);
    self->LinkEndChild(material_tag.self);
  }

  TiXmlElement* self;
};

struct InertialTag
{
  InertialTag(const std::string& mass_value = "1",
              const InertiaTag& inertia_tag = InertiaTag(),
              const OriginTag& origin_tag = OriginTag())
    : self(new TiXmlElement ("inertial"))
    , mass(new TiXmlElement ("mass"))
  {
    mass->SetAttribute("value", mass_value);
    self->LinkEndChild(mass);
    self->LinkEndChild(inertia_tag.self);
    self->LinkEndChild(origin_tag.self);
  }

  TiXmlElement* self;
  TiXmlElement* mass;
};

struct RobotTag
{
  RobotTag(const std::string& name,
           const VisualTag& visual_tag,
           const CollisionTag& collision_tag,
           const InertialTag& inertial_tag = InertialTag())
    : self(new TiXmlElement ("robot"))
    , link(new TiXmlElement ("link"))
  {
    self->SetAttribute("name", name);
    self->SetAttribute("xmlns:xacro", "https://www.ros.org/wiki/xacro");
    link->SetAttribute("name", name + "_base_link");

    link->LinkEndChild(visual_tag.self);
    link->LinkEndChild(collision_tag.self);
    link->LinkEndChild(inertial_tag.self);
    self->LinkEndChild(link);
  }

  TiXmlElement* self;
  TiXmlElement* link;
};

std::string createObjectURDF(const std::string& object_name,
                             const std::string& mesh_filename);

} // namespace simulation
} // namespace hrwros

#endif // GILBRETH_GAZEBO_URDF_CREATOR_H
