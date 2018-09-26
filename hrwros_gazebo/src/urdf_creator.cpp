#include "hrwros_gazebo/urdf_creator.h"
#include <ros/console.h>

namespace hrwros
{
namespace simulation
{

std::string createObjectURDF(const std::string& object_name,
                             const std::string& mesh_filename)
{
  // Set up the XML document
  TiXmlDocument doc;

  // Set up the XML declaration line
  TiXmlDeclaration* decl (new TiXmlDeclaration ("1.0", "", ""));

  // Create URDF tags
  GeometryTag visual_geometry (mesh_filename);
  VisualTag visual (visual_geometry);
  GeometryTag collision_geometry (mesh_filename);
  CollisionTag collision (collision_geometry);
  RobotTag robot (object_name, visual, collision);

  // Set up hierarchy
  doc.LinkEndChild(decl);
  doc.LinkEndChild(robot.self);

  // Set up the printer
  TiXmlPrinter printer;
  printer.SetIndent("  ");
  doc.Accept(&printer);
  std::string xmltext = printer.CStr();

  return xmltext;
}

} // namespace simulation
} // namespace hrwros

