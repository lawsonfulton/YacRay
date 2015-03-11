#include "scene.hpp"
#include "ray.hpp"

#include <iostream>

SceneNode::SceneNode(const std::string& name)
  : m_name(name)
{
}

SceneNode::~SceneNode()
{
}

std::list<GeometryNode> SceneNode::getFlattened() const {
  std::list<GeometryNode> flattened;

  for (auto nodeIt = m_children.begin(); nodeIt != m_children.end(); ++nodeIt) {
    std::list<GeometryNode> childFlattened = (*nodeIt)->getFlattened();

    //Transform them
    for(auto childIt = childFlattened.begin(); childIt != childFlattened.end(); ++childIt) {
      childIt->m_trans = m_trans * childIt->m_trans;
      childIt->m_invtrans = childIt->m_invtrans * m_invtrans;
      flattened.push_back(*childIt);
    }

    // //Extend the list
    // flattened.reserve(flattened.size() + distance(childFlattened.begin(),childFlattened.end()));
    // flattened.insert(flattened.end(),childFlattened.begin(),childFlattened.end());
  }

  return flattened;
}

void SceneNode::rotate(char axis, double angle)
{
  switch(axis) {
    case 'x':
        m_trans.rotate(angle, QVector3D(1,0,0));
        break;
    case 'y':
        m_trans.rotate(angle, QVector3D(0,1,0));
        break;
    case 'z':
        m_trans.rotate(angle, QVector3D(0,0,1));
        break;
    default:
        break;
  }
  set_transform(m_trans);
}

void SceneNode::scale(const Vector3D& amount)
{
    m_trans.scale(amount);
    set_transform(m_trans);
}

void SceneNode::translate(const Vector3D& amount)
{
    // m_trans.translate(amount);    
    // set_transform(m_trans);

      QMatrix4x4 trans;
    trans.translate(amount);

    if(is_joint()) {
        m_trans = trans * m_trans; //
    }
    else {
        m_trans.translate(amount);    
    }

    set_transform(m_trans);
}

bool SceneNode::is_joint() const
{
  return false;
}

JointNode::JointNode(const std::string& name)
  : SceneNode(name)
{
}

JointNode::~JointNode()
{
}

bool JointNode::is_joint() const
{
  return true;
}

bool SceneNode::is_geometry() const
{
  return false;
}
bool GeometryNode::is_geometry() const
{
  return true;
}

void JointNode::set_joint_x(double min, double init, double max)
{
  m_joint_x.min = min;
  m_joint_x.init = init;
  m_joint_x.max = max;

  rotate('x', init);
}

void JointNode::set_joint_y(double min, double init, double max)
{
  m_joint_y.min = min;
  m_joint_y.init = init;
  m_joint_y.max = max;

  rotate('y', init);
}

GeometryNode::GeometryNode(const std::string& name, Primitive* primitive)
  : SceneNode(name),
    m_primitive(primitive)
{
}

GeometryNode::~GeometryNode()
{
}
 
bool GeometryNode::computeIntersection(const Ray &ray, Intersection &i) {
  double t = DBL_INF;
  Vector3D normal;
  Point3D point;

  Matrix4x4 normalMatrix = Matrix4x4(m_trans.normalMatrix());

  Ray transformedRay(ray);
  transformedRay.transform(m_invtrans);
  bool intersects = m_primitive->rayIntersection(transformedRay, t, normal, point);

  //Transform it back into world space
  if(intersects) {
    point = m_trans * point;
    t = copysign((point - ray.origin()).length(), t);
    normal = (normalMatrix * normal).normalized();
  }

  //TODO major error here because we are checking for t < MIN_INTERSECT_DIST in model space after scale
  //if(t > MIN_INTERSECT_DIST) intersects = true; //The speckles are because it is in a shadow?

  i.node = this;
  i.ray = &ray;
  i.t = t;
  i.normal = normal; //TODO adjust normal
  i.point = point;

  return intersects;
}

const Material* GeometryNode::get_material() const {
  return m_material;
}

Material* GeometryNode::get_material() {
  return m_material;
}

std::list<GeometryNode> GeometryNode::getFlattened() const {
  std::list<GeometryNode> flattened(1, *this);

  for (auto nodeIt = m_children.begin(); nodeIt != m_children.end(); ++nodeIt) {
    std::list<GeometryNode> childFlattened = (*nodeIt)->getFlattened();

    //Transform them
    for(auto childIt = childFlattened.begin(); childIt != childFlattened.end(); ++childIt) {
      childIt->m_trans = m_trans * childIt->m_trans;
      childIt->m_invtrans = childIt->m_invtrans * m_invtrans;
      flattened.push_back(*childIt);
    }
  }

  return flattened;
}



