#ifndef MESH_VISUAL_H
#define MESH_VISUAL_H

#include <decomp_geometry/polyhedron.h>
#include <Eigen/Eigenvalues>

#include <OgreVector.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <OgreManualObject.h>
#include <OgreMaterial.h>

namespace decomp_rviz_plugins {
  class MeshVisual {
    public:
      MeshVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node);

      virtual ~MeshVisual();

      void setMessage(const vec_E<vec_Vec3f> &bds);
      void setFramePosition(const Ogre::Vector3 &position);
      void setFrameOrientation(const Ogre::Quaternion &orientation);

      void setColor(float r, float g, float b, float a);

    private:
      // rviz2 has no MeshShape; ROS 1 rviz::MeshShape was a ManualObject triangle list
      Ogre::ManualObject *obj_;
      Ogre::MaterialPtr material_;

      Ogre::SceneNode *frame_node_;

      Ogre::SceneManager *scene_manager_;
  };
}

#endif
