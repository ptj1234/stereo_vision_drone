#include "mesh_visual.h"

#include <OgreTechnique.h>
#include <OgreMaterialManager.h>
#include <rviz_rendering/material_manager.hpp>

namespace decomp_rviz_plugins {
  MeshVisual::MeshVisual(Ogre::SceneManager *scene_manager,
                         Ogre::SceneNode *parent_node) {
    scene_manager_ = scene_manager;
    frame_node_ = parent_node->createChildSceneNode();
    static int count = 0;
    const std::string name = "DecompMeshVisual" + std::to_string(count++);
    obj_ = scene_manager_->createManualObject(name);
    frame_node_->attachObject(obj_);
    material_ = rviz_rendering::MaterialManager::createMaterialWithLighting(name + "Material");
    material_->setCullingMode(Ogre::CULL_NONE);
  }

  MeshVisual::~MeshVisual() {
    scene_manager_->destroyManualObject(obj_);
    Ogre::MaterialManager::getSingleton().remove(material_);
    scene_manager_->destroySceneNode(frame_node_);
  }

  void MeshVisual::setMessage(const vec_E<vec_Vec3f> &bds) {
    obj_->clear();

    if (bds.empty())
      return;

    obj_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, material_->getGroup());
    int free_cnt = 0;
    for (const auto &vs: bds) {
      if (vs.size() > 2) {
        Vec3f p0 = vs[0];
        Vec3f p1 = vs[1];
        Vec3f p2 = vs[2];
        Vec3f n = (p2-p0).cross(p1-p0);
        n = n.normalized();
        if(std::isnan(n(0)))
          n = Vec3f(0, 0, -1);

        int ref_cnt = free_cnt;
        Ogre::Vector3 normal(n(0), n(1), n(2));
        for (unsigned int i = 0; i < vs.size(); i++) {
          obj_->position(Ogre::Vector3(vs[i](0), vs[i](1), vs[i](2)));
          obj_->normal(normal);
          if (i > 1 && i < vs.size())
            obj_->triangle(ref_cnt, free_cnt - 1, free_cnt);
          free_cnt++;
        }
      }
    }
    obj_->end();
  }

  // Position and orientation are passed through to the SceneNode.
  void MeshVisual::setFramePosition(const Ogre::Vector3 &position) {
    frame_node_->setPosition(position);
  }

  void MeshVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
    frame_node_->setOrientation(orientation);
  }

  void MeshVisual::setColor(float r, float g, float b, float a) {
    Ogre::Technique *technique = material_->getTechnique(0);
    technique->setAmbient(r * 0.5f, g * 0.5f, b * 0.5f);
    technique->setDiffuse(r, g, b, a);
    rviz_rendering::MaterialManager::enableAlphaBlending(material_, a);
  }
}
