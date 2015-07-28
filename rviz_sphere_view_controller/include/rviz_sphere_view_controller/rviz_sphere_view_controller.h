#ifndef RVIZ_SPHERE_VIEW_CONTROLLER_H
#define RVIZ_SPHERE_VIEW_CONTROLLER_H

#include "rviz/frame_position_tracking_view_controller.h"

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

namespace rviz {
class FloatProperty;
class VectorProperty;
}


namespace rviz_sphere_view_controller {

class SphereViewController : public rviz::FramePositionTrackingViewController {
    Q_OBJECT
public:
    SphereViewController();
    virtual ~SphereViewController();

    virtual void onInitialize();
    virtual void handleMouseEvent(ViewportMouseEvent& evt);
    virtual void lookAt(const Ogre::Vector3& point);
    virtual void reset();

    /** @brief Configure the settings of this view controller to give,
     * as much as possible, a similar view as that given by the
     * @a source_view.
     *
     * @a source_view must return a valid @c Ogre::Camera* from getCamera(). */
    virtual void mimic(ViewController* source_view);

    virtual void update(float dt, float ros_dt);

protected:
    virtual void onTargetFrameChanged(const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation);

    void setPropertiesFromCamera(Ogre::Camera* source_camera);

    void updateCamera();

    Ogre::Quaternion getOrientation(); ///< Return a Quaternion based on the yaw and pitch properties.

    FloatProperty* distance_property_;
    VectorProperty* pos_offset_property_;
    VectorProperty* ori_offset_property_;

    Ogre::Vector3 camera_position_;
    Ogre::Quaternion camera_orientation_;
};

}  // namespace rviz_sphere_view_controller

#endif // RVIZ_SPHERE_VIEW_CONTROLLER_H
