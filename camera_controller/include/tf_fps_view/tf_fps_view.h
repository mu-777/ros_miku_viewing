#ifndef TF_FPS_VIEW
#define TF_FPS_VIEW

#ifndef Q_MOC_RUN
#include <boost/shared_ptr.hpp>
#include <tf/transform_broadcaster.h>
#endif
#include <OgreVector3.h>
#include <OgreQuaternion.h>

#include "rviz/frame_position_tracking_view_controller.h"

namespace rviz
{
class FloatProperty;
class SceneNode;
class Shape;
class VectorProperty;
}

namespace miku_viewer
{
/** @brief A first-person camera, controlled by yaw, pitch, and position. */
class TFFPSViewController : public FramePositionTrackingViewController
{
Q_OBJECT
public:
  TFFPSViewController();
  virtual ~TFFPSViewController();

  virtual void onInitialize();
  virtual void handleMouseEvent(ViewportMouseEvent& evt);
  virtual void lookAt( const Ogre::Vector3& point );
  virtual void reset();

  /** @brief Configure the settings of this view controller to give,
   * as much as possible, a similar view as that given by the
   * @a source_view.
   *
   * @a source_view must return a valid @c Ogre::Camera* from getCamera(). */
  virtual void mimic( ViewController* source_view );
  virtual void update(float dt, float ros_dt);

protected:
  virtual void onTargetFrameChanged(const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation);

  void setPropertiesFromCamera( Ogre::Camera* source_camera );

  void updateCamera();

  Ogre::Quaternion getOrientation(); ///< Return a Quaternion based on the yaw and pitch properties.

  FloatProperty* dist_property_;
  VectorProperty* offset_pos_property_;
};

} // end namespace miku_viewer



#endif