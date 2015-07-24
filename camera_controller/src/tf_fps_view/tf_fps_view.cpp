#include <OgreViewport.h>
#include <OgreQuaternion.h>
#include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreCamera.h>

#include "rviz/uniform_string_stream.h"
#include "rviz/display_context.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/geometry.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/vector_property.h"

#include "fps_view_controller.h"

namespace miku_viewer
{

static const Ogre::Quaternion ROBOT_TO_CAMERA_ROTATION =
  Ogre::Quaternion( Ogre::Radian( -Ogre::Math::HALF_PI ), Ogre::Vector3::UNIT_Y ) *
  Ogre::Quaternion( Ogre::Radian( -Ogre::Math::HALF_PI ), Ogre::Vector3::UNIT_Z );

static const float PITCH_LIMIT_LOW = -Ogre::Math::HALF_PI + 0.001;
static const float PITCH_LIMIT_HIGH = Ogre::Math::HALF_PI - 0.001;

TFFPSViewController::TFFPSViewController()
{
  dist_property_ = new FloatProperty( "Distance", 0, "Distance to look-up point.", this );
  dist_property_->setMin(0.01);
  offset_pos_property_ = new VectorProperty( "Offset", Ogre::Vector3( 0, 0, 0 ), "Offset from the camera.", this );
}

TFFPSViewController::~TFFPSViewController()
{
}

void TFFPSViewController::onInitialize()
{
  FramePositionTrackingViewController::onInitialize();
  camera_->setProjectionType( Ogre::PT_PERSPECTIVE );
}

void TFFPSViewController::reset()
{
  camera_->setPosition( Ogre::Vector3( 5, 5, 10 ));
  camera_->lookAt( 0, 0, 0 );
  setPropertiesFromCamera( camera_ );

  // Hersh says: why is the following junk necessary?  I don't know.
  // However, without this you need to call reset() twice after
  // switching from TopDownOrtho to FPS.  After the first call the
  // camera is in the right position but pointing the wrong way.
  updateCamera();
  camera_->lookAt( 0, 0, 0 );
  setPropertiesFromCamera( camera_ );
}

void TFFPSViewController::handleMouseEvent(ViewportMouseEvent& event)
{
  if ( event.shift() )
  {
    setStatus( "<b>Left-Click:</b> Move X/Y.  <b>Right-Click:</b>: Move Z." );
  }
  else
  {
    setStatus( "<b>Left-Click:</b> Rotate.  <b>Middle-Click:</b> Move X/Y.  <b>Right-Click:</b>: Zoom.  <b>Shift</b>: More options." );
  }

  bool moved = false;

  int32_t diff_x = 0;
  int32_t diff_y = 0;

  if( event.type == QEvent::MouseMove )
  {
    diff_x = event.x - event.last_x;
    diff_y = event.y - event.last_y;
    moved = true;
  }

  if( event.left() && !event.shift() )
  {
    setCursor( Rotate3D );
    yaw( -diff_x*0.005 );
    pitch( diff_y*0.005 );
  }
  else if( event.middle() || ( event.shift() && event.left() ))
  {
    setCursor( MoveXY );
    move( diff_x*0.01, -diff_y*0.01, 0.0f );
  }
  else if( event.right() )
  {
    setCursor( MoveZ );
    move( 0.0f, 0.0f, diff_y*0.1 );
  }
  else
  {
    setCursor( event.shift() ? MoveXY : Rotate3D );
  }

  if ( event.wheel_delta != 0 )
  {
    int diff = event.wheel_delta;
    move( 0.0f, 0.0f, -diff * 0.01 );

    moved = true;
  }

  if (moved)
  {
    context_->queueRender();
  }
}

void TFFPSViewController::setPropertiesFromCamera( Ogre::Camera* source_camera )
{
  Ogre::Quaternion quat = source_camera->getOrientation() * ROBOT_TO_CAMERA_ROTATION.Inverse();
  float yaw = quat.getRoll( false ).valueRadians(); // OGRE camera frame looks along -Z, so they call rotation around Z "roll".
  float pitch = quat.getYaw( false ).valueRadians(); // OGRE camera frame has +Y as "up", so they call rotation around Y "yaw".

  Ogre::Vector3 direction = quat * Ogre::Vector3::NEGATIVE_UNIT_Z;

  if ( direction.dotProduct( Ogre::Vector3::NEGATIVE_UNIT_Z ) < 0 )
  {
    if ( pitch > Ogre::Math::HALF_PI )
    {
      pitch -= Ogre::Math::PI;
    }
    else if ( pitch < -Ogre::Math::HALF_PI )
    {
      pitch += Ogre::Math::PI;
    }

    yaw = -yaw;

    if ( direction.dotProduct( Ogre::Vector3::UNIT_X ) < 0 )
    {
      yaw -= Ogre::Math::PI;
    }
    else
    {
      yaw += Ogre::Math::PI;
    }
  }

  pitch_property_->setFloat( pitch );
  yaw_property_->setFloat( mapAngleTo0_2Pi( yaw ));
  position_property_->setVector( source_camera->getPosition() );
}

void TFFPSViewController::mimic( ViewController* source_view )
{
  FramePositionTrackingViewController::mimic( source_view );
  setPropertiesFromCamera( source_view->getCamera() );
}

void TFFPSViewController::update(float dt, float ros_dt)
{
  FramePositionTrackingViewController::update( dt, ros_dt );
  updateCamera();
}

void TFFPSViewController::lookAt( const Ogre::Vector3& point )
{
  camera_->lookAt( point );
  setPropertiesFromCamera( camera_ );
}

void TFFPSViewController::onTargetFrameChanged(const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation)
{
  position_property_->add( old_reference_position - reference_position_ );
}

void TFFPSViewController::updateCamera()
{
  camera_->setOrientation( getOrientation() );
  camera_->setPosition( position_property_->getVector() );
}

void TFFPSViewController::yaw( float angle )
{
  yaw_property_->setFloat( mapAngleTo0_2Pi( yaw_property_->getFloat() + angle ));
}

void TFFPSViewController::pitch( float angle )
{
  pitch_property_->add( angle );
}

Ogre::Quaternion TFFPSViewController::getOrientation()
{
  Ogre::Quaternion pitch, yaw;

  yaw.FromAngleAxis( Ogre::Radian( yaw_property_->getFloat() ), Ogre::Vector3::UNIT_Z );
  pitch.FromAngleAxis( Ogre::Radian( pitch_property_->getFloat() ), Ogre::Vector3::UNIT_Y );

  return yaw * pitch * ROBOT_TO_CAMERA_ROTATION;
}

void TFFPSViewController::move( float x, float y, float z )
{
  Ogre::Vector3 translate( x, y, z );
  position_property_->add( getOrientation() * translate );
}

} // end namespace miku_viewer

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( miku_viewer::TFFPSViewController, rviz::ViewController )