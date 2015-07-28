package com.three_seven.mu.imu_publisher;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorListener;
import android.hardware.SensorManager;
import android.os.SystemClock;
import android.util.Log;
import android.view.View;

import org.ros.message.*;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import java.lang.String;

import sensor_msgs.Imu;
import std_msgs.*;

public class ImuPublisher implements NodeMain {

    private static final String TAG = "IMUPublisher";
    private ConnectedNode node;
    private Publisher<sensor_msgs.Imu> publisher;
    private mySensorListener sensorListener;

    private SensorManager sensorManager;
    private final Sensor accelSensor;
    private final Sensor gyroSensor;
    private final Sensor quatSensor;


    public ImuPublisher(SensorManager sensorManager) {
        Log.d(TAG, "constructor");
        this.sensorManager = sensorManager;
        this.accelSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        this.gyroSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        this.quatSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
    }

    //@Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("android_ros_sample/talker");
    }

    //@Override
    public void onError(Node node, Throwable throwable) {
    }

    //@Override
    public void onStart(ConnectedNode node) {
        this.publisher = node.newPublisher("android/phone_imu", "sensor_msgs/Imu");
        this.sensorListener = new mySensorListener(this.publisher);
        this.node = node;
        this.sensorManager.registerListener(this.sensorListener, this.accelSensor, SensorManager.SENSOR_DELAY_UI);
        this.sensorManager.registerListener(this.sensorListener, this.gyroSensor, SensorManager.SENSOR_DELAY_UI);
        this.sensorManager.registerListener(this.sensorListener, this.quatSensor, SensorManager.SENSOR_DELAY_UI);
    }

    //@Override
    public void onShutdown(Node arg0) {
        this.sensorManager.unregisterListener(this.sensorListener);
    }

    //@Override
    public void onShutdownComplete(Node arg0) {
    }

    private class mySensorListener implements SensorEventListener {
        private Publisher<Imu> publisher;
        private sensor_msgs.Imu imu;

        private mySensorListener(Publisher<Imu> publisher) {
            this.publisher = publisher;
            this.imu = this.publisher.newMessage();
            this.imu.getHeader().setFrameId("/android_imu");
        }

        //	@Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {
        }

        //	@Override
        public void onSensorChanged(SensorEvent event) {
            if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
                this.imu.getLinearAcceleration().setX(event.values[0]);
                this.imu.getLinearAcceleration().setY(event.values[1]);
                this.imu.getLinearAcceleration().setZ(event.values[2]);
            } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
                this.imu.getAngularVelocity().setX(event.values[0]);
                this.imu.getAngularVelocity().setY(event.values[1]);
                this.imu.getAngularVelocity().setZ(event.values[2]);
            } else if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
                float[] quaternion = new float[4];
                SensorManager.getQuaternionFromVector(quaternion, event.values);
                this.imu.getOrientation().setW(quaternion[0]);
                this.imu.getOrientation().setX(quaternion[1]);
                this.imu.getOrientation().setY(quaternion[2]);
                this.imu.getOrientation().setZ(quaternion[3]);
            }
            long time_delta_millis = System.currentTimeMillis() - SystemClock.uptimeMillis();
            this.imu.getHeader().setStamp(org.ros.message.Time.fromMillis(time_delta_millis + event.timestamp / 1000000));
            this.imu.getHeader().setSeq(this.imu.getHeader().getSeq() + 1);
            this.publisher.publish(this.imu);
        }
    }

}
