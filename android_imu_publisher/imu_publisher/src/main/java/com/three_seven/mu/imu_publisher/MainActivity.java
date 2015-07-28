package com.three_seven.mu.imu_publisher;

/**
 * Created by ryosuke on 15/07/28.
 */

import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;

import org.ros.android.RosActivity;
import org.ros.node.NodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.address.InetAddressFactory;

public class MainActivity extends RosActivity {

    private static final String TAG = "MainActivity";
    private ImuPublisher mIMUPub;
    private SensorManager mSensorManager;
    private Button mMsgButton;

    public MainActivity() {
        super("notificationTicker", "notificationTitle");
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.d(TAG, "onCreate");
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
    }

    @Override
    protected void onPause() {
        super.onPause();
    }

    @Override
    protected void onResume() {
        super.onResume();
    }


    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {

        Log.d(TAG, "init");
        NodeConfiguration nodeConfigurator = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfigurator.setMasterUri(getMasterUri());
        nodeConfigurator.setNodeName("android_imu_publisher");

        this.mIMUPub = new ImuPublisher(mSensorManager);
        nodeMainExecutor.execute(this.mIMUPub, nodeConfigurator);
    }
}
