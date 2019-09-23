package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.app.Activity;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.os.*;
import android.util.Log;

import java.util.Arrays;
import java.util.List;

/**
 * Created by Zachary Collins on 7/23/19.
 * Instagram: @hilariously_random
 *
 * An android phone sensor interface
 */

public class PhoneSensors extends Activity implements SensorEventListener {
    private final SensorManager mSensorManager;
    private LinearOpMode opMode = null;
    //private final Sensor mAccelerometer;
    public boolean bInitialized;

    public PhoneSensors() {
        opMode.telemetry.addData("1","1");
        opMode.telemetry.update();

        mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);

        if (mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) == null ||
                mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE) == null ||
                mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION) == null) {
            bInitialized = false;
        }
        else {
            bInitialized = true;

            initListeners();
        }
    }
    private void initListeners(){
        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                SensorManager.SENSOR_DELAY_FASTEST);

        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
                SensorManager.SENSOR_DELAY_FASTEST);

        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
                SensorManager.SENSOR_DELAY_FASTEST);

        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION),
                SensorManager.SENSOR_DELAY_FASTEST);
    }

    protected void onResume() {
        super.onResume();
        //mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_NORMAL);
    }

    protected void onPause() {
        super.onPause();
        mSensorManager.unregisterListener(this);
    }

    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }

    public void onSensorChanged(SensorEvent event) {
        if(event.sensor.getType() == Sensor.TYPE_ACCELEROMETER){
            //lux = event.values.clone()[0];
        }
    }
    public String listAvailableSensors() {
        List<Sensor> sensorList = mSensorManager.getSensorList(Sensor.TYPE_ALL); // gets list of all sensors available on the phone
        return sensorList.toString();
    }

}
