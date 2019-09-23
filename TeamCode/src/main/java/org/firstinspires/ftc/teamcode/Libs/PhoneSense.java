package org.firstinspires.ftc.teamcode.Libs;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.List;

/**
 * Created by Zachary Collins on 7/23/19.
 * Instagram: @hilariously_random
 *
 * An android phone sensor interface
 */

public class PhoneSense implements SensorEventListener {
    private final String TAG = "Phone Sense";
    private final SensorManager sensorManager;
    private LinearOpMode opMode = null;
    private final Sensor accelerometer;
    public boolean bInitialized;
    private final Context context;


    public PhoneSense(Context context) {
        this.context = context;

        Log.d(TAG, "PhoneSense: ");
        sensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);

        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_NORMAL);
    }
//    private void initListeners(){
//        mSensorManager.registerListener(this,
//                mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
//                SensorManager.SENSOR_DELAY_FASTEST);
//
//        mSensorManager.registerListener(this,
//                mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
//                SensorManager.SENSOR_DELAY_FASTEST);
//
//        mSensorManager.registerListener(this,
//                mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
//                SensorManager.SENSOR_DELAY_FASTEST);
//
//        mSensorManager.registerListener(this,
//                mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION),
//                SensorManager.SENSOR_DELAY_FASTEST);
//    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if(event.sensor.getType() == Sensor.TYPE_ACCELEROMETER){
            //lux = event.values.clone()[0];
            opMode.telemetry.addData("data", event.values.clone());
            opMode.telemetry.update();
            Log.d(TAG, event.values.clone().toString());
        }
    }
    public String listAvailableSensors() {
        List<Sensor> sensorList = sensorManager.getSensorList(Sensor.TYPE_ALL); // gets list of all sensors available on the phone
        return sensorList.toString();
    }

}
