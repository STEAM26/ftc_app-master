package org.firstinspires.ftc.teamcode.Libs;

import android.app.Activity;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.os.Handler;
import android.os.Looper;

import java.util.Arrays;

/**
 * Created by Zachary Collins on 7/23/19.
 * Instagram: @hilariously_random
 *
 * An android phone sensor interface
 */


public class PhoneSensors extends Activity implements SensorEventListener {
    private final SensorManager mSensorManager;
    private final Sensor mAccelerometer;
    private SensorEvent mAccelerometerEvent;

    public PhoneSensors() {
        mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
        mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
    }

    //ignore
    protected void onResume() {
        super.onResume();
        mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_NORMAL);
    }

    //ignore
    protected void onPause() {
        super.onPause();
        mSensorManager.unregisterListener(this);
    }

    //ignore
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }

    /**
     * updates as fast as possible, data is raw
     * @param event contains event caller and data
     */
    public void onSensorChanged(SensorEvent event) {
        //event[0..2] for Accel x,y,z same got gyro
        if(event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            mAccelerometerEvent = event;
        }
    }

    /**
     * callback for acceleration, gravity is already subtracted
     * @return array of acceleration values per axis [x,y,z] in  m/s^2
     */
    public double[] getAccelerometer(){
        double[] values = convertFloatsToDoubles(mAccelerometerEvent.values.clone());
        return values;
    }

    /**
     *
     * @param input array of floats
     * @return output array of doubles
     */
    public static double[] convertFloatsToDoubles(float[] input)
    {
        if (input == null)
        {
            return null; // Or throw an exception - your choice
        }
        double[] output = new double[input.length];
        for (int i = 0; i < input.length; i++)
        {
            output[i] = input[i];
        }
        return output;
    }
}
