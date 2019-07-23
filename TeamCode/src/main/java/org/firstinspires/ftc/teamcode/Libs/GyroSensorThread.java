package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardwareProfiles.HardwareTestPlatform;

import org.firstinspires.ftc.teamcode.Libs.PIDController;


/**
 * Created by Zachary Collins on 6/26/19.
 * Instagram: @hilariously_random
 *
 * Description:
 *  Gyro sensor controller on a separate thread
 *
 *
 * Usage:
 *
 *  GyroSensorThread mygyro new GyroSensorThread(HardwareTestPlatform, OpMode, false);
 *  mygyro.start();         //Starts the threaded process
 *  mygyro.initialize()     //Starts the initialization process
 *  mygyro.isInitializing() //Checks if still initializing
 *  mygyro.getHeading()     //Get current heading
 *  mygyro.heading          //Public variable storing current heading
 *  mygyro.state            //Public variable storing current state
 *
 * Also:
 *  Other Local variables are accessible and useful
 */


public class GyroSensorThread extends Thread {
    public double rawx    = 0;
    public double rawy    = 0;
    public double rawz    = 0;
    public double heading = 0;
    private boolean log;

    private State state = State.WAITING;

    private LinearOpMode opMode;
    private HardwareTestPlatform robot;
    private GyroSensor gyro = robot.sensorGyro;
    private DataLogger Dl = new DataLogger("PID_LOG");


    // TODO: remove HardwareTestPlatform dependency from this function
    public GyroSensorThread(HardwareTestPlatform myRobot, LinearOpMode myOpMode, boolean useLog){
        this.robot  = myRobot;
        this.opMode = myOpMode;
        this.log    = useLog;

        if(this.log){
            // TODO: add log functionality
        }
    }

    //this function will run as a thread
    public void run() {
        //TODO: change condition for while loop, may not be thread safe

        while(!opMode.isStopRequested()) {
            switch (state) {

                //Thread is active but no commands have been sent
                case WAITING:
                    //bypass and push to INIT?
                    break;

                //Init command has been sent and  the process has started
                case INIT:
                    gyro.calibrate();
                    if (!isInitializing()) { //init complete
                        state = State.RUNNING;
                    }
                    break;

                //Automatic transition to RUNNING state on init completion
                case RUNNING:
                    this.heading = gyro.getHeading();
                    this.rawx = gyro.rawX();
                    this.rawy = gyro.rawY();
                    this.rawz = gyro.rawZ();
                    break;
            }
        }
    }

    //initialize Gyro
    public void initialize() {
        if(state == State.WAITING){
            state = State.INIT;
        }
    }

    //check if gyro is initializing
    public boolean isInitializing() {
        return robot.sensorGyro.isCalibrating();
    }

    // returns current heading
    public double getHeading(){
        return AngleUnit.DEGREES.normalize(this.heading);
    }

    //Not functioning
    public boolean closeLog(){
        try {
            Dl.closeDataLogger();
        }
        catch (Exception e) {
            return false;
        }
        return true;
    }

    //All possible states of the gyro sensor
    private enum State{
        WAITING, INIT, RUNNING
    }
}
