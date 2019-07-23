/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.Opmodes;

/**
 * Import the classes we need to have local access to.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HardwareProfiles.HardwareTestPlatform;
import org.firstinspires.ftc.teamcode.Libs.DataLogger;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

import java.util.List;


/**
 * Name the opMode and put it in the appropriate group
 */
@Autonomous(name = "WORLD AutoCrater", group = "Autonomous")
/**
 * This opMode attempts to acquire and trigger the rightRed beacons.
 *
 * Assumptions:
 *
 *  - The robot starts facing the positive Y axis (facing the the rightBlue wall
 *  with the rightBlue images, legos and tools)
 *  -  The robot starts with the left side wheels just to the left of the seam between tiles 2 and 3
 *  on the rightRed team wall with tile one being the left most tile on the wall, ~ -1500 X from origin
 */
public class WORLDAutoCrater extends LinearOpMode {


    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AWLOnnD/////AAAAGUkCQDlQKUEVie7jg6bzwOsIdO360LbYDcYryrOUvM7ISMTrmHv4Z3WRq5IydTQEhQYFOCQhOD6wsaCEHdx3+K/HibQdTtWHzc5xTm//yzcfMcYBwNQsUFGghDV4ccGnbSXHALbYnv63U/n7VeCY91NtLLBe4rB3/U0q22IO6o3Q7Pui+06i3VlTiomIqptoGpbI0kuEwok+6Mq6818ECggYxwpW4UATAy7Rl0eDzp8BzkYEWM8Qe3ykRiEk9D4DBApyx8p3AERmPlQU8rIA/JDAs4tCEJSMNycVw2RKdE1qTrNfVqPe+mYWNOpypVq67odTh7tTHE+BGqdh6znE4NlTia2vr6vmAHjDsQuxn5bm";
    /**
     * Instantiate all objects needed in this class
     */
    boolean TelemitryQ = true;
    private State state = State.SCAN; //land////////=======================================================Start
    int targetPOS = 0;
    private String vuMarkValue = "UNK";
    private double timeout = 0;
    private int count = 1;
    int lowerEncoderRange;
    int startingEncoderRange; //At least 650 clicks (-650) above lowerEncoderRange
    int higherEncoderRange;
    long futureTime;
    private LinearOpMode opMode = this;                     //Opmode
    private DataLogger Dl;
    DriveMecanum drive = new DriveMecanum(robot, opMode, Dl);
    int goldPOS = 0; // 0=N/A 1=Left 2=Center 3=Right
    boolean FORCE_GOLD_POS = false;

    int counter;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private final static HardwareTestPlatform robot = new HardwareTestPlatform();


    /**
     * PID variables
     */

    private void begin() {

        /**
         * Inititialize the robot's hardware.  The hardware configuration can be found in the
         * HardwareTestPlatform.java class.
         */

        robot.init(hardwareMap);


        robot.motorLift.setPower(-1.0);
        sleep(500);
        robot.motorLift.setPower(0.0);
        robot.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.servoGrabber.setDirection(DcMotorSimple.Direction.REVERSE);

        /**
         * Set Marker Holder POS
         */
        robot.servoLeft.setPosition(0.3);

        /**
         * Lift up
        */

        //set starting encoder
        startingEncoderRange = robot.motorLift.getCurrentPosition();

        telemetry.addData("Initializing", "");
        telemetry.addData("","");
        telemetry.update();sleep(150);

        telemetry.addData("Status: ", "Initializing");
        telemetry.addData("","");
        telemetry.addData("Lift: ","waiting");
        telemetry.addData("Vuforia: ","waiting");
        telemetry.addData("Tensorflow: ","waiting");
        telemetry.addData("Gyro: ", "waiting");
        telemetry.update();sleep(100);
        //move down to switch
        while (!robot.touchSensor.isPressed()) {
            telemetry.addData("Status: ", "Initializing");
            telemetry.addData("","");
            telemetry.addData("Lift: ","init");
            telemetry.addData("Vuforia: ","waiting");
            telemetry.addData("Tensorflow: ","waiting");
            telemetry.addData("Gyro: ", "waiting");
            telemetry.update();
            targetPOS = robot.motorLift.getCurrentPosition() + 100;
            robot.motorLift.setTargetPosition(targetPOS);
            robot.motorLift.setPower(0.8);
        }

        targetPOS = robot.motorLift.getCurrentPosition() + 300;
        robot.motorLift.setTargetPosition(targetPOS);
        robot.motorLift.setPower(0.8);
        while(robot.motorLift.getCurrentPosition() < robot.motorLift.getTargetPosition()){
            telemetry.addData("Status: ", "Initializing");
            telemetry.addData("","");
            telemetry.addData("Lift: ","init");
            telemetry.addData("Vuforia: ","waiting");
            telemetry.addData("Tensorflow: ","waiting");
            telemetry.addData("Gyro: ", "waiting");
            telemetry.update();
        }
        //stop motor
        robot.motorLift.setPower(0);

        //set absolute bottom var
        lowerEncoderRange = robot.motorLift.getCurrentPosition();



        /**
         * Vuforia and tfod
         */
        telemetry.addData("Status: ", "Initializing");
        telemetry.addData("","");
        telemetry.addData("Lift: ","READY");
        telemetry.addData("Vuforia: ","init");
        telemetry.addData("Tensorflow: ","waiting");
        telemetry.addData("Gyro: ", "waiting");
        telemetry.update();sleep(100);
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        telemetry.addData("Status: ", "Initializing");
        telemetry.addData("","");
        telemetry.addData("Lift: ","READY");
        telemetry.addData("Vuforia: ","READY");
        telemetry.addData("Tensorflow: ","init");
        telemetry.addData("Gyro: ", "waiting");
        telemetry.update();sleep(100);

        long startTime = System.currentTimeMillis();
        futureTime = System.currentTimeMillis() + 7000; //7 seconds before calibrating gyro
        while(System.currentTimeMillis() < futureTime){
            telemetry.addData("Status: ", "Initializing");
            telemetry.addData("","");
            telemetry.addData("Lift: ","READY");
            telemetry.addData("Vuforia: ","READY");
            telemetry.addData("Tensorflow: ","READY");
            telemetry.addData("Gyro: ", "init: " + (int)((System.currentTimeMillis()-startTime * 100.0f)/(7140)) + "%");
            //telemetry.addData("Calibrating GYRO in (ms): ", ((futureTime-System.currentTimeMillis())/1000));
            telemetry.update();
        }
        telemetry.addData("Status: ", "Initializing");
        telemetry.addData("","");
        telemetry.addData("Lift: ","READY");
        telemetry.addData("Vuforia: ","READY");
        telemetry.addData("Tensorflow: ","READY");
        telemetry.addData("Gyro: ", "init: 99%");
        telemetry.update(); sleep(100);

        /**
         * Calibrate the MR Gyro
         */
        robot.sensorGyro.calibrate();
        while (robot.sensorGyro.isCalibrating()) {
            telemetry.addData("Status: ", "Initializing");
            telemetry.addData("","");
            telemetry.addData("Lift: ","READY");
            telemetry.addData("Vuforia: ","READY");
            telemetry.addData("Tensorflow: ","READY");
            telemetry.addData("Gyro: ", "init: 99%");
            telemetry.update();
        }
        telemetry.addData("Status: ", "READY");
        telemetry.addData("","");
        telemetry.addData("Lift: ","READY");
        telemetry.addData("Vuforia: ","READY");
        telemetry.addData("Tensorflow: ","READY");
        telemetry.addData("Gyro: ", "READY");
        telemetry.update(); sleep(150);

    }

    public void runOpMode() {
        begin();

        /**
         * Start the opMode
         */

        telemetry.addData("Status: ", "READY");
        telemetry.addData("","");
        telemetry.addData("Driver -> ", "Press Play to start tracking");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.update();

            switch (state) {

                case SCAN:
                    if (opModeIsActive()) {
                        /** Activate Tensor Flow Object Detection. */
                        if (tfod != null) {
                            tfod.activate();
                        }

                        boolean exitTensor = false;
                        long tensorEndTime = System.currentTimeMillis() + 3000; // 3 seconds for reading the elements
                        //remove timeout



                        while (opModeIsActive() && !exitTensor) {
                            if (tfod != null) {
                                // getUpdatedRecognitions() will return null if no new information is available since
                                // the last time that call was made.
                                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                                //0,0 is top left
                                // 1000,700 is bottom right
                                int imageHeight = 0; //y
                                int imageWidth = 0; //x

                                //LEFT, CENTER, RIGHT
                                // 0 - 1000/3
                                // 1000/3 - 1000*2/3
                                // 1000*2/3 - 1000


                                if (updatedRecognitions != null) {
                                    telemetry.addData("Number of Objects detected: ", updatedRecognitions.size());
                                    counter = updatedRecognitions.size();
                                    if (updatedRecognitions.size() == 3 && false) {
                                        int goldMineralX = -1;
                                        int silverMineral1X = -1;
                                        int silverMineral2X = -1;
                                        for (Recognition recognition : updatedRecognitions) {
                                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                                goldMineralX = (int) recognition.getLeft();
                                            } else if (silverMineral1X == -1) {
                                                silverMineral1X = (int) recognition.getLeft();
                                            } else {
                                                silverMineral2X = (int) recognition.getLeft();
                                            }
                                        }
                                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                                telemetry.addData("Gold Mineral Position", "Left");
                                                goldPOS = 1;
                                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                                telemetry.addData("Gold Mineral Position", "Right");
                                                goldPOS = 3;
                                            } else {
                                                telemetry.addData("Gold Mineral Position", "Center");
                                                goldPOS = 2;
                                            }
                                        }
                                    }

                                    else if (updatedRecognitions.size() < 3) {
                                        int goldMineralX = -1;
                                        int silverMineral1X = -1;
                                        int silverMineral2X = -1;
                                        for (Recognition recognition : updatedRecognitions) {
                                            imageWidth = recognition.getImageWidth();
                                            imageWidth = recognition.getImageHeight();

                                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                                goldMineralX = (int) recognition.getTop();
                                            } else if (silverMineral1X == -1) {
                                                silverMineral1X = (int) recognition.getTop();
                                            } else {
                                                silverMineral2X = (int) recognition.getTop();
                                            }

                                        }
                                        if (goldMineralX != -1){ // if gold pos is known
                                            if(goldMineralX > imageWidth*2/3){
                                                telemetry.addData("Gold Mineral Position", "Right");
                                                goldPOS = 3;
                                            }
                                            else if(goldMineralX > imageWidth*1/3){
                                                telemetry.addData("Gold Mineral Position", "Center");
                                                goldPOS = 2;
                                            }
                                            else {
                                                telemetry.addData("Gold Mineral Position", "Left");
                                                goldPOS = 1;
                                            }
                                        }

                                        else if(silverMineral1X != -1 && silverMineral2X != -1){ //both silver are known
                                            if (silverMineral1X > imageWidth*1/3 && silverMineral2X > imageWidth*1/3){ // both are on right 2 thirds
                                                telemetry.addData("Gold Mineral Position", "Left");
                                                goldPOS = 1;
                                            }
                                            else if (silverMineral1X < imageWidth*2/3 && silverMineral2X < imageWidth*2/3){ // both are on left 2 thirds
                                                telemetry.addData("Gold Mineral Position", "Right");
                                                goldPOS = 3;
                                            }
                                            else{
                                                telemetry.addData("Gold Mineral Position", "Center");
                                                goldPOS = 2;
                                            }
                                        }
                                        else { //only one silver is known
                                            if(silverMineral1X > imageWidth*2/3 || silverMineral2X > imageWidth*2/3){
                                                telemetry.addData("Gold Mineral Position", "Center");
                                                goldPOS = 2;
                                            }
                                            else{
                                                telemetry.addData("Gold Mineral Position", "Right");
                                                goldPOS = 3;
                                            }
                                        }

                                        telemetry.addData("X: ", goldMineralX);
                                        telemetry.update();
                                    }

                                    telemetry.addData("Time Left to Acquire Reading (ms): ", (tensorEndTime - System.currentTimeMillis()));
                                    telemetry.update();
                                }


                            }
                            exitTensor = System.currentTimeMillis() > tensorEndTime || goldPOS != 0;
                            telemetry.addData("Number of Objects detected: ", counter);
                            telemetry.addData("Gold POS", goldPOS);
                            telemetry.update();
                            sleep(10);
                        }
                    }

                    if (tfod != null) {
                        tfod.shutdown();
                    }


                    state = State.LOWER;
                    break;

                case LOWER:
                    //lower robot and un hook

                    //lower
                    targetPOS = robot.motorLift.getCurrentPosition() - 2000;
                    robot.motorLift.setTargetPosition(targetPOS);
                    robot.motorLift.setPower(1.0);
                    sleep(1000);

                    //move up to switch
                    while (!robot.touchSensor.isPressed()) {
                        telemetry.addData("Moving to UPPER Limit Switch", "");
                        telemetry.addData("Current List POS: ", robot.motorLift.getCurrentPosition());
                        telemetry.addData("Target Lift POS: ", robot.motorLift.getTargetPosition());
                        telemetry.addData("Limit Switch is pressed = ", robot.touchSensor.isPressed());
                        telemetry.update();
                        targetPOS = robot.motorLift.getCurrentPosition() - 200;
                        robot.motorLift.setTargetPosition(targetPOS);
                        robot.motorLift.setPower(0.8);
                    }
                    targetPOS = robot.motorLift.getCurrentPosition() + 170;
                    robot.motorLift.setTargetPosition(targetPOS);
                    robot.motorLift.setPower(1.0);
                    sleep(500);

                    robot.motorLift.setPower(0.0);
                    higherEncoderRange = robot.motorLift.getCurrentPosition();

                    //unhook
                    drive.translateTime(0.5, 0.35, 270);
                    drive.translateTime(0.6, 0.2, 0);
                    drive.translateTime(0.5, 0.35, 90);

                    state = State.MINER;
                    break;

                case MINER:

                    switch (goldPOS) {

                        case 0:
                            telemetry.addData("N/A", goldPOS);
                            telemetry.addData("Random Choice for N/A today is: Center", "");
                            state = State.MtoCORNER;
                            break;
                        case 1:
                            telemetry.addData("Left", goldPOS);
                            state = State.LtoCORNER;
                            break;
                        case 2:
                            telemetry.addData("Center", goldPOS);
                            state = State.MtoCORNER;
                            break;
                        case 3:
                            telemetry.addData("Right", goldPOS);
                            state = State.RtoCORNER;
                            break;
                        case 4:
                            state = State.MtoCORNER; //just in case
                            break;
                    }
                    telemetry.update();
                    sleep(100);
                    break;

                case LtoCORNER: //Left
                    //Move Forward
                    drive.translateTime(0.8, 0.4, 0);
                    sleep(100);

                    //Strafe Left
                    drive.translateTime(1.4, 0.4, 270);
                    sleep(100);

                    //Move forward and hit element
                    drive.translateTime(0.7, 0.4,0);
                    sleep(100);

                    //Move back
                    drive.translateTime(0.5, 0.4,180);
                    sleep(100);

                    //turn left 90
                    drive.pivotLeft(0.2, 88);
                    sleep(100);

                    //move to wall
                    drive.translateTime(1.5, 0.5, 0);
                    sleep(100);

                    //turn left 45
                    drive.pivotLeft(0.2, 36);
                    sleep(100);

                    //strafe right to wall
                    drive.translateTime(0.5,0.5,90);
                    sleep(100);

                    //move forward to depot, hugging wall
                    drive.translateTime(1.0,0.6,0);
                    sleep(100);

                    //hit back wall

                    //back up
                    //drive.translateTime(1.5,0.4,0);
                    //sleep(100);

                    //dump marker
                    robot.servoGrabber.setPower(-1.0);
                    sleep(500);

                    //move back to crater, hugging wall
                    drive.translateTime(2.0,0.6,175);


                    state = State.END;
                    break;
                case MtoCORNER: //Middle
                    //Move Forward
                    drive.translateTime(0.8, 0.4, 0);
                    sleep(100);

                    //Move forward and hit element
                    drive.translateTime(0.7, 0.4,0);
                    sleep(100);

                    //Move back
                    drive.translateTime(0.5, 0.4,180);
                    sleep(100);

                    //Strafe Left
                    drive.translateTime(1.4, 0.4, 270);
                    sleep(100);

                    //turn left 90
                    drive.pivotLeft(0.2, 88);
                    sleep(100);

                    //move to wall
                    drive.translateTime(1.5, 0.5, 0);
                    sleep(100);

                    //turn left 45
                    drive.pivotLeft(0.2, 36);
                    sleep(100);

                    //strafe right to wall
                    drive.translateTime(0.6,0.5,90);
                    sleep(100);

                    //move forward to depot, hugging wall
                    drive.translateTime(1.0,0.6,0);
                    sleep(100);

                    //hit back wall

                    //back up
                    //drive.translateTime(1.5,0.4,0);
                    //sleep(100);

                    //dump marker
                    robot.servoGrabber.setPower(-1.0);
                    sleep(500);

                    //move back to crater, hugging wall
                    drive.translateTime(2.0,0.6,175);

                    state = State.END;
                    break;
                case RtoCORNER: //Right
                    //Move Forward
                    drive.translateTime(0.7, 0.4, 0);
                    sleep(100);

                    //Strafe Right
                    drive.translateTime(1.1, 0.4, 90);///1.3/1.4 befroe
                    sleep(100);

                    //Move forward and hit element
                    drive.translateTime(0.7, 0.4,0);
                    sleep(100);

                    //Move back
                    drive.translateTime(0.5, 0.4,180);
                    sleep(100);

                    //Strafe Left
                    drive.translateTime(1.4, 0.4, 270);
                    sleep(100);

                    //Strafe Left 2
                    drive.translateTime(1.4, 0.4, 270);
                    sleep(100);

                    //error
                    drive.translateTime(0.25,0.4,180);

                    //turn left 90
                    drive.pivotLeft(0.2, 88);
                    sleep(50);

                    //move to wall
                    drive.translateTime(1.8, 0.5, 0);
                    sleep(50);

                    //turn left 45
                    drive.pivotLeft(0.2, 36);
                    sleep(100);

                    //strafe right to wall
                    drive.translateTime(0.6,0.5,90);
                    sleep(50);

                    //move forward to depot, hugging wall
                    drive.translateTime(1.0,0.6,0);
                    sleep(100);

                    //hit back wall

                    //back up
                    //drive.translateTime(1.5,0.4,0);
                    //sleep(100);

                    //dump marker
                    robot.servoGrabber.setPower(-1.0);
                    sleep(500);

                    //move back to crater, hugging wall
                    drive.translateTime(1.7,0.8,175);

                    state = State.END;
                    break;

                case END:
                    drive.translateTime(0.3,0.6,270);
                    sleep(100);

                    drive.pivotLeft(0.3,170);
                    sleep(100);

                    drive.translateTime(0.5,0.3, 0);
                    sleep(100);

                    robot.motorArm.setPower(1.0);
                    robot.servoGrabber.setPower(1.0);

                    sleep(2000);

                    state = State.HALT;
                    break;

                case TEST:
                    robot.servoLeft.setPosition(0.2);
                    state = State.END;
                    break;
                case HALT:
                    robot.motorLF.setPower(0);
                    robot.motorRF.setPower(0);
                    robot.motorLR.setPower(0);
                    robot.motorRR.setPower(0);
                    robot.motorLift.setPower(0);
                    robot.motorArm.setPower(0);
                    robot.motorShoulder.setPower(0);
                    robot.servoGrabber.setPower(0);
                    telemetry.addData("Gold pos", goldPOS);
                    telemetry.addData("halt", 0);
                    telemetry.update();
                    break;

            }

            idle();

        }

        //Exit the OpMode
        requestOpModeStop();
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }


    enum State { //Corner == End
        SCAN, LOWER, MINER, LtoCORNER, MtoCORNER, RtoCORNER, END, HALT, TEST
    }
    /**
     * SCAN
     * -- Looks for 3 minerals and sends position of gold mineral to State.MINER
     * -- Timeout is included if 3 minerals cannot be seen
     *
     * LOWER
     * -- Lowers fast and slows to find top limit switch
     *
     * MINER
     * -- Takes position of gold mineral and continues to State. LtoCORNER, MtoCORNER,
     */
}

