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
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
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

//robotcore (Internal)
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//FIRST (external)
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

//Custom
import org.firstinspires.ftc.teamcode.HardwareProfiles.HardwareTestPlatform;
import org.firstinspires.ftc.teamcode.Libs.DataLogger;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

//Java (Internal)
import java.util.List;


/**
 * Name the opMode and put it in the appropriate group
 */
@Autonomous(name = "AutoBase", group = "Autonomous")


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
public class AutoBase extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AWLOnnD/////AAAAGUkCQDlQKUEVie7jg6bzwOsIdO360LbYDcYryrOUvM7ISMTrmHv4Z3WRq5IydTQEhQYFOCQhOD6wsaCEHdx3+K/HibQdTtWHzc5xTm//yzcfMcYBwNQsUFGghDV4ccGnbSXHALbYnv63U/n7VeCY91NtLLBe4rB3/U0q22IO6o3Q7Pui+06i3VlTiomIqptoGpbI0kuEwok+6Mq6818ECggYxwpW4UATAy7Rl0eDzp8BzkYEWM8Qe3ykRiEk9D4DBApyx8p3AERmPlQU8rIA/JDAs4tCEJSMNycVw2RKdE1qTrNfVqPe+mYWNOpypVq67odTh7tTHE+BGqdh6znE4NlTia2vr6vmAHjDsQuxn5bm";

    /**
     * Instantiate all objects needed in this class
     */
    private State state = State.STEP1;
    private String vuMarkValue = "UNK";
    private double timeout = 0;
    long futureTime;
    private LinearOpMode opMode = this;
    private DataLogger Dl;
    DriveMecanum drive = new DriveMecanum(robot, opMode, Dl);

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    /**
     * Inititialize the robot's hardware.  The hardware configuration can be found in the
     * HardwareTestPlatform.java class.
     */
    private final static HardwareTestPlatform robot = new HardwareTestPlatform();


    // region INIT
    /* Setup the initial state of the robot.  The actions in the begin() method are run when you hit the
     * init button on the driver station app.
     */

    private void begin() {

        /**
         * Vuforia and tfod
         */
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        futureTime = System.currentTimeMillis() + 8000; //8 seconds before calibrating gyro
        while(System.currentTimeMillis() < futureTime){
            telemetry.addData("Calibrating GYRO and VUFORIA in (ms): ", futureTime-System.currentTimeMillis());
            telemetry.update();
        }

        /**
         * Calibrate the MR Gyro
         */
        robot.sensorGyro.calibrate();
        while (robot.sensorGyro.isCalibrating()) {
            telemetry.addData("Waiting on Gyro Calibration", "");
            telemetry.update();
        }

    }
    // endregion


    //region OpMode
    public void runOpMode() {
        begin();
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();


        /**
         * Start the opMode
         */
        while (opModeIsActive()) {

            telemetry.update();

            switch (state) {
                //region START
                case START: //not init
                    /**
                     * hit ball off stand
                     */
                    state = State.STEP1;
                    break;
                //endregion


                //region STEP1
                case STEP1:
                    //diamond shape
                    drive.translateTime(2.0,0.25,315);
                    drive.translateTime(2.0,0.25,45);
                    drive.translateTime(2.0,0.25,135);
                    drive.translateTime(2.0,0.25,225);
                    state = State.STEP2;
                    break;
                //endregion


                //region STEP2
                case STEP2:
                    //turn
                    drive.pivotRight(0.2,90); //90 degrees to the right
                    drive.pivotRight(0.2, 45);//45 more degrees to the right
                    //pivotLeft
                    state = State.STEP3;
                    break;
                //endregion


                //region STEP3
                case STEP3:
                    state = State.HALT;
                    break;
                //endregion


                //region HALT
                case HALT:
                    robot.motorLF.setPower(0);
                    robot.motorRF.setPower(0);
                    robot.motorLR.setPower(0);
                    robot.motorRR.setPower(0);
                    robot.motorLift.setPower(0);
                    robot.motorArm.setPower(0);
                    robot.motorShoulder.setPower(0);
                    robot.servoGrabber.setPower(0);
                    telemetry.update();
                    break;
                //endregion
            }

            idle();

        }

        //Exit the OpMode
        requestOpModeStop();
    }
    //endregion

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


    enum State {
        START, STEP1, STEP2, STEP3, HALT
    }
}

