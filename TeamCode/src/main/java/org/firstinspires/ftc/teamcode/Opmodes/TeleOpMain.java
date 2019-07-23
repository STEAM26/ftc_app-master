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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.HardwareProfiles.HardwareTestPlatform;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Teleop M", group = "COMP")

public class TeleOpMain extends LinearOpMode {
    /**
     * Instantiate all objects needed in this class
     */

    long xstopTime = System.currentTimeMillis();
    long ystopTime = System.currentTimeMillis();
    long astopTime = System.currentTimeMillis();
    long bstopTime = System.currentTimeMillis();
    long lbstopTime = System.currentTimeMillis();
    long rbstopTime = System.currentTimeMillis();
    long ltstopTime = System.currentTimeMillis();
    long rtstopTime = System.currentTimeMillis();
    long accelerationStopTime1 = System.currentTimeMillis();
    long accelerationStopTime2 = System.currentTimeMillis();
    long runningStopTime = System.currentTimeMillis();

    boolean xToggleMode = false;
    boolean yToggleMode = false;
    boolean aToggleMode = false;
    boolean bToggleMode = false;
    boolean lbToggleMode = false;
    boolean rbToggleMode = false;

    double maxMultiplier = 1.0; //max for variable below
    double multiplier = 0.4;
    double maxTurnMultiplier = 0.5;
    double turnMltiplier = 0.1;

    int lowerEncoderRange;
    int higherEncoderRange;

    int loadingPos = 0;
    //0 = |
    //1 = /
    //2 = -
    //3 = \
    //4 = 0
    double output = 0.0;

    private final static HardwareTestPlatform robot = new HardwareTestPlatform();

    @Override
    public void runOpMode() {
        begin();

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //robot.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.servoGrabber.setDirection(DcMotorSimple.Direction.REVERSE);


        long timerInterval;
        int liftTargetPOS = 0;
        int armTargetPOS = 0;
        int shoulderTargetPOS = 0;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();


        higherEncoderRange = robot.motorLift.getCurrentPosition();
        //arm is in between the sensors


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double r = Math.hypot((gamepad1.left_stick_x * -1), gamepad1.left_stick_y);
            double robotAngle = Math.atan2((gamepad1.left_stick_y * -1), (gamepad1.left_stick_x )) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            double rightY = 0.5*gamepad1.right_stick_y;
            final double v1 = r * Math.cos(robotAngle) + rightX; // + rightY
            final double v2 = r * Math.sin(robotAngle) - rightX; // + rightY
            final double v3 = r * Math.sin(robotAngle) + rightX; // + rightY
            final double v4 = r * Math.cos(robotAngle) - rightX; // + rightY

            robot.motorLF.setPower(v1*multiplier);
            robot.motorRF.setPower(v2*multiplier);
            robot.motorLR.setPower(v3*multiplier);
            robot.motorRR.setPower(v4*multiplier);


            timerInterval = 1000;
            if (gamepad1.x == true && xstopTime < System.currentTimeMillis()) {
                xstopTime = System.currentTimeMillis() + timerInterval;

                //action(s)

                //toggle actions
                if (xToggleMode) {
                    xToggleMode = false;
                    //action 1
                    maxMultiplier = 1.0;
                    maxTurnMultiplier = 1.0;
                    multiplier = 1.0;
                    //
                } else {
                    xToggleMode = true;
                    //action 2
                    maxMultiplier = 0.4;
                    maxTurnMultiplier = 0.5;
                    multiplier = 0.4;
                    //
                }
            }

            if (gamepad1.y == true) {
                robot.servoGrabber.setPower(1.0);
            }
            else if (gamepad2.y == true) {
                robot.servoGrabber.setPower(-1.0);
            }
            else if (gamepad1.a == true || gamepad2.a == true) {
                robot.servoGrabber.setPower(0.0);
            }


            timerInterval = 500;
            if (gamepad1.b == true && bstopTime < System.currentTimeMillis()) {
                bstopTime = System.currentTimeMillis() + timerInterval;

                //action(s)

                //toggle actions
                if (bToggleMode) {
                    bToggleMode = false;
                    //action 1
                    robot.servoDoor.setPosition(0.0);
                    //
                } else {
                    bToggleMode = true;
                    //action 2
                    robot.servoDoor.setPosition(0.4);
                    //
                }
            }

            /**
             * bumpers
             */

//            timerInterval = 1;
////            if (gamepad1.left_bumper == true && lbstopTime < System.currentTimeMillis()) {
////                lbstopTime = System.currentTimeMillis() + timerInterval;
////                //action(s)
////                //liftTargetPOS = robot.motorLift.getCurrentPosition() + 200; //Down
////
////                if (robot.motorLift.getCurrentPosition() < 600) {
////                    liftTargetPOS = robot.motorLift.getCurrentPosition() + 100;
////
////                } else {
////                    if (!robot.touchSensor.isPressed()) {
////                        liftTargetPOS = robot.motorLift.getCurrentPosition() + 100;
////                    }
////                }
////                robot.motorLift.setTargetPosition(liftTargetPOS);
////                robot.motorLift.setPower(0.5);
////
////                //toggle actions
////                if (lbToggleMode) {
////                    lbToggleMode = false;
////                    //action 1
////
////                    //
////                } else {
////                    lbToggleMode = true;
////                    //action 2
////
////                    //
////                }
////            }
////            timerInterval = 1;
////            if (gamepad1.right_bumper == true && rbstopTime < System.currentTimeMillis()) {
////                rbstopTime = System.currentTimeMillis() + timerInterval;
////
////                //action(s)
////                liftTargetPOS = higherEncoderRange;
////
////                robot.motorLift.setTargetPosition(liftTargetPOS);
////                robot.motorLift.setPower(0.5);
////
////                //toggle actions
////                if (lbToggleMode) {
////                    rbToggleMode = false;
////                    //action 1
////
////                    //
////                } else {
////                    rbToggleMode = true;
////                    //action 2
////
////                    //
////                }
////            }
            timerInterval = 1;
            if (gamepad2.left_bumper && lbstopTime < System.currentTimeMillis()) {
                lbstopTime = System.currentTimeMillis() + timerInterval;

                //action(s)
                armTargetPOS = robot.motorArm.getCurrentPosition() - 200;
                robot.motorArm.setTargetPosition(armTargetPOS);
                robot.motorArm.setPower(1.0);
            }

            timerInterval = 1;
            if (gamepad2.right_bumper && rbstopTime < System.currentTimeMillis()) {
                rbstopTime = System.currentTimeMillis() + timerInterval;

                //action(s)
                armTargetPOS = robot.motorArm.getCurrentPosition() + 200;
                robot.motorArm.setTargetPosition(armTargetPOS);
                robot.motorArm.setPower(1.0);
            }

            /**
             * triggers
             */
            timerInterval = 1;
            if (gamepad2.left_trigger > 0.05 && ltstopTime < System.currentTimeMillis()) {
                ltstopTime = System.currentTimeMillis() + timerInterval;

                //action(s)
                shoulderTargetPOS = robot.motorShoulder.getCurrentPosition() + 100;
                robot.motorShoulder.setTargetPosition(shoulderTargetPOS);
                robot.motorShoulder.setPower(1.0);

            }
            timerInterval = 1;
            if (gamepad2.right_trigger > 0.05 && rtstopTime < System.currentTimeMillis()) {
                rtstopTime = System.currentTimeMillis() + timerInterval;

                //action(s)
                shoulderTargetPOS = robot.motorShoulder.getCurrentPosition() - 50;

                robot.motorShoulder.setTargetPosition(shoulderTargetPOS);
                robot.motorShoulder.setPower(1.0);
            }


            if(gamepad1.left_bumper){
                robot.motorLift.setPower(1.0);
            }
            else if(gamepad1.right_bumper){
                robot.motorLift.setPower(-1.0);
            }
            else{
                robot.motorLift.setPower(0.0);
            }



            //telemetry
            telemetry.addData("Current Time (ms): ", System.currentTimeMillis());
            telemetry.addData("Acceleration Multiplier: ", multiplier);
            telemetry.addData("Turn Acceleration Multiplier: ", turnMltiplier);
            telemetry.addData("Robot Target Strafe Angle:", robotAngle);
            telemetry.addData("Pot ", robot.pot.getVoltage());
            telemetry.addData("Touch: ", robot.touchSensor.isPressed());
            telemetry.addData("Touch Analog: ", robot.touchSensor.getValue());
            telemetry.addData("Shoulder pos", robot.motorShoulder.getCurrentPosition());
            telemetry.addData("Shoulder target pos", robot.motorShoulder.getTargetPosition());
            telemetry.addData("Arm pos", robot.motorArm.getCurrentPosition());
            telemetry.addData("Arm target pos", robot.motorArm.getTargetPosition());
            telemetry.addData("x", gamepad1.left_stick_x);
            telemetry.addData("y", gamepad1.left_stick_y);

            telemetry.update();

            //idle
            idle();

        }
    }
    public void motorsHalt() {
        robot.motorLF.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRR.setPower(0);
    }

    private void begin() {

        /**
         * Inititialize the robot's hardware.  The hardware configuration can be found in the
         * HardwareTestPlatform.java class.
         */
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}
