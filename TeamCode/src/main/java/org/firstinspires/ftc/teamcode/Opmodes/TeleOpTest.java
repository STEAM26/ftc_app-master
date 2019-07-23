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
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.HardwareProfiles.HardwareTestPlatform;
import org.firstinspires.ftc.teamcode.Libs.PIDController;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Teleop world owo", group = "COMP")

public class TeleOpTest extends LinearOpMode {
    /**
     * Instantiate all objects needed in this class
     */

    PIDController PID_Arm = new PIDController(1.2,0.0,0.0,false); //1.2 to 1.5

    long xstopTime = System.currentTimeMillis();
    long ystopTime = System.currentTimeMillis();
    long astopTime = System.currentTimeMillis();
    long bstopTime = System.currentTimeMillis();
    long lbstopTime = System.currentTimeMillis();
    long rbstopTime = System.currentTimeMillis();
    long ltstopTime = System.currentTimeMillis();
    long rtstopTime = System.currentTimeMillis();
    long runningStopTime = System.currentTimeMillis();

    boolean xToggleMode = false;
    boolean yToggleMode = false;
    boolean aToggleMode = false;
    boolean bToggleMode = false;
    boolean lbToggleMode = false;
    boolean rbToggleMode = false;

    boolean lowerArm = true;
    double targetPot = 0.0;
    double output = 0.0;

    double tempPower = 0.0;

    double multiplier = 0.45;

    int lowerEncoderRange;
    int higherEncoderRange;

    boolean intake = false;



    private final static HardwareTestPlatform robot = new HardwareTestPlatform();

    @Override
    public void runOpMode() {
        begin();

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //robot.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.servoGrabber.setDirection(DcMotorSimple.Direction.FORWARD);

        //our motor = 3,892 encoder clicks per rotation
        double degreesToClicks = 3892/360;
        double clicksToDegrees = 360/3892;

        //     = 0
        double s1 = 45;   //[Degrees]
        double s2 = 67.5; //[Degrees]
        //     = 90

        double shoulderHighestRange = 90 * degreesToClicks * 2; //[Encoder Clicks]

        double S1 = s1 * degreesToClicks * 2; //[Encoder Clicks]
        double S2 = s2 * degreesToClicks * 2; //[Encoder Clicks]

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double r = Math.hypot((gamepad1.left_stick_x * -1), gamepad1.left_stick_y);
            double robotAngle = Math.atan2((gamepad1.left_stick_y * -1), (gamepad1.left_stick_x )) - Math.PI / 4;
            double rightX = 0.75*gamepad1.right_stick_x;
            double rightY = -0.4*gamepad1.right_stick_y;

            final double v1 = r * Math.cos(robotAngle) + rightX + rightY; // + rightY
            final double v2 = r * Math.sin(robotAngle) - rightX + rightY; // + rightY
            final double v3 = r * Math.sin(robotAngle) + rightX + rightY; // + rightY
            final double v4 = r * Math.cos(robotAngle) - rightX + rightY; // + rightY

            robot.motorLF.setPower(v1*multiplier);
            robot.motorRF.setPower(v2*multiplier);
            robot.motorLR.setPower(v3*multiplier);
            robot.motorRR.setPower(v4*multiplier);

            /**
             * Gamepad 1
             *  -----------------------------
             * |Joy 1    |Strafe             |
             * |Joy 2    |Turn               |
             * |x        |Speed toggle       |
             * |y        |Reverse intake     |
             * |a        |                   |
             * |b        |trapdoor           |
             * |lb       |Stop Lower         |
             * |rb       |Reverse            |
             * |lt       |                   |
             * |rt       |                   |
             *  -----------------------------

             * Gamepad 2
             *  -----------------------------
             * |Joy 1    |Extend/Retract     |
             * |Joy 2    |                   |
             * |x        |Stop intake        |
             * |y        |Forward intake     |
             * |a        |Lower Arm          |
             * |b        |Raise arm          |
             * |lb       |Raise 20%          |
             * |rb       |Raise 40%          |
             * |lt       |                   |
             * |rt       |                   |
             *  -----------------------------
             */

            double theta = 90 - (robot.pot.getVoltage() / 0.639)*90;

            /*
            timerInterval = 1000;
            if (gamepad1.x == true && xstopTime < System.currentTimeMillis()) {
                xstopTime = System.currentTimeMillis() + timerInterval;

                //action(s)

                //toggle actions
                if (xToggleMode) {
                    xToggleMode = false;
                } else {
                    xToggleMode = true;
                }
            }*/
            /*
            else {
                if (xToggleMode) {
                    //action 1
                    multiplier = 0.4;
                } else {
                    //action 2
                    multiplier = 1.0;
                }
            }*/

            if(gamepad1.b){
                bstopTime = System.currentTimeMillis() + 4000;
            }
            if(bstopTime > System.currentTimeMillis()){
                robot.servoDoor.setPosition(0.0);
            }
            else{
                robot.servoDoor.setPosition(0.4);
            }

            if (gamepad2.y == true) {
                robot.servoGrabber.setPower(-1.0);
            }
            else if (gamepad1.y == true) {
                robot.servoGrabber.setPower(1.0);
            }
            else if (gamepad2.x == true) {
                robot.servoGrabber.setPower(0.0);
            }


            if (gamepad1.left_trigger > 0.1) {
                //action(s)
                robot.motorLift.setPower(-gamepad1.left_trigger);
            }
            else if (gamepad1.right_trigger > 0.1) {
                //action(s)
                robot.motorLift.setPower(gamepad1.right_trigger);
            }
            else {
                robot.motorLift.setPower(0.0);
            }

            robot.motorArm.setPower(-gamepad2.left_stick_y*Math.abs(gamepad2.left_stick_y)*0.8);


            if(gamepad2.a && false){ /** Disabled  */
                targetPot = 0.7;// = 0 || 3.34 = bottom
                double currentPot = robot.pot.getVoltage();
                if(currentPot > 1.8 && currentPot < 3){
                    output = 2.2 * 0.138 * (Math.sin(theta) * Math.sin(theta)) + 0.1;
                }
                else{
                    output = 0.1;
                }
            }
            if(gamepad2.a && false){ /** Disabled  */
                double currentPot = robot.pot.getVoltage();
                if(currentPot < 1.8){
                    output = 0.25;
                }
                else {
                    output = 2.2 * 0.138 * (Math.sin(theta) * Math.sin(theta)) + 0.1;
                }
            }
            if(gamepad2.a){
                double currentPot = robot.pot.getVoltage();
                if(currentPot < 1.8){
                    output = 0.25;
                }
                else {
                    output = 0.138 * (Math.sin(theta*Math.PI/180) * Math.sin(theta*Math.PI/180)) + 0.1;
                }
            }
            else if(gamepad2.b){
                lowerArm = false;//lift works
                targetPot = 0.0; // = 90
                output = PID_Arm.getOutput(robot.pot.getVoltage(), targetPot) - 0.138*Math.cos(theta*Math.PI/180); //degree to rad test
            }
            else if(gamepad1.a){ /** REMOVE IF TEST FAILS */
                lowerArm = false;//lift works
                targetPot = 1.7; // = 90
                output = PID_Arm.getOutput(robot.pot.getVoltage(), targetPot);
            }
            else if(gamepad2.left_bumper){
                output = -0.7;
            }
            else if(gamepad2.right_bumper){
                output = -0.4;
            }
            else{
               output = 0.0;
            }if(gamepad1.left_bumper && gamepad1.x){ output = 0.3;}


            /**
             * Set final power
             */
            robot.motorShoulder.setPower(output);



            //telemetry
            telemetry.addData("Current Time (ms): ", System.currentTimeMillis());
            telemetry.addData("Acceleration Multiplier: ", multiplier);
            telemetry.addData("Touch: ", robot.touchSensor.isPressed());
            telemetry.addData("Pot Analog: ", robot.pot.getVoltage());
            telemetry.addData("Theta~: ", theta);
            telemetry.addData("Output: ", output);
            telemetry.addData("art", "owo");
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
