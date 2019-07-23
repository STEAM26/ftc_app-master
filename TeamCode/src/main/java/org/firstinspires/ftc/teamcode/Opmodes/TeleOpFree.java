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

import org.firstinspires.ftc.teamcode.HardwareProfiles.HardwareTestPlatform;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Teleopfreedom", group = "COMP")

public class TeleOpFree extends LinearOpMode {
    /**
     * Instantiate all objects needed in this class
     */
    boolean editMode = false;
    long xstopTime = System.currentTimeMillis();


    private final static HardwareTestPlatform robot = new HardwareTestPlatform();
    double myCurrentLauncherPosition = 0;
    int launcherEncoderStop = 0;

    @Override
    public void runOpMode() {

        begin();

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //robot.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        long timerInterval = 0;
        int targetPOS = 0;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("TeleOpRearWheelDrive Active", "");
        telemetry.update();
        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {

            //robot.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION); //// =======

            double r = Math.hypot((gamepad1.left_stick_x * -1), gamepad1.left_stick_y);
            double robotAngle = Math.atan2((gamepad1.left_stick_y * -1), (gamepad1.left_stick_x )) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            double rightY = 0.5*gamepad1.right_stick_y;
            final double v1 = r * Math.cos(robotAngle) + rightX; // + rightY
            final double v2 = r * Math.sin(robotAngle) - rightX; // + rightY
            final double v3 = r * Math.sin(robotAngle) + rightX; // + rightY
            final double v4 = r * Math.cos(robotAngle) - rightX; // + rightY

            robot.motorLF.setPower(v1*0.3);
            robot.motorRF.setPower(v2*0.3);
            robot.motorLR.setPower(v3*0.3);
            robot.motorRR.setPower(v4*0.3);



            if(gamepad1.x == true && xstopTime < System.currentTimeMillis()) {
                xstopTime = System.currentTimeMillis() + 1000;
                editMode = !editMode;
            }



            if(gamepad1.left_trigger > 0.1) {
                robot.motorShoulder.setPower(1.0);
            }
            else if(gamepad1.right_trigger > 0.1) {
                robot.motorShoulder.setPower(-1.0);
            }
            else {
                robot.motorShoulder.setPower(0.0);
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


            telemetry.addData("Edit Mode", editMode);
            telemetry.addData("Current Time", System.currentTimeMillis());
            telemetry.addData("Stop Time x", xstopTime);
            telemetry.addData("Lift encoder", robot.motorLift.getCurrentPosition());
            telemetry.addData("Target Encoder Pos", targetPOS);
            telemetry.update();
            //8014 7120 7200
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
