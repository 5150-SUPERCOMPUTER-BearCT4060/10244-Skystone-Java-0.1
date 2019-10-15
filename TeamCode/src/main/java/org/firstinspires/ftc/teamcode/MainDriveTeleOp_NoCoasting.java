/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Two Person Drive NOOASTING", group="Main")

public class MainDriveTeleOp_NoCoasting extends OpMode {

    // Declare OpMode members.


    //Motors
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left_mtr = null;
    private DcMotor right_mtr = null;
    private DcMotor elbow_mtr = null;
    private DcMotor arm_mtr = null;


    //Servos
    //   private Servo colorArm = null;
    private Servo claw_1 = null;
    private Servo claw_2 = null;

    private Double LeftValue ;
    private Double RightValue;

   // private float leftPos = left_mtr.getCurrentPosition();
   // private float rightPos = right_mtr.getCurrentPosition();
   // private float armPos = elbow_mtr.getCurrentPosition();
   // private float armPos = arm_mtr.getCurrentPosition();

    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //Motors
        left_mtr = hardwareMap.get(DcMotor.class, "left_mtr");
        right_mtr = hardwareMap.get(DcMotor.class, "right_mtr");
        arm_mtr = hardwareMap.get(DcMotor.class, "arm_mtr");
        elbow_mtr = hardwareMap.get(DcMotor.class, "elbow_mtr");

        //Servos
        claw_1 = hardwareMap.get(Servo.class, "claw_1");
        claw_2 = hardwareMap.get(Servo.class, "claw_2");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        left_mtr.setDirection(DcMotor.Direction.FORWARD);
        right_mtr.setDirection(DcMotor.Direction.REVERSE);
        arm_mtr.setDirection(DcMotor.Direction.FORWARD);
        elbow_mtr.setDirection(DcMotor.Direction.FORWARD);

        left_mtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_mtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_mtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow_mtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_mtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_mtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_mtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow_mtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left_mtr.setPower(0);
        right_mtr.setPower(0);
        arm_mtr.setPower(0);
        elbow_mtr.setPower(0);


        // Wait for the game to start (driver presses PLAY)
        runtime.reset();
    }


    // run until the end of the match (driver presses STOP)
    public void loop() {

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        //Motors
        double drive = gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;
        leftPower = Range.clip(drive + turn, -1, 1);
        rightPower = Range.clip(drive - turn, -1, 1);


        if (gamepad2.left_stick_y < 0.1 || gamepad2.left_stick_y > -0.1) {
           // liftMotor.setPower(gamepad2.left_stick_y);
        }

        else if (!gamepad2.dpad_down || !gamepad2.dpad_up) {
            //liftMotor.setPower(0);
        }

        // arm is trigger use numbers
        if (gamepad2.left_trigger == 1) {
            arm_mtr.setPower(-0.5);
        } else if (gamepad2.right_trigger <9) {
            arm_mtr.setPower(1);
        } else if (gamepad2.left_trigger <-1 || gamepad2.right_trigger <-1) {
            arm_mtr.setPower(0.0);
        }



    //elbow is bumpers use !
        if (gamepad2.left_bumper) {
            elbow_mtr.setPower(-0.5);
        } else if (gamepad2.right_bumper) {
            elbow_mtr.setPower(1);
        } else if (!gamepad2.left_bumper && !gamepad2.right_bumper) {
            elbow_mtr.setPower(0.0);
        }


        //Servos
        LeftValue = 0.0;
        RightValue = 0.0;


        if (gamepad2.x) {

            //clawHand.setPosition(0);

        } else if (gamepad2.y) {
            //lHand.setPosition(1.0);
        } else if (gamepad2.a) {
            claw_1.setPosition(0);
        } else if (gamepad2.b) {
            claw_2.setPosition(1);
        }



         if (gamepad2.y) {
        //rHand.setPosition(1.0);
        } else if (gamepad2.a) {
        claw_1.setPosition(0);
        } else if (gamepad2.b) {
        claw_2.setPosition(1);
        }

        // Send calculated power to wheels
        left_mtr.setPower(leftPower);
        right_mtr.setPower(rightPower);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
       //  telemetry.addData("Left POS", "(%.2f)", leftPos);
       // telemetry.addData("Right POS", "(%.2f)", rightPos);
       // telemetry.addData("Arm  POS", "(%.2f)", armPos);
       // telemetry.addData("elbow  POS", "(%.2f)", armPos);
        telemetry.update();
    }

        public void stop() {
            left_mtr.setPower(0);
            right_mtr.setPower(0);
            arm_mtr.setPower(0);
            elbow_mtr.setPower(0);
            }
}