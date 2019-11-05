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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;


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

@Autonomous(name="Autonomous_BuildSide", group="Main")

public class Autonomous_BuildSide extends OpMode {

    // Declare OpMode members.
    //arm_motor = intake_mtr
    //


    //Motors
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left_mtr = null;
    private DcMotor right_mtr = null;
    private DcMotor l_lift = null;
    private DcMotor r_lift = null;
    private DcMotor intake_mtr = null;
    //Servos
    //   private Servo colorArm = null;

    private Servo claw_1 = null;
    private Servo claw_2 = null;

    private Double LeftValue;
    private Double RightValue;

   /* private float leftPos = left_mtr.getCurrentPosition();
    private float rightPos = right_mtr.getCurrentPosition();
    private float IntakePos = intake_mtr.getCurrentPosition();
    private float LliftPos = l_lift.getCurrentPosition();
    private float RliftPos = r_lift.getCurrentPosition(); */

    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //Motors
        left_mtr = hardwareMap.get(DcMotor.class, "left_mtr");
        right_mtr = hardwareMap.get(DcMotor.class, "right_mtr");
        l_lift = hardwareMap.get(DcMotor.class, "l_lift");
        r_lift = hardwareMap.get(DcMotor.class, "r_lift");
        intake_mtr = hardwareMap.get(DcMotor.class, "intake_mtr");
        //Servos

        claw_1 = hardwareMap.get(Servo.class, "claw_1");
        claw_2 = hardwareMap.get(Servo.class, "claw_2");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        left_mtr.setDirection(DcMotor.Direction.FORWARD);
        right_mtr.setDirection(DcMotor.Direction.REVERSE);
        l_lift.setDirection(DcMotor.Direction.FORWARD);
        r_lift.setDirection(DcMotor.Direction.REVERSE);
        intake_mtr.setDirection(DcMotor.Direction.FORWARD);

        left_mtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_mtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_mtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_mtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_mtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        l_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake_mtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left_mtr.setPower(0);
        right_mtr.setPower(0);
        l_lift.setPower(0);
        r_lift.setPower(0);
        intake_mtr.setPower(0);

        claw_1.setPosition(0);
        claw_2.setPosition(0);


        // Wait for the game to start (driver presses PLAY)
        runtime.reset();

    }
    long maxTime = 0;

    //creates the pause function
    public void pause(long time){
        maxTime = System.currentTimeMillis() + time;
        while(System.currentTimeMillis() < maxTime) {
        }

    }

    @Override
    public void loop() {

        //76.2cm/sec
        //Start on Pad side
        //reverse all drive power

        //DriveForward(-1,100);
        //Forward 61 cm


        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

            //use pause(time) for a delay
        /*
            DriveForward(-1, 900);
            pause(900);
        left_mtr.setPower(0);
        right_mtr.setPower(0);
            TurnRight(1, 2000);
            pause(2000);
        left_mtr.setPower(0);
        right_mtr.setPower(0);
            DriveForward(-1,2500);
            pause(2500);
        left_mtr.setPower(0);
        right_mtr.setPower(0);
        */
         //red turn left, blue turn right
        //buildside
         DriveForward(-1,1200);
        pause(1200);
         left_mtr.setPower(0);
        right_mtr.setPower(0);

        SuckIt(-1,1000);
        GrabIt(500);
        pause(1500);
        intake_mtr.setPower(0);

        DriveReverse(-1,1200);
        pause(1200);
         left_mtr.setPower(0);
        right_mtr.setPower(0);

        DropIt(500);
        pause(500);

        TurnLeft(1,1000);
        pause(900);
        left_mtr.setPower(0);
        right_mtr.setPower(0);

        DriveForward(-1,400);
        pause(400);
        left_mtr.setPower(0);
        right_mtr.setPower(0);


        pause(26000);



    }



    public void DriveForward(double power, double time) {
        left_mtr.setPower(power);
        right_mtr.setPower(power);
    }

    public void DriveForwardTime(double power, long time) throws InterruptedException {
        TurnLeft(power, time);
        right_mtr.setPower(power);
        sleep(time);
    }

    public void TurnRight(double power, double time) {
        left_mtr.setPower(-power);
    }

    public void TurnRightTime(double power, long time) throws InterruptedException {
        TurnLeft(power, time);
        left_mtr.setPower(-power);
        sleep(time);
    }

    public void TurnLeft(double power, double time) {
        right_mtr.setPower(-power);
    }

    public void TurnLeftTime(double power, long time) throws InterruptedException {
        TurnLeft(power, time);
        right_mtr.setPower(-power);
        sleep(time);
    }

    public void DriveReverse(double power, double time) {
        left_mtr.setPower(-power);
        right_mtr.setPower(-power);
    }

    public void DriveReverseTime(double power, long time) throws InterruptedException {
        DriveReverse(power, time);
        right_mtr.setPower(-power);
        right_mtr.setPower(-power);
        sleep(time);
    }

    public void SuckIt(double power, double time) {
        intake_mtr.setPower(-power);
    }

    public void SuckItTime(double power, long time) throws InterruptedException {
        TurnLeft(power, time);
        intake_mtr.setPower(power);
        sleep(time);
    }

    public void BlowIt(double power) {
        intake_mtr.setPower(-power);
    }

    public void BlowIt(double power, long time) throws InterruptedException {
        TurnLeft(power, time);
        intake_mtr.setPower(-power);
        sleep(time);
    }

    public void GrabIt(double time) {
        claw_1.setPosition(0);
        claw_2.setPosition(0);
    }

    public void DropIt( double time) {
        claw_1.setPosition(1);
        claw_2.setPosition(1);
    }

    public void StopDriving() {
        DriveForward(0, 0);
    }

    private void justWait(int miliseconds) {

        double currTime = getRuntime();
        double waitUntil = currTime + (double) (miliseconds / 1000);
        while (getRuntime() < waitUntil) {

        }

//protected void pause(long millis){
        //ElapsedTime() = System.currentTimeMillis() + millis;
        //While(System.currentTimeMillis() < maxtime && !singleThread.isInterupted())
    }


}



