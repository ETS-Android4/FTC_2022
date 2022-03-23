/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backwards               Left-joystick Forward/Backwards
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backwards when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Driver:Test ", group="Test_FTC_20222")
public class TestDrive extends LinearOpMode {

    // Main chassis drive
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor centerDrive = null;

    // Lift and intake
    private DcMotor liftDrive = null;
    private DcMotor rightIntake = null;
    private DcMotor leftIntake = null;

    //carousel spinner
    private DcMotor caroussel = null;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive  = hardwareMap.get(DcMotor.class, "right_drive");
        centerDrive = hardwareMap.get(DcMotor.class, "center_drive");

        liftDrive = hardwareMap.get(DcMotor.class, "lift_drive");
        rightIntake = hardwareMap.get(DcMotor.class, "right_intake");
        leftIntake = hardwareMap.get(DcMotor.class, "left_intake");
        caroussel = hardwareMap.get(DcMotor.class, "carousel");

        // Most robots need the motors on one side to be reversed to drive forward.
        // When you first test your robot, push the left joystick forward
        // and flip the direction ( FORWARD <-> REVERSE ) of any wheel that runs backwards
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        centerDrive.setDirection(DcMotor.Direction.FORWARD);

        liftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotor.Direction.REVERSE);

        caroussel.setDirection(DcMotor.Direction.FORWARD);


        // main variables
        double left_drive_power = 0;
        double right_drive_power = 0;
        double center_drive_power = 0;

        double lift_drive_power = 0;
        double right_intake_power = 0;
        double left_intake_power = 0;

        double carousel_power = 0;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            left_drive_power = -gamepad1.left_stick_y;
            right_drive_power = -gamepad1.right_stick_y;

            if(gamepad1.dpad_left){
                center_drive_power = 1;
            }else if(gamepad1.dpad_right){
                center_drive_power = -1;
            }else{
                center_drive_power = 0;
            }

            // the intake and lift controls

            //lift_drive_power =

            if(gamepad2.dpad_up){
                lift_drive_power = 1;
            }else if(gamepad2.dpad_down){
                lift_drive_power = -1;
            }else{
                lift_drive_power = 0;
            }

            if(gamepad2.a){
                right_intake_power = 1;
                left_intake_power = 1;
            }else if(gamepad2.b){
                right_intake_power = -1;
                left_intake_power = -1;
            }else{
                right_intake_power = 0;
                left_intake_power = 0;
            }

            if(gamepad2.x){
                carousel_power = 1;
            }else if(gamepad2.y){
                carousel_power = -1;
            }else{
                carousel_power = 0;
            }


            // Send calculated power to wheels
            leftDrive.setPower(left_drive_power);
            rightDrive.setPower(right_drive_power);
            centerDrive.setPower(center_drive_power);

            liftDrive.setPower(lift_drive_power);
            rightIntake.setPower(right_intake_power);
            leftIntake.setPower(left_intake_power);

            caroussel.setPower(carousel_power);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", left_drive_power, right_drive_power);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", center_drive_power, lift_drive_power);
            telemetry.update();
        }
    }}
