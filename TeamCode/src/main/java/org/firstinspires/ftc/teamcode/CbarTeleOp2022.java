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


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



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

@TeleOp(name="Comp Opmode", group="Practice")

public class CbarTeleOp2022 extends OpMode {

    //DriveTrain Motor Variables
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor centerDrive;


    //DriveTrain Motor power variables
    double leftDrivePower;
    double rightDrivePower;
    double centerDrivePower;

    //LiftSystem Motor variables
    DcMotor liftMotor;
    DcMotor leftIntake;
    DcMotor rightIntake;

    //LiftSystem Motor power variables
    double liftPower;
    double leftIntakePower;
    double rightIntakePower;

    //Carosel
    DcMotor caroselMotor;

    double caroselPower;


    @Override
    public void init(){
        //DriveTrain Hardware mapping
        leftDrive=hardwareMap.dcMotor.get("leftDrive");
        rightDrive=hardwareMap.dcMotor.get("rightDrive");
        centerDrive=hardwareMap.dcMotor.get("centerDrive");



        //Intake Hardware mapping
        liftMotor=hardwareMap.dcMotor.get("liftMotor");
        leftIntake=hardwareMap.dcMotor.get("leftIntake");
        rightIntake=hardwareMap.dcMotor.get("rightIntake");

        //Set directions
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        centerDrive.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        caroselMotor.setDirection(DcMotor.Direction.FORWARD);
    }
    @Override
    public void loop(){
        leftDrivePower=-gamepad1.left_stick_y;
        rightDrivePower=-gamepad1.right_stick_y;


        if(gamepad1.dpad_left==true){
            centerDrivePower=1;
        }
        else if(gamepad1.dpad_right==true){
            centerDrivePower=-1;
        }
        else{
            centerDrivePower=0;
        }

        leftDrive.setPower(leftDrivePower);
        rightDrive.setPower(rightDrivePower);
        centerDrive.setPower(centerDrivePower);

        liftPower=gamepad2.left_stick_y;
        liftMotor.setPower(liftPower);

        //Intake
        if(gamepad2.x==true){
            leftIntake.setPower(1);
            rightIntake.setPower(1);
        }else if(gamepad2.y==true){
            leftIntake.setPower(-1);
            rightIntake.setPower(-1);
        }else{
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }

        //Carosel movement
        if(gamepad2.a==true){
            caroselMotor.setPower(1);
        }
        else if(gamepad2.b==true){
            caroselMotor.setPower(-1);
        }
        else{
            caroselMotor.setPower(0);
        }
    }
}
