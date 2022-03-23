package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This configuration file consists of the main setup for each
 * sensors and actuators on the motor. Each sensors names, directions,
 * speeds and limits.
 * It is divided into :
 * variable declarations
 * initialization
 * hardware mapping
 */
public class CbarMainConfig {
    public ElapsedTime runtime = new ElapsedTime();  // main time tracking object
    HardwareMap hardwareMap; // main hardware mapping variable

    //DriveTrain Motor Variables

    DcMotor leftDrive = null;
    DcMotor rightDrive = null;
    DcMotor centerDrive = null;

    //LiftSystem Motor variables
    DcMotor liftMotor = null;
    DcMotor leftIntake = null;
    DcMotor rightIntake = null;

    //Carosel
    DcMotor caroselMotor = null;

    public CbarMainConfig(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

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

}
