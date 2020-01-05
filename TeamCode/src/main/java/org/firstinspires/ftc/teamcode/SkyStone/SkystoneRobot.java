package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// This class extends the mecanum drivetrain autonomus code and is spicific to the Skystone robot Geared Up created.
// It holds all the attachment code and sensor algorithms.
// Our compitition autonomuses extend this and utilize its algorithms.
//Created By Karina
public class SkystoneRobot extends MechenumDriving {

    public DcMotor lift;
    public Servo dropper;
    public Servo clamp1;
    public Servo clamp2;
    public DcMotor intakeLeft;
    public DcMotor intakeRight;

    public void initAttachCode(){
        //init the Attachments
        intakeLeft = hardwareMap.dcMotor.get("il");
        intakeRight = hardwareMap.dcMotor.get("ir");
        lift = hardwareMap.dcMotor.get("lift");
        dropper = hardwareMap.servo.get("drop");
        clamp1 = hardwareMap.servo.get("clamp1");
        clamp2 = hardwareMap.servo.get("clamp2");

        //set attatchment direction
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        // set attachment power variables
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
        lift.setPower(0);
    }
    // This code uses the reflected light value of the skystone in order to determine weather or not it is the correct one.
    //If the value is below 15 it collects, and  if it is above 15 it moves on.
    //These values are estimated at about a Distance of 4-6 inches in a medium light enviorment with the LED on
    public void findSkyStoneAlpha() {
        if (colorSensor.alpha() >= 15){
            forwardForever();
        } else if (colorSensor.alpha() <= 15){
            stopMotors();
            intake(1);
            forward(10, -.5);
            sideRight(15,.5);
            forward(10, .5);
        }
    }

    // This code uses the reflected red light value of the skystone in order to determine weather or not it is the correct one.
    //If the value is below ____________ it collects, and  if it is above __________ it moves on.
    //These values are estimated at about a Distance of 4-6 inches in a medium light enviorment with the LED on
    public void findSkyStoneRed(){
        if (colorSensor.red() >= 15){
            forwardForever();
        } if (colorSensor.red() <= 15){
            stopMotors();
            intake(1);
            forward(10, -.5);
            sideRight(15,.5);
            forward(10, .5);
        }
    }

    //Turns both sides of the intake on.
    public void intake(double power){
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }

    // sends the light sensor readings to the telemetry on the phone for testing pourposes.
    public void readColorSensor(){
        colorSensor.enableLed(true);
        colorSensor.alpha();
        colorSensor.red();
        telemetry.addData("LightVal", colorSensor.alpha());
        telemetry.addData("RedVal", colorSensor.red());
        telemetry.update();
    }

}
