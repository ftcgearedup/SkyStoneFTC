package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// This class extends the mecanum drivetrain autonomus code and is spicific to the Skystone robot Geared Up created.
// It holds all the attachment code and sensor algorithms.
// Our compitition autonomuses extend this and utilize its algorithms.
//Created By Karina
public class SkystoneRobot extends MechenumDriving {

    public DcMotor lift;
    public Servo release;
    public Servo clamp1;
    public Servo clamp2;
    public Servo intake;


    public void initAttachCode() {
        //init the Attachments
        intake = hardwareMap.servo.get("IN");
        lift = hardwareMap.dcMotor.get("lift");

        release = hardwareMap.servo.get("AIR");
        clamp1 = hardwareMap.servo.get("FC");
        clamp2 = hardwareMap.servo.get("BC");

        //set attatchment direction
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        // set attachment power variables
        lift.setPower(0);

    }


    // This code uses the reflected light value of the skystone in order to determine weather or not it is the correct one.
    //If the value is below 15 it collects, and  if it is above 15 it moves on.
    //These values are estimated at about a Distance of 4-6 inches in a medium light enviorment with the LED on
    public void findSkyStoneAlpha() {
        if (colorSensor.alpha() >= 15) {
            forwardForever();
        } else if (colorSensor.alpha() <= 15) {
            stopMotors();
            intake.setPosition(1);
            forward(10, -.5);
            sideRight(15, .5);
            forward(10, .5);
        }
    }

    // This code uses the reflected red light value of the skystone in order to determine weather or not it is the correct one.
    //If the value is below 10 it collects, and  if it is above 10 it moves on.
    //These values are estimated at about a Distance of 4-6 inches in a medium light enviorment with the LED on
    public void findSkyStoneRed() {
        while (colorSensor.red() >= 10) {
            forwardForever();
            if (colorSensor.red() <= 10) {
                stopMotors();
                intake.setPosition(1);
                forward(10, -.5);
                sideRight(15, .5);
                forward(10, .5);
                break;
            }
        }

    }

    public void intakeStone() {
        intake.setPosition(1);
        while (colorSensor.red() >= 15) {
            intake.setPosition(1);
        }
        if (colorSensor.red() <= 15) {
            intake.setPosition(1);
        }
    }

    public void seaStone() {
        while (colorSensor.red() <= 15) {
            intake.setPosition(1);
        }
        if (colorSensor.red() >= 15) {
            intake.setPosition(1);

        }

    }
}