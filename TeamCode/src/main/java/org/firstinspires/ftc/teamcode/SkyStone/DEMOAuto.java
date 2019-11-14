package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class DEMOAuto extends LinearOpMode {

    private DcMotor frontRight;
    private DcMotor frontleft;
    private DcMotor backRight;
    private DcMotor backleft;

    public void intihardware(){
        frontRight = hardwareMap.dcMotor.get("fR");
        frontleft = hardwareMap.dcMotor .get ("fL");
        backRight = hardwareMap .dcMotor .get ("bR");
        backleft = hardwareMap .dcMotor .get ("bL");

    }

    public void pause (){
        frontRight .setPower(0);
        frontleft .setPower(0);
        backRight .setPower(0);
        backleft .setPower(0);

    }
    public void foreward (){
        frontRight .setPower(.5);
        frontleft .setPower(.5);
        backRight .setPower (.5);
        backleft .setPower (.5);

    }
    public void turn () {
        frontRight .setPower(1);
        frontleft .setPower(-1);
        backRight .setPower(1);
        backleft .setPower(-1);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        foreward();
        pause();
        turn();
    }
}