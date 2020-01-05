package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

public class AutoRedRight extends MechenumDriving {

    //Attachments
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private DcMotor lift;
    private Servo dropper;
    private Servo clamp1;
    private Servo clamp2;
    private Gyroscope imu;
    private AngularVelocity angleV;
    private ColorSensor colorSensor;
    private double degree = 0;

    public void initHardware() {

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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.loggingEnabled = false;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(Gyroscope.class, "imu");
    }

    public void runOpMode() {
        initHardware();
        setZeroPowBehv(DcMotor.ZeroPowerBehavior.BRAKE);
        setDirection();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
    }

    public void intake(double power) {
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }
}
