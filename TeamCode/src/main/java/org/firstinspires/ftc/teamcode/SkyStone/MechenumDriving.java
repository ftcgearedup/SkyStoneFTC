package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

// This class contains code for a functioning autonomus mecanum drivetrain
// In order to use this, you may extend class you ar working on with this.
// it contains Linear opmode classess as well as Vuforia and tensorflow code.
// Created By Karina
public class MechenumDriving extends VuforiaSkyStoneNavigationWebcam {

    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor backLeft;
    public DcMotor frontLeft;
    public Gyroscope imu;
    public AngularVelocity angleV;
    public ColorSensor colorSensor;
    public double degree = 0;

    private double ticksPerRevNR20 = 560;
    private double ticksPerRevNR40 = 1120;
    private double ticksPerRevNR60 = 1680;

    //The post gear box gear ratio.
    private double gearRatio = 1.0;
    //The circumference of the drive wheel.
    private double wheelCircumference = 31.9024; // ??
    //Formula to calculate ticks per centimeter for the current drive set up.FORWARDS/BACKWARD ONLY
    private double ticksPerCm = (ticksPerRevNR40 * gearRatio) / wheelCircumference;
    //Formula to calculate ticks per centimeter for the current drive set up.SIDEWAYS

    boolean calib;

    public void initDriveHardware(){
        // init the motors
        frontRight = hardwareMap.dcMotor.get("fr");
        backRight = hardwareMap.dcMotor.get("br");
        frontLeft = hardwareMap.dcMotor.get("fl");
        backLeft = hardwareMap.dcMotor.get("bl");

        // set wheel direction
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);


        // tHIS Is set up for vuforia stuff
        //Please don't use vuforia unless you are a master programmer of it gets much better in the futurw
        //its a bit of a waste of time
        //The color sensor can do the same thing, and navigation targets are hard
        //you could always try tensor flow. That works better but is a bit tricky to set up
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = false;
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        //Gyro stuff
        imu = hardwareMap.get(Gyroscope.class, "imu");

        colorSensor = hardwareMap.colorSensor.get("color");

    }

    //can be either breake or float
    public void setZeroPowBehv(DcMotor.ZeroPowerBehavior behv) {
        frontLeft.setZeroPowerBehavior(behv);
        frontRight.setZeroPowerBehavior(behv);
        backLeft.setZeroPowerBehavior(behv);
        backRight.setZeroPowerBehavior(behv);
    }

    // Sometimes the motors arwe on backwords and this fixes that
    public void setDirection() {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    //for backwards use negative power
    //Pass in centimeters.
    public void forward(double targetDistance, double power) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double targetDistanceTicks = Math.abs(targetDistance * ticksPerCm);
        double currentDistanceTicks = 0;
        while ((Math.abs(currentDistanceTicks) < targetDistanceTicks) && opModeIsActive()) {
            telemetry.addData("Target pos ticks: ", targetDistanceTicks);
            telemetry.addData("Target Distance:", targetDistance + "cm");
            currentDistanceTicks = (frontRight.getCurrentPosition() +
                    frontLeft.getCurrentPosition() +
                    backRight.getCurrentPosition() +
                    backLeft.getCurrentPosition()) / 4.0;
            telemetry.addData("Current pos ticks Avg: ", currentDistanceTicks);
            telemetry.addData("Current Distance cm", currentDistanceTicks / ticksPerCm);
            telemetry.update();

            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
        }
        stopMotors();
    }
    //stop. Just stop
    public void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    //move forward forever
    public void forwardForever() {
        frontLeft.setPower(1);
        frontRight.setPower(1);
        backLeft.setPower(1);
        backRight.setPower(1);
    }

    public void setDriveMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    //Rightwards
    //D
    public void sideRight(double targetDistance, double power) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double currentDistance = 0;
        while ((currentDistance < targetDistance) && opModeIsActive()) {
            currentDistance = frontLeft.getCurrentPosition();
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(-power);
            backRight.setPower(power);
        }

    }

    public void sideLeft(double targetDistance, double power) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double currentDistance = 0;
        while (currentDistance < targetDistance && opModeIsActive()) {
            currentDistance = frontRight.getCurrentPosition();
            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(-power);
        }
    }
    //clockwise is 0 cc is 1
    public void pivotCW(double degree, double power) {

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentDegree = 0;
        while ((currentDegree < degree) && opModeIsActive()) {
            currentDegree = (frontLeft.getCurrentPosition() + backLeft.getCurrentPosition())/2;
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backRight.setPower(-power);
        }
        stopMotors();
    }

    public void pivotCC(double degree, double power) {

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentDegree = 0;
        while (currentDegree < degree && opModeIsActive()) {
            currentDegree = (frontRight.getCurrentPosition() + backRight.getCurrentPosition())/2;
            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(-power);
            backRight.setPower(power);
        }
        stopMotors();
    }

    public void sidewaysWithProportionalDrive(double targetDistance, double power){
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double currentDistance = 0;
        while ((currentDistance < targetDistance) && opModeIsActive()) {
            currentDistance = Math.abs(frontLeft.getCurrentPosition());
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(-power);
            backRight.setPower(power);
        }
        proportionalDrive();
        if (degree == 0 || (degree > 0 && degree < 5) || (degree < 0 && Math.abs(degree) > 5)){
            stopMotors();
        }else {
            while (((degree != 0) && (Math.abs(degree) > 5))){
                if(degree > 5){
                    backLeft.setPower(.3);
                    frontLeft.setPower(.3);
                }if (degree < -5){
                    backRight.setPower(.3);
                    frontRight.setPower(.3);
                }
            }
        }
    }

    public void forwardWithProportionalDrive(double targetDistance, double power){
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double targetDistanceTicks = Math.abs(targetDistance * ticksPerCm);
        double currentDistanceTicks = 0;
        proportionalDrive();
        while ((Math.abs(currentDistanceTicks) < targetDistanceTicks) && opModeIsActive()) {
            telemetry.addData("Target pos ticks: ", targetDistanceTicks);
            telemetry.addData("Target Distance:", targetDistance + "cm");
            currentDistanceTicks = (frontRight.getCurrentPosition() +
                    frontLeft.getCurrentPosition() +
                    backRight.getCurrentPosition() +
                    backLeft.getCurrentPosition()) / 4.0;
            telemetry.addData("Current pos ticks Avg: ", currentDistanceTicks);
            telemetry.addData("Current Distance cm", currentDistanceTicks / ticksPerCm);
            telemetry.update();

            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
        }if (degree == 0 || (degree > 0 && degree < 5) || (degree < 0 && Math.abs(degree) > 5)){
            stopMotors();
        }else {
            while (((degree != 0) && (Math.abs(degree) > 5))){
                if(degree > 5){
                    backLeft.setPower(.3);
                    frontLeft.setPower(.3);
                }if (degree < -5){
                    backRight.setPower(.3);
                    frontRight.setPower(.3);
                }
            }
        }
        stopMotors();
    }

    public void proportionalDrive(){
        angleV = imu.getAngularVelocity(AngleUnit.DEGREES);
        telemetry.addData("AV",imu.getAngularVelocity(AngleUnit.DEGREES));
        long last = angleV.acquisitionTime;
        degree = 0;
        while (opModeIsActive()) {
            AngularVelocity rate = imu.getAngularVelocity(AngleUnit.DEGREES);
            long current = rate.acquisitionTime;
            double degreeChange = (current-last)*rate.yRotationRate/1.0e9;
            last = current;
            degree = degreeChange+degree;
        }
    }
}
