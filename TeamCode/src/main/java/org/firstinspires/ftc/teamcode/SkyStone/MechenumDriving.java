package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

// This class contains code for a functioning autonomus mecanum drivetrain
// In order to use this, you may extend class you ar working on with this.
// it contains Linear opmode classess as well as Vuforia and tensorflow code.
// Created By Karina
public class MechenumDriving extends VuforiaSkyStoneNavigationWebcam {

    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor backLeft;
    public DcMotor frontLeft;
    public AngularVelocity angleV;
    public ColorSensor colorSensor;
    public double degree = 0;
    public BNO055IMU imu;


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
        //Please don't use vuforia unless you are a master programmer or it gets much better in the futurw
        //its a bit of a waste of time
        //The color sensor can do the same thing, and navigation targets are hard
        //you could always try tensor flow. That works better but is a bit tricky to set up
        //ask lucas
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = false;
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        //Gyro stuff
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        colorSensor = hardwareMap.colorSensor.get("color");

    }

    //can be either break or float
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
//move left.
    //when the robot reaches the target encoder tick distance, it will stop,(hopefully)
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
        stopMotors();
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

    //this is not calibrated for the robot itself yet-sorry
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
        double targetDistanceTicks = Math.abs(targetDistance * ticksPerCm);
        double currentDistanceTicks = 0;
        degree = 0;
        AngularVelocity lastAngleV = proportionalDriveStuff(imu.getAngularVelocity());
        while ((Math.abs(currentDistanceTicks) < targetDistanceTicks) && opModeIsActive()) {
            lastAngleV = proportionalDriveStuff(lastAngleV);

            telemetry.addData("Target pos ticks: ", targetDistanceTicks);
            telemetry.addData("Target Distance:", targetDistance + "cm");
            currentDistanceTicks = (frontRight.getCurrentPosition() +
                    frontLeft.getCurrentPosition() +
                    backRight.getCurrentPosition() +
                    backLeft.getCurrentPosition()) / 4.0;
            telemetry.addData("Current pos ticks Avg: ", currentDistanceTicks);
            telemetry.addData("Current Distance cm", currentDistanceTicks / ticksPerCm);
            telemetry.update();
            //Straighten out
            while (Math.abs(degree) > 5) {
                if (degree > 5) {
                    frontLeft.setPower(power+ degree/360*.3);
                    frontRight.setPower(-power);
                    backLeft.setPower(power+ degree/360*.3);
                    backRight.setPower(-power);
                }
                if (degree < -5) {
                    frontLeft.setPower(power);
                    frontRight.setPower(-power - degree/360*.3);
                    backLeft.setPower(power);
                    backRight.setPower(-power- degree/360*.3);
                }
            }

        }
        stopMotors();
    }

    public void forwardWithProportionalDrive(double targetDistance, double power){
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double targetDistanceTicks = Math.abs(targetDistance * ticksPerCm);
        double currentDistanceTicks = 0;
        degree = 0;
        AngularVelocity lastAngleV = proportionalDriveStuff(imu.getAngularVelocity());
        while ((Math.abs(currentDistanceTicks) < targetDistanceTicks) && opModeIsActive()) {
            lastAngleV = proportionalDriveStuff(lastAngleV);

            telemetry.addData("Target pos ticks: ", targetDistanceTicks);
            telemetry.addData("Target Distance:", targetDistance + "cm");
            currentDistanceTicks = (frontRight.getCurrentPosition() +
                    frontLeft.getCurrentPosition() +
                    backRight.getCurrentPosition() +
                    backLeft.getCurrentPosition()) / 4.0;
            telemetry.addData("Current pos ticks Avg: ", currentDistanceTicks);
            telemetry.addData("Current Distance cm", currentDistanceTicks / ticksPerCm);
            telemetry.update();
            //Straighten out
            while (Math.abs(degree) > 5) {
                if (degree > 5) {
                    frontLeft.setPower(power+ degree/360*.3);
                    frontRight.setPower(power);
                    backLeft.setPower(power+ degree/360*.3);
                    backRight.setPower(power);
                }
                if (degree < -5) {
                    frontLeft.setPower(power);
                    frontRight.setPower(power - degree/360*.3);
                    backLeft.setPower(power);
                    backRight.setPower(power- degree/360*.3);
                }
            }

        }
        stopMotors();
    }

    public AngularVelocity proportionalDriveStuff(AngularVelocity angleV){
        telemetry.addData("AV",imu.getAngularVelocity());
        long last = angleV.acquisitionTime;
        AngularVelocity rate = imu.getAngularVelocity();
        long current = rate.acquisitionTime;
        double degreeChange = (current-last)*rate.yRotationRate/1.0e9;
        last = current;
        degree = degreeChange + degree;
        return rate;
    }

    // pulled from daniel's code
    // Below this is Daniel based versions

    public static final double GYRO_ERROR_THRESHOLD = 5;
    public static final double P_GYRO_TURN_COEFF = 0.008;
    //reference
    // imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES)

    private double getGyroError(double targetAngle) {
        double error = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES).firstAngle;

        // keep the error on a range of -179 to 180
        while(opModeIsActive() && error > 180)  error -= 360;
        while(opModeIsActive() && error <= -180) error += 360;

        return error;
    }

    public void gyroPivot(double speed, double angle) {
        double steer;
        double threshold = getGyroError(angle) > 0 ? GYRO_ERROR_THRESHOLD
                : -GYRO_ERROR_THRESHOLD;

        while(opModeIsActive() && Math.abs(getGyroError(angle)) > threshold) {

            steer = Range.clip(getGyroError(angle)
                    * P_GYRO_TURN_COEFF , -1, 1);

            double proportionalSpeed = speed * steer;

            frontLeft.setPower(proportionalSpeed);
            frontRight.setPower(proportionalSpeed);
            backLeft.setPower(proportionalSpeed);
            backRight.setPower(proportionalSpeed);
        }

        // when we're on target, stop the robot
        stopMotors();
    }


}
