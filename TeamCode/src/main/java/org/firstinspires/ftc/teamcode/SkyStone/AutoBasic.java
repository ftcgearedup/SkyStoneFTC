package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaSkyStoneNavigationWebcam;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

@Autonomous(name = "MainAuto", group = "Autonomous")
public class AutoBasic extends VuforiaSkyStoneNavigationWebcam {
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;

    //Attachments
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private DcMotor lift;
    private Servo dropper;
    private Servo clamp1;
    private Servo clamp2;
    private Gyroscope imu;
    private AngularVelocity angleV;
    private double degree = 0;

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

    public void initHardware() {
        // init the motors
        frontRight = hardwareMap.dcMotor.get("fr");
        backRight = hardwareMap.dcMotor.get("br");
        frontLeft = hardwareMap.dcMotor.get("fl");
        backLeft = hardwareMap.dcMotor.get("bl");

        //init the Attachments
        intakeLeft = hardwareMap.dcMotor.get("il");
        intakeRight = hardwareMap.dcMotor.get("ir");
        lift = hardwareMap.dcMotor.get("lift");
        dropper = hardwareMap.servo.get("drop");
        clamp1 = hardwareMap.servo.get("clamp1");
        clamp2 = hardwareMap.servo.get("clamp2");

        // set wheel direction
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //set attatchment direction
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        // set wheel power variables
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // set attachment power variables
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
        lift.setPower(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.loggingEnabled = false;
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(Gyroscope.class, "imu");




    }

    public void runOpMode(){

        initHardware();
        setZeroPowBehv(DcMotor.ZeroPowerBehavior.BRAKE);
        setDirection();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chooseprogram();
           waitForStart();
    }

    public void setZeroPowBehv(DcMotor.ZeroPowerBehavior behv) {
        frontLeft.setZeroPowerBehavior(behv);
        frontRight.setZeroPowerBehavior(behv);
        backLeft.setZeroPowerBehavior(behv);
        backRight.setZeroPowerBehavior(behv);
    }

    public void setDirection() {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setDriveMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
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

    public void chooseprogram() {
        telemetry.addData("ChoosePosition", "RED1-A, RED2-B, Blue1-X, Blue2-Y");
        while (!isStarted()) {
            if (gamepad1.a) {
                telemetry.addData("RobotPosition", "RED1");
                red1();
                telemetry.update();
                break;
            } else if (gamepad1.b) {
                telemetry.addData("RobotPosition", "RED2");
                red2();
                telemetry.update();
                break;
            } else if (gamepad1.x) {
                telemetry.addData("RobotPosition", "BLUE1");
                blue1();
                telemetry.update();
                break;
            } else if (gamepad1.y) {
                telemetry.addData("RobotPosition", "BLUE2");
                blue2();
                telemetry.update();
                break;
            }else {
                telemetry.addData("waiting", "help");
                telemetry.update();
            }
        }
    }

    public void red1(){
        waitForStart();
        while (opModeIsActive()) {
            forward(65, .5);
            double encoderVal = frontLeft.getCurrentPosition();

            // Unused Vuforia Code
           /* while (lastSkyStoneLocation == null) {
                setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //driveLeft
                frontLeft.setPower(-.25);
                frontRight.setPower(.25);
                backLeft.setPower(.25);
                backRight.setPower(-.25);
                encoderVal = frontLeft.getCurrentPosition();
            }
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
*/
           // intake a Skystone
           sideLeft(1000,.75);
           intake(1);
           forward(30, .75);
           forward(30, -.75);
           forward(0,0);
           sideRight(1050, .75);
           forward(60,.75);
           forward(70, -.75);
           intake(0);

           // Unused but useful
            /*setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (frontLeft.getCurrentPosition() >= -encoderVal) {
                frontLeft.setPower(-.5);
                frontRight.setPower(.5);
                backLeft.setPower(.5);
                backRight.setPower(-.5);
            }
            */

           //Move to Foundation
           sidewaysWithProportionalDrive(5300,1);
           forwardWithProportionalDrive(40, 1);

           // Get the foundation
           sideRight(30, 1);
           clamp1.setPosition(0);
           clamp2.setPosition(1);

           // Move the foundation
           forwardWithProportionalDrive(300, -1);
           clamp1.setPosition(.75);
           clamp2.setPosition(0);

           // Park
           sideLeft(700,1);
           break;
        }
    }
    public void red2(){
        waitForStart();
        while (opModeIsActive()){
            // Move to Skystones
            sidewaysWithProportionalDrive(4000, -1);
            forwardWithProportionalDrive(65, 1);

            // intake a Skystone
            sideLeft(1000,.75);
            intake(1);
            forward(30, .75);
            forward(30, -.75);
            forward(0,0);
            sideRight(1050, .75);
            forward(60,.75);
            forward(70, -.75);
            intake(0);

            //Move to Foundation
            sidewaysWithProportionalDrive(5300,1);
            forwardWithProportionalDrive(40, 1);

            // Get the foundation
            sideRight(30, 1);
            clamp1.setPosition(0);
            clamp2.setPosition(1);

            // Move the foundation
            forwardWithProportionalDrive(300, -1);
            clamp1.setPosition(.75);
            clamp2.setPosition(0);

            // Park
            sideLeft(700,1);
            break;
        }
    }
    public void blue1(){
        waitForStart();
        while (opModeIsActive()) {
            forward(65, .5);
            double encoderVal = frontLeft.getCurrentPosition();

            // intake a Skystone
            sideLeft(1000, .75);
            intake(1);
            forward(30, .75);
            forward(30, -.75);
            forward(0, 0);
            sideRight(1050, .75);
            forward(60, .75);
            forward(70, -.75);
            intake(0);

            //Move to Foundation
            sidewaysWithProportionalDrive(5300, 1);
            forwardWithProportionalDrive(40, 1);

            // Get the foundation
            sideRight(30, 1);
            clamp1.setPosition(0);
            clamp2.setPosition(1);

            // Move the foundation
            forwardWithProportionalDrive(300, -1);
            clamp1.setPosition(.75);
            clamp2.setPosition(0);

            // Park
            sideLeft(700, 1);
            break;
        }
    }
    public void blue2(){
        waitForStart();
        while (opModeIsActive()){
            // Move to Skystones
            sidewaysWithProportionalDrive(2000, 1);
            forwardWithProportionalDrive(65, 1);

            // intake a Skystone
            sideRight(1000,.75);
            intake(1);
            forward(30, .75);
            forward(30, -.75);
            forward(0,0);
            sideLeft(1050, .75);
            forward(60,.75);
            forward(70, -.75);
            intake(0);

            //Move to Foundation
            sidewaysWithProportionalDrive(5300,1);
            forwardWithProportionalDrive(40, 1);

            // Get the foundation
            sideLeft(30, 1);
            clamp1.setPosition(0);
            clamp2.setPosition(1);

            // Move the foundation
            forwardWithProportionalDrive(300, -1);
            clamp1.setPosition(.75);
            clamp2.setPosition(0);

            // Park
            sideRight(700,1);
            break;
        }
    }
    public void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

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
    public void intake(double power){
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
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
            currentDistance = frontLeft.getCurrentPosition();
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
