package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaSkyStoneNavigationWebcam;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

@Autonomous(name = "MainAuto", group = "Autonomous")
public class AutoMain extends SkystoneRobot {

    private Gyroscope imu;
    private AngularVelocity angleV;
    private ColorSensor colorSensor;
    private double degree = 0;

    public void initHardware() {

        initDriveHardware();
        initAttachCode();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = false;
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(Gyroscope.class, "imu");

        colorSensor = hardwareMap.colorSensor.get("color");

    }

    public void runOpMode(){
        initHardware();
        setZeroPowBehv(DcMotor.ZeroPowerBehavior.BRAKE);
        setDirection();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chooseprogram();
           waitForStart();
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
            findSkyStoneAlpha();
           // transport a Skystone
            sideLeft(15,.5);
            forward(35,-.5);
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
}
