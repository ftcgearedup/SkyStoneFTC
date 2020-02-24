package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

@Autonomous(name = "SimpleAuto", group = "Autonomous")
public class SimpleAuto extends SkystoneRobot {

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
        initDriveHardware();
        initAttachCode();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.loggingEnabled = false;
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(Gyroscope.class, "imu");
    }

    public void chooseprogram() {
        telemetry.addData("ChoosePosition", "RED1-A, RED2-B, Blue1-X, Blue2-Y");
        while (!isStarted()) {
            if (gamepad1.a) {
                telemetry.addData("Program", "RightPark");
                telemetry.update();
                waitForStart();
                while (opModeIsActive()){
                    sideRight(1800, .7 );
                    break;
                }
                telemetry.update();
                break;
            } else if (gamepad1.b) {
                telemetry.addData("RobotPosition", "RightForwardPark");
                telemetry.update();
                waitForStart();
                while (opModeIsActive()){
                    forward(50, .7);
                    stopMotors();
                    sideRight(1800, .7 );
                    break;
                }
                telemetry.update();
                break;
            } else if (gamepad1.x) {
                telemetry.addData("RobotPosition", "LeftPark");
                telemetry.update();
                waitForStart();
                while (opModeIsActive()){
                    sideLeft(1800, .7 );
                    stopMotors();
                }
                telemetry.update();
                break;
            } else if (gamepad1.y) {
                telemetry.addData("RobotPosition", "LeftForwardPark");
                telemetry.update();
                waitForStart();
                while (opModeIsActive()){
                    forward(50, .7);
                    stopMotors();
                    sideRight(1800, .7 );
                    stopMotors();

                }
                telemetry.update();
                break;
            }else {
                telemetry.addData("waiting", "help");
                telemetry.update();
            }
        }
    }

    @Override
    public void runOpMode() {
        initHardware();
        chooseprogram();

    }

}
