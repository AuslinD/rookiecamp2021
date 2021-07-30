package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class MazeAuto extends LinearOpMode {

    ElapsedTime runTime = new ElapsedTime();

    DcMotor left;
    DcMotor right;
    // Our sensors, motors, and other devices go here, along with other long term state
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;

    public void moveTime(int speed, int time){
        left.setPower(speed);
        right.setPower(speed);
        sleep(time);
    }
    public void moveEncoder(int speed, int distance){
        while(distance > left.getCurrentPosition()){
            left.setPower(speed);
            right.setPower(speed);
        }
    }
    public void turnRight(int speed, int time){
        left.setPower(speed);
        right.setPower(-speed);
        sleep(time);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void proportionalTurn(double speed, int target){
        double currAngle = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));// maybe second angle or thirdAngle
        double error = target - currAngle;
        double kP = .6 / 90;
        double kI = 0.1;
        double kD = 0.02 / 90;
        double prevError = 0;
        double prevTime = 0;
        double deltaTime;
        double deltaError;
        double P;
        double I;
        double D;
        while(target != currAngle){


            currAngle = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            error = target - currAngle;
            deltaTime = runTime.time() - prevTime;
            deltaError = error - prevError;
            P = error * kP;
            I = deltaTime * error * kI;
            D = deltaError / deltaTime;
            D *= kD;


            prevTime = runTime.time();
            prevError = error;

            left.setPower(P + I - D);
            right.setPower(-(P + I - D));

        }
    }


    @Override
    public void runOpMode() throws InterruptedException {


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        left = hardwareMap.dcMotor.get("l");
        right = hardwareMap.dcMotor.get("r");

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        waitForStart();
        runTime.reset();
        moveEncoder(1, 500);//change this?
        proportionalTurn(-1, 90);
        moveEncoder(1, 10000);//change this
        proportionalTurn(1, 0);
        moveEncoder(1, 1000);//change this


        //go forward some ticks
        // turn 90 degrees (maybe PID?)
        // go forward some ticks
        // turn 90 degrees (back to original orientation)
        //go forward some ticks

    }
}
