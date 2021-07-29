package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MazeAuto extends LinearOpMode {
    DcMotor left;
    DcMotor right;
    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.dcMotor.get("l");
        right = hardwareMap.dcMotor.get("r");

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        //go forward some ticks
        // turn 90 degrees (maybe PI?)
        // go forward some ticks
        // turn 90 degrees (back to original orientation)
        //go forward some ticks

    }
}
