package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Teleop", group = "Tele")
public class NewTeleop extends OpMode {
    DcMotor left;
    DcMotor right;
    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("l");
        right = hardwareMap.dcMotor.get("r");

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




    }

    @Override
    public void loop() {

        if(Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1){
            double lefty = gamepad1.left_stick_y * gamepad1.left_stick_y;
            double rightx = gamepad1.right_stick_x * gamepad1.right_stick_x;
            double leftPower = lefty - rightx;
            double rightPower = lefty - rightx;
            if(leftPower > 1){
                leftPower = 1;
            }
            if(rightPower > 1){
                rightPower = 1;
            }
            if(gamepad1.right_trigger > 0.1){
                left.setPower(leftPower * 0.35);// may change this later
                right.setPower(rightPower * 0.35);
            }
            else{
                left.setPower(leftPower);
                right.setPower(rightPower);
            }

        }
        else{
            left.setPower(0);
            right.setPower(0);
        }



    }
}
