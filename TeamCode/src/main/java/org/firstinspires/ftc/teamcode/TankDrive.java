package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TankDrive extends OpMode {
    DcMotor right, left, shooter, intake;

    public void init() {
        right = hardwareMap.get(DcMotor.class, "motor2");
        left = hardwareMap.get(DcMotor.class, "motor1");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("status", "initialized");
        telemetry.update();
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {
        double forward = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;


        if (gamepad1.a) {
            shooter.setPower(0.7);
        }
        if (gamepad1.b) {
            shooter.setPower(-1);

        }
        if (gamepad1.x) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
        if (gamepad1.y) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }
            right.setPower(forward + turn);
            left.setPower(forward - turn);
        }
    }
