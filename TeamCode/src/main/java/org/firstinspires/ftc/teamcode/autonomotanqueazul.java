package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "autonomotanqueazul", group = "Autonomous")
public class autonomotanqueazul extends LinearOpMode {

    private DcMotor leftMotor, rightMotor, shooter, intake;
    private IMU imu;

    static final double TICKS_PER_CM = 17.82; // Ajuste conforme o diâmetro da roda e a redução
    static final double TURN_TOLERANCE = 2.0; // graus de margem de erro

    @Override
    public void runOpMode() {
        // Inicialização do hardware
        leftMotor = hardwareMap.get(DcMotor.class, "motor1");
        rightMotor = hardwareMap.get(DcMotor.class, "motor2");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(myIMUparameters);

        // Configuração de direção (ajuste se o robô andar para o lado errado)
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();

        waitForStart();

        if (opModeIsActive()) {
            moveCM(100, 1.0); // anda 50 cm pra frente
            turnToAngle(30); // gira 180 graus
            moveCM(100, 0.5); // anda 30 cm pra trás
            sleep(15000);
            shooterVai(1);
            intakeVai(1);

        }
    }

    private void resetEncoders() {
        for (DcMotor m : new DcMotor[]{leftMotor, rightMotor}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void moveCM(double cm, double power) {
        int ticks = (int) (cm * TICKS_PER_CM);

        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + ticks);
        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + ticks);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(Math.abs(power));
        rightMotor.setPower(Math.abs(power));
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        while (opModeIsActive() && (leftMotor.isBusy() || rightMotor.isBusy())) {
            telemetry.addLine("Movendo para frente/trás...");
            telemetry.update();
        }

        stopAllMotors();
    }

    private void turnToAngle(double targetAngle) {
        double currentYaw = getYaw();
        double error = angleDiff(targetAngle, currentYaw);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && Math.abs(error) > TURN_TOLERANCE) {
            double power = 0.3 * Math.signum(error);

            // Gira no próprio eixo
            leftMotor.setPower(-power);
            rightMotor.setPower(power);

            currentYaw = getYaw();
            error = angleDiff(targetAngle, currentYaw);

            telemetry.addData("Yaw", currentYaw);
            telemetry.addData("Target", targetAngle);
            telemetry.update();
        }

        stopAllMotors();
    }

    private double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double angleDiff(double target, double current) {
        double error = target - current;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    private void stopAllMotors() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    public void shooterVai(double potencia){
        shooter.setPower(potencia);
    }
    public void intakeVai(double power){
        intake.setPower(power);
    }
}

