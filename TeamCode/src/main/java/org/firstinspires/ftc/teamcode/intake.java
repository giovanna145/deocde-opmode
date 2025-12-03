package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class intake extends OpMode {

    // Declaração dos motores que serão usados no robô
    DcMotor left, right, intake;

    // Método chamado uma única vez quando o modo é iniciado
    public void init(){
        // Faz o mapeamento dos motores com os nomes configurados no Driver Station
        left = hardwareMap.get(DcMotor.class, "motor1");
        right = hardwareMap.get(DcMotor.class, "motor2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER) ;
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Inverte a direção do motor esquerdo
        // Isso é necessário porque motores montados em lados opostos geralmente giram em sentidos contrários
        left.setDirection(DcMotorSimple.Direction.REVERSE);

        // Envia uma mensagem para o Driver Station informando que o robô foi inicializado
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
    }

    // Método chamado repetidamente durante o modo TeleOp
    // É aqui que o controle do robô acontece
    public void loop(){
        double SPEED_MULT = 5.0;

        double forward = gamepad1.left_stick_y * SPEED_MULT;
        double horizontal = gamepad1.left_stick_x * SPEED_MULT;
        double sideways = gamepad1.left_stick_x * SPEED_MULT;



        //double forward = gamepad1.right_stick_y;

        // Lê o valor do eixo X do joystick direito (rotação do robô)

        // Lê o valor do gatilho direito (controle do lançador)
        //double power = gamepad1.left_trigger;

        // A soma e subtração permitem que o robô gire e ande ao mesmo tempo
        double turn = 0;
        left.setPower((forward - turn) * SPEED_MULT);
        right.setPower((forward + turn) * SPEED_MULT);

        // Define a potência do motor do lançador (shooter); sei

        double power = -1;

        if (gamepad1.a){
            intake.setPower(power);
        }
        if (gamepad1.b){
            intake.setPower(-power);

        }
    }
}
