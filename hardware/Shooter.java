package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.util.NominalVoltage;


import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Shooter extends Mechanism {
    private PIDController controller;

    private final DcMotorEx[] motors = new DcMotorEx[2];

    private VoltageSensor voltage;

    public static double INTAKE_POS = 0.89; // 0.18
    public static double AUTO_INTAKE_POS = 0.18;
    public static double AUTO_SCORE_POS = 0.27;
    public static double AUTO_CONE_STACK_POS = 0.57;
    public static double SCORE_POS = 0.27; //0.76
    public static double GROUND_SCORE_POS = 0.82;

    public Shooter(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        voltage = hwMap.voltageSensor.iterator().next();
        motors[0] = hwMap.get(DcMotorEx.class, "shooterLeftMotor");
        motors[1] = hwMap.get(DcMotorEx.class, "shooterRightMotor");

        motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motors[0].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // might be wrong RunMode
        motors[1].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motors[0].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[1].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motors[0].setDirection(DcMotorEx.Direction.REVERSE);
        motors[1].setDirection(DcMotorEx.Direction.REVERSE);

    }

    public void intakePos() {
        leftShoot.setPosition(INTAKE_POS);
        rightArm.setPosition(INTAKE_POS);
    }

    public void autoIntakePos() {
        leftShoot.setPosition(AUTO_INTAKE_POS);
        rightArm.setPosition(AUTO_INTAKE_POS);
    }

    public void autoScorePos() {
        leftShoot.setPosition(AUTO_SCORE_POS);
        rightArm.setPosition(AUTO_SCORE_POS);
    }

    public void autoConeStackPos() {
        leftArm.setPosition(AUTO_CONE_STACK_POS);
        rightArm.setPosition(AUTO_CONE_STACK_POS);
    }

    public void scorePos() {
        leftArm.setPosition(SCORE_POS);
        rightArm.setPosition(SCORE_POS);
    }

    public void groundScorePos() {
        leftArm.setPosition(GROUND_SCORE_POS);
        rightArm.setPosition(GROUND_SCORE_POS);
    }

    public void moveToPos(double pos) {
        leftArm.setPosition(pos);
        rightArm.setPosition(pos);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            scorePos();
        } else if (gamepad.dpad_down) {
            intakePos();
        } else if (gamepad.dpad_right) {
            autoScorePos();
        } else if (gamepad.dpad_left) {
            autoConeStackPos();
        }
    }

}
