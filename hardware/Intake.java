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


//import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class Intake extends Mechanism {

    private DcMotor intake;
    private PIDController controller;

    public static double LEFT_POWER = 1;

    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotor.class, "intake");
    }

    public void intake() {
        intake.setPower(LEFT_POWER);
    }

    public void stop() {
        intake.setPower(0);
    }

    public void outtake() {
        intake.setPower(-LEFT_POWER);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.right_trigger > 0) {
            intake();
        } else if (gamepad.left_trigger > 0) {
            outtake();
        } else {
            stop();
        }
    }

}
