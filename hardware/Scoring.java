package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.command.CommandSequence;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.util.Mechanism;

@Config
public class Scoring extends Mechanism {
    private Drivetrain drivetrain = new Drivetrain(opMode); // OPMODE NULL HERE
    private Intake intake = new Intake(opMode);
    private Transfer transfer = new Transfer(opMode);
    public Shooter shooter = new Shooter(opMode);

    private State state = State.EMPTY;

    private enum State {
        FULL,
        SHOOTING,
        EMPTY,
    }

    public static double SHOT_WAIT = 0.1;

    private boolean climbPressed = false;
    private boolean frontClicked = false;
    private boolean dpadClicked = false;
    private boolean rightStickClicked = false;
    private boolean shortIntake = false;
    private boolean scoreClicked = false;

    public Scoring(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    private Command intakeCommand = () -> intake.intake();
    private Command outakeCommand = () -> intake.outtake();
    private Command stopIntake = () -> intake.stop();


    @Override
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        intake.init(hwMap);
        shooter.init(hwMap);
        transfer.init(hwMap);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("state", state);
        telemetry.addData("scoreClicked", scoreClicked);
        drivetrain.telemetry(telemetry);
        intake.telemetry(telemetry);
        shooter.telemetry(telemetry);
        transfer.telemetry(telemetry);
    }
}
