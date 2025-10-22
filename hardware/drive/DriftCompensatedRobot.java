package org.firstinspires.ftc.teamcode.hardware.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.teamcode.hardware.drive.DriftCompensatedDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.Scoring;

public class DriftCompensatedRobot extends Mechanism {
    private Scoring scoring = new Scoring(opMode);
    private DriftCompensatedDrivetrain dt = new DriftCompensatedDrivetrain(opMode);

    public DriftCompensatedRobot(LinearOpMode opMode) { this.opMode = opMode; }

    public void init(HardwareMap hwMap) {
        dt.init(hwMap);
        scoring.init(hwMap);
    }

    public void loop(Gamepad gamepad) {
        dt.loop(gamepad);
        scoring.loop(gamepad);
    }
}
