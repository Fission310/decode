package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.teamcode.hardware.drivebase.FieldCentricDrivetrain;

public class FieldCentricRobot extends Mechanism {
    private Scoring scoring = new Scoring(opMode);
    private FieldCentricDrivetrain dt = new FieldCentricDrivetrain(opMode);

    public FieldCentricRobot(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        dt.init(hwMap);
        scoring.init(hwMap);
    }

    @Override
    public void loop(Gamepad gamepad) {
        dt.loop(gamepad);
        scoring.loop(gamepad);
    }
}
