package com.stuypulse.robot.commands.Intake;

import com.stuypulse.robot.commands.Intake.IntakePosition.IntakeSetPositionDown;
import com.stuypulse.robot.commands.Intake.IntakePosition.IntakeSetPositionStop;
import com.stuypulse.robot.commands.Intake.IntakeState.IntakeSetStateStow;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SuperStructureStow extends SequentialCommandGroup {
    public SuperStructureStow() {
        addCommands(
            new IntakeSetPositionDown(),
            new IntakeSetStateStow().withTimeout(3), //TODO: update with seconds or alternative.
            new IntakeSetPositionStop()
        );
    }
}
