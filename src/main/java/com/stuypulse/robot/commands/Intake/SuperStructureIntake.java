package com.stuypulse.robot.commands.Intake;

import com.stuypulse.robot.commands.Intake.IntakePosition.IntakeSetPositionUp;
import com.stuypulse.robot.commands.Intake.IntakeState.IntakeSetStateIntake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SuperStructureIntake extends SequentialCommandGroup {
    public SuperStructureIntake() {
        addCommands(
            new IntakeSetPositionUp(), //TODO: add timeouts if necessary
            new IntakeSetStateIntake()
        );
    }
}
