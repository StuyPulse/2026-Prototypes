package com.stuypulse.robot.commands.Intake.IntakePosition;

import com.stuypulse.robot.subsystems.Intake;

public class IntakeSetPositionStop extends IntakeSetPosition {
    public IntakeSetPositionStop() {
        super(Intake.IntakePosition.STOP);
    }
}
