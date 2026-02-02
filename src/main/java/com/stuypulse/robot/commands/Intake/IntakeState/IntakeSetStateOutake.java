package com.stuypulse.robot.commands.Intake.IntakeState;

import com.stuypulse.robot.subsystems.Intake;

public class IntakeSetStateOutake extends IntakeSetState {
    public IntakeSetStateOutake() {
        super(Intake.IntakeState.OUTTAKE);
    }
}
