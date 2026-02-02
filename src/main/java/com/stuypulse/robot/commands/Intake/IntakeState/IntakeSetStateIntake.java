package com.stuypulse.robot.commands.Intake.IntakeState;

import com.stuypulse.robot.subsystems.Intake;

public class IntakeSetStateIntake extends IntakeSetState {
    public IntakeSetStateIntake() {
        super(Intake.IntakeState.INTAKE);
    }
}
