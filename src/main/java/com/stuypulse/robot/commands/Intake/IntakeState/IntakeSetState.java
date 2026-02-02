package com.stuypulse.robot.commands.Intake.IntakeState;

import com.stuypulse.robot.subsystems.Intake;
import com.stuypulse.robot.subsystems.Intake.IntakeState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeSetState extends InstantCommand {
    private final Intake intake;
    private IntakeState state;

    public IntakeSetState(IntakeState state) {
        intake = Intake.getInstance();
        this.state = state;
        
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeState(state);
    }
}
