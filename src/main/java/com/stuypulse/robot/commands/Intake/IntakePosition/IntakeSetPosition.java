package com.stuypulse.robot.commands.Intake.IntakePosition;

import com.stuypulse.robot.subsystems.Intake;
import com.stuypulse.robot.subsystems.Intake.IntakePosition;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeSetPosition extends InstantCommand{
    private final Intake intake;
    private IntakePosition position;

    public IntakeSetPosition(IntakePosition position) {
        intake = Intake.getInstance();
        this.position = position;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakePosition(position);
    }
}
