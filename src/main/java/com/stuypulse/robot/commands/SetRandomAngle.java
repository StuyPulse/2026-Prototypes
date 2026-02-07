package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Sim;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetRandomAngle extends InstantCommand {
    public final Sim sim;

    public SetRandomAngle() {
        sim = Sim.getInstance();

        addRequirements(sim);
    }

    @Override
    public void execute() {
        sim.setRandomAngle((int) (Math.random() * 360));
    }
    
}
