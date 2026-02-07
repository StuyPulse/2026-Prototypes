package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Sim;
import com.stuypulse.robot.subsystems.Sim.State;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetState extends InstantCommand {
    public final Sim sim;
    public State state;

    public SetState(State state) {
        sim = Sim.getInstance();

        this.state = state;

        addRequirements(sim);
    }

    @Override
    public void execute() {
        sim.setState(this.state);
    }
}
