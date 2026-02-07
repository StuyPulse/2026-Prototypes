package com.stuypulse.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sim extends SubsystemBase {
    public static final Sim instance;
    public static int randomAngle = 0;

    static {
        instance = new SimImpl();
    }

    public static Sim getInstance() {
        return instance;
    }

    public enum State { 
        TOTARGET(() -> randomAngle, 1), //Speed is multiplier on voltage, either on or off
        STOP(() -> 0, 0); //speed values are good as they are imo
        
        Supplier<Integer> angle;
        int speed;

        private State(Supplier<Integer> angle, int speed) {
            this.angle = angle;
            this.speed = speed;
        }

        public Supplier<Integer> getAngle() {
            return angle;
        }

        public int getSpeed() {
            return speed;
        }
    }

    private State state;

    public Sim() {
        state = State.STOP;
    }

    public State getState() {
        return state;
    }

    public void setState(State state) {
        this.state = state;
    }

    public void setRandomAngle(int angle) {
        this.randomAngle = angle;
    }

    @Override
    public void periodic() {
        //TODO: add stuff here? -> not necessary ig?
    }
    
}
