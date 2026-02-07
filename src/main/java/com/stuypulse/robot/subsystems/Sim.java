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
        TOTARGET(() -> randomAngle, 1, 3), //Speed is multiplier on voltage, either on or off
        STOP(() -> 0, 0, -3); //speed values are good as they are imo 
        //TODO: stop while debugging was being used to go backwards
        
        Supplier<Integer> angle;
        int speed;
        int voltage;

        private State(Supplier<Integer> angle, int speed, int voltage) {
            this.angle = angle;
            this.speed = speed;
            this.voltage = voltage;
        }

        public Supplier<Integer> getAngle() {
            return angle;
        }

        public int getSpeed() {
            return speed;
        }

        public int getVoltage() {
            return voltage;
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
