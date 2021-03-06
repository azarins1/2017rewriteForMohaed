package com.team1678.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
//import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
//import com.team1678.frc2020.subsystems.Canifier;
//import com.team1678.frc2020.subsystems.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import com.team254.lib.drivers.MotorChecker;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonSRXFactory;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.team254.lib.drivers.BaseTalonChecker;
//import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.ReflectingCSVWriter;
//import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Climber extends Subsystem {

    // Constants, make sure that you have public static before each of them
    public static double kClimbingVoltage = -12.0;
    public static double kTopVoltage = -2.0;

    public static double kSpinUpVelocity = 0.42;
    public static double kStartClimbingVelocity = 0.35;
    public static double kFinalVelocity = 0.15;

    private static Climber mInstance;

    public enum WantedAction {
        // Wanted actions
        NONE, CLIMB
    }

    private enum State {
        // States
        IDLE, SPINUP, APPROACHING, CLIMBING, AT_TOP
    }

    private State mState = State.IDLE;

    // Any private variables you may need

    private double current_vel;
    private PeriodicIO mPeriodicIO = new PeriodicIO();

    // Any motor, solenoid, sensor you need. These will be referred to as
    // actuators(convert signal into energy) and sensors
    private final TalonSRX mMaster;

    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private Climber()
    {
        // Set each actuator to their ID's
        mMaster = TalonSRXFactory.createDefaultTalon(Constants.kClimberMasterId);
    }

    public synchronized static Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        }
        return mInstance;
    }

    @Override
    public synchronized void outputTelemetry() {
        // Anything you want displayed on the SmartDashboard

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    @Override
    public void stop() {
        setOpenLoop(0);
    }

    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.demand = percentage;
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState =  State.IDLE;
                // startLogging();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Climber.this) {
                 // What happens while the robot is on. This is usually a state machine
                 runStateMachine();
				}
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
                stopLogging();
            }
        });
    }

    public void runStateMachine() {
        current_vel = mPeriodicIO.velocity;
        switch (mState) {
            //IDLE, SPINUP, APPROACHING, CLIMBING, AT_TOP
            // Cases for each state and what the actuators should be at those states
            case IDLE:
                setOpenLoop(0);
            case SPINUP:
                setOpenLoop(kClimbingVoltage);
                if (current_vel > kSpinUpVelocity){
                    mState = State.APPROACHING;
                }
            case APPROACHING:
                setOpenLoop(kClimbingVoltage);
                if (current_vel < kStartClimbingVelocity){
                    mState = State.CLIMBING;
                }
            case CLIMBING:
                setOpenLoop(kClimbingVoltage);
                if (current_vel < kFinalVelocity){
                    mState = State.AT_TOP;
                }
            case AT_TOP:
                setOpenLoop(kTopVoltage);
                break;
        }
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            // Cases to switch
            case NONE:
                mState = State.IDLE;
            case CLIMB:
                mState = State.SPINUP;
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        // Update anything you want to read from sensors or actuators, these are usually
        // in your inputs under periodicIO
        mPeriodicIO.position = mMaster.getSelectedSensorPosition(0);
        mPeriodicIO.velocity = mMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.current = mMaster.getSupplyCurrent();
        mPeriodicIO.voltage = mMaster.getMotorOutputVoltage();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // Use .set on each of your actuators to whatever output you have been setting
        // from periodicIO. This is also a good place to add limits to your code.
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand);
    }

    @Override
    public boolean checkSystem() {
        // Use CheckMotor function on any motor you have that uses PID, else just return
        // true in this block
        return true;
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>(
                    "/home/lvuser/CLIMBER-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double voltage;
        public double velocity;
        public double current;
        public double position;

        // OUTPUTS
        public double demand;
    }
}