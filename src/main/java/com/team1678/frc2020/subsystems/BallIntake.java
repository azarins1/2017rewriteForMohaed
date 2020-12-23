package com.team1678.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.util.ReflectingCSVWriter;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SolenoidBase;

public class BallIntake extends Subsystem {
    // Constants, make sure that you have public static before each of them
    public static double kClimbingVoltage = -12.0;
    public static double kTopVoltage = -2.0;

    public static double kSpinUpVelocity = 0.42;
    public static double kStartClimbingVelocity = 0.35;
    public static double kFinalVelocity = 0.15;

    private static BallIntake mInstance;

    public enum WantedAction {
        // Wanted actions
        NONE, CLIMB
    }

    private enum State {
        // States
        IDLE, INTAKE, INTAKE_SLOW, OUTTAKE
    }

    private State mState = State.IDLE;

    // Any private variables you may need

    private double current_vel;
    private PeriodicIO mPeriodicIO = new PeriodicIO();

    // Any motor, solenoid, sensor you need. These will be referred to as
    // actuators(convert signal into energy) and sensors
    private final TalonSRX mMaster;
    private final Solenoid mSolenoid;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private BallIntake() {
        // Set each actuator to their ID's
        mMaster = TalonSRXFactory.createDefaultTalon(Constants.kClimberMasterId);
        mSolenoid = Constants.makeSolenoidForId(Constants.kDeploySolenoidId);
    }

    public synchronized static BallIntake getInstance() {
        if (mInstance == null) {
            mInstance = new BallIntake();
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
                mState = State.IDLE;
                // startLogging();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (BallIntake.this) {
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
            // IDLE, SPINUP, APPROACHING, CLIMBING, AT_TOP
            // Cases for each state and what the actuators should be at those states
            case IDLE:
                setOpenLoop(0);
                mPeriodicIO.intake_up = false;
                break;
            case INTAKE_SLOW:
                setOpenLoop(6);
                mPeriodicIO.intake_up = false;
                break;
            case OUTTAKE:
                setOpenLoop(-10);
                mPeriodicIO.intake_up = false;
                break;
            case INTAKE:
                setOpenLoop(10);
                mPeriodicIO.intake_up = false;
                break;
        }
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            // Cases to switch
            case NONE:
                mState = State.IDLE;
            case CLIMB:
                mState = State.IDLE;
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
        // runStateMachine(); does it for us
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand);
        mSolenoid.set(mPeriodicIO.intake_up);
    }

    @Override
    public boolean checkSystem() {
        // Use CheckMotor function on any motor you have that uses PID, else just return
        // true in this block
        return true;
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/BallIntake-LOGS.csv", PeriodicIO.class);
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
        public boolean intake_up;
    }
}
