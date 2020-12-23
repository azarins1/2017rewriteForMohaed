//GearIntake.java is for intaking the gears

package com.team1678.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.team1678.frc2020.subsystems.Canifier;
import com.team1678.frc2020.subsystems.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import com.team254.lib.drivers.MotorChecker;
import com.team254.lib.drivers.TalonSRXFactory;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.team254.lib.drivers.BaseTalonChecker;
import com.team254.lib.util.ReflectingCSVWriter;

import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.Solenoid;

public class GearIntake extends Subsystem {

    // Constants, make sure that you have public static before each of them
    public static double kIntakeVoltage = 12.0;
    public static double kPickupVoltage = 2.5;
    public static double kCarryVoltage = 1.5;
    public static double kScoreVoltage = -12.0;
    public static double kOuttakeVoltage = -4.0;
    public static int kPickupTicks = 300;
    public static double kCurrentThreshold = 60.0;

    private static GearIntake mInstance;

    public enum WantedAction {
        // Wanted actions
        DROP, RISE, SCORE, NONE, OUTTAKE, START_DROPPING_BALLS, STOP_DROPPING_BALLS,
    }

    private enum State {
        // States
        IDLE, INTAKING, DROP_BALL_WITH_GEAR, DROP_BALL_WITHOUT_GEAR, PICKING_UP, CARRYING, SCORING, OUTTAKING
    }

    private State mState = State.IDLE;

    // Any private variables you may need
    private int pickup_timer_ = 0;
    private boolean current_spiked = false;
    private PeriodicIO mPeriodicIO = new PeriodicIO();

    // Any motor, solenoid, sensor you need. These will be referred to as
    // actuators(convert signal into energy) and sensors
    private final TalonSRX mMaster;
    private final Solenoid mSolenoid;

    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private GearIntake() {
        // Set each actuator to their ID's
        mMaster = TalonSRXFactory.createDefaultTalon(Constants.kClimberMasterId);
        mSolenoid =  Constants.makeSolenoidForId(Constants.kDeploySolenoidId);
    }

    public synchronized static GearIntake getInstance() {
        if (mInstance == null) {
            mInstance = new GearIntake();
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
        mState = State.IDLE;
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
                synchronized (GearIntake.this) {
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

    private double voltage = 0;
    private boolean intake_down = false;

    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.demand = percentage;
    }

    public void runStateMachine() {
        switch (mState) {
            // Cases for each state and what the actuators should be at those states
            case IDLE:
                setOpenLoop(0.0);
                mPeriodicIO.voltage = 0.0;
                mPeriodicIO.intake_down = false;
            case INTAKING:
                setOpenLoop(kIntakeVoltage);
                mPeriodicIO.voltage = kIntakeVoltage;
                mPeriodicIO.intake_down = true;
                if (mPeriodicIO.current > kCurrentThreshold) {
                    current_spiked = true;
                    mState = State.PICKING_UP;
                    pickup_timer_ = kPickupTicks;
                    mPeriodicIO.voltage = kPickupVoltage;
                }
                break;
            case DROP_BALL_WITH_GEAR:
                setOpenLoop(2.5);
                mPeriodicIO.voltage = 2.5;
                mPeriodicIO.intake_down = true;
                break;
            case DROP_BALL_WITHOUT_GEAR:
                setOpenLoop(0);
                mPeriodicIO.voltage = 0;
                mPeriodicIO.intake_down = true;
                break;
            case PICKING_UP:
                setOpenLoop(kPickupVoltage);
                mPeriodicIO.voltage = kPickupVoltage;
                mPeriodicIO.intake_down = false;
                if (--pickup_timer_ < 0) {
                    mState = State.CARRYING;
                }
                break;
            case CARRYING:
                setOpenLoop(kCarryVoltage);
                mPeriodicIO.voltage = kCarryVoltage;
                mPeriodicIO.intake_down = false;
                break;
            case SCORING:
                setOpenLoop(kScoreVoltage);
                mPeriodicIO.voltage = kScoreVoltage;
                mPeriodicIO.intake_down = false;
                break;
            case OUTTAKING:
                setOpenLoop(kOuttakeVoltage);
                mPeriodicIO.voltage = kOuttakeVoltage;
                mPeriodicIO.intake_down = true;
                break;
        }
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            // Cases to switch
            case DROP:
                mState = State.INTAKING;
                break;
            case RISE:
                if (mState == State.INTAKING) {
                    mState = State.IDLE;
                }
                break;
            case SCORE:
                if (mState == State.CARRYING || mState == State.IDLE) {
                    mState = State.SCORING;
                }
                break;
            case NONE:
                if (mState == State.SCORING) {
                    mState = State.IDLE;
                }
                break;
            case OUTTAKE:
                mState = State.OUTTAKING;
                break;
            case START_DROPPING_BALLS:
                if (mState == State.PICKING_UP || mState == State.CARRYING) {
                    mState = State.DROP_BALL_WITH_GEAR;
                }
                if (mState == State.INTAKING || mState == State.IDLE) {
                    mState = State.DROP_BALL_WITHOUT_GEAR;
                }
                break;
            case STOP_DROPPING_BALLS:
                if (mState == State.DROP_BALL_WITHOUT_GEAR) {
                    mState = State.IDLE;
                }
                if (mState == State.DROP_BALL_WITH_GEAR) {
                    mState = State.CARRYING;
                }
                break;
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        // Update anything you want to read from sensors or actuators, these are usually
        // in your inputs under periodicIO

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {

        // Use .set on each of your actuators to whatever output you have been setting
        // from periodicIO. This is also a good place to add limits to your code.
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand);
        mSolenoid.set(mPeriodicIO.intake_down);
    }

    @Override
    public boolean checkSystem() {
        // Use CheckMotor function on any motor you have that uses PID, else just return
        // true in this block
        return true;
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/GEARINTAKE/-LOGS.csv", PeriodicIO.class);
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
        public boolean intake_down;
        public double current;

        // OUTPUTS
        public double demand;
        public double solenoidDemand;
    }
}