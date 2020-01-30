package com.team1678.frc2020.subsystems;

import java.util.stream.Stream;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj.Spark;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ColorMatch;
import com.revrobotics.ControlType;

// Control panel manipulator
public class Roller extends Subsystem {
    // Constants
    public static double kRotateVoltage = 3.0; // Positive value rotates the control panel counter-clockwise

    // Motors, solenoids and sensors
    public I2C.Port i2cPort;
    public ColorSensorV3 mColorSensor;
    private final Spark mRollerMotor;
    public Solenoid mPopoutSolenoid;

    // Color sensing
    private final ColorMatch mColorMatcher = new ColorMatch();
    ColorMatchResult mMatch;

    private final Color kBlueTarget = ColorMatch.makeColor(0.134, 0.432, 0.434);
    private final Color kGreenTarget = ColorMatch.makeColor(0.178, 0.571, 0.251);
    private final Color kRedTarget = ColorMatch.makeColor(0.485, 0.364, 0.150);
    private final Color kYellowTarget = ColorMatch.makeColor(0.314, 0.553, 0.120);

    private int mColorCounter = 0;

    private Color mInitialColor;
    private Color mOneColorAgo;
    private Color mTwoColorsAgo;

    private Color mColorPositionTarget;
    private Color mSlowDownTarget;

    // Game data
    private String gameData;

    // State management
    public enum WantedAction {
        NONE, ACHIEVE_ROTATION_CONTROL, ACHIEVE_POSITION_CONTROL
    }

    private enum State {
        IDLE, ACHIEVING_ROTATION_CONTROL, ACHIEVING_POSITION_CONTROL
    }

    private State mState = State.IDLE;

    // General management
    private static Roller mInstance;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private Roller() {
        mRollerMotor = new Spark(Constants.kRollerId);
        mPopoutSolenoid = Constants.makeSolenoidForId(Constants.kRollerSolenoid);

        i2cPort = I2C.Port.kOnboard;
        mColorSensor = new ColorSensorV3(i2cPort);

        mColorMatcher.addColorMatch(kBlueTarget);
        mColorMatcher.addColorMatch(kGreenTarget);
        mColorMatcher.addColorMatch(kRedTarget);
        mColorMatcher.addColorMatch(kYellowTarget);   
    }

    public synchronized static Roller getInstance() {
        if (mInstance == null) {
            mInstance = new Roller();
        }

        return mInstance;
    }

    // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.detected_color = mColorSensor.getColor();
        gameData = DriverStation.getInstance().getGameSpecificMessage();

        if(gameData.length() > 0) {
            // Accounts for the FMS detecting the color two wedges down
            switch (gameData.charAt(0)) {
                case 'B' :
                    mColorPositionTarget = kRedTarget;
                    mSlowDownTarget = kGreenTarget;
                    break;
                case 'G' :
                    mColorPositionTarget = kYellowTarget;
                    mSlowDownTarget = kRedTarget;
                    break;
                case 'R' :
                    mColorPositionTarget = kBlueTarget;
                    mSlowDownTarget = kYellowTarget;
                    break;
                case 'Y' :
                    mColorPositionTarget = kGreenTarget;
                    mSlowDownTarget = kBlueTarget;
                    break;
                default :
                    System.out.println("Invalid color from FMS!");
                    break;
            }
        } else {
            // No data has been recieved
        }
    }

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public synchronized void writePeriodicOutputs() {
        mRollerMotor.set(mPeriodicIO.roller_demand / 12.0);
        mPopoutSolenoid.set(mPeriodicIO.pop_out_solenoid);
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.IDLE;
            } 

            @Override 
            public void onLoop(double timestamp) {
                synchronized (Roller.this) {
                    runStateMachine(true);
                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
            }

        });
    }

    // TODO: Only perform actions if 'modifyOutputs' is true
    public void runStateMachine(boolean modifyOutputs) {
        switch(mState) {
            case IDLE:
                mPeriodicIO.pop_out_solenoid = false;
                mPeriodicIO.roller_demand = 0.0;
                break;
            case ACHIEVING_ROTATION_CONTROL:
                mPeriodicIO.pop_out_solenoid = true;
                mMatch = mColorMatcher.matchClosestColor(mPeriodicIO.detected_color);

                if (mColorCounter < 7) {
                    mPeriodicIO.roller_demand = kRotateVoltage;
            
                    if (mMatch.color != mOneColorAgo) {
                      if (mMatch.color == mInitialColor) {
                        // Necessary to avoid color confusion between red/green and blue/yellow
                        // Haven't tested this logic yet
                        if (!(mInitialColor == kYellowTarget && mMatch.color == kYellowTarget && mOneColorAgo == kGreenTarget && mTwoColorsAgo == kBlueTarget)
                        || !(mInitialColor == kGreenTarget && mMatch.color == kGreenTarget && mOneColorAgo == kYellowTarget && mTwoColorsAgo == kRedTarget)) {
                            mColorCounter++;
                        }
                      }
                    }
                  }
            
                  if (mColorCounter >= 7) {
                    mColorCounter = 0;
                    setState(WantedAction.NONE);
                  }

                  if (mOneColorAgo != mMatch.color) {
                    mTwoColorsAgo = mOneColorAgo;
                    mOneColorAgo = mMatch.color;
                  }

                break;
            case ACHIEVING_POSITION_CONTROL:
                mMatch = mColorMatcher.matchClosestColor(mPeriodicIO.detected_color);

                if (gameData.length() > 0) {
                    mPeriodicIO.pop_out_solenoid = true;

                    if (mMatch.color != mColorPositionTarget) {
                        mPeriodicIO.roller_demand = kRotateVoltage;
            
                        if (mMatch.color == mSlowDownTarget) {
                            mPeriodicIO.roller_demand = kRotateVoltage - 2;
                        }
                    } else {
                        mColorCounter = 0;
                        setState(WantedAction.NONE);
                    }
                }

                break;
            default:
                System.out.println("Invalid roller goal!");
                break;
        }
    }

    public void zeroSensors() {}

    @Override
    public void stop() {
        mPeriodicIO.roller_demand = 0.0;
        mPeriodicIO.pop_out_solenoid = false;
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putNumber("Red", mPeriodicIO.detected_color.red);
        SmartDashboard.putNumber("Green", mPeriodicIO.detected_color.green);
        SmartDashboard.putNumber("Blue", mPeriodicIO.detected_color.blue);
    }

    public void setState(WantedAction action) {
        switch(action) {
            case NONE:
                mState = State.IDLE;
                break;
            case ACHIEVE_ROTATION_CONTROL:
                // This is frowned upon by Java developers, so I'm willing to change it
                mInitialColor = mOneColorAgo = mTwoColorsAgo = mMatch.color;
                mState = State.ACHIEVING_ROTATION_CONTROL;
                break;
            case ACHIEVE_POSITION_CONTROL:
                mState = State.ACHIEVING_POSITION_CONTROL;
                break;
            default:
                System.out.println("Invalid roller action!");
                break;
        }
    }

    public static class PeriodicIO {
        // INPUTS
        public Color detected_color;

        // OUTPUTS
        public double roller_demand;
        public boolean pop_out_solenoid;
    }
}
