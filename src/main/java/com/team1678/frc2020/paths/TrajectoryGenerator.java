package com.team1678.frc2020.paths;

import com.team1678.frc2020.planners.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryUtil;
import com.team254.lib.trajectory.timing.CentripetalAccelerationConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 130.0;
    private static final double kMaxAccel = 100.0;
    private static final double kMaxCentripetalAccel = 110.0;
    private static final double kMaxVoltage = 9.0;

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(boolean reversed,
            final List<Pose2d> waypoints, final List<TimingConstraint<Pose2dWithCurvature>> constraints, double max_vel, // inches/s
            double max_accel, // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(boolean reversed,
            final List<Pose2d> waypoints, final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel, // inches/s
            double end_vel, // inches/s
            double max_vel, // inches/s
            double max_accel, // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel,
                max_accel, max_voltage);
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle
    // of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the left.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x
    // axis for LEFT)
    //
    // NAMING
    // poses are positions that the robot ends or starts at during it's trajectory
    // waypoints are positions that the robot follows in between poses
    // turned are positions after a point turn

    public static final Pose2d kTestStartPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kTestFarPose = new Pose2d(50.0, 0.0, Rotation2d.fromDegrees(0.0));

    public static final Pose2d kStartingPose = new Pose2d(140.0, -140.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kFirstIntakePose = new Pose2d(230.0, -140.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kTrenchExitWaypoint = new Pose2d(200.0, -110.0, Rotation2d.fromDegrees(-80.0));
    public static final Pose2d kShotPose = new Pose2d(180.0, 75.0, Rotation2d.fromDegrees(-90.0));
    public static final Pose2d kShotPoseTurned = new Pose2d(180.0, 75.0, Rotation2d.fromDegrees(0.0));

    public static final Pose2d kRVEnterWaypoint = new Pose2d(220.0, 75.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kSecondIntakePose = new Pose2d(245.0, 55.0, Rotation2d.fromDegrees(-70.0));
    public static final Pose2d kPillarWaypoint = new Pose2d(240.0, 20.0, Rotation2d.fromDegrees(200.0));
    public static final Pose2d kThirdIntakePose = new Pose2d(240.0, -20.0, Rotation2d.fromDegrees(-55.0));
    // public static final Pose2d kShotPoseWaypoint = new Pose2d(240.0, -40.0, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d kThirdIntakePoseTurned = new Pose2d(240.0, -20.0, Rotation2d.fromDegrees(0.0));

    public static final Pose2d kShotPoseWaypoint2 = new Pose2d(220.0, -20.0, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d kSecondShotPose = new Pose2d(180.0, 75.0, Rotation2d.fromDegrees(-65.0));

    public static final Pose2d kTestPoint1 = new Pose2d(265, 80, Rotation2d.fromDegrees(-110));

    public class TrajectorySet {

        public final Trajectory<TimedState<Pose2dWithCurvature>> testPath;
        public final Trajectory<TimedState<Pose2dWithCurvature>> testPathReversed;

        public final Trajectory<TimedState<Pose2dWithCurvature>> startToSteal;
        public final Trajectory<TimedState<Pose2dWithCurvature>> stealToFirstShot;
        public final Trajectory<TimedState<Pose2dWithCurvature>> intakeCells;
        public final Trajectory<TimedState<Pose2dWithCurvature>> intakeToSecondShot;

        private TrajectorySet() {
            testPath = getTestPath();
            testPathReversed = getTestPathReversed();

            startToSteal = getStartToSteal();
            stealToFirstShot = getStealToFirstShot();
            intakeCells = getIntakeCells();
            intakeToSecondShot = getIntakeToSecondShot();
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kTestStartPose);
            waypoints.add(kTestFarPose);
            return generateTrajectory(false, waypoints,
                    Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)), kMaxVelocity, kMaxAccel,
                    kMaxVoltage);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPathReversed() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kTestFarPose);
            waypoints.add(kTestStartPose);
            return generateTrajectory(true, waypoints,
                    Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)), kMaxVelocity, kMaxAccel,
                    kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToSteal() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kStartingPose);
            waypoints.add(kFirstIntakePose);
            return generateTrajectory(false, waypoints,
                    Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)), kMaxVelocity, kMaxAccel,
                    kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStealToFirstShot() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kFirstIntakePose);
            waypoints.add(kTrenchExitWaypoint);
            waypoints.add(kShotPose);
            return generateTrajectory(true, waypoints,
                    Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)), kMaxVelocity, kMaxAccel,
                    kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getIntakeCells() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kShotPoseTurned);
            waypoints.add(kRVEnterWaypoint);
            waypoints.add(kSecondIntakePose);
            waypoints.add(kPillarWaypoint);
            waypoints.add(kThirdIntakePose);
        //    waypoints.add(kShotPoseWaypoint);
            return generateTrajectory(false, waypoints,
                    Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)), kMaxVelocity, kMaxAccel,
                    kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getIntakeToSecondShot() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kThirdIntakePoseTurned);
     // waypoints.add(kShotPoseWaypoint2);
            waypoints.add(kSecondShotPose);
            return generateTrajectory(true, waypoints,
                    Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)), kMaxVelocity, kMaxAccel,
                    kMaxVoltage);
        }
    }
}
