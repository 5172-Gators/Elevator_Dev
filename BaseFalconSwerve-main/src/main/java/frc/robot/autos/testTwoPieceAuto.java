package frc.robot.autos;

// Constants
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Position;

// Commands
import frc.robot.commands.SetAllPositions;
import frc.robot.commands.Elevator.ElevatorSetPositionHigh;

// Subsystems
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.WristSub;
import frc.robot.subsystems.ShoulderSub;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.IntakeSub;


import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class testTwoPieceAuto extends SequentialCommandGroup {

    private final ElevatorSub s_Elevator = new ElevatorSub();
    private final WristSub s_Wrist = new WristSub();
    private final ShoulderSub s_Shoulder = new ShoulderSub();
    private final IntakeSub s_Intake = new IntakeSub();

    public testTwoPieceAuto(Swerve s_Swerve, ShoulderSub s_Shoulder, ElevatorSub s_Elevator, WristSub s_Wrist, IntakeSub s_Intake){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory trajectory1 =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // continue moving forward
                null,
                // End 3 meters straight ahead of where we started
                new Pose2d(-3, 0, new Rotation2d(180)),
                
                config);

        Trajectory trajectory2 = 
            TrajectoryGenerator.generateTrajectory(
                // turn around
                new Pose2d(-3, 0, new Rotation2d(180)),
                // no interior waypoints
                null,
                // return to start + get ready to place a piece
                new Pose2d(0,0, new Rotation2d(0)), 
                config);

        Trajectory trajectory3 = 
            TrajectoryGenerator.generateTrajectory(
                // turn around
                new Pose2d(0, 0, new Rotation2d(180)),
                // no interior waypoints
                null,
                // stay turned around
                null, 
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand1 =
            new SwerveControllerCommand(
                trajectory1,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand swerveControllerCommand2 =
            new SwerveControllerCommand(
                trajectory2,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        
        SwerveControllerCommand swerveControllerCommand3 =
            new SwerveControllerCommand(
                trajectory3,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(

            // place cone loaded into robot
            new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.HIGH, () -> GamePiece.CONE),
            new InstantCommand(() -> s_Intake.setMotor(-0.75)),
            new WaitCommand(0.5),
            new InstantCommand(() -> s_Intake.stopIntake()),

            // start trajectory 1
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectory1.getInitialPose())),
            swerveControllerCommand1,

            // stow elevator + intake
            new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.STOWED, () -> GamePiece.CUBE),

            // drive across field
            new WaitCommand(3),

            // intake cube
            new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.CUBEINTAKE, () -> GamePiece.CUBE),
            new InstantCommand(() -> s_Intake.setMotor(0.75)),
            new WaitCommand(0.5),
            new InstantCommand(() -> s_Intake.stopIntake()),

            // start trajectory 2
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectory2.getInitialPose())),
            swerveControllerCommand2,

            // place cube
            new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.HIGH, () -> GamePiece.CUBE),
            new InstantCommand(() -> s_Intake.setMotor(0.75)),
            new WaitCommand(0.5),
            new InstantCommand(() -> s_Intake.stopIntake()),

            // stow elevator
            new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.STOWED, () -> GamePiece.CUBE),

            // turn around/start trajectory 3
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectory3.getInitialPose())),
            swerveControllerCommand3,

            // reset gyro
            new InstantCommand(() -> s_Swerve.zeroGyro())

        );
    }
}
