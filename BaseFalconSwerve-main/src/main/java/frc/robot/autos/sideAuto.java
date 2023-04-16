package frc.robot.autos;

// Constants
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Position;

// Commands
import frc.robot.commands.SetAllPositions;

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

public class sideAuto extends SequentialCommandGroup {

    private final ElevatorSub s_Elevator = new ElevatorSub();
    private final WristSub s_Wrist = new WristSub();
    private final ShoulderSub s_Shoulder = new ShoulderSub();
    private final IntakeSub s_Intake = new IntakeSub();
    private final Swerve s_Swerve = new Swerve();

    public sideAuto(Swerve s_Swerve){
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
                // Pass through these two interior waypoints, making an 's' curve path
                null,
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-4.1, 0, new Rotation2d(0)),
                config);

        Trajectory backInside =
            TrajectoryGenerator.generateTrajectory(
                // start outside of community
                new Pose2d(-4.1, 0, new Rotation2d(0)),
                // no waypoints
                null,
                // end on charge station
                new Pose2d(-1, 0, new Rotation2d(180)), 
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
                backInside,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);



        

        addCommands(

            // place preloaded gamepiece
            new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.CUBEHIGH, () -> GamePiece.CUBE),
            new InstantCommand(() -> s_Intake.setMotor(0.75)),
            new WaitCommand(0.5),
            new InstantCommand(() -> s_Intake.stopIntake()),

            // stow arm
            new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.STOWED, () -> GamePiece.CUBE),

            // wait before starting path
            new WaitCommand(0.75),

            // start path
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectory1.getInitialPose())),
            swerveControllerCommand1,

            // wait before starting next command
            new WaitCommand(0.5),

            // back into community
            new InstantCommand(() -> s_Swerve.resetOdometry(backInside.getInitialPose())),
            swerveControllerCommand2,

            // reset gyro
            new InstantCommand(() -> s_Swerve.zeroGyro())


        );
    }
}
