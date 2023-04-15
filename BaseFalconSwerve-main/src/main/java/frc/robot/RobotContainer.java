package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Position;
/* Autos */
import frc.robot.autos.*;
import frc.robot.commands.SetAllPositions;
/* Commands */
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.commands.Elevator.ElevatorSetPosition;
import frc.robot.commands.Elevator.ElevatorSetPositionHigh;
import frc.robot.commands.Elevator.TeleopElevator;
import frc.robot.commands.Wrist.TeleopWrist;
import frc.robot.commands.Wrist.WristSetPosition;
import frc.robot.commands.Shoulder.ShoulderSetPosition;
import frc.robot.commands.Shoulder.TeleopShoulder;
/* Subsystems */
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    public static GamePiece gamePiece = GamePiece.CONE;

    /* Controllers */
    private final static Joystick translateStick = new Joystick(0);
    private final static Joystick rotateStick = new Joystick(1);
    private final static Joystick operatorStick = new Joystick(2);

    /* Drive Controls */

    private final int j_translationAxis = Joystick.AxisType.kY.value;
    private final int j_strafeAxis = Joystick.AxisType.kX.value;
    private final int j_rotationAxis = Joystick.AxisType.kX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(rotateStick, 2);
    private final JoystickButton robotCentric = new JoystickButton(operatorStick, 1);

    // private static final JoystickButton driveFastTrigger = new
    // JoystickButton(translateStick, 1);
    // private static final JoystickButton driveSlowButton = new
    // JoystickButton(translateStick, 2);
    // private static final JoystickButton robotCentric = new
    // JoystickButton(translateStick, 3);

    // Left Stick

    private static final JoystickButton intakeTrigger = new JoystickButton(translateStick, 1);
    private static final JoystickButton outtakeButton = new JoystickButton(translateStick, 2);
    private static final JoystickButton selectConeButton = new JoystickButton(translateStick, 3);
    private static final JoystickButton selectCubeButton = new JoystickButton(translateStick, 4);

    // Right Stick Buttons
    private static final JoystickButton selectGamepieceTrigger = new JoystickButton(rotateStick, 1);
    private static final JoystickButton resetGyroButton = new JoystickButton(rotateStick, 2);

    // Operator Controls
    private static final int elevatorAxis = Joystick.AxisType.kY.value;
    private static final int wristAxis = Joystick.AxisType.kX.value;

    private static final JoystickButton stopIntake = new JoystickButton(operatorStick, 1);
    private static final JoystickButton outTakeSlowButton = new JoystickButton(operatorStick, 2);
    private static final JoystickButton outTakeFastButton = new JoystickButton(operatorStick, 3);
    // private static final JoystickButton toggleLEDButton = new
    // JoystickButton(operatorStick, 4);
    private static final JoystickButton pickHumanPlayerButton = new JoystickButton(operatorStick, 5);
    private static final JoystickButton pickStandingConeButton = new JoystickButton(operatorStick, 6);
    private static final JoystickButton pickTippedConeButton = new JoystickButton(operatorStick, 7);
    private static final JoystickButton pickCubeButton = new JoystickButton(operatorStick, 8);
    private static final JoystickButton stowIntakeButton = new JoystickButton(operatorStick, 9);
    private static final JoystickButton placeHighButton = new JoystickButton(operatorStick, 10);
    private static final JoystickButton placeMidButton = new JoystickButton(operatorStick, 11);
    private static final JoystickButton placeLowButton = new JoystickButton(operatorStick, 12);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final IntakeSub s_Intake = new IntakeSub();
    private final WristSub s_Wrist = new WristSub();
    private final ElevatorSub s_Elevator = new ElevatorSub();
    private final ShoulderSub s_Shoulder = new ShoulderSub();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -translateStick.getRawAxis(j_translationAxis),
                        () -> -translateStick.getRawAxis(j_strafeAxis),
                        () -> -rotateStick.getRawAxis(j_rotationAxis),
                        () -> robotCentric.getAsBoolean()));

        s_Elevator.setDefaultCommand(
                new TeleopElevator(
                        s_Elevator,
                        () -> -operatorStick.getY() * 1000));
        s_Shoulder.setDefaultCommand(
            new TeleopShoulder(
                s_Shoulder,
                () -> operatorStick.getX() * 1000));

        s_Wrist.setDefaultCommand(new TeleopWrist(
                s_Wrist,
                () -> operatorStick.getTwist() * 1000));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */

        // Translate Stick

        intakeTrigger.onTrue(new InstantCommand(() -> s_Intake.setMotor(1))); // button 1
        outtakeButton.onTrue(new InstantCommand(() -> s_Intake.setMotor(-1))); // button 2
        // selectConeButton.onTrue(new InstantCommand(()->
        // setGamePiece(GamePiece.CONE))); // button 3
        // selectCubeButton.onTrue(new InstantCommand(()->
        // setGamePiece(GamePiece.CUBE))); // button 4

        // Rotate Stick

        // selectGamepieceTrigger.onTrue(new InstantCommand(() ->
        // setGamePiece(GamePiece.CONE)));
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro())); // button 2

        /* Operator Buttons */

        stopIntake.onTrue(new InstantCommand(() -> s_Intake.setMotor(0)));// button 1

         outTakeSlowButton.whileTrue(new InstantCommand(() -> s_Intake.setMotor(0.5)));
         // button 2

         outTakeFastButton.whileTrue(new InstantCommand(()-> s_Intake.setMotor(-0.5)));
        // // button 3

        // // button 4

        // pickHumanPlayerButton.onTrue(new SequentialCommandGroup( // button 5
        // new InstantCommand(() -> setGamePiece(GamePiece.CONE)),
        // new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder,
        // Position.HUMANPLAYERINTAKE, () -> GamePiece.CONE)));

        // pickStandingConeButton.onTrue(new SequentialCommandGroup( // button 6
        // new InstantCommand(() -> setGamePiece(GamePiece.CONE)),
        // new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder,
        // Position.STANDINGCONEINTAKE, () -> GamePiece.CONE)));

        // pickTippedConeButton.onTrue(new SequentialCommandGroup( // button 7
        // new InstantCommand(() -> setGamePiece(GamePiece.CONE)),
        // new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder,
        // Position.TIPPEDCONEINTAKE, () -> GamePiece.CONE)));

        // pickCubeButton.onTrue(new SequentialCommandGroup( // button 8
        // new InstantCommand(() -> setGamePiece(GamePiece.CUBE)),
        // new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.CUBEINTAKE, ()
        // -> GamePiece.CUBE)));

        // stowIntakeButton.onTrue(new SequentialCommandGroup( // button 9
        // new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.STOWED, () ->
        // GamePiece.CONE)));

        // placeHighButton.onTrue(new SequentialCommandGroup( // button 10
        // new InstantCommand(() -> setGamePiece(GamePiece.CUBE)),
        // new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.HIGH, () ->
        // GamePiece.CUBE)));

        // placeMidButton.onTrue(new SequentialCommandGroup( // button 11
        // new InstantCommand(() -> setGamePiece(GamePiece.CUBE)),
        // new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.MID, () ->
        // GamePiece.CUBE)));

        // placeLowButton.onTrue(new SequentialCommandGroup( // button 12
        // new InstantCommand(() -> setGamePiece(GamePiece.CUBE)),
        // new SetAllPositions(s_Wrist, s_Elevator, s_Shoulder, Position.LOW, () ->
        // GamePiece.CUBE)));

    }

    public static GamePiece getGamePiece() {
        return gamePiece;
    }

    public static void setGamePiece(GamePiece piece) {
        gamePiece = piece;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
