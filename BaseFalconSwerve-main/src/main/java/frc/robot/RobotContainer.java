package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/* Autos */
import frc.robot.autos.*;

/* Commands */
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.commands.Elevator.ElevatorSetPosition;
import frc.robot.commands.Elevator.ElevatorSetPositionHigh;
import frc.robot.commands.Elevator.TeleopElevator;
import frc.robot.commands.Wrist.WristSetPosition;
import frc.robot.commands.Shoulder.ShoulderSetPosition;

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

    private static final JoystickButton driveFastTrigger = new JoystickButton(translateStick, 1);
    private static final JoystickButton driveSlowButton = new JoystickButton(translateStick, 2);
    // private static final JoystickButton robotCentric = new
    // JoystickButton(translateStick, 3);

    private static final JoystickButton gridLineUpButton = new JoystickButton(translateStick, 4);
    // rotateStick Buttons
    private static final JoystickButton selectGamepieceTrigger = new JoystickButton(rotateStick, 1);
    private static final JoystickButton resetGyroButton = new JoystickButton(rotateStick, 2);
    private static final JoystickButton autoBalanceButton = new JoystickButton(rotateStick, 3);
    private static final JoystickButton followPathButton = new JoystickButton(rotateStick, 4);

    // Operator Controls
    private static final int elevatorAxis = Joystick.AxisType.kY.value;
    private static final int wristAxis = Joystick.AxisType.kX.value;

    private static final JoystickButton intakeTrigger = new JoystickButton(translateStick, 1);
    private static final JoystickButton outTakeSlowButton = new JoystickButton(operatorStick, 2);
    private static final JoystickButton outTakeFastButton = new JoystickButton(operatorStick, 3);
    private static final JoystickButton toggleLEDButton = new JoystickButton(operatorStick, 4);
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
    private final Intake s_Intake = new Intake();
    private final Wrist s_Wrist = new Wrist();
    private final Elevator s_Elevator = new Elevator();
    private final Shoulder s_Shoulder = new Shoulder();

    private double elevatorDesiredPosition = 0;
    // private final Wrist s_Wrist = new Wrist();
    // private final Shoulder s_Shoulder = new Shoulder();
    // private final ElevatorTest s_ElevatorTest = new ElevatorTest();

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
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        /* Operator Button */
        // robotCentric.onTrue(new ElevatorSetPosition(s_ElevatorTest));
       // placeHighButton.onTrue(new ElevatorSetPositionHigh(s_Elevator));
      // placeHighButton.onTrue(new ElevatorSetPositionHigh(s_Elevator));
        // placeLowButton.onTrue(new SetPositionLow(s_ElevatorTest));
        outTakeSlowButton.onTrue(new InstantCommand(() -> s_Intake.setMotor(1.0)));

        //outTakeFastButton.onTrue( new InstantCommand(()-> s_Shoulder.setPosition(-3500.0)));
        
        //outTakeFastButton.onTrue(new InstantCommand(() -> s_Wrist.setPosition(8000)));
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
