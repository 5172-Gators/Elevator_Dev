// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.driverstation;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Joystick wrapper class for the main driver translation stick
 */
public class DriveTranslateStick implements ThrustmasterJoystick {
    // Variables
    private static int joystickPort = 0;
    private static Joystick m_translateStick;

    private static double rateLimitedX = 0.0;
    private static double rateLimitedY = 0.0;

    // Button IDs
    private static final int k_intakeInButton = 1;
    private static final int k_intakeOutButton = 2;
    private static final int k_intakeDeployButton = 3;
    private static final int k_IntakeStowButton = 4;
    private static final int k_climberSoftLimitOverrideButton = 11;
    private static final int k_deployClimberHooksButton = 14;
    private static final int k_stowClimberHooks = 15;
    private static final int k_resetClimberWithoutTurretButton = 5;
    
    /**
     * Default constructor for the main driver translation stick
     */
    public DriveTranslateStick() {
        m_translateStick = new Joystick(joystickPort);
    }

    /**
     * Overload constructor for the main driver translation stick
     * @param portNum port number of joystick
     */
    public DriveTranslateStick(int portNum) {
        DriveTranslateStick.joystickPort = portNum;
        m_translateStick = new Joystick(joystickPort);
    }
    /**
     * Returns a reference to the Joystick object
     */
    @Override
    public Joystick getJoystick() {
        return m_translateStick;
    }

    /**
     * Returns the double value of the x axis between -1.0 and 1.0
     */
    @Override
    public double getX() {
        return m_translateStick.getX();
    }

    /**
     * Returns the slew rate limited joystick value of the Y axis
     * @param maxAllowableChangePerLoop maximum allowed change in value per code loop
     */
    public double getXRateLimited(double maxAllowableChangePerLoop) {
        double change = getX() - rateLimitedX;
        if (change > maxAllowableChangePerLoop) change = maxAllowableChangePerLoop;
        else if (change < -maxAllowableChangePerLoop) change = -maxAllowableChangePerLoop;
        rateLimitedX += change;
        return rateLimitedX;
    }

    /**
     * Returns the double value of the y axis between -1.0 and 1.0
     */
    @Override
    public double getY() {
        return -m_translateStick.getY();
    }

    /**
     * Returns the slew rate limited joystick value of the Y axis
     * @param maxAllowableChangePerLoop maximum allowed change in value per code loop
     */
    public double getYRateLimited(double maxAllowableChangePerLoop) {
        double change = getY() - rateLimitedY;
        if (change > maxAllowableChangePerLoop) change = maxAllowableChangePerLoop;
        else if (change < -maxAllowableChangePerLoop) change = -maxAllowableChangePerLoop;
        rateLimitedY += change;
        return rateLimitedY;
    }

    /**
     * Returns the value of the twist axis between -1.0 and 1.0
     */
    @Override
    public double getTwist() {
        return m_translateStick.getZ();
    }

    /**
     * Returns the value of the slider axis between -1.0 and 1.0
     */
    @Override
    public double getSlider() {
        return -m_translateStick.getRawAxis(4);
    }

    /**
     * Returns the boolean value of the trigger
     */
    @Override
    public boolean getTrigger() {
        return m_translateStick.getTrigger();
    }

    /**
     * Returns the boolean value of any button on the joystick
     * @param buttonNum ID of the desired button
     */
    @Override
    public boolean getButtonValue(int buttonNum) {
        if(buttonNum <= m_translateStick.getButtonCount())
            return m_translateStick.getRawButton(buttonNum);
        else
            return false;
    }

    public boolean getIntakeInButton() {
        return m_translateStick.getRawButton(k_intakeInButton);
    }

    public boolean getIntakeOutButton() {
        return m_translateStick.getRawButton(k_intakeOutButton);
    }

    public boolean getIntakeStowButton() {
        return m_translateStick.getRawButton(k_IntakeStowButton);
    }
    public boolean getIntakeDeployButton (){
        return m_translateStick.getRawButton(k_intakeDeployButton);
    }

    public boolean getClimberSoftLimitOverrideButton() {
        return m_translateStick.getRawButton(k_climberSoftLimitOverrideButton);
    }

    public boolean getClimberDeployHooksButton() {
        return m_translateStick.getRawButton(k_deployClimberHooksButton);
    }

    public boolean getClimberStowHooksButton() {
        return m_translateStick.getRawButton(k_stowClimberHooks);
    }

    public boolean getResetClimberWithoutTurretButton() {
        return m_translateStick.getRawButton(k_resetClimberWithoutTurretButton);
    }
}
