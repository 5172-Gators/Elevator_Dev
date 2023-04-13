// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.driverstation;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Joystick wrapper class for the main driver rotation joystick
 */
public class DriveRotateStick implements ThrustmasterJoystick {
    // Variables
    private static int joystickPort = 1;
    private static Joystick m_rotateStick;

    private static double rateLimitedX = 0.0;
    private static double rateLimitedY = 0.0;


    // Button IDs
    private static final int k_resetGyroButton = 2;
    private static final int k_spitBallButton = 3;

    /**
     * Default constructor for the main driver rotation stick
     */
    public DriveRotateStick() {
        m_rotateStick = new Joystick(joystickPort);
    }

    /**
     * Overload constructor for the main driver rotation stick
     * @param portNum
     */
    public DriveRotateStick(int portNum) {
        DriveRotateStick.joystickPort = portNum;
        m_rotateStick = new Joystick(joystickPort);
    }

    /**
     * Returns a reference to the Joystick object
     */
    @Override
    public Joystick getJoystick() {
        return m_rotateStick;
    }

    /**
     * Returns the double value of the x axis between -1.0 and 1.0
     */
    @Override
    public double getX() {
        return m_rotateStick.getX();
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
        return -m_rotateStick.getY();
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
        return m_rotateStick.getZ();
    }

    /**
     * Returns the value of the slider axis between -1.0 and 1.0
     */
    @Override
    public double getSlider() {
        return -m_rotateStick.getRawAxis(4);
    }

    /**
     * Returns the boolean value of the trigger
     */
    @Override
    public boolean getTrigger() {
        return m_rotateStick.getTrigger();
    }

    /**
     * Returns the boolean value of any button on the joystick
     * @param buttonNum ID of the desired button
     */
    @Override
    public boolean getButtonValue(int buttonNum) {
        if(buttonNum <= m_rotateStick.getButtonCount())
            return m_rotateStick.getRawButton(buttonNum);
        else
            return false;
    }

    public boolean getResetGyroButton() {
        return m_rotateStick.getRawButton(k_resetGyroButton);
    }

    public boolean getSpitBallButton() {
        return m_rotateStick.getRawButton(k_spitBallButton);
    }

}
