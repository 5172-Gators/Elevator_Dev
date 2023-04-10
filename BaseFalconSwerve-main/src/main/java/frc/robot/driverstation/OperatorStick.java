// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.driverstation;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Joystick wrapper class for the operator joystick
 */
public class OperatorStick implements ThrustmasterJoystick{
    // Variables
    private static int joystickPort = 2;
    private static Joystick m_operatorStick;

    private static double rateLimitedX = 0.0;
    private static double rateLimitedY = 0.0;

    // Button IDs
    private static final int k_fireButton = 1;
   // private static final int k_cancelAimModesButton = 2;
    private static final int k_forceFireButton = 4;
    private static final int k_autoAimModeButton = 3;
    private static final int k_climberSafetyButton = 10;
    private static final int k_calAimModeButton = 16;
    private static final int k_calModeRPMUp = 13;
    private static final int k_calModeRPMDown = 14;
    private static final int k_calModeHoodUp = 12;
    private static final int k_calModeHoodDown = 15;
    private static final int k_calModeSavePointButton = 11;
    private static final int k_calModeSaveTableButton = 6;
    private static final int k_calModeClearTableButton = 5;
    private static final int k_incrementAdjustableDistanceOffsetButton = 7;
    private static final int k_decrementAdjustableDistanceOffsetButton = 8;
    private static final int k_resetAdjustableDistanceOffsetButton = 9;

    // Better Button Bindings

    private static final int k_Button_1 = 1;
    private static final int k_button_2 = 2;
    private static final int k_button_3 = 3;
    private static final int k_button_4 = 4;
    private static final int k_button_5 = 5;
    private static final int k_button_6 = 6;
    private static final int k_button_7 = 7;
    private static final int k_button_8 = 8;
    private static final int k_button_9 = 9;
    private static final int k_button_10 = 10;
    private static final int k_button_11 = 11;
    private static final int k_button_12 = 12;
    private static final int k_button_13 = 13;
    private static final int k_button_14 = 14;
    private static final int k_button_15 = 15;
    private static final int k_button_16 = 16;



    /**
     * Default constructor for the operator joystick
     */
    public OperatorStick() {
        m_operatorStick = new Joystick(joystickPort);
    }

    /**
     * Overload constructor for the operator joystick
     * @param portNum
     */
    public OperatorStick(int portNum) {
        OperatorStick.joystickPort = portNum;
        m_operatorStick = new Joystick(joystickPort);
    }
/**
     * Returns a reference to the Joystick object
     */
    @Override
    public Joystick getJoystick() {
        return m_operatorStick;
    }

    /**
     * Returns the double value of the x axis between -1.0 and 1.0
     */
    @Override
    public double getX() {
        return m_operatorStick.getX();
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
        return -m_operatorStick.getY();
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
        return m_operatorStick.getZ();
    }

    /**
     * Returns the value of the slider axis between -1.0 and 1.0
     */
    @Override
    public double getSlider() {
        return -m_operatorStick.getRawAxis(4);
    }

    /**
     * Returns the boolean value of the trigger
     */
    @Override
    public boolean getTrigger() {
        return m_operatorStick.getTrigger();
    }

    /**
     * Returns the boolean value of any button on the joystick
     * @param buttonNum ID of the desired button
     */
    @Override
    public boolean getButtonValue(int buttonNum) {
        if(buttonNum <= m_operatorStick.getButtonCount())
            return m_operatorStick.getRawButton(buttonNum);
        else
            return false;
    }

    /**
     * Used to get the boolean state of the fire button
     * @return value of fire button
     */
    public boolean getFireButton() {
        return m_operatorStick.getRawButton(k_fireButton);
    }
    
    /**
     * Used to get the boolean state of the cancel aim modes button
     * @return value of cancel aim modes button
     */

    public boolean getForceFireButton() {
        return m_operatorStick.getRawButton(k_forceFireButton);
    }

    public boolean getAutoAimModeButton() {
        return m_operatorStick.getRawButton(k_autoAimModeButton);
    }

    public boolean getClimberSafetyButton() {
        return m_operatorStick.getRawButton(k_climberSafetyButton);
    }

    public boolean getCalAimModeButton() {
        return m_operatorStick.getRawButton(k_calAimModeButton);
    }

    public boolean getCalModeRPMUpButton() {
        return m_operatorStick.getRawButton(k_calModeRPMUp);
    }

    public boolean getCalModeRPMDownButton() {
        return m_operatorStick.getRawButton(k_calModeRPMDown);
    }

    public boolean getCalModeHoodUp() {
        return m_operatorStick.getRawButton(k_calModeHoodUp);
    }

    public boolean getCalModeHoodDown() {
        return m_operatorStick.getRawButton(k_calModeHoodDown);
    }

    public boolean getCalModeSavePoint() {
        return m_operatorStick.getRawButton(k_calModeSavePointButton);
    }

    public boolean getCalModeSaveTableButton() {
        return m_operatorStick.getRawButton(k_calModeSaveTableButton);
    }

    public boolean getCalModeClearTable() {
        return m_operatorStick.getRawButton(k_calModeClearTableButton);
    }

    public boolean getIncrementAdjustableDistanceOffsetButton() {
        return m_operatorStick.getRawButton(k_incrementAdjustableDistanceOffsetButton);
    }

    public boolean getDecrementAdjustableDistanceOffsetButton() {
        return m_operatorStick.getRawButton(k_decrementAdjustableDistanceOffsetButton);
    }

    public boolean getResetAdjustableDistanceOffsetButton() {
        return m_operatorStick.getRawButton(k_resetAdjustableDistanceOffsetButton);
    }

    public boolean getButton1() {
        return m_operatorStick.getRawButton(k_Button_1);
    }

    public boolean getButton2() {
     return m_operatorStick.getRawButton(k_button_2);
    }

    public boolean getButton3() {
        return m_operatorStick.getRawButton(k_button_3);
    }

    public boolean getButton4() {
     return m_operatorStick.getRawButton(k_button_4);
    }

    public boolean getButton5() {
        return m_operatorStick.getRawButton(k_button_5);
    }

    public boolean getButton6() {
        return m_operatorStick.getRawButton(k_button_6);
    }


    public boolean getButton7() {
        return m_operatorStick.getRawButton(k_button_7);
    }

    public boolean getButton8() {
        return m_operatorStick.getRawButton(k_button_8);
    }

    public boolean getButton9() {
        return m_operatorStick.getRawButton(k_button_9);
    }

    public boolean getButton10() {
        return m_operatorStick.getRawButton(k_button_10);
    }

    public boolean getButton11() {
     return m_operatorStick.getRawButton(k_button_11);
    }

    public boolean getButton12() {
       return m_operatorStick.getRawButton(k_button_12);
    }

    public boolean getButton13() {
        return m_operatorStick.getRawButton(k_button_13);
    }

    public boolean getButton14() {
        return m_operatorStick.getRawButton(k_button_14);
    }

    public boolean getButton15() {
        return m_operatorStick.getRawButton(k_button_15);
    }

    public boolean getButton16() {
        return m_operatorStick.getRawButton(k_button_16);
    }
}
