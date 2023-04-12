// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.driverstation;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Joystick interface for thrustmaster joysticks
 */
public interface ThrustmasterJoystick {
    // Methods
    public Joystick getJoystick();

    public double getX();
    public double getY();
    public double getXRateLimited(double maxAllowableChangePerLoop);
    public double getYRateLimited(double maxAllowableChangePerLoop);
    public double getTwist();
    public double getSlider();

    public boolean getTrigger();
    public boolean getButtonValue(int buttonNum);
}
