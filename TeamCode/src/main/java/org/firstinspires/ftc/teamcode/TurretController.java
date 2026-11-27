package org.firstinspires.ftc.teamcode;

public class TurretController {

// Converts a desired angle into encoder ticks.
public static double angleToTicks(
double angleDegrees,
double ticksPerRevolution,
double zeroPosition
) {
double ticksPerDegree = ticksPerRevolution / 360.0;

return zeroPosition + (angleDegrees * ticksPerDegree);
}

// Calculates how many ticks the turret is away from its target.
public static double calculateError(
double targetTicks,
double currentTicks
) {
return targetTicks - currentTicks;
}
}