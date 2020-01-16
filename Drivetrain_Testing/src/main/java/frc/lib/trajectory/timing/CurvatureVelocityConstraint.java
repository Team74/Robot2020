package frc.lib.trajectory.timing;

import frc.robot.Constants;

import frc.lib.trajectory.timing.TimingConstraints;

import frc.lib.utils.geometry.Pose2dWithCurvature;

public class CurvatureVelocityConstraint implements TimingConstraints<Pose2dWithCurvature> {

	public double getMaxVelocity(final Pose2dWithCurvature state){
		return Constants.kRobotMaxVelocity / (1 + Math.abs(4.0*state.getCurvature()));//6.0
	}
	
	public MinMaxAcceleration getMinMaxAcceleration(final Pose2dWithCurvature state, final double velocity){
		return MinMaxAcceleration.kNoLimits;
	}
	
}