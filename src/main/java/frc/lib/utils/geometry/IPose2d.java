package frc.lib.utils.geometry;

import frc.lib.utils.geometry.IRotation2d;
import frc.lib.utils.geometry.ITranslation2d;

public interface IPose2d<S> extends IRotation2d<S>, ITranslation2d<S> {
    public Pose2d getPose();

    public S transformBy(Pose2d transform);

    public S mirror();
}