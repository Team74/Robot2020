package frc.lib.utils.geometry;

import frc.lib.utils.geometry.State;

public interface ICurvature<S> extends State<S> {
    double getCurvature();

    double getDCurvatureDs();
}