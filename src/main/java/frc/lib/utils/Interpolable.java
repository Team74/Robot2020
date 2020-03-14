package frc.lib.utils;

/*
Interpolable is an interface used by an Interpolating Tree as the Value type. Given two end points and an
interpolation parameter on [0, 1], it calculates a new Interpolable representing the interpolated value.
*/

/*
Interpolates between this value and an other value according to a given parameter. If x is 0, the method should
return this value. If x is 1, the method should return the other value. If 0 < x < 1, the return value should be
interpolated proportionally between the two.
*/
public interface Interpolable<T> {
    public T interpolate(T other, double x);
}