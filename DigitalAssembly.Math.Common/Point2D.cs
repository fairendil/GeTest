using DigitalAssembly.Math.Common.Interfaces;
using MathNet.Numerics.LinearAlgebra;
using static System.Math;

namespace DigitalAssembly.Math.Common;

public class Point2D<T> : IVectorDouble2D
    where T : Point2D<T>
{
    public double X { get; set; }
    public double Y { get; set; }

    public Vector<double> Coordinate { get; }

    protected Point2D(double x, double y)
    {
        X = x;
        Y = y;
        Coordinate = Vector<double>.Build.DenseOfArray(new double[] { X, Y });
    }

    public static T operator -(Point2D<T> left, T right) => (T)Activator.CreateInstance(typeof(T), left.X - right.X, left.Y - right.Y)!;

    public static T operator +(Point2D<T> left, T right) => (T)Activator.CreateInstance(typeof(T), left.X + right.X, left.Y + right.Y)!;

    public static T operator *(Point2D<T> left, T right) => (T)Activator.CreateInstance(typeof(T), left.X * right.X, left.Y * right.Y)!;

    public static T operator /(Point2D<T> left, T right) => (T)Activator.CreateInstance(typeof(T), left.X / right.X, left.Y / right.Y)!;

    public static T operator *(Point2D<T> left, double parameter) => (T)Activator.CreateInstance(typeof(T), left.X * parameter, left.Y * parameter)!;

    public static T operator /(Point2D<T> left, double parameter) => (T)Activator.CreateInstance(typeof(T), left.X / parameter, left.Y / parameter)!;

    [Obsolete("Use Equals() instead")]
    public static bool operator ==(Point2D<T> left, T right) => throw new InvalidOperationException();

    [Obsolete("Use Equals() instead")]
    public static bool operator !=(Point2D<T> left, T right) => throw new InvalidOperationException();

    public double L2Norm() => Coordinate.L2Norm();

    public Vector<double> Homogenous => Vector<double>.Build.DenseOfArray(new double[] { X, Y, 1 });

    public override bool Equals(object? obj)
    {
        if (obj is not Point2D<T> point)
        {
            return false;
        }

        // TODO: Add precition from globals
        return ReferenceEquals(this, obj) || Equals(point, 1e-5);
    }

    public bool Equals(Point2D<T> point, double precision) => Abs(X - point.X) < precision && Abs(Y - point.Y) < precision;

    public override string ToString() => $"({X} {Y})";
}
