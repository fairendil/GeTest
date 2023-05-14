using DigitalAssembly.Math.Common.Interfaces;
using MathNet.Numerics.LinearAlgebra;
using static System.Math;

namespace DigitalAssembly.Math.Common;

public class Point3D<T> : IVectorDouble3D
    where T : Point3D<T>
{
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }

    public Vector<double> Coordinate { get; }

    protected Point3D(double x, double y, double z)
    {
        X = x;
        Y = y;
        Z = z;
        Coordinate = Vector<double>.Build.DenseOfArray(new double[] { X, Y, Z });
    }

    public static T operator -(Point3D<T> left, T right) => (T)Activator.CreateInstance(typeof(T), left.X - right.X, left.Y - right.Y, left.Z - right.Z)!;

    public static T operator +(Point3D<T> left, T right) => (T)Activator.CreateInstance(typeof(T), left.X + right.X, left.Y + right.Y, left.Z + right.Z)!;

    public double L2Norm() => Coordinate.L2Norm();

    public Vector<double> Homogenous => Vector<double>.Build.DenseOfArray(new double[] { X, Y, Z, 1 });

    public override bool Equals(object? obj)
    {
        if (obj is not Point3D<T> point)
        {
            return false;
        }

        // TODO: Add precition from globals
        return ReferenceEquals(this, obj) || Equals(point, 1e-5);
    }

    public bool Equals(Point3D<T> point, double precision) => Abs(X - point.X) < precision && Abs(Y - point.Y) < precision && Abs(Z - point.Z) < precision;

    public override string ToString() => $"({X} {Y} {Z})";
}