using DigitalAssembly.Math.Common.Interfaces;
using MathNet.Numerics.LinearAlgebra;

namespace DigitalAssembly.Photogrammetry.Geometry.Common;

public sealed class PixelSize : IVectorDouble2D
{
    public PixelSize(double x, double y)
    {
        X = x;
        Y = y;
    }

    public double X { get; }
    public double Y { get; }

    public Vector<double> Coordinate => Vector<double>.Build.DenseOfArray(new double[] { X, Y });
}
