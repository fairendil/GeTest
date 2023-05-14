using MathNet.Numerics.LinearAlgebra;

namespace DigitalAssembly.Math.Common.Interfaces;

public interface IVectorDouble3D: IPoint
{
    public Vector<double> Coordinate { get; }
}
