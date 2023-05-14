using MathNet.Numerics.LinearAlgebra;

namespace DigitalAssembly.Math.Common.Interfaces;

public interface IVectorDouble2D: IPoint
{
    Vector<double> Coordinate { get; }

    bool Equals(object obj);
}
