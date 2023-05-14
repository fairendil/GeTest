using MathNet.Numerics.LinearAlgebra;

namespace DigitalAssembly.Math.PointClouds.Transformation;

public sealed class TransformationFittingResult
{
    public TransformationFittingResult(Matrix<double> transformation, double meanSquaredFittingError)
    {
        Transformation = transformation;
        MeanSquaredFittingError = meanSquaredFittingError;
    }

    public Matrix<double> Transformation { get; }

    public double MeanSquaredFittingError { get; }
}
