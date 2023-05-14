using DigitalAssembly.GoldenEye.Objects;

namespace DigitalAssembly.GoldenEye.Classifier;
public class ClassificationResult
{
    public MovableObject MeasuredObject { get; }

    public double MeanSquaredFittingError { get; }

    public ClassificationResult(MovableObject measuredObject, double meansquaredFittingError)
    {
        MeasuredObject = measuredObject;
        MeanSquaredFittingError = meansquaredFittingError;
    }
}
