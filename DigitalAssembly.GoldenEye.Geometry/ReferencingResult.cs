using DigitalAssembly.GoldenEye.Objects;

namespace DigitalAssembly.GoldenEye.Geometry;

public class ReferencingResult
{
    public List<Adapter> MeasuredObjects { get; }

    public double ReferencingError { get; }

    public ReferenceStatus IsReferenceGood { get; }

    public ReferencingResult(List<Adapter> measuredObjects, double referencingError, ReferenceStatus isReferenceGood)
    {
        MeasuredObjects = measuredObjects;
        ReferencingError = referencingError;
        IsReferenceGood = isReferenceGood;
    }
}
