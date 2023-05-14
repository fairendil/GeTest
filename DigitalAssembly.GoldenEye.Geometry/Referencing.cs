using DigitalAssembly.GoldenEye.Classifier;
using DigitalAssembly.GoldenEye.DB;
using DigitalAssembly.GoldenEye.Objects;
using MathNet.Numerics.LinearAlgebra;

namespace DigitalAssembly.GoldenEye.Geometry;

// TODO: discuss referencing with metrologists
public sealed class Referencing
{
    private readonly Reference _selectedReference;
    private readonly double _referenceErrorTolerance;

    public Referencing(ReferenceModel referenceObject, double referenceErrorTolerance)
    {
        _selectedReference = new(referenceObject);
        _referenceErrorTolerance = referenceErrorTolerance;
    }

    private static Matrix<double> ComputeAdapterToReferenceTransformation(Matrix<double> referenceTransformation, Matrix<double> adapterTransformation)
    {
        return referenceTransformation == null || referenceTransformation.ColumnCount != 4 || referenceTransformation.RowCount != 4 ||
                adapterTransformation == null || adapterTransformation.ColumnCount != 4 || adapterTransformation.RowCount != 4
                ? throw new ArgumentException("Both matrices: adapter and reference - should be 4x4 matrices of homogrgaphy")
                : referenceTransformation.Inverse() * adapterTransformation;
    }

    public ReferencingResult EstimateTransformations(List<ClassificationResult> measuredObjects)
    {
        Reference? measuredReference = null;
        double referenceError = double.NaN;
        int referenceIndexInObjectList = -1;
        foreach (ClassificationResult? measuredObject in from ClassificationResult measuredObject in measuredObjects
                                                           where _selectedReference.Equals(measuredObject.MeasuredObject)
                                                           select measuredObject)
        {
            measuredReference = (Reference)measuredObject.MeasuredObject;
            referenceError = measuredObject.MeanSquaredFittingError;
            referenceIndexInObjectList = measuredObjects.IndexOf(measuredObject);
        }

        // Reference object not found, return all found elements as is
        if (measuredReference == null || referenceIndexInObjectList == -1)
        {
            return new(measuredObjects.Select(element => (Adapter)element.MeasuredObject).ToList(), double.NaN, ReferenceStatus.NotFound);
        }

        measuredObjects.RemoveAt(referenceIndexInObjectList);
        Matrix<double> referenceTransformation = measuredReference.Position.TransformationMatrix;
        List<Adapter> result = new();
        foreach (Adapter adapter in measuredObjects.Where(element => element.MeasuredObject is Adapter)
                                                    .Select(element => (Adapter)element.MeasuredObject))
        {
            Matrix<double> adapterTransformation = adapter.Position.TransformationMatrix;
            Matrix<double> adapterToReferenceTransformation = ComputeAdapterToReferenceTransformation(referenceTransformation, adapterTransformation);
            result.Add(adapter.UpdateTransformation(adapterToReferenceTransformation));
        }

        ReferenceStatus status = referenceError < _referenceErrorTolerance 
                               ? ReferenceStatus.Good 
                               : ReferenceStatus.Bad;
        return new(result, referenceError, status);
    }
}
