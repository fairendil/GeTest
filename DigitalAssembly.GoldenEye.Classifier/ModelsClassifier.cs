using DigitalAssembly.GoldenEye.DB;
using DigitalAssembly.GoldenEye.Objects;
using DigitalAssembly.Math.PointClouds;
using DigitalAssembly.Photogrammetry;
using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;
using System.Diagnostics;

namespace DigitalAssembly.GoldenEye.Classifier;

public class ModelsClassifier
{
    private List<Model> _objectsDb;
    private ClassifyAndComputeTransformation _transformationFinder;

    public ModelsClassifier(List<Model> objects)
    {
        _objectsDb = objects;
        double tolerance = 1;
        double similarity = 0.85;
        _transformationFinder = new(tolerance, similarity);
    }

    public void UpdateObjectsDB(List<Model> objects) => _objectsDb = objects;

    public void SetParametersForAlgorithm(double tolerance, double similarity) => _transformationFinder = new(tolerance, similarity);

    public List<ClassificationResult> Classify(MarkPoint<ModelCsPoint>[] pointCloud)
    {
        List<ClassificationResult> classificationResults = new();
        List<ModelCsPoint> points = pointCloud.Select(point => point.Point).ToList();
        foreach(Model model in _objectsDb)
        {
            List<ModelCsPoint> modelPoints = model.Points.Select(point => point.Point).ToList();
            PointCloudClassificationResult<ModelCsPoint> classificationResult = _transformationFinder.Estimate(modelPoints.ToList(), points);
            if (classificationResult.ClassificationStatus == Math.PointClouds.Classification.ClassificationStatus.NotClassified) 
            {
                continue;
            }

            MovableObject found = model switch
            {
                AdapterModel => new Adapter((AdapterModel)model),
                ReferenceModel => new Reference((ReferenceModel)model),
                _ => throw new UnreachableException(),
            };

            found.SetClassificationResult(classificationResult);
            classificationResults.Add(new(found, classificationResult.FittingQuality));
        }

        return classificationResults;
    }
}
