using DigitalAssembly.GoldenEye.DB;
using DigitalAssembly.Math.Common;
using DigitalAssembly.Photogrammetry;
using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;
using DigitalAssembly.Photogrammetry.Serializers;
using static System.Math;

namespace DigitalAssembly.GoldenEye.Serializers;

public static class ModelSerializer
{
    private const int DOUBLE_COUNT = 4;

    public static MarkPoint<T>[] LoadPointsFromFile<T>(string filename)
        where T : Point3D<T>
    {
        List<MarkPoint<T>> points = new();
        double[][] doubleValues = DoubleCsvSerializer.LoadFromFile(filename, DOUBLE_COUNT, @"\,?\s+");
        for (int i = 0; i < doubleValues.Length; ++i)
        {
            double[] doubles = doubleValues[i];
            if (Abs(doubles[0] % 1) > double.Epsilon)
            {
                throw new SerializerException($"Mark code in line '{i}' is not int value");
            }

            int code = (int)doubles[0];
            T point = (T)Activator.CreateInstance(typeof(T), doubles[1], doubles[2], doubles[3])!;
            points.Add(MarkPoint<T>.FromCode(code, MarkCodeType.BitCode14b, point));
        }

        return points.ToArray();
    }

    public static Model LoadAdapterFromFile(string fileName)
    {
        try
        {
            MarkPoint<ModelCsPoint>[] modelMarks = LoadPointsFromFile<ModelCsPoint>(fileName);
            return new AdapterModel(Path.GetFileNameWithoutExtension(fileName), modelMarks, new() { new ModelCsPoint(0, 0, 0) });
        }
        catch (Exception ex)
        {
            throw new SerializerException($"Adapeter from file {fileName} cannot be loaded: {ex}");
        }
    }

    public static Model LoadReferenceFromFile(string fileName)
    {
        try
        {
            MarkPoint<ModelCsPoint>[] modelMarks = LoadPointsFromFile<ModelCsPoint>(fileName);
            return new ReferenceModel(Path.GetFileNameWithoutExtension(fileName), modelMarks, new() { new ModelCsPoint(0, 0, 0) });
        }
        catch (Exception ex)
        {
            throw new SerializerException($"Reference from file {fileName} cannot be loaded: {ex}");
        }
    }
}