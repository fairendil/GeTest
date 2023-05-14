using DigitalAssembly.Math.Common;

namespace DigitalAssembly.Photogrammetry.Serializers;

public static class MarkPointsSerializer3D
{
    private const int DOUBLE_COUNT = 4;

    public static MarkPoint<T>[] LoadFromFile<T>(string filename, string delimeterRegex, MarkCodeType codeType)
        where T : Point3D<T>
    {
        List<MarkPoint<T>> points = new();
        double[][] doubleValues = DoubleCsvSerializer.LoadFromFile(filename, DOUBLE_COUNT, delimeterRegex);
        for (int i = 0; i < doubleValues.Length; ++i)
        {
            double[] doubles = doubleValues[i];
            if (System.Math.Abs(doubles[0] % 1) > double.Epsilon)
            {
                throw new SerializerException($"Mark code in line '{i}' is not int value");
            }

            int code = (int)doubles[0];
            T point = (T)Activator.CreateInstance(typeof(T), doubles[1], doubles[2], doubles[3])!;
            points.Add(MarkPoint<T>.FromCode(code, codeType, point));
        }

        return points.ToArray();
    }
}
