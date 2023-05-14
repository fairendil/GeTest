using DigitalAssembly.Math.Common;

namespace DigitalAssembly.Photogrammetry.Serializers;

public static class MarkPointsSerializer2D
{
    private const int DOUBLE_COUNT = 3;

    public static MarkPoint<T>[] LoadFromFile<T>(string filename, string delimeterRegex, MarkCodeType codeType)
        where T : Point2D<T>
    {
        List<MarkPoint<T>> points = new();
        double[][] doubleValues = DoubleCsvSerializer.LoadFromFile(filename, DOUBLE_COUNT, delimeterRegex);
        for(int i = 0; i < doubleValues.Length; ++i)
        {
            double[] doubles = doubleValues[i];
            if (System.Math.Abs(doubles[0] % 1) > double.Epsilon)
            {
                throw new SerializerException($"Mark code in line '{i}' is not int value");
            }

            int code = (int)doubles[0];
            T point = (T)Activator.CreateInstance(typeof(T), doubles[1], doubles[2])!;
            points.Add(MarkPoint<T>.FromCode(code, codeType, point));
        }

        return points.ToArray();
    }

    public static MarkPoint<T>[] LoadFromPawlinCsv<T>(string filename, MarkCodeType codeType)
        where T : Point2D<T>
    {
        List<MarkPoint<T>> points = new();
        double[][] doubleValues = DoubleCsvSerializer.LoadFromFile(filename, 8, @"\;");
        for (int i = 0; i < doubleValues.Length; ++i)
        {
            double[] doubles = doubleValues[i];
            if (System.Math.Abs(doubles[1] % 1) > double.Epsilon)
            {
                throw new SerializerException($"Mark code in line '{i}' is not int value");
            }

            int code = (int)doubles[1];
            T point = (T)Activator.CreateInstance(typeof(T), doubles[6], doubles[7])!;
            points.Add(MarkPoint<T>.FromCode(code, codeType, point));
        }

        return points.ToArray();
    }

}
