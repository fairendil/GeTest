using BenchmarkDotNet.Attributes;
using DigitalAssembly.Photogrammetry.Camera;
using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;
using FastSerialization;
using System.Text.RegularExpressions;

namespace DigitalAssembly.Photogrammetry.Serializers;

[MemoryDiagnoser]
public class Aicon3DCalibrationSerializer
{
    private const int INTRINSIC_LINES = 10;

    private static readonly Regex Pattern = new(@"(?<name>[a-zA-Z0-9]+)\s*:\s*(?<value>[-+eE0-9.]+)", RegexOptions.IgnoreCase | RegexOptions.Compiled);
    private static readonly string _intrisicFile = Path.GetFullPath("c:\\Repositories\\goldeneye-1\\Test\\SystemParameters-MI\\Параметры калибровки камер MI.txt");//.Join("SystemParameters-MI", $"Параметры калибровки камер MI.txt");

    public IEnumerable<string> Lines { get; }

    public static IntrisicParameters LoadIntrisicParameters(string filename, bool isLeft)
    {
        string blockStart = "Camera/R0:               " + (isLeft ? "1" : "2");
        IEnumerable<string> lines = File.ReadLines(_intrisicFile)
                                .SkipWhile(i => !i.Trim().StartsWith(blockStart))
                                .Skip(1)
                                .Take(INTRINSIC_LINES);
        List<string> names = new(new string[] { "ck", "xh", "yh", "a1", "a2", "a3", "b1", "b2", "c1", "c2" });
        CameraModelSerializer.SerializeParametersArray(lines, out double[] parametrsArray, Pattern, names);
        return new(parametrsArray[0], new(parametrsArray[1], parametrsArray[2]), new(
            parametrsArray[0], parametrsArray[1], parametrsArray[2], parametrsArray[3], parametrsArray[4],
            parametrsArray[5], parametrsArray[6]));
    }

    public static IEnumerable<MarkPoint<PictureCsPoint>[]> SerialazeCameraCsPoints(string filename, bool isLeft)
    {
        string blockStart = "*** Image coordinates ***";
        string blockEnd = "*** Summary ***";// + (isLeft ? "1" : "2");
        string[] array = File.ReadLines(_intrisicFile)
                                        .Where(line => !string.IsNullOrWhiteSpace(line) && !line.TrimEnd().EndsWith("***"))
                                        .SkipWhile(line => !line.Trim().StartsWith(blockStart))
                                        .Skip(2)
                                        .TakeWhile(line => !line.Trim().StartsWith(blockEnd))
                                        .SkipLast(3)
                                        .ToArray();
        //Span<string> lines = new(array);
        //Span<string> except = lines.TrimEnd(new ReadOnlySpan<string>("***"));
        //Span<string> linesSpan = new Span<string>(lines.Except(except).To)

        List<double[]> list = DoubleCsvSerializer.Serialize(12, @"\s+", array);
        foreach (int pictureIndex in list.Select(i => (int)i[1]).Distinct())
        {
            yield return list.Where(i => i[1] == pictureIndex).Select(i => MarkPoint<PictureCsPoint>.FromCode((int)i[0], MarkCodeType.BitCode14b, new PictureCsPoint(i[2], i[4]))).ToArray();
        }
    }
}
