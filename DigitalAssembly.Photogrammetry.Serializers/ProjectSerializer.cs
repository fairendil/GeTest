using DigitalAssembly.Math.Common;
using DigitalAssembly.Math.Common.Enums;
using DigitalAssembly.Photogrammetry.Camera;
using DigitalAssembly.Photogrammetry.Camera.DistortionModels;
using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;
using MathNet.Spatial.Euclidean;
using MathNet.Spatial.Units;
using System.Text.RegularExpressions;

namespace DigitalAssembly.Photogrammetry.Serializers;

/// <summary>
/// Reads and writes project from/into files in one directory:
/// CameraLeft.dat
/// CameraDistortionLeft.dat
/// CameraViewLeft.dat
/// CameraRight.dat
/// CameraDistortionRight.dat
/// CameraViewRight.dat
/// </summary>
public static class CameraModelSerializer
{
    private static readonly Regex _pattern = new(@"(?<name>[a-zA-Z0-9]+)\s*=\s*(?<value>[-+eE0-9.]+)", RegexOptions.IgnoreCase | RegexOptions.Compiled);
    private static void LoadDistorionParameters(string filename, out ClassicDistortionParameters distortion)
                        => ParseDistorionParameters(File.ReadLines(filename), out distortion, _pattern);

    public static void ParseDistorionParameters(IEnumerable<string> lines, out ClassicDistortionParameters distortion, Regex? pattern = null, List<string>? names = null)
    {
        double[] distortionClassic = Enumerable.Range(0, 7).Select(_ => double.NaN).ToArray();

        pattern ??= new Regex("");
        names ??= new(new string[] { "a1", "a2", "a3", "b1", "b2", "c1", "c2" });
        foreach (string line in lines)
        {
            if (string.IsNullOrEmpty(line))
            {
                continue;
            }

            Match match = pattern.Match(line);
            if (!match.Success)
            {
                continue;
            }
            string name = match.Groups["name"].Value.ToLower();
            int index = names.IndexOf(name);
            if (index == -1)
            {
                throw new SerializerException($"Key name '{name}' not supported");
            }

            string value = match.Groups["value"].Value;
            if (!double.TryParse(value, out double v))
            {
                throw new SerializerException($"Value for key '{name}' should be double. Got {value}");
            }

            distortionClassic[index] = v;
        }

        if (distortionClassic.Any(double.IsNaN))
        {
            throw new SerializerException("Distortion parameters serialized incorrectly");
        }

        distortion = new(distortionClassic[0], distortionClassic[1], distortionClassic[2],
                                               distortionClassic[3], distortionClassic[4], distortionClassic[5],
                                               distortionClassic[6]);
    }

    public static void SerializeParametersArray(IEnumerable<string> lines, out double[] parametrsArray,
                                                Regex pattern, List<string>? names = null)
    {
        names ??= new(new string[] { "a1", "a2", "a3", "b1", "b2", "c1", "c2" });
        parametrsArray = Enumerable.Range(0, names.Count).Select(_ => double.NaN).ToArray();

        foreach (string line in lines)
        {
            if (string.IsNullOrEmpty(line))
            {
                continue;
            }

            Match match = pattern.Match(line);
            if (!match.Success)
            {
                continue;
            }
            string name = match.Groups["name"].Value.ToLower();
            int index = names.IndexOf(name);
            if (index == -1)
            {
                throw new SerializerException($"Key name '{name}' not supported");
            }

            string value = match.Groups["value"].Value;
            if (!double.TryParse(value, out double v))
            {
                throw new SerializerException($"Value for key '{name}' should be double. Got {value}");
            }

            parametrsArray[index] = v;
        }

        if (parametrsArray.Any(double.IsNaN))
        {
            throw new SerializerException("Parameters serialized incorrectly");
        }
    }

    private static void LoadIntrisicParameters(string filename, string distortionFile, out IntrisicParameters intrisic, out Size imageSize, out Size matrixSize)
    {
        double focus = 0;
        double[] ppoint = Enumerable.Range(0, 2).Select(_ => double.NaN).ToArray();
        double[] imgSize = Enumerable.Range(0, 2).Select(_ => double.NaN).ToArray();
        double[] matSize = Enumerable.Range(0, 2).Select(_ => double.NaN).ToArray();

        LoadDistorionParameters(distortionFile, out ClassicDistortionParameters distortion);

        using StreamReader file = new(filename);
        while (!file.EndOfStream)
        {
            try
            {
                string? line = file.ReadLine();
                if (string.IsNullOrEmpty(line))
                {
                    continue;
                }

                string[] t = Regex.Split(line, @"\s*=\s*").ToArray();
                switch (t[0].ToLower())
                {
                    case "width":
                        imgSize[0] = double.Parse(t[1]);
                        break;
                    case "height":
                        imgSize[1] = double.Parse(t[1]);
                        break;
                    case "matrixwidth":
                        matSize[0] = double.Parse(t[1]);
                        break;
                    case "matrixheight":
                        matSize[1] = double.Parse(t[1]);
                        break;
                    case "focus":
                        focus = double.Parse(t[1]);
                        break;
                    case "x0":
                        ppoint[0] = double.Parse(t[1]);
                        break;
                    case "y0":
                        ppoint[1] = double.Parse(t[1]);
                        break;
                }
            }
            catch
            {
                ;
            }
        }

        if (ppoint.Any(double.IsNaN) || imgSize.Any(double.IsNaN) || matSize.Any(double.IsNaN))
        {
            throw new SerializerException("Intrisic parameters serialized incorrectly");
        }

        intrisic = new(focus, new PictureCsPoint(ppoint[0], ppoint[1]), distortion);
        imageSize = new(imgSize[0], imgSize[1]);
        matrixSize = new(matSize[0], matSize[1]);
    }

    private static void LoadExtrisicParameters(string filename, out Transformation3D<ModelCsPoint> extrisic)
    {
        double[] view = Enumerable.Range(0, 6).Select(_ => double.NaN).ToArray();
        using StreamReader file = new(filename);

        while (!file.EndOfStream)
        {
            try
            {
                string? line = file.ReadLine();
                if (string.IsNullOrEmpty(line))
                {
                    continue;
                }

                string[] t = Regex.Split(line, @"\s*=\s*").ToArray();
                switch (t[0].ToLower())
                {
                    case "xs":
                        view[0] = double.Parse(t[1]);
                        break;
                    case "ys":
                        view[1] = double.Parse(t[1]);
                        break;
                    case "zs":
                        view[2] = double.Parse(t[1]);
                        break;
                    case "omega":
                        view[3] = double.Parse(t[1]);
                        break;
                    case "phi":
                        view[4] = double.Parse(t[1]);
                        break;
                    case "kappa":
                        view[5] = double.Parse(t[1]);
                        break;
                }
            }
            catch
            {
                ;
            }
        }

        if (view.Any(double.IsNaN))
        {
            throw new SerializerException("Extrisic parameters serialized incorrectly");
        }

        EulerAngles angles = new(Angle.FromDegrees(view[3]),
                                 Angle.FromDegrees(view[4]),
                                 Angle.FromDegrees(view[5]));
        Rotation rotation = new(angles, EulerAngleConvention.XYZ);
        ModelCsPoint translation = new(view[0], view[1], view[2]);

        //TODO: проверить сходимость от разных вариантов начальных приближений        
        EulerAngles anglesCv = new(Angle.FromDegrees(view[3]),
                                 Angle.FromDegrees(view[4]),
                                 Angle.FromDegrees(view[5]));
        //-z-yx
        Rotation rotationCv = new(anglesCv, EulerAngleConvention.XYZ);
        ModelCsPoint translationCv = new(-view[0], -view[1], -view[2]);
        extrisic = new Transformation3D<ModelCsPoint>(rotation, translation, rotationCv, translationCv);
    }

    /// <summary>
    /// 
    /// </summary>
    /// <exception cref="SerializerException"></exception>
    /// <param name="dirName"></param>
    /// <param name="cameraName"></param>
    public static CameraModel LoadCameraModel(string dirName, string cameraName)
    {
        string intrisicFile = Path.Join(dirName, $"Camera{cameraName}.dat");
        string extrisicFile = Path.Join(dirName, $"CameraView{cameraName}.dat");
        string distortionFile = Path.Join(dirName, $"CameraDistortion{cameraName}.dat");

        LoadExtrisicParameters(extrisicFile, out Transformation3D<ModelCsPoint> extrisic);
        LoadIntrisicParameters(intrisicFile, distortionFile, out IntrisicParameters intrisic, out Size imageSize, out Size matrixSize);
        return new CameraModel(extrisic, intrisic, imageSize, matrixSize);
    }
}
