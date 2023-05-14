using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using Aladdin.HASP.Envelope;

namespace DigitalAssembly.Math.Primitives;

public class Point
{

    public double X { get; set; }

    public double Y { get; set; }

    public double Z { get; set; }

    public Point()
    {
        X = 0;
        Y = 0;
        Z = 0;
    }
    public Point(double xIn, double yIn, double zIn)
    {
        X = xIn;
        Y = yIn;
        Z = zIn;
    }

    public static Dictionary<string, Point> FromFileWithNames(string fileName)
    {
        Dictionary<string, Point> mapPoints = new();
        FileStream file1 = new(fileName, FileMode.Open);
        StreamReader reader = new(file1);
        string[] dataFromFile = reader.ReadToEnd().Split("\r\n", StringSplitOptions.RemoveEmptyEntries);

        int numPoints = dataFromFile.Length;

        for (int i = 0; i < numPoints; i++)
        {
            string[] dataPoint = dataFromFile[i]
                .Replace(".",",")
                .Split(" ", StringSplitOptions.RemoveEmptyEntries);

            TryGetDouble(dataPoint[1], out double tempPointX);
            TryGetDouble(dataPoint[2], out double tempPointY);
            TryGetDouble(dataPoint[3], out double tempPointZ);

            Point tempPoint = new(tempPointX, tempPointY, tempPointZ);

            mapPoints.Add(dataPoint[0], tempPoint);
        }

        reader.Close();

        return mapPoints;
    }

    private static bool TryGetDouble(string value, out double outValue)
    {
        //Try parsing in the current culture
        if (!double.TryParse(value, NumberStyles.Any, CultureInfo.CurrentCulture, out outValue) &&
            //Then try in US english
            !double.TryParse(value, NumberStyles.Any, CultureInfo.GetCultureInfo("en-US"), out outValue) &&
            //Then in neutral language
            !double.TryParse(value, NumberStyles.Any, CultureInfo.InvariantCulture, out outValue))
        {
            outValue = double.NaN;
            return false;
        }

        return true;
    }

    public static Point[] FromFile(string fileName)
    {
        FileStream file1 = new(fileName, FileMode.Open);
        StreamReader reader = new(file1);
        string[] dataFromFile = reader.ReadToEnd().Split("\n", StringSplitOptions.RemoveEmptyEntries);

        int numPoints = dataFromFile.Length;
        Point[] PointsOut = new Point[numPoints];

        for (int i = 0; i < numPoints; i++)
        {
            string[] dataPoint = dataFromFile[i]
                .Replace(".", ",")
                .Split(" ", StringSplitOptions.RemoveEmptyEntries);

            TryGetDouble(dataPoint[0], out double tempPointX);
            TryGetDouble(dataPoint[1], out double tempPointY);
            TryGetDouble(dataPoint[2], out double tempPointZ);

            PointsOut[i] = new Point(
                tempPointX, tempPointY,
                tempPointZ);
        }

        reader.Close();

        return PointsOut;
    }

    public static void ExportFileWithNames(
        Dictionary<string, Point> PointIn, string fileName)
    {
        FileStream file1 = new(fileName, FileMode.OpenOrCreate);
        file1.SetLength(0);
        using (StreamWriter writer = new(file1, Encoding.UTF8))
        {
            foreach (KeyValuePair<string, Point> point in PointIn)
            {
                writer.WriteLine($"{point.Key} ".Replace(",", ".") +
                    $"{point.Value.X} {point.Value.Y} ".Replace(",", ".") +
                    $"{point.Value.Z}".Replace(",","."));
            }
        }

        file1.Close();
    }
}
