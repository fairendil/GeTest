using DigitalAssembly.Math.Common;
using MathNet.Numerics.LinearAlgebra;
using static System.Math;

namespace DigitalAssembly.Math.PointClouds.Classification;

public class ChineseСrosswordClassification : IPcClassification
{
    private readonly double Tolerance;
    private readonly double Similarity;

    public ChineseСrosswordClassification(double tolerance, double similarity)
    {
        Tolerance = tolerance;
        Similarity = similarity;
    }

    /// <summary>
    /// Выделение соответствующих друг другу точек на исходном и целевом облаках точек. Выбираем из точек только те, которые видны среди targetPoints
    /// </summary>
    /// <typeparam name="PT">Point of type Point3D</typeparam>
    /// <param name="initialPoints"></param>
    /// <param name="targetPoints"></param>
    /// <returns></returns>
    public (List<PT> initialPointsFound, List<PT> targetPointsFound) SelectPoints<PT>(List<PT> initialPoints, List<PT> targetPoints)
        where PT : Point3D<PT>
    {
        List<(int initialIndex, int targetIndex)> pairs = EstimatePairs(initialPoints, targetPoints);
        List<PT> initialPointsFound = new();
        List<PT> targetPointsFound = new();
        for (int i = 0; i < pairs.Count; i++)
        {
            initialPointsFound.Add(initialPoints[pairs[i].initialIndex]);
            targetPointsFound.Add(targetPoints[pairs[i].targetIndex]);
        }

        return (initialPointsFound, targetPointsFound);
    }

    // TODO: research for another pairing algorithm
    /// <summary>
    /// "Китайский кроссворд" (выбираем самую "похожую" точку: если соответствующие расстояния между точками совпадают с точностью до tolerance, точки "похожи", остальные зануляем),
    /// </summary>
    /// <typeparam name="PT">Point of type Point3D</typeparam>
    /// <param name="initialPoints"></param>
    /// <param name="targetPoints"></param>
    /// <returns></returns>
    private List<(int initialIndex, int targetIndex)> EstimatePairs<PT>(List<PT> initialPoints, List<PT> targetPoints)
        where PT : Point3D<PT>
    {
        List<(int initialIndex, int targetIndex)> resultPairs = new();

        int srcPtsCount = initialPoints.Count;
        int tgtPtsCount = targetPoints.Count;
        // матрица "похожести"
        Vector<double> similarityMatrix = Vector<double>.Build.Dense(srcPtsCount * tgtPtsCount);

        Matrix<double> srcDist = Matrix<double>.Build.Dense(srcPtsCount, srcPtsCount);
        Matrix<double> tgtDist = Matrix<double>.Build.Dense(tgtPtsCount, tgtPtsCount);

        for (int i = 0; i < srcPtsCount; i++)
        {
            for (int j = 0; j < srcPtsCount; j++)
            {
                srcDist[i, j] = (initialPoints[i] - initialPoints[j]).L2Norm();
            }
        }

        for (int i = 0; i < tgtPtsCount; i++)
        {
            for (int j = 0; j < tgtPtsCount; j++)
            {
                tgtDist[i, j] = (targetPoints[i] - targetPoints[j]).L2Norm();
            }
        }

        // заполнение матрицы "похожести" (если соответствующие расстояния между точками совпадают с точностью до tolerance, точки "похожи")
        for (int i = 0; i < srcPtsCount; i++)
        {
            Vector<double> vSrc = srcDist.Row(i);
            for (int j = 0; j < tgtPtsCount; j++)
            {
                Vector<double> vTgt = tgtDist.Row(j);
                double count = 0;
                for (int k = 0; k < vSrc.Count; k++)
                {
                    if (k == i || vTgt.Count == 0)
                    {
                        continue;
                    }

                    double x = vSrc[k];
                    Vector<double> vecTgtCopy = Vector<double>.Build.Dense(vTgt.Count);
                    vTgt.CopyTo(vecTgtCopy);
                    for (int l = 0; l < vecTgtCopy.Count; l++)
                    {
                        vecTgtCopy[l] = Abs(vecTgtCopy[l] - x);
                    }

                    double minElement = vecTgtCopy.Min();
                    if (Abs(minElement) < Tolerance)
                    {
                        count += 1.0 - (minElement / Tolerance);
                    }
                }

                similarityMatrix[(i * tgtPtsCount) + j] = count;
            }
        }

        // нормирование
        double maxElement = 0;
        for (int i = 0; i < srcPtsCount; i++)
        {
            for (int j = 0; j < tgtPtsCount; j++)
            {
                if (similarityMatrix[(i * tgtPtsCount) + j] > maxElement)
                {
                    maxElement = similarityMatrix[(i * tgtPtsCount) + j];
                }
            }
        }

        for (int i = 0; i < srcPtsCount; i++)
        {
            for (int j = 0; j < tgtPtsCount; j++)
            {
                similarityMatrix[(i * tgtPtsCount) + j] /= maxElement;
            }
        }

        // "китайский кроссворд"
        for (int i = 0; i < srcPtsCount; i++)
        {
            for (int j = 0; j < tgtPtsCount; j++)
            {
                if (similarityMatrix[i * tgtPtsCount + j] > Similarity)
                {
                    resultPairs.Add((i, j));
                    for (int l = 0; l < tgtPtsCount; l++)
                    {
                        similarityMatrix[i * tgtPtsCount + l] = 0.0;
                    }

                    for (int k = 0; k < srcPtsCount; k++)
                    {
                        similarityMatrix[k * tgtPtsCount + j] = 0.0;
                    }

                    continue;
                }
            }
        }

        return resultPairs;
    }
}
