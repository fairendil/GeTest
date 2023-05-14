using DigitalAssembly.Math.Common;

namespace DigitalAssembly.Math.PointClouds.Transformation;

public interface IPcTransformation
{
    /// <summary>
    /// Вычисление матрицы перехода от одного облака точек (системы координат) к другому (другой системе координат)
    /// </summary>
    /// <typeparam name="PT">Point of type Point3D</typeparam>
    /// <param name="initialPoints"></param>
    /// <param name="targetPoints"></param>
    /// <returns></returns>
    public TransformationFittingResult Estimate<PT>(List<PT> initialPoints, List<PT> targetPoints)
       where PT : Point3D<PT>;
}
