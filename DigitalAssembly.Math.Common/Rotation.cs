using DigitalAssembly.Math.Common.Enums;
using DigitalAssembly.Math.Common.Extentions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Spatial.Euclidean;
using System.Diagnostics;
using static MathNet.Spatial.Euclidean.Matrix3D;

namespace DigitalAssembly.Math.Common;

public sealed class Rotation
{
    /// <summary>
    /// Rotation in euler angle with chosen convention
    /// </summary>
    /// <param name="eulerAngles">X-Y-Z order</param>
    /// <param name="convention"></param>
    public Rotation(EulerAngles eulerAngles, EulerAngleConvention convention)
    {
        EulerAngles = eulerAngles;
        Convention = convention;
        Matrix = RotationMatrix();
    }

    public EulerAngles EulerAngles { get; }

    public EulerAngleConvention Convention { get; }

    public Matrix<double> Matrix { get; }

    private Matrix<double> RotationMatrix()
    {
        switch (Convention)
        {
            case EulerAngleConvention.ZYX:
                {
                    return RotationAroundZAxis(EulerAngles.Gamma) *
                        RotationAroundYAxis(EulerAngles.Beta) *
                        RotationAroundXAxis(EulerAngles.Alpha);
                }
            case EulerAngleConvention.ZYZ:
                {
                    return RotationAroundZAxis(EulerAngles.Gamma) *
                        RotationAroundYAxis(EulerAngles.Beta) *
                        RotationAroundZAxis(EulerAngles.Gamma);
                }
            case EulerAngleConvention.ZXY:
                {
                    return RotationAroundZAxis(EulerAngles.Gamma) *
                        RotationAroundXAxis(EulerAngles.Alpha) *
                        RotationAroundYAxis(EulerAngles.Beta);
                }
            case EulerAngleConvention.XYZ:
                {
                    return RotationAroundXAxis(EulerAngles.Alpha) *
                        RotationAroundYAxis(EulerAngles.Beta) *
                        RotationAroundZAxis(EulerAngles.Gamma);
                }
            case EulerAngleConvention.YXZ:
                {
                    return RotationAroundYAxis(EulerAngles.Beta) *
                        RotationAroundXAxis(EulerAngles.Alpha) *
                        RotationAroundZAxis(EulerAngles.Gamma);
                }
            default:
                throw new UnreachableException();
        }
    }

    /// <summary>
    /// Parse rotation matrix into Euler angles
    /// </summary>
    /// <param name="rotationMatrix"></param>
    /// <param name="convention"></param>
    /// <returns></returns>
    /// <exception cref="ArgumentException">If matrix not 3x3</exception>
    public static Rotation FromRotationMatrix(Matrix<double> rotationMatrix, EulerAngleConvention convention)
    {
        return rotationMatrix == null || rotationMatrix.ColumnCount != 3 || rotationMatrix.RowCount != 3
            ? throw new ArgumentException("Rotation matrix should be 3x3 not null matrix of homography")
            : rotationMatrix.ToEulerAngles(convention);
    }
}
