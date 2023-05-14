using DigitalAssembly.Math.Common.Enums;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Spatial.Euclidean;
using MathNet.Spatial.Units;
using static System.Math;

namespace DigitalAssembly.Math.Common.Extentions;

public static class EulerRotationAngles
{
    public static Rotation ToEulerAngles(this Matrix<double> matrix, EulerAngleConvention convention)
    {
        if (matrix == null || matrix.ColumnCount != 3 || matrix.RowCount != 3)
        {
            throw new ArgumentException("Rotation matrix should be 3x3 not null matrix of rotation");
        }

        EulerAngles rotation = convention switch
        {
            EulerAngleConvention.ZYX => FromMatrixToZyx(matrix),
            EulerAngleConvention.ZYZ => FromMatrixToZyz(matrix),
            EulerAngleConvention.ZXY => FromMatrixToZxy(matrix),
            EulerAngleConvention.XYZ => FromMatrixToXyz(matrix),
            EulerAngleConvention.YXZ => FromMatrixToYxz(matrix),
            _ => throw new NotImplementedException()
        };

        return new Rotation(rotation, convention);
    }

    private static EulerAngles FromMatrixToZyx(Matrix<double> matrix)
    {
        if (matrix.RowCount != 3 && matrix.ColumnCount != 3)
        {
            throw new ArgumentException("Rotation matrix should be 3x3 not null matrix of rotation");
        }

        double sy = Sqrt((matrix[0, 0] * matrix[0, 0]) +
                         (matrix[1, 0] * matrix[1, 0]));
        bool singular = sy < 1e-6;
        double x, y, z;
        if (!singular)
        {
            x = Atan2(matrix[2, 1], matrix[2, 2]);
            y = Atan2(-matrix[2, 0], sy);
            z = Atan2(matrix[1, 0], matrix[0, 0]);
        }
        else
        {
            x = Atan2(-matrix[1, 2], matrix[1, 1]);
            y = Atan2(-matrix[2, 0], sy);
            z = 0;
        }

        EulerAngles angles = new(Angle.FromRadians(x), Angle.FromRadians(y), Angle.FromRadians(z));
        return angles;
    }

    private static EulerAngles FromMatrixToZxy(Matrix<double> matrix)
    {
        if (matrix.RowCount != 3 && matrix.ColumnCount != 3)
        {
            throw new ArgumentException("Rotation matrix should be 3x3 not null matrix of rotation");
        }

        double x, y, z;
        if (matrix[2, 1] < 1)
        {
            if (matrix[2, 1] > -1)
            {
                x = Asin(matrix[2, 1]);
                z = Atan2(-matrix[0, 1], matrix[1, 1]);
                y = Atan2(-matrix[2, 0], matrix[2, 2]);
            }
            else
            {
                x = -PI / 2;
                z = -Atan2(matrix[0, 2], matrix[0, 0]);
                y = 0;
            }
        }
        else
        {
            x = PI / 2;
            z = Atan2(matrix[0, 2], matrix[0, 0]);
            y = 0;
        }

        EulerAngles angles = new(Angle.FromRadians(x), Angle.FromRadians(y), Angle.FromRadians(z));
        return angles;
    }

    private static EulerAngles FromMatrixToYxz(Matrix<double> matrix)
    {
        if (matrix.RowCount != 3 && matrix.ColumnCount != 3)
        {
            throw new ArgumentException("Rotation matrix should be 3x3 not null matrix of rotation");
        }

        double x, y, z;
        if (matrix[1, 2] < 1)
        {
            if (matrix[1, 2] > -1)
            {
                x = Asin(-matrix[1, 2]);
                z = Atan2(matrix[1, 0], matrix[1, 1]);
                y = Atan2(matrix[0, 2], matrix[2, 2]);
            }
            else
            {
                x = PI / 2;
                z = -Atan2(-matrix[0, 1], matrix[0, 0]);
                y = 0;
            }
        }
        else
        {
            x = -PI / 2;
            y = Atan2(-matrix[0, 1], matrix[0, 0]);
            z = 0;
        }

        EulerAngles angles = new(Angle.FromRadians(x), Angle.FromRadians(y), Angle.FromRadians(z));
        return angles;
    }

    private static EulerAngles FromMatrixToXyz(Matrix<double> matrix)
    {
        if (matrix.RowCount != 3 && matrix.ColumnCount != 3)
        {
            throw new ArgumentException("Rotation matrix should be 3x3 not null matrix of rotation");
        }

        double x, y, z;
        if (matrix[0, 2] < 1)
        {
            if (matrix[0, 2] > -1)
            {
                y = Asin(matrix[0, 2]);
                x = Atan2(-matrix[1, 2], matrix[2, 2]);
                z = Atan2(-matrix[0, 1], matrix[0, 0]);
            }
            else
            {
                y = -PI / 2;
                x = -Atan2(matrix[1, 0], matrix[1, 1]);
                z = 0;
            }
        }
        else
        {
            y = PI / 2;
            x = Atan2(matrix[1, 0], matrix[1, 1]);
            z = 0;
        }

        EulerAngles angles = new(Angle.FromRadians(x), Angle.FromRadians(y), Angle.FromRadians(z));
        return angles;
    }

    private static EulerAngles FromMatrixToZyz(Matrix<double> matrix)
    {
        if (matrix.RowCount != 3 && matrix.ColumnCount != 3)
        {
            throw new ArgumentException("Rotation matrix should be 3x3 not null matrix of rotation");
        }

        double sy = Sqrt((matrix[2, 1] * matrix[2, 1]) +
                (matrix[2, 0] * matrix[2, 0]));

        bool singular = sy < 1e-6;

        double z1, y, z2;
        if (!singular)
        {
            z1 = Atan2(matrix[1, 2], matrix[0, 2]);
            y = Atan2(sy, matrix[2, 2]);
            z2 = Atan2(matrix[2, 1], -matrix[2, 0]);
        }
        else
        {
            z1 = Atan2(-matrix[0, 1], matrix[1, 1]);
            y = Atan2(sy, matrix[2, 2]);
            z2 = 0;
        }

        EulerAngles angles = new(Angle.FromRadians(z1), Angle.FromRadians(y), Angle.FromRadians(z2));
        return angles;
    }
}
