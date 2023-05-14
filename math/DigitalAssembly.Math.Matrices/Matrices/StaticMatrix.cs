using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;
using MathNet.Spatial.Euclidean;
using MathNet.Spatial.Units;
using System;
using static System.Math;

namespace DigitalAssembly.Math.Matrices;

public static partial class StaticMatrix
{
    public static Matrix<double> GetTransformationFromEulerAngles(this MatrixBuilder<double> M,
        Point3D pos, EulerAngles angles,
        NotationAngles notationAngles = NotationAngles.ZYX)
    {
        if (M is null)
        {
            throw new ArgumentNullException(nameof(M));
        }

        //calculate forward transform as succession of translations and rotations
        Matrix<double> MatrixRotation = Matrix<double>.Build.Dense(3, 3);

        //Calculate matrix rotation, use necessary notation
        switch (notationAngles)
        {
            case NotationAngles.XZX:
                MatrixRotation = Matrix3D.RotationAroundXAxis(Angle.FromRadians(angles.Alpha.Radians)) *
                            Matrix3D.RotationAroundZAxis(Angle.FromRadians(angles.Beta.Radians)) *
                            Matrix3D.RotationAroundXAxis(Angle.FromRadians(angles.Gamma.Radians));
                break;

            case NotationAngles.XYX:
                MatrixRotation = Matrix3D.RotationAroundXAxis(Angle.FromRadians(angles.Alpha.Radians)) *
                            Matrix3D.RotationAroundYAxis(Angle.FromRadians(angles.Beta.Radians)) *
                            Matrix3D.RotationAroundXAxis(Angle.FromRadians(angles.Gamma.Radians));
                break;

            case NotationAngles.YXY:
                MatrixRotation = Matrix3D.RotationAroundYAxis(Angle.FromRadians(angles.Alpha.Radians)) *
                            Matrix3D.RotationAroundXAxis(Angle.FromRadians(angles.Beta.Radians)) *
                            Matrix3D.RotationAroundYAxis(Angle.FromRadians(angles.Gamma.Radians));
                break;

            case NotationAngles.YZY:
                MatrixRotation = Matrix3D.RotationAroundYAxis(Angle.FromRadians(angles.Alpha.Radians)) *
                            Matrix3D.RotationAroundZAxis(Angle.FromRadians(angles.Beta.Radians)) *
                            Matrix3D.RotationAroundYAxis(Angle.FromRadians(angles.Gamma.Radians));
                break;

            case NotationAngles.ZYZ:
                MatrixRotation = Matrix3D.RotationAroundZAxis(Angle.FromRadians(angles.Alpha.Radians)) *
                            Matrix3D.RotationAroundYAxis(Angle.FromRadians(angles.Beta.Radians)) *
                            Matrix3D.RotationAroundZAxis(Angle.FromRadians(angles.Gamma.Radians));
                break;

            case NotationAngles.ZXZ:
                MatrixRotation = Matrix3D.RotationAroundZAxis(Angle.FromRadians(angles.Alpha.Radians)) *
                            Matrix3D.RotationAroundXAxis(Angle.FromRadians(angles.Beta.Radians)) *
                            Matrix3D.RotationAroundZAxis(Angle.FromRadians(angles.Gamma.Radians));
                break;

            case NotationAngles.XZY:
                MatrixRotation = Matrix3D.RotationAroundXAxis(Angle.FromRadians(angles.Alpha.Radians)) *
                            Matrix3D.RotationAroundZAxis(Angle.FromRadians(angles.Beta.Radians)) *
                            Matrix3D.RotationAroundYAxis(Angle.FromRadians(angles.Gamma.Radians));
                break;

            case NotationAngles.XYZ:
                MatrixRotation = Matrix3D.RotationAroundXAxis(Angle.FromRadians(angles.Alpha.Radians)) *
                            Matrix3D.RotationAroundYAxis(Angle.FromRadians(angles.Beta.Radians)) *
                            Matrix3D.RotationAroundZAxis(Angle.FromRadians(angles.Gamma.Radians));
                break;

            case NotationAngles.YXZ:
                MatrixRotation = Matrix3D.RotationAroundYAxis(Angle.FromRadians(angles.Alpha.Radians)) *
                            Matrix3D.RotationAroundXAxis(Angle.FromRadians(angles.Beta.Radians)) *
                            Matrix3D.RotationAroundZAxis(Angle.FromRadians(angles.Gamma.Radians));
                break;

            case NotationAngles.YZX:
                MatrixRotation = Matrix3D.RotationAroundYAxis(Angle.FromRadians(angles.Alpha.Radians)) *
                            Matrix3D.RotationAroundZAxis(Angle.FromRadians(angles.Beta.Radians)) *
                            Matrix3D.RotationAroundXAxis(Angle.FromRadians(angles.Gamma.Radians));
                break;

            case NotationAngles.ZYX:
                MatrixRotation = Matrix3D.RotationAroundZAxis(Angle.FromRadians(angles.Alpha.Radians)) *
                            Matrix3D.RotationAroundYAxis(Angle.FromRadians(angles.Beta.Radians)) *
                            Matrix3D.RotationAroundXAxis(Angle.FromRadians(angles.Gamma.Radians));
                break;

            case NotationAngles.ZXY:
                MatrixRotation = Matrix3D.RotationAroundZAxis(Angle.FromRadians(angles.Alpha.Radians)) *
                            Matrix3D.RotationAroundXAxis(Angle.FromRadians(angles.Beta.Radians)) *
                            Matrix3D.RotationAroundYAxis(Angle.FromRadians(angles.Gamma.Radians));
                break;
        }

        VectorBuilder<double> V = Vector<double>.Build;

        //Add 4th column with XYZ
        Vector<double> VectorXYZ = V.Dense(new double[] { pos.X, pos.Y, pos.Z });

        //Add 4th row for homogeneous matrix
        Vector<double> ZerosColumn = V.Dense(new double[] { 0, 0, 0, 1 });

        return MatrixRotation.InsertColumn(3, VectorXYZ)
                .InsertRow(3, ZerosColumn);
    }

    public static Matrix<double> GetTransformationFromQuaternion(this MatrixBuilder<double> M,
        Point3D pos, Quaternion quat)
    {
        Matrix<double> matrixOut = M.Dense(4, 4);

        matrixOut[0, 0] = 1 - 2 * quat.ImagY * quat.ImagY - 2 * quat.ImagZ * quat.ImagZ;
        matrixOut[0, 1] = 2 * quat.ImagX * quat.ImagY - 2 * quat.ImagZ * quat.Real;
        matrixOut[0, 2] = 2 * quat.ImagX * quat.ImagZ + 2 * quat.ImagY * quat.Real;
        matrixOut[0, 3] = pos.X;

        matrixOut[1, 0] = 2 * quat.ImagX * quat.ImagY + 2 * quat.ImagZ * quat.Real;
        matrixOut[1, 1] = 1 - 2 * quat.ImagX * quat.ImagX - 2 * quat.ImagZ * quat.ImagZ;
        matrixOut[1, 2] = 2 * quat.ImagY * quat.ImagZ - 2 * quat.ImagX * quat.Real;
        matrixOut[1, 3] = pos.Y;

        matrixOut[2, 0] = 2 * quat.ImagX * quat.ImagZ - 2 * quat.ImagY * quat.Real;
        matrixOut[2, 1] = 2 * quat.ImagY * quat.ImagZ + 2 * quat.ImagX * quat.Real;
        matrixOut[2, 2] = 1 - 2 * quat.ImagX * quat.ImagX - 2 * quat.ImagY * quat.ImagY;
        matrixOut[2, 3] = pos.Z;

        matrixOut[3, 3] = 1;

        return matrixOut;
    }

    public static Matrix<double> GetTransformationFromAxisAnlge(this MatrixBuilder<double> M,
        Point3D pos, AxisAngle axisAngle)
    {
        Quaternion quatFromAxisAngle = axisAngle.GetQuaternion();

        return M.GetTransformationFromQuaternion(pos, quatFromAxisAngle);
    }

    public static Matrix<double> GetTransformationFromModernDH(this MatrixBuilder<double> M,
        double alpha, double a, double d, double theta)
    {
        VectorBuilder<double> V = Vector<double>.Build;
        Vector<double> ZerosRow = V.Dense(new double[] { 0, 0, 0, 1 });
        Vector<double> ZerosColumn = V.Dense(new double[] { 0, 0, 0 });

        Matrix<double> matRotX = Matrix3D.RotationAroundXAxis(Angle.FromRadians(alpha)).InsertColumn(3, ZerosColumn)
                .InsertRow(3, ZerosRow);

        Matrix<double> matTransX = M.DenseIdentity(4, 4);
        matTransX[0, 3] = a;

        Matrix<double> matTransZ = M.DenseIdentity(4, 4);
        matTransZ[2, 3] = d;

        Matrix<double> matRotZ = Matrix3D.RotationAroundZAxis(Angle.FromRadians(theta)).InsertColumn(3, ZerosColumn)
                .InsertRow(3, ZerosRow);

        return matRotX * matTransX * matTransZ * matRotZ;
    }

    public static Matrix<double> GetTransformationFromClassicDH(this MatrixBuilder<double> M,
        double theta, double d, double a, double alpha)
    {
        VectorBuilder<double> V = Vector<double>.Build;
        Vector<double> ZerosRow = V.Dense(new double[] { 0, 0, 0, 1 });
        Vector<double> ZerosColumn = V.Dense(new double[] { 0, 0, 0 });

        Matrix<double> matRotZ = Matrix3D.RotationAroundZAxis(Angle.FromRadians(theta)).InsertColumn(3, ZerosColumn)
                .InsertRow(3, ZerosRow);

        Matrix<double> matTransZ = M.DenseIdentity(4, 4);
        matTransZ[2, 3] = d;

        Matrix<double> matTransX = M.DenseIdentity(4, 4);
        matTransX[0, 3] = a;

        Matrix<double> matRotX = Matrix3D.RotationAroundXAxis(Angle.FromRadians(alpha)).InsertColumn(3, ZerosColumn)
                .InsertRow(3, ZerosRow);

        return matRotZ * matTransZ * matTransX * matRotX;
    }

    public static Matrix<double> GetTransformationFromExtendedDH(this MatrixBuilder<double> M,
        double theta, double d, double a, double b, double alpha, double beta)
    {
        VectorBuilder<double> V = Vector<double>.Build;
        Vector<double> ZerosRow = V.Dense(new double[] { 0, 0, 0, 1 });
        Vector<double> ZerosColumn = V.Dense(new double[] { 0, 0, 0 });

        Matrix<double> matRotZ = Matrix3D.RotationAroundZAxis(Angle.FromRadians(theta)).InsertColumn(3, ZerosColumn)
                .InsertRow(3, ZerosRow);

        Matrix<double> matTransX = M.DenseIdentity(4, 4);
        matTransX[0, 3] = a;

        Matrix<double> matTransY = M.DenseIdentity(4, 4);
        matTransY[1, 3] = b;

        Matrix<double> matTransZ = M.DenseIdentity(4, 4);
        matTransZ[2, 3] = d;

        Matrix<double> matRotX = Matrix3D.RotationAroundXAxis(Angle.FromRadians(alpha)).InsertColumn(3, ZerosColumn)
                .InsertRow(3, ZerosRow);

        Matrix<double> matRotY = Matrix3D.RotationAroundYAxis(Angle.FromRadians(beta)).InsertColumn(3, ZerosColumn)
                .InsertRow(3, ZerosRow);

        return matRotZ * matTransZ * matTransX * matTransY * matRotX * matRotY;
    }

    public static EulerAngles GetEulerAngles(this Matrix<double> rotationMatrix,
        NotationAngles notationAngles = NotationAngles.ZYX)
    {
        if ((rotationMatrix.RowCount == 3) && (rotationMatrix.ColumnCount == 3))
        {
            double sy;

            
            switch (notationAngles)
            {
                case NotationAngles.ZYX:
                    sy = Sqrt((rotationMatrix[0, 0] * rotationMatrix[0, 0]) +
                    (rotationMatrix[1, 0] * rotationMatrix[1, 0]));
                    break;
                case NotationAngles.ZYZ:
                    sy = Sqrt((rotationMatrix[2, 1] * rotationMatrix[2, 1]) +
                    (rotationMatrix[2, 0] * rotationMatrix[2, 0]));
                    break;
                case NotationAngles.XYZ:
                    sy = Sqrt((rotationMatrix[0, 0] * rotationMatrix[0, 0]) +
                    (rotationMatrix[0, 1] * rotationMatrix[0, 1]));
                    break;
                default:
                    sy = 1;
                    break;
            }

            bool singular = sy < 1e-6;


            Angle angle1 = new Angle();
            Angle angle2 = new Angle();
            Angle angle3 = new Angle();
            //Singular case is considered only notations:
            //ZYZ, ZYX, XYZ - most popular and have singularity result in web
            if (!singular)
            {
                switch (notationAngles)
                {
                    case NotationAngles.XZX:
                        angle1 = Angle.FromRadians(Atan2(rotationMatrix[2, 0], rotationMatrix[1, 0]));
                        angle2 = Angle.FromRadians(Acos(rotationMatrix[0, 0]));
                        angle3 = Angle.FromRadians(Atan2(rotationMatrix[0, 2], -rotationMatrix[0, 1]));
                        break;

                    case NotationAngles.XYX:
                        angle1 = Angle.FromRadians(Atan2(rotationMatrix[1, 0], -rotationMatrix[2, 0]));
                        angle2 = Angle.FromRadians(Acos(rotationMatrix[0, 0]));
                        angle3 = Angle.FromRadians(Atan2(rotationMatrix[0, 1], rotationMatrix[0, 2]));
                        break;

                    case NotationAngles.YXY:
                        angle1 = Angle.FromRadians(Atan2(rotationMatrix[0, 1], rotationMatrix[2, 1]));
                        angle2 = Angle.FromRadians(Acos(rotationMatrix[1, 1]));
                        angle3 = Angle.FromRadians(Atan2(rotationMatrix[1, 0], -rotationMatrix[1, 2]));
                        break;

                    case NotationAngles.YZY:
                        angle1 = Angle.FromRadians(Atan2(rotationMatrix[2, 1], -rotationMatrix[0, 1]));
                        angle2 = Angle.FromRadians(Acos(rotationMatrix[1, 1]));
                        angle3 = Angle.FromRadians(Atan2(rotationMatrix[1, 2], rotationMatrix[1, 0]));
                        break;

                    case NotationAngles.ZYZ:
                        angle1 = Angle.FromRadians(Atan2(rotationMatrix[1, 2], rotationMatrix[0, 2]));
                        angle2 = Angle.FromRadians(Atan2(sy, rotationMatrix[2, 2]));
                        angle3 = Angle.FromRadians(Atan2(rotationMatrix[2, 1], -rotationMatrix[2, 0]));
                        break;

                    case NotationAngles.ZXZ:
                        angle1 = Angle.FromRadians(Atan2(rotationMatrix[0, 2], -rotationMatrix[1, 2]));
                        angle2 = Angle.FromRadians(Acos(rotationMatrix[2, 2]));
                        angle3 = Angle.FromRadians(Atan2(rotationMatrix[2, 0], rotationMatrix[2, 1]));
                        break;

                    case NotationAngles.XZY:
                        angle1 = Angle.FromRadians(Atan2(rotationMatrix[2, 1], rotationMatrix[1, 1]));
                        angle2 = Angle.FromRadians(Asin(-rotationMatrix[0, 1]));
                        angle3 = Angle.FromRadians(Atan2(rotationMatrix[0, 2], rotationMatrix[0, 0]));
                        break;

                    case NotationAngles.XYZ:
                        angle1 = Angle.FromRadians(Atan2(-rotationMatrix[1, 2], rotationMatrix[2, 2]));
                        angle2 = Angle.FromRadians(Atan2(rotationMatrix[0, 2], sy));
                        angle3 = Angle.FromRadians(Atan2(-rotationMatrix[0, 1], rotationMatrix[0, 0]));
                        break;

                    case NotationAngles.YXZ:
                        angle1 = Angle.FromRadians(Atan2(rotationMatrix[0, 2], rotationMatrix[2, 2]));
                        angle2 = Angle.FromRadians(Asin(-rotationMatrix[1, 2]));
                        angle3 = Angle.FromRadians(Atan2(rotationMatrix[1, 0], rotationMatrix[1, 1]));
                        break;

                    case NotationAngles.YZX:
                        angle1 = Angle.FromRadians(Atan2(-rotationMatrix[2, 0], rotationMatrix[0, 0]));
                        angle2 = Angle.FromRadians(Asin(-rotationMatrix[1, 0]));
                        angle3 = Angle.FromRadians(Atan2(-rotationMatrix[1, 2], rotationMatrix[1, 1]));
                        break;

                    case NotationAngles.ZYX:
                        angle1 = Angle.FromRadians(Atan2(rotationMatrix[1, 0], rotationMatrix[0, 0]));
                        angle2 = Angle.FromRadians(Atan2(-rotationMatrix[2, 0], sy));
                        angle3 = Angle.FromRadians(Atan2(rotationMatrix[2, 1], rotationMatrix[2, 2]));
                        break;

                    case NotationAngles.ZXY:
                        angle1 = Angle.FromRadians(Atan2(-rotationMatrix[0, 1], rotationMatrix[1, 1]));
                        angle2 = Angle.FromRadians(Asin(rotationMatrix[2, 1]));
                        angle3 = Angle.FromRadians(Atan2(-rotationMatrix[2, 0], rotationMatrix[2, 2]));
                        break;
                }
            }
            else
            {
                switch (notationAngles)
                {
                    case NotationAngles.ZYX:
                        angle1 = Angle.FromRadians(0);
                        angle2 = Angle.FromRadians(Atan2(-rotationMatrix[2, 0], sy));
                        angle3 = Angle.FromRadians(Atan2(-rotationMatrix[1, 2], rotationMatrix[1, 1]));
                        break;
                    case NotationAngles.ZYZ:
                        angle1 = Angle.FromRadians(Atan2(-rotationMatrix[0, 1], rotationMatrix[1, 1]));
                        angle2 = Angle.FromRadians(Atan2(sy, rotationMatrix[2, 2]));
                        angle3 = Angle.FromRadians(0);
                        break;
                    case NotationAngles.XYZ:
                        angle1 = Angle.FromRadians(Atan2(-rotationMatrix[1, 2], rotationMatrix[1, 1]));
                        angle2 = Angle.FromRadians(Atan2(rotationMatrix[0, 2], sy));
                        angle3 = Angle.FromRadians(0);
                        break;
                }
            }

            return new EulerAngles(angle1, angle2, angle3);
        }
        else
        {
            return new EulerAngles();
        }
    }

    public static Quaternion GetQuaternion(this Matrix<double> rotationMatrix)
    {
        if ((rotationMatrix.RowCount == 3) && (rotationMatrix.ColumnCount == 3))
        {
            double K11 = rotationMatrix[0, 0] - rotationMatrix[1, 1] - rotationMatrix[2, 2];
            double K12 = rotationMatrix[0, 1] + rotationMatrix[1, 0];
            double K13 = rotationMatrix[0, 2] + rotationMatrix[2, 0];
            double K14 = rotationMatrix[2, 1] - rotationMatrix[1, 2];

            double K22 = rotationMatrix[1, 1] - rotationMatrix[0, 0] - rotationMatrix[2, 2];
            double K23 = rotationMatrix[1, 2] + rotationMatrix[2, 1];
            double K24 = rotationMatrix[0, 2] - rotationMatrix[2, 0];

            double K33 = rotationMatrix[2, 2] - rotationMatrix[0, 0] - rotationMatrix[1, 1];
            double K34 = rotationMatrix[1, 0] - rotationMatrix[0, 1];

            double K44 = rotationMatrix[0, 0] + rotationMatrix[1, 1] + rotationMatrix[2, 2];

            Matrix<double> K = Matrix<double>.Build.DenseOfArray(new double[4, 4]
            {
                        {K11,   K12,    K13,    K14},
                        {K12,   K22,    K23,    K24},
                        {K13,   K23,    K33,    K34},
                        {K14,   K24,    K34,    K44}
            }) / 3;

            Evd<double> eigVal = K.Evd();

            int indMax = eigVal.EigenValues.Real().AbsoluteMaximumIndex();

            Quaternion quatOut = new Quaternion(
                eigVal.EigenVectors[3, indMax],
                eigVal.EigenVectors[0, indMax],
                eigVal.EigenVectors[1, indMax],
                eigVal.EigenVectors[2, indMax]);

            if (quatOut.Real < 0)
            {
                return quatOut.Negate();
            }
            else
            {
                return quatOut;
            }
        }
        else
        {
            return new Quaternion();
        }
    }

    public static AxisAngle GetAxisAngle(this Matrix<double> rotationMatrix)
    {
        if ((rotationMatrix.RowCount == 3) && (rotationMatrix.ColumnCount == 3))
        {
            Quaternion quatFromRotationMatrix = rotationMatrix.GetQuaternion();
            return AxisAngle.FromQuaternion(quatFromRotationMatrix);
        }
        else
        {
            return new AxisAngle();
        }
    }

    public static (Point3D, EulerAngles) GetCoordinatesAndEulerAngles(this Matrix<double> transformationMatrix,
        NotationAngles notationAngles = NotationAngles.ZYX)
    {
        if ((transformationMatrix.RowCount == 4) && (transformationMatrix.ColumnCount == 4))
        {
            Matrix<double> rotationMatrix = transformationMatrix.SubMatrix(0, 3, 0, 3);

            EulerAngles eulerAnglesOut = GetEulerAngles(rotationMatrix, notationAngles);

            Point3D posOut = new Point3D(
                transformationMatrix[0, 3],
                transformationMatrix[1, 3],
                transformationMatrix[2, 3]);

            return (posOut, eulerAnglesOut);
        }
        else
        {
            return (new Point3D(), new EulerAngles());
        }
    }

    public static (Point3D, Quaternion) GetCoordinatesAndQuaternion(this Matrix<double> transformationMatrix)
    {
        if ((transformationMatrix.RowCount == 4) && (transformationMatrix.ColumnCount == 4))
        {
            Matrix<double> rotationMatrix = transformationMatrix.SubMatrix(0, 3, 0, 3);

            Quaternion quaternionOut = GetQuaternion(rotationMatrix);

            Point3D posOut = new Point3D(
                transformationMatrix[0, 3],
                transformationMatrix[1, 3],
                transformationMatrix[2, 3]);

            return (posOut, quaternionOut);
        }
        else
        {
            return (new Point3D(), new Quaternion());
        }
    }

    public static (Point3D, AxisAngle) GetCoordinatesAndAxisAngle(this Matrix<double> transformationMatrix)
    {
        if ((transformationMatrix.RowCount == 4) && (transformationMatrix.ColumnCount == 4))
        {
            Matrix<double> rotationMatrix = transformationMatrix.SubMatrix(0, 3, 0, 3);

            AxisAngle axisAngleOut = GetAxisAngle(rotationMatrix);

            Point3D posOut = new Point3D(
                transformationMatrix[0, 3],
                transformationMatrix[1, 3],
                transformationMatrix[2, 3]);

            return (posOut, axisAngleOut);
        }
        else
        {
            return (new Point3D(), new AxisAngle());
        }
    }

}
