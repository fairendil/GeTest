using MathNet.Numerics.LinearAlgebra;

namespace DigitalAssembly.Math.Common;

public sealed class Transformation3D<T>
    where T : Point3D<T>
{
    public Transformation3D(Rotation rotation, T point3D,
                            Rotation? rotationCV = null,
                            T? point3DCV = null)
    {
        Rotation = rotation;
        Point3D = point3D;
        TransformationMatrix = ComputeTransformation(Rotation,
                                                     Point3D,
                                                     Matrix<double>.Build.DenseDiagonal(4, 4, 1));
        RotationCV = rotationCV ?? rotation;
        Point3DCV = point3DCV ?? point3D;
        TransformationMatrixCV = ComputeTransformation(RotationCV,
                                                       Point3DCV,
                                                       Matrix<double>.Build.DenseDiagonal(4, 4, 1));
    }

    private Matrix<double> ComputeTransformation(Rotation? rotation,
                                                 T? point3D,
                                                 Matrix<double> matrix)
    {
        if (rotation is null || point3D is null)
        {
            return matrix;
        }

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                matrix[i, j] = rotation.Matrix[i, j];
            }

            matrix[i, 3] = point3D.Coordinate[i];
        }
        return matrix;
    }

    public Rotation Rotation { get; }
    public Rotation? RotationCV { get; }

    public T Point3D { get; }
    public T? Point3DCV { get; }

    public Matrix<double> TransformationMatrix { get; }
    public Matrix<double> TransformationMatrixCV { get; }

}
