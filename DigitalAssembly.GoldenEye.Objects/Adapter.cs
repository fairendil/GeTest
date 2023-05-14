using DigitalAssembly.GoldenEye.DB;
using MathNet.Numerics.LinearAlgebra;

namespace DigitalAssembly.GoldenEye.Objects;

public class Adapter: MovableObject
{ 
    public Adapter(AdapterModel model): base(model)
    {
    }

    public Adapter UpdateTransformation(Matrix<double> transformation)
    {
        Adapter result = new(new(ModelName, Points, TCP));
        result.SetTransformation(transformation);
        return result;
    }
}
