using MathNet.Numerics.LinearAlgebra;

namespace RealRadarSim.Tracking
{
    public interface ITrackerFilter
    {
        Vector<double> State { get; set; }
        Matrix<double> Covariance { get; set; }
        void Predict(double dt);
        void Update(Vector<double> z);
        void Update(Vector<double> z, Matrix<double> customMeasurementCov);
        Vector<double> H(Vector<double> x);
        Matrix<double> S();
        Matrix<double> GetMeasurementNoiseCov();
    }
}
