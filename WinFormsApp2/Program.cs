using System;
using System.Windows.Forms;
using RealRadarSim.Forms;

namespace RealRadarSim
{
    static class Program
    {
        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new RadarForm());
        }
    }
}
