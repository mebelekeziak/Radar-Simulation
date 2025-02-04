using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Windows.Forms;
using MathNet.Numerics.LinearAlgebra;
using RealRadarSim.Engine;
using RealRadarSim.Models;
using RealRadarSim.Tracking;

namespace RealRadarSim.Forms
{


    public partial class RadarForm : Form
    {
        private SimulationEngine engine;
        private System.Windows.Forms.Timer simulationTimer;
        private double scale;
        private int radarDisplayRadius;

        // Reusable drawing objects
        private Pen circlePen = new Pen(Color.Green, 1);
        private Pen crossPen = new Pen(Color.Green, 1);
        private Pen sweepPen = new Pen(Color.Lime, 2);
        private Font trackFont = new Font("Arial", 10);
        private Brush textBrush = new SolidBrush(Color.White);
        private Brush targetDetailBrush = new SolidBrush(Color.LimeGreen);
        private Pen headingLinePen = new Pen(Color.Yellow, 2);
        private Image planeImage;

        // Mouse tracking
        private Point mousePos;

        // Zooming
        private float zoomFactor = 1.0f;
        private const float zoomStep = 0.1f;
        private const float minZoom = 0.1f;
        private const float maxZoom = 5.0f;

        // Panning
        private float panX = 0.0f;
        private float panY = 0.0f;
        private bool isPanning = false;
        private Point lastMousePos;

        // Debug mode flag
        private bool debugMode = false;

        public RadarForm()
        {
            InitializeComponent();

            // Load the plane image (ensure plane.png exists in your executable’s folder)
            try
            {
                planeImage = Image.FromFile("plane.png");
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error loading plane.png: " + ex.Message);
                Environment.Exit(1);
            }

            // Initialize the simulation engine (with Lua support inside SimulationEngine)
            engine = new SimulationEngine(new Random());

            // Set up simulation timer
            simulationTimer = new System.Windows.Forms.Timer();
            simulationTimer.Interval = (int)(engine.GetDt() * 1000);
            simulationTimer.Tick += SimulationTimer_Tick;
            simulationTimer.Start();

            // Form properties
            this.DoubleBuffered = true;
            this.Width = 1200;
            this.Height = 1000;
            this.Text = "Radar";
            this.BackColor = Color.Black;

            // Calculate display scale based on radar max range.
            radarDisplayRadius = Math.Min(this.ClientSize.Width, this.ClientSize.Height) / 2 - 20;
            scale = radarDisplayRadius / engine.GetMaxRange();

            // Subscribe to mouse and key events for zooming and panning.
            this.MouseMove += Form_MouseMove;
            this.MouseWheel += Form_MouseWheel;
            this.KeyDown += Form_KeyDown;
            this.MouseDown += Form_MouseDown;
            this.MouseUp += Form_MouseUp;


        }

        private void Form_MouseDown(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
            {
                isPanning = true;
                lastMousePos = e.Location;
            }
        }

        private void Form_MouseUp(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
                isPanning = false;
        }

        private void Form_MouseMove(object sender, MouseEventArgs e)
        {
            mousePos = e.Location;
            if (isPanning)
            {
                int dx = e.X - lastMousePos.X;
                int dy = e.Y - lastMousePos.Y;
                panX += dx;
                panY += dy;
                lastMousePos = e.Location;
                this.Invalidate();
            }
            else
            {
                this.Invalidate();
            }
        }

        private void Form_MouseWheel(object sender, MouseEventArgs e)
        {
            if (e.Delta > 0)
            {
                zoomFactor += zoomStep;
                if (zoomFactor > maxZoom)
                    zoomFactor = maxZoom;
            }
            else
            {
                zoomFactor -= zoomStep;
                if (zoomFactor < minZoom)
                    zoomFactor = minZoom;
            }
            this.Invalidate();
        }

        private void Form_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.Oemplus)
            {
                zoomFactor += zoomStep;
                if (zoomFactor > maxZoom)
                    zoomFactor = maxZoom;
                this.Invalidate();
            }
            else if (e.KeyCode == Keys.OemMinus)
            {
                zoomFactor -= zoomStep;
                if (zoomFactor < minZoom)
                    zoomFactor = minZoom;
                this.Invalidate();
            }
            // Toggle debug mode with the D key
            else if (e.KeyCode == Keys.D)
            {
                debugMode = !debugMode;
                this.Invalidate();
            }
        }

        private void SimulationTimer_Tick(object sender, EventArgs e)
        {
            engine.Update();
            this.Invalidate();
        }

        private void RadarForm_Load(object sender, EventArgs e)
        {
            // Kod inicjalizacyjny
        }


        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            Graphics g = e.Graphics;
            int cx = this.ClientSize.Width / 2;
            int cy = this.ClientSize.Height / 2;

            // Apply transforms for center, panning, and zooming.
            g.TranslateTransform(cx, cy);
            g.TranslateTransform(panX, panY);
            g.ScaleTransform(zoomFactor, zoomFactor);

            // Draw range circles.
            for (int i = 1; i <= 4; i++)
            {
                int r = (radarDisplayRadius * i) / 4;
                g.DrawEllipse(circlePen, -r, -r, 2 * r, 2 * r);
            }

            // Draw cross lines.
            g.DrawLine(crossPen, -radarDisplayRadius, 0, radarDisplayRadius, 0);
            g.DrawLine(crossPen, 0, -radarDisplayRadius, 0, radarDisplayRadius);

            // Draw radar sweep line.
            double beamAngle = engine.GetCurrentBeamAngle();
            float x2 = (float)(radarDisplayRadius * Math.Cos(beamAngle));
            float y2 = (float)(radarDisplayRadius * Math.Sin(beamAngle));
            g.DrawLine(sweepPen, 0, 0, x2, y2);

            // In debug mode, display raw measurements.
            if (debugMode)
            {
                var rawMeasurements = engine.GetLastMeasurements();
                using (Pen measPen = new Pen(Color.Red, 2))
                using (Brush measBrush = new SolidBrush(Color.Red))
                {
                    foreach (var meas in rawMeasurements)
                    {
                        double mx = meas.Range * Math.Cos(meas.Azimuth);
                        double my = meas.Range * Math.Sin(meas.Azimuth);
                        int ix = (int)(mx * scale);
                        int iy = (int)(my * scale);
                        g.DrawEllipse(measPen, ix - 3, iy - 3, 6, 6);
                    }
                }
                // Show count of raw measurements.
                string debugText = $"Raw Measurements: {rawMeasurements.Count}";
                g.ResetTransform();
                g.DrawString(debugText, trackFont, textBrush, 10, 10);
                // Restore transform.
                g.TranslateTransform(cx, cy);
                g.TranslateTransform(panX, panY);
                g.ScaleTransform(zoomFactor, zoomFactor);
            }

            // Draw confirmed tracks.
            var tracks = engine.GetTracks();
            foreach (var trk in tracks)
            {
                if (trk.ExistenceProb < 0.20)
                    continue;

                double x = trk.Filter.State[0];
                double y = trk.Filter.State[1];
                int tx = (int)(x * scale);
                int ty = (int)(y * scale);

                double vx = trk.Filter.State[3];
                double vy = trk.Filter.State[4];
                double headingRad = Math.Atan2(vy, vx);
                double headingDeg = headingRad * 180.0 / Math.PI;

                GraphicsState gs = g.Save();
                g.TranslateTransform(tx, ty);
                float rotationAngle = (float)(90 - headingDeg);
                g.RotateTransform(rotationAngle);
                int imgWidth = planeImage.Width;
                int imgHeight = planeImage.Height;
                g.DrawImage(planeImage, -imgWidth / 2, -imgHeight / 2, imgWidth, imgHeight);
                g.DrawLine(headingLinePen, 0, 0, 0, -imgHeight / 2);
                g.Restore(gs);

                string flightName = trk.FlightName;
                string details = $"T{trk.TrackId} [{flightName}]\nP={trk.ExistenceProb:F2}, H={headingDeg:F0}°";
                g.DrawString(details, trackFont, targetDetailBrush, tx + imgWidth / 2 + 5, ty - imgHeight / 2);
            }

            // Display target info on mouse hover.
            float invZoom = 1.0f / zoomFactor;
            float realMouseX = (mousePos.X - cx - panX);
            float realMouseY = (mousePos.Y - cy - panY);
            float radarX = realMouseX * invZoom;
            float radarY = realMouseY * invZoom;

            var targets = engine.GetTargets();
            foreach (var tgt in targets)
            {
                double x = tgt.State[0];
                double y = tgt.State[1];
                double z = tgt.State[2];
                int tx = (int)(x * scale);
                int ty = (int)(y * scale);
                float dx = radarX - tx;
                float dy = radarY - ty;
                float distPixels = (float)Math.Sqrt(dx * dx + dy * dy);
                if (distPixels < 10)
                {
                    double realDistance = Math.Sqrt(x * x + y * y + z * z);
                    double velocity = tgt.State[3];
                    double height = z;
                    string info = $"Dist: {realDistance:F0} m, Vel: {velocity:F0} m/s, Hgt: {height:F0} m";
                    g.DrawString(info, trackFont, textBrush, tx + 10, ty + 10);
                }
            }

        }
    }
}
