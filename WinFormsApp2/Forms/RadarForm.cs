using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Windows.Forms;
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

        private Pen circlePen = new Pen(Color.Green, 1);
        private Pen crossPen = new Pen(Color.Green, 1);
        private Pen sweepPen = new Pen(Color.Lime, 2);

        private Font trackFont = new Font("Arial", 10);
        private Brush textBrush = new SolidBrush(Color.White);
        private Brush targetDetailBrush = new SolidBrush(Color.LimeGreen);
        private Pen headingLinePen = new Pen(Color.Yellow, 2);

        private Image planeImage;
        private Point mousePos;

        private float zoomFactor = 1.0f;
        private const float zoomStep = 0.1f;
        private const float minZoom = 0.1f;
        private const float maxZoom = 5.0f;

        private float panX = 0;
        private float panY = 0;
        private bool isPanning = false;
        private Point lastMousePos;
        private bool debugMode = false;

        // NEW: The track currently hovered by the mouse (if any)
        private JPDA_Track hoveredTrack = null;

        public RadarForm()
        {
            InitializeComponent();

            // Load plane image
            try
            {
                planeImage = Image.FromFile("plane.png");
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error loading plane.png: " + ex.Message);
                Environment.Exit(1);
            }

            // Create simulation engine
            engine = new SimulationEngine(new Random());

            // Timer for simulation updates
            simulationTimer = new System.Windows.Forms.Timer();
            simulationTimer.Interval = (int)(engine.GetDt() * 1000.0);
            simulationTimer.Tick += SimulationTimer_Tick;
            simulationTimer.Start();

            // Form settings
            this.DoubleBuffered = true;
            this.Width = 1200;
            this.Height = 1000;
            this.Text = "Radar Sim";
            this.BackColor = Color.Black;

            // Scale: from real distances to screen pixels
            radarDisplayRadius = Math.Min(this.ClientSize.Width, this.ClientSize.Height) / 2 - 20;
            scale = radarDisplayRadius / engine.GetMaxRange();

            // Event handlers
            this.MouseMove += Form_MouseMove;
            this.MouseWheel += Form_MouseWheel;
            this.KeyDown += Form_KeyDown;
            this.MouseDown += Form_MouseDown;
            this.MouseUp += Form_MouseUp;
        }

        private void SimulationTimer_Tick(object? sender, EventArgs e)
        {
            engine.Update();
            this.Invalidate();
        }

        private void Form_MouseDown(object? sender, MouseEventArgs e)
        {
            // If left click and we have a hovered track, lock it
            if (e.Button == MouseButtons.Left)
            {
                if (hoveredTrack != null)
                {
                    var radar = engine.GetRadar();
                    if (radar.RadarType.ToLower() == "aircraft")
                    {
                        // Build a synthetic TargetCT from hoveredTrack
                        var lockedTgt = new TargetCT(
                            hoveredTrack.Filter.State[0],
                            hoveredTrack.Filter.State[1],
                            hoveredTrack.Filter.State[2],
                            0, 0, 0, 0, 1.0,    // dummy speed, turn, etc.
                            hoveredTrack.FlightName,
                            10.0,
                            new Random()
                        );
                        radar.LockTarget(lockedTgt);
                        Console.WriteLine($"[RadarForm] Locked trackID={hoveredTrack.TrackId}, P={hoveredTrack.ExistenceProb:F2}");
                    }
                }
                else
                {
                    // Start panning
                    isPanning = true;
                    lastMousePos = e.Location;
                }
            }
            // Right-click or middle-click can also be used for panning or something else
        }

        private void Form_MouseUp(object? sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
                isPanning = false;
        }

        private void Form_MouseMove(object? sender, MouseEventArgs e)
        {
            mousePos = e.Location;

            if (isPanning)
            {
                int dx = e.X - lastMousePos.X;
                int dy = e.Y - lastMousePos.Y;
                panX += dx;
                panY += dy;
                lastMousePos = e.Location;
            }

            this.Invalidate();
        }

        private void Form_MouseWheel(object? sender, MouseEventArgs e)
        {
            if (e.Delta > 0)
            {
                zoomFactor += zoomStep;
                if (zoomFactor > maxZoom) zoomFactor = maxZoom;
            }
            else
            {
                zoomFactor -= zoomStep;
                if (zoomFactor < minZoom) zoomFactor = minZoom;
            }
            this.Invalidate();
        }

        private void Form_KeyDown(object? sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.Oemplus || e.KeyCode == Keys.Add)
            {
                zoomFactor += zoomStep;
                if (zoomFactor > maxZoom) zoomFactor = maxZoom;
            }
            else if (e.KeyCode == Keys.OemMinus || e.KeyCode == Keys.Subtract)
            {
                zoomFactor -= zoomStep;
                if (zoomFactor < minZoom) zoomFactor = minZoom;
            }
            else if (e.KeyCode == Keys.D)
            {
                debugMode = !debugMode;
            }
            // Press U to unlock
            else if (e.KeyCode == Keys.U)
            {
                var radar = engine.GetRadar();
                radar.UnlockTarget();
                Console.WriteLine("[RadarForm] Radar unlocked (bar-scan resumed).");
            }

            this.Invalidate();
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);

            Graphics g = e.Graphics;
            int cx = this.ClientSize.Width / 2;
            int cy = this.ClientSize.Height / 2;

            g.TranslateTransform(cx, cy);
            g.TranslateTransform(panX, panY);
            g.ScaleTransform(zoomFactor, zoomFactor);

            // Draw range rings & cross
            for (int i = 1; i <= 4; i++)
            {
                int r = (radarDisplayRadius * i) / 4;
                g.DrawEllipse(circlePen, -r, -r, 2 * r, 2 * r);
            }
            g.DrawLine(crossPen, -radarDisplayRadius, 0, radarDisplayRadius, 0);
            g.DrawLine(crossPen, 0, -radarDisplayRadius, 0, radarDisplayRadius);

            var radar = engine.GetRadar();
            if (radar.RadarType.ToLower() == "aircraft")
            {
                // Aircraft wedge
                double az = radar.CurrentAzimuth;
                double bw = radar.BeamWidthRad;
                float startAngle = (float)((az - bw / 2.0) * 180.0 / Math.PI);
                float sweepAngle = (float)(bw * 180.0 / Math.PI);

                using (Brush wedgeBrush = new SolidBrush(Color.FromArgb(80, Color.Lime)))
                {
                    g.FillPie(wedgeBrush,
                        -radarDisplayRadius, -radarDisplayRadius,
                        radarDisplayRadius * 2, radarDisplayRadius * 2,
                        startAngle, sweepAngle);
                }
                g.DrawPie(sweepPen,
                    -radarDisplayRadius, -radarDisplayRadius,
                    radarDisplayRadius * 2, radarDisplayRadius * 2,
                    startAngle, sweepAngle);
            }
            else
            {
                // Ground radar single sweep line
                double beamAngle = engine.GetCurrentBeamAngle();
                float x2 = (float)(radarDisplayRadius * Math.Cos(beamAngle));
                float y2 = (float)(radarDisplayRadius * Math.Sin(beamAngle));
                g.DrawLine(sweepPen, 0, 0, x2, y2);
            }

            // Elevation bars info
            if (radar.RadarType.ToLower() == "aircraft" && radar.ShowElevationBars)
            {
                double barSpacingDeg = 2.0;
                double barHalfRad = (barSpacingDeg * Math.PI / 180.0) * 0.5;
                double eLow = radar.CurrentElevation - barHalfRad;
                double eHigh = radar.CurrentElevation + barHalfRad;

                double[] coverageRanges = { 10000, 20000, 30000, 40000, 50000 };
                string barInfo =
                    $"Elev Bar {radar.CurrentElevationBar + 1}/{radar.AntennaHeight}\n" +
                    $"Angles: {eLow * 180.0 / Math.PI:F1}° to {eHigh * 180.0 / Math.PI:F1}°\n" +
                    "Coverage:\n";

                foreach (var rngM in coverageRanges)
                {
                    double altLow = rngM * Math.Sin(eLow);
                    double altHigh = rngM * Math.Sin(eHigh);
                    barInfo += $" @ {rngM / 1000.0:F1} km: {altLow / 1000.0:F1}–{altHigh / 1000.0:F1} km\n";
                }

                g.ResetTransform();
                g.DrawString(barInfo, trackFont, textBrush, this.ClientSize.Width - 230, 20);
                g.TranslateTransform(cx, cy);
                g.TranslateTransform(panX, panY);
                g.ScaleTransform(zoomFactor, zoomFactor);
            }

            // Debug raw measurements
            if (debugMode)
            {
                var rawMeas = engine.GetLastMeasurements();
                using (Pen mp = new Pen(Color.Red, 2))
                {
                    foreach (var m in rawMeas)
                    {
                        double mx = m.Range * Math.Cos(m.Azimuth);
                        double my = m.Range * Math.Sin(m.Azimuth);
                        int ix = (int)(mx * scale);
                        int iy = (int)(my * scale);
                        g.DrawEllipse(mp, ix - 3, iy - 3, 6, 6);
                    }
                }
                g.ResetTransform();
                g.DrawString($"Raw Meas: {engine.GetLastMeasurements().Count}", trackFont, textBrush, 10, 10);
                g.TranslateTransform(cx, cy);
                g.TranslateTransform(panX, panY);
                g.ScaleTransform(zoomFactor, zoomFactor);
            }

            // Draw tracks & highlight hovered
            var tracks = engine.GetTracks();
            hoveredTrack = null; // we'll find if any track is near the mouse
            float closestDist = 999999f;

            // Convert from screen coords to radar coords
            float invZoom = 1.0f / zoomFactor;
            float realMouseX = (mousePos.X - cx - panX) * invZoom;
            float realMouseY = (mousePos.Y - cy - panY) * invZoom;

            foreach (var trk in tracks)
            {
                if (trk.ExistenceProb < 0.20) continue;
                double x = trk.Filter.State[0];
                double y = trk.Filter.State[1];
                int tx = (int)(x * scale);
                int ty = (int)(y * scale);

                double vx = trk.Filter.State[3];
                double vy = trk.Filter.State[4];
                double headingRad = Math.Atan2(vy, vx);
                double headingDeg = headingRad * 180.0 / Math.PI;

                // Check mouse distance from the track
                float dx = (float)(tx - realMouseX);
                float dy = (float)(ty - realMouseY);
                float distPix = (float)Math.Sqrt(dx * dx + dy * dy);

                if (distPix < 12f && distPix < closestDist)
                {
                    closestDist = distPix;
                    hoveredTrack = trk;
                }

                // Draw track (plane icon)
                var gs = g.Save();
                g.TranslateTransform(tx, ty);

                float rot = (float)(90.0 - headingDeg);
                g.RotateTransform(rot);

                int imgW = planeImage.Width;
                int imgH = planeImage.Height;
                g.DrawImage(planeImage, -imgW / 2, -imgH / 2, imgW, imgH);
                g.DrawLine(headingLinePen, 0, 0, 0, -imgH / 2);

                g.Restore(gs);

                string details = $"T{trk.TrackId} [{trk.FlightName}]\n" +
                                 $"P={trk.ExistenceProb:F2},H={headingDeg:F0}°";
                g.DrawString(details, trackFont, targetDetailBrush, tx + imgW / 2 + 5, ty - imgH / 2);
            }

            // If we found a hovered track, highlight it
            if (hoveredTrack != null)
            {
                double xx = hoveredTrack.Filter.State[0];
                double yy = hoveredTrack.Filter.State[1];
                int hx = (int)(xx * scale);
                int hy = (int)(yy * scale);
                // draw a small circle or something
                using (Pen highlightPen = new Pen(Color.Magenta, 2))
                {
                    g.DrawEllipse(highlightPen, hx - 10, hy - 10, 20, 20);
                }
            }

            // Optional: Show info if the mouse is near a known "real targetCT" too:
            var targets = engine.GetTargets();
            foreach (var tgt in targets)
            {
                double tx = tgt.State[0];
                double ty = tgt.State[1];
                double tz = tgt.State[2];
                int px = (int)(tx * scale);
                int py = (int)(ty * scale);

                float dx = (float)(px - realMouseX);
                float dy = (float)(py - realMouseY);
                float distPix = (float)Math.Sqrt(dx * dx + dy * dy);
                if (distPix < 10.0f)
                {
                    double distM = Math.Sqrt(tx * tx + ty * ty + tz * tz);
                    double speed = tgt.State[3];
                    double alt = tz;
                    string info = $"Dist={distM:F0}m, V={speed:F0}m/s, Alt={alt:F0}m";
                    g.DrawString(info, trackFont, textBrush, px + 10, py + 10);
                }
            }
        }

        private void RadarForm_Load(object sender, EventArgs e)
        {
        }
    }
}
