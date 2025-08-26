using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Text;
using System.IO;
using System.Linq;
using System.Reflection;
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

        // Neon pen/brush set
        private Pen circlePen = new Pen(Color.FromArgb(90, 0, 255, 160), 2) { DashStyle = DashStyle.Dot };
        private Pen crossPen = new Pen(Color.FromArgb(90, 120, 255, 240), 1.5f);
        private Pen sweepPen = new Pen(Color.FromArgb(255, 64, 255, 128), 2.5f);
        private Pen shadowPen = new Pen(Color.FromArgb(35, 64, 255, 128), 16) { DashStyle = DashStyle.Solid };
        private Pen headingLinePen = new Pen(Color.FromArgb(200, 255, 255, 50), 2.2f) { DashCap = DashCap.Round };

        private Font trackFont;
        private Font uiFont;
        private Brush textBrush = new SolidBrush(Color.White);
        private Brush labelShadow = new SolidBrush(Color.FromArgb(50, 0, 0, 0));
        private Brush targetDetailBrush = new SolidBrush(Color.Cyan);
        private Brush highlightBrush = new SolidBrush(Color.FromArgb(180, 255, 0, 255));
        private Brush statusPanelBrush = new SolidBrush(Color.FromArgb(200, 20, 30, 40));
        private Pen highlightPen = new Pen(Color.FromArgb(200, 255, 40, 220), 3.5f);

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

        private JPDA_Track hoveredTrack = null;
        private JPDA_Track lockedTrack = null;

        private PrivateFontCollection privateFonts = new PrivateFontCollection();

        public RadarForm()
        {
            InitializeComponent();

            try
            {
                Assembly assembly = Assembly.GetExecutingAssembly();
                string resourceName = "RadarEKF.Assets.Images.plane.png";
                using (Stream stream = assembly.GetManifestResourceStream(resourceName))
                {
                    if (stream == null)
                        throw new Exception($"Did not find '{resourceName}'.");
                    planeImage = Image.FromStream(stream);
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error loading plane.png: " + ex.Message);
                Environment.Exit(1);
            }

            try
            {
                Assembly assembly = Assembly.GetExecutingAssembly();
                string fontResourceName = "RadarEKF.Assets.Fonts.RobotoCondensed.ttf";
                using (Stream fontStream = assembly.GetManifestResourceStream(fontResourceName))
                {
                    if (fontStream == null)
                        throw new Exception($"Did not find embedded font resource '{fontResourceName}'.");

                    string tempFontPath = Path.Combine(Path.GetTempPath(), "RobotoCondensed.ttf");
                    using (FileStream fs = new FileStream(tempFontPath, FileMode.Create, FileAccess.Write))
                        fontStream.CopyTo(fs);

                    privateFonts.AddFontFile(tempFontPath);
                }
                trackFont = new Font(privateFonts.Families[0], 11, FontStyle.Bold);
                uiFont = new Font(privateFonts.Families[0], 12, FontStyle.Bold);
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error loading font RobotoCondensed.ttf: " + ex.Message);
                trackFont = new Font("Segoe UI", 11, FontStyle.Bold);
                uiFont = new Font("Segoe UI", 12, FontStyle.Bold);
            }

            engine = new SimulationEngine(new Random());

            simulationTimer = new System.Windows.Forms.Timer();
            simulationTimer.Interval = (int)(engine.GetDt() * 1000.0);
            simulationTimer.Tick += SimulationTimer_Tick;
            simulationTimer.Start();

            this.DoubleBuffered = true;
            this.Width = 1200;
            this.Height = 1000;
            this.Text = "Radar Sim";
            this.BackColor = Color.Black;

            radarDisplayRadius = Math.Min(this.ClientSize.Width, this.ClientSize.Height) / 2 - 32;
            scale = radarDisplayRadius / engine.GetMaxRange();

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
            if (e.Button == MouseButtons.Left)
            {
                if (hoveredTrack != null)
                {
                    var radar = engine.GetRadar();
                    if (radar.RadarType.ToLower() == "aircraft")
                    {
                        radar.LockTarget(hoveredTrack);
                        lockedTrack = hoveredTrack;
                        Console.WriteLine($"[RadarForm] Locked trackID={hoveredTrack.TrackId}, P={hoveredTrack.ExistenceProb:F2}");
                    }
                }
                else
                {
                    isPanning = true;
                    lastMousePos = e.Location;
                }
            }
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
            if (e.Delta > 0) zoomFactor = Math.Min(zoomFactor + zoomStep, maxZoom);
            else zoomFactor = Math.Max(zoomFactor - zoomStep, minZoom);
            this.Invalidate();
        }

        private void Form_KeyDown(object? sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.Oemplus || e.KeyCode == Keys.Add)
                zoomFactor = Math.Min(zoomFactor + zoomStep, maxZoom);
            else if (e.KeyCode == Keys.OemMinus || e.KeyCode == Keys.Subtract)
                zoomFactor = Math.Max(zoomFactor - zoomStep, minZoom);
            else if (e.KeyCode == Keys.D)
                debugMode = !debugMode;
            else if (e.KeyCode == Keys.U)
            {
                var radar = engine.GetRadar();
                radar.UnlockTarget();
                lockedTrack = null;
                Console.WriteLine("[RadarForm] Radar unlocked (bar-scan resumed).");
            }
            this.Invalidate();
        }

        private TargetCT FindTargetByFlightName(string flightName)
        {
            foreach (var tgt in engine.GetTargets())
                if (tgt.AircraftName == flightName)
                    return tgt;
            return null;
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            Graphics g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.TextRenderingHint = TextRenderingHint.ClearTypeGridFit;

            int cx = this.ClientSize.Width / 2;
            int cy = this.ClientSize.Height / 2;

            g.TranslateTransform(cx, cy);
            g.TranslateTransform(panX, panY);
            g.ScaleTransform(zoomFactor, zoomFactor);

            // --- Range rings and cross
            for (int i = 1; i <= 4; i++)
            {
                int r = (radarDisplayRadius * i) / 4;
                g.DrawEllipse(circlePen, -r, -r, 2 * r, 2 * r);
            }
            g.DrawLine(crossPen, -radarDisplayRadius, 0, radarDisplayRadius, 0);
            g.DrawLine(crossPen, 0, -radarDisplayRadius, 0, radarDisplayRadius);

            // --- Radar sweep/beam
            var radar = engine.GetRadar();
            if (radar.RadarType.ToLower() == "aircraft")
            {
                if (radar.UseAesaMode)
                {
                    float sectorStartAngle = -70.0f, sectorSweepAngle = 140.0f;
                    using (GraphicsPath p = new GraphicsPath())
                    {
                        p.AddPie(-radarDisplayRadius, -radarDisplayRadius,
                                 radarDisplayRadius * 2, radarDisplayRadius * 2,
                                 sectorStartAngle, sectorSweepAngle);
                        using (PathGradientBrush grad = new PathGradientBrush(p))
                        {
                            grad.CenterColor = Color.FromArgb(50, 0, 255, 255);
                            grad.SurroundColors = new[] { Color.Transparent };
                            g.FillPath(grad, p);
                        }
                    }
                    AdvancedRadar advRadar = radar;
                    if (advRadar.AesaBeams != null)
                    {
                        foreach (var beam in advRadar.AesaBeams)
                        {
                            double az = beam.CurrentAzimuth;
                            double bw = radar.BeamWidthRad;
                            float startAngle = (float)((az - bw / 2.0) * 180.0 / Math.PI);
                            float sweepAngle = (float)(bw * 180.0 / Math.PI);
                            using (GraphicsPath beamPath = new GraphicsPath())
                            {
                                beamPath.AddPie(-radarDisplayRadius, -radarDisplayRadius,
                                    radarDisplayRadius * 2, radarDisplayRadius * 2,
                                    startAngle, sweepAngle);
                                using (PathGradientBrush grad = new PathGradientBrush(beamPath))
                                {
                                    grad.CenterColor = Color.FromArgb(150, 0, 255, 128);
                                    grad.SurroundColors = new[] { Color.Transparent };
                                    g.FillPath(grad, beamPath);
                                }
                            }
                            g.DrawPie(sweepPen, -radarDisplayRadius, -radarDisplayRadius,
                                radarDisplayRadius * 2, radarDisplayRadius * 2, startAngle, sweepAngle);
                        }
                    }
                }
                else
                {
                    double az = radar.CurrentAzimuth;
                    double bw = radar.BeamWidthRad;
                    float startAngle = (float)((az - bw / 2.0) * 180.0 / Math.PI);
                    float sweepAngle = (float)(bw * 180.0 / Math.PI);
                    using (GraphicsPath wedge = new GraphicsPath())
                    {
                        wedge.AddPie(-radarDisplayRadius, -radarDisplayRadius,
                            radarDisplayRadius * 2, radarDisplayRadius * 2,
                            startAngle, sweepAngle);
                        using (PathGradientBrush grad = new PathGradientBrush(wedge))
                        {
                            grad.CenterColor = Color.FromArgb(130, 64, 255, 128);
                            grad.SurroundColors = new[] { Color.Transparent };
                            g.FillPath(grad, wedge);
                        }
                    }
                    g.DrawPie(sweepPen, -radarDisplayRadius, -radarDisplayRadius,
                        radarDisplayRadius * 2, radarDisplayRadius * 2,
                        startAngle, sweepAngle);
                }
            }
            else
            {
                double beamAngle = engine.GetCurrentBeamAngle();
                float x2 = (float)(radarDisplayRadius * Math.Cos(beamAngle));
                float y2 = (float)(radarDisplayRadius * Math.Sin(beamAngle));
                // Neon line with shadow
                g.DrawLine(shadowPen, 0, 0, x2, y2);
                g.DrawLine(sweepPen, 0, 0, x2, y2);
            }

            // Debug-only: draw antenna beam/lobe intensity by antennaPattern/beamwidth
            if (debugMode)
            {
                DrawDebugAntennaBeam(g, radar);
            }

            var tracks = engine.GetTracks();
            hoveredTrack = null;
            float closestDist = float.MaxValue;
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

                // SNR text
                string snrInfo = "";
                if (!string.IsNullOrEmpty(trk.FlightName))
                {
                    TargetCT target = FindTargetByFlightName(trk.FlightName);
                    if (target != null)
                    {
                        double z = trk.Filter.State[2];
                        double r = Math.Sqrt(x * x + y * y + z * z);
                        double snr_dB = radar.GetSNR_dB(target.RCS, r);
                        snrInfo = $", SNR={snr_dB:F1} dB";
                    }
                }

                string details = $"T{trk.TrackId} [{trk.FlightName}]\n" +
                                 $"P={trk.ExistenceProb:F2}, H={headingDeg:F0}°{snrInfo}";

                float dx = (float)(tx - realMouseX);
                float dy = (float)(ty - realMouseY);
                float distPix = (float)Math.Sqrt(dx * dx + dy * dy);
                if (distPix < 18f && distPix < closestDist)
                {
                    closestDist = distPix;
                    hoveredTrack = trk;
                }

                var gs = g.Save();
                g.TranslateTransform(tx, ty);
                float rot = (float)(90.0 - headingDeg);
                g.RotateTransform(rot);

                int imgW = planeImage.Width, imgH = planeImage.Height;
                g.DrawImage(planeImage, -imgW / 2, -imgH / 2, imgW, imgH);
                g.DrawLine(headingLinePen, 0, 0, 0, -imgH / 2);

                g.Restore(gs);

                if (trk == hoveredTrack || trk == lockedTrack)
                {
                    float t = (float)(DateTime.Now.Millisecond / 1000.0f);
                    int pulse = 12 + (int)(Math.Abs(Math.Sin(t * Math.PI * 2)) * 6);
                    using (Pen neon = new Pen(Color.Magenta, pulse))
                    {
                        neon.Color = Color.FromArgb(120, neon.Color);
                        g.DrawEllipse(neon, tx - 16, ty - 16, 32, 32);
                    }
                    g.DrawEllipse(highlightPen, tx - 11, ty - 11, 22, 22);
                }

                g.DrawString(details, trackFont, labelShadow, tx + imgW / 2 + 7, ty - imgH / 2 + 2);
                g.DrawString(details, trackFont, targetDetailBrush, tx + imgW / 2 + 5, ty - imgH / 2);
            }

            if (hoveredTrack != null)
            {
                double xx = hoveredTrack.Filter.State[0];
                double yy = hoveredTrack.Filter.State[1];
                int hx = (int)(xx * scale);
                int hy = (int)(yy * scale);
                using (Pen pulse = new Pen(Color.Cyan, 2.5f))
                {
                    pulse.DashStyle = DashStyle.DashDot;
                    g.DrawEllipse(pulse, hx - 24, hy - 24, 48, 48);
                }
            }
            // true positions
            foreach (var tgt in engine.GetTargets())
            {
                double tx = tgt.State[0];
                double ty = tgt.State[1];
                double tz = tgt.State[2];
                int px = (int)(tx * scale);
                int py = (int)(ty * scale);

                float dx2 = (float)(px - realMouseX);
                float dy2 = (float)(py - realMouseY);
                float distPix2 = (float)Math.Sqrt(dx2 * dx2 + dy2 * dy2);
                if (distPix2 < 12.0f)
                {
                    double distM = Math.Sqrt(tx * tx + ty * ty + tz * tz);
                    double speed = tgt.State[3];
                    double alt = tz;
                    string info = $"Dist={distM:F0}m, V={speed:F0}m/s, Alt={alt:F0}m";
                    g.DrawString(info, trackFont, textBrush, px + 12, py + 8);
                }
                using (Brush b = new SolidBrush(Color.FromArgb(110, 60, 180, 255)))
                    g.FillEllipse(b, px - 5, py - 5, 10, 10);
            }

            g.ResetTransform();
            DrawStatusPanel(g, radar, tracks.Count, cx, cy);

            if (debugMode)
            {
                var rawMeas = engine.GetLastMeasurements();
                using (Pen mp = new Pen(Color.Red, 2))
                {
                    foreach (var m in rawMeas)
                    {
                        // Convert polar (range, azimuth) to world XY (meters)
                        double mx = m.Range * Math.Cos(m.Azimuth);
                        double my = m.Range * Math.Sin(m.Azimuth);
                        // Apply the same world->screen mapping as the main scene:
                        // screen = center + pan + zoom * (meters * scale)
                        int ix = (int)(cx + panX + zoomFactor * (mx * scale));
                        int iy = (int)(cy + panY + zoomFactor * (my * scale));
                        g.DrawEllipse(mp, ix - 4, iy - 4, 8, 8);
                    }
                }
            }
        }


        private void DrawStatusPanel(Graphics g, AdvancedRadar radar, int trackCount, int cx, int cy)
        {
            string status =
                $"Radar: {(radar.UseAesaMode ? "AESA" : "Mechanical")}\n" +
                $"Range: {engine.GetMaxRange() / 1000:F1} km\n" +
                $"Zoom: {zoomFactor:F2}x\n" +
                $"Tracks: {trackCount}\n" +
                $"[D]ebug: {(debugMode ? "ON" : "off")}\n" +
                $"[+/-] Zoom  [U] Unlock\n" +
                $"Pan: Drag background";

            var warnings = engine.GetConfigWarnings();
            if (warnings != null && warnings.Count > 0)
            {
                status += "\n\nWARN: Unsupported config value(s):\n";
                foreach (var w in warnings)
                {
                    status += "- " + w + "\n";
                }
            }

            SizeF sz = g.MeasureString(status, uiFont);
            RectangleF panelRect = new RectangleF(18, this.ClientSize.Height - sz.Height - 36, sz.Width + 32, sz.Height + 24);

            using (SolidBrush bg = new SolidBrush(Color.FromArgb(170, 30, 38, 45)))
                g.FillRectangle(bg, panelRect);

            using (Pen shadow = new Pen(Color.FromArgb(60, 0, 0, 0), 8))
                g.DrawRectangle(shadow, Rectangle.Round(panelRect));

            using (Pen neon = new Pen(Color.FromArgb(80, 60, 255, 255), 3))
                g.DrawRectangle(neon, Rectangle.Round(panelRect));

            g.DrawString(status, uiFont, textBrush, panelRect.Left + 12, panelRect.Top + 10);
        }

        private void RadarForm_Load(object sender, EventArgs e)
        {

        }

        // --------------------------- Debug beam/lobe rendering ---------------------------
        private static double OffBoresightGainUI(string pattern, double thetaRad, double bwRad)
        {
            if (thetaRad <= 0) return 1.0;
            double bw = Math.Max(1e-6, bwRad);
            string pat = (pattern ?? "gaussian").ToLower();
            if (pat == "sinc2" || pat == "sinc^2" || pat == "sinc")
            {
                const double u_hp = 1.39156; // solves sinc(u_hp) = 1/sqrt(2)
                double k = 2.0 * u_hp / bw;
                double u = k * thetaRad;
                double s = Math.Abs(u) < 1e-8 ? 1.0 : Math.Sin(u) / u;
                double g = s * s;
                return Math.Max(0.0, Math.Min(1.0, g));
            }
            else
            {
                double x = thetaRad / bw;
                double g = Math.Exp(-4.0 * Math.Log(2.0) * x * x); // -3 dB at BW/2
                return Math.Max(0.0, Math.Min(1.0, g));
            }
        }

        private void DrawPatternForBeam(Graphics g, double centerAzRad, double bwRad, string pattern, Color baseColor)
        {
            int segments = 72; // angular resolution of the lobe
            double start = centerAzRad - 0.5 * bwRad;
            double dphi = bwRad / segments;

            for (int i = 0; i < segments; i++)
            {
                // Mid-angle for intensity sampling
                double a0 = start + i * dphi;
                double amid = a0 + 0.5 * dphi;
                double theta = Math.Abs(amid - centerAzRad);
                double gain = OffBoresightGainUI(pattern, theta, bwRad);
                // Small gamma to improve visibility of sidelobes
                gain = Math.Pow(gain, 0.7);

                int alpha = (int)(gain * 185); // cap max alpha
                if (alpha <= 0) continue;

                using (Brush b = new SolidBrush(Color.FromArgb(alpha, baseColor)))
                {
                    float startDeg = (float)(a0 * 180.0 / Math.PI);
                    float sweepDeg = (float)(dphi * 180.0 / Math.PI);
                    g.FillPie(b,
                        -radarDisplayRadius, -radarDisplayRadius,
                        radarDisplayRadius * 2, radarDisplayRadius * 2,
                        startDeg, sweepDeg);
                }
            }
        }

        private void DrawDebugAntennaBeam(Graphics g, AdvancedRadar radar)
        {
            // Choose a vivid base color for the lobe overlay
            Color baseColor = Color.FromArgb(0, 255, 180);
            double bw = radar.BeamWidthRad;

            if (radar.RadarType.ToLower() == "aircraft")
            {
                if (radar.UseAesaMode && radar.AesaBeams != null && radar.AesaBeams.Count > 0)
                {
                    foreach (var beam in radar.AesaBeams)
                    {
                        DrawPatternForBeam(g, beam.CurrentAzimuth, bw, radar.AntennaPattern, baseColor);
                    }
                }
                else
                {
                    DrawPatternForBeam(g, radar.CurrentAzimuth, bw, radar.AntennaPattern, baseColor);
                }
            }
            else
            {
                // Ground/mechanical: use current beam angle from engine helper
                double az = engine.GetCurrentBeamAngle();
                DrawPatternForBeam(g, az, bw, radar.AntennaPattern, baseColor);
            }
        }
    }
}
