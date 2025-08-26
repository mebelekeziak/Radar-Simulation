using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Text;
 
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

        private readonly Pen ringPen = new Pen(Color.FromArgb(60, 200, 200, 200), 1);
        private readonly Pen axisPen = new Pen(Color.FromArgb(50, 200, 200, 200), 1);
        private readonly Pen sweepEdgePen = new Pen(Color.FromArgb(120, 0, 180, 255), 2);
        private readonly Brush sweepFill = new SolidBrush(Color.FromArgb(35, 0, 180, 255));
        private readonly Pen headingPen = new Pen(Color.FromArgb(180, 230, 230, 230), 1);
        private readonly Pen highlightPen = new Pen(Color.FromArgb(200, 255, 120, 120), 2);
        private readonly Brush labelBrush = new SolidBrush(Color.Gainsboro);
        private readonly Brush targetBrush = new SolidBrush(Color.FromArgb(150, 120, 170, 255));
        private readonly Brush panelBg = new SolidBrush(Color.FromArgb(140, 20, 20, 20));

        private Font trackFont = new Font("Segoe UI", 10f, FontStyle.Regular);
        private Font uiFont = new Font("Segoe UI Semibold", 10.5f, FontStyle.Bold);

        private readonly Brush confirmedTrackBrush = new SolidBrush(Color.Orange);
        private Point mousePos;

        private float zoomFactor = 1.0f;
        private const float zoomStep = 0.1f;
        private const float minZoom = 0.1f;
        private const float maxZoom = 5.0f;

        private float panX = 0, panY = 0;
        private bool isPanning = false;
        private Point lastMousePos;
        private bool debugMode = false;

        private JPDA_Track hoveredTrack = null;
        private JPDA_Track lockedTrack = null;

        public RadarForm()
        {
            InitializeComponent();

            // Track marker: bright orange dot replaces previous PNG plane icon

            engine = new SimulationEngine(new Random());

            simulationTimer = new System.Windows.Forms.Timer { Interval = (int)(engine.GetDt() * 1000.0) };
            simulationTimer.Tick += (_, __) => { engine.Update(); Invalidate(); };
            simulationTimer.Start();

            DoubleBuffered = true;
            Width = 1200;
            Height = 1000;
            Text = "Radar Sim";
            BackColor = Color.Black;

            radarDisplayRadius = Math.Min(ClientSize.Width, ClientSize.Height) / 2 - 32;
            scale = radarDisplayRadius / engine.GetMaxRange();

            MouseMove += Form_MouseMove;
            MouseWheel += Form_MouseWheel;
            KeyDown += Form_KeyDown;
            MouseDown += Form_MouseDown;
            MouseUp += Form_MouseUp;
        }

        // Designer wires this: Load += RadarForm_Load;
        private void RadarForm_Load(object? sender, EventArgs e)
        {
            // No-op: initialization happens in constructor. Keep handler to satisfy designer.
        }

        private void Form_MouseDown(object? sender, MouseEventArgs e)
        {
            if (e.Button != MouseButtons.Left) return;

            if (hoveredTrack != null)
            {
                var radar = engine.GetRadar();
                if (radar.RadarType.Equals("aircraft", StringComparison.OrdinalIgnoreCase))
                {
                    radar.LockTarget(hoveredTrack);
                    lockedTrack = hoveredTrack;
                }
            }
            else
            {
                isPanning = true;
                lastMousePos = e.Location;
            }
        }

        private void Form_MouseUp(object? sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left) isPanning = false;
        }

        private void Form_MouseMove(object? sender, MouseEventArgs e)
        {
            mousePos = e.Location;
            if (isPanning)
            {
                panX += e.X - lastMousePos.X;
                panY += e.Y - lastMousePos.Y;
                lastMousePos = e.Location;
            }
            Invalidate();
        }

        private void Form_MouseWheel(object? sender, MouseEventArgs e)
        {
            zoomFactor = e.Delta > 0
                ? Math.Min(zoomFactor + zoomStep, maxZoom)
                : Math.Max(zoomFactor - zoomStep, minZoom);
            Invalidate();
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
            }
            Invalidate();
        }

        private TargetCT FindTargetByFlightName(string flightName)
        {
            foreach (var tgt in engine.GetTargets())
                if (tgt.AircraftName == flightName) return tgt;
            return null;
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            var g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.TextRenderingHint = TextRenderingHint.ClearTypeGridFit;

            int cx = ClientSize.Width / 2;
            int cy = ClientSize.Height / 2;

            // World transform
            g.TranslateTransform(cx, cy);
            g.TranslateTransform(panX, panY);
            g.ScaleTransform(zoomFactor, zoomFactor);

            // ——— Range rings + axes (simple, quiet) ———
            for (int i = 1; i <= 4; i++)
            {
                int r = (radarDisplayRadius * i) / 4;
                g.DrawEllipse(ringPen, -r, -r, 2 * r, 2 * r);
            }
            g.DrawLine(axisPen, -radarDisplayRadius, 0, radarDisplayRadius, 0);
            g.DrawLine(axisPen, 0, -radarDisplayRadius, 0, radarDisplayRadius);

            var radar = engine.GetRadar();

            // ——— Beam / sweep (flat fill, no gradients) ———
            if (radar.RadarType.Equals("aircraft", StringComparison.OrdinalIgnoreCase))
            {
                if (radar.UseAesaMode && radar.AesaBeams != null)
                {
                    foreach (var beam in radar.AesaBeams)
                        DrawFlatBeam(g, beam.CurrentAzimuth, radar.BeamWidthRad);
                }
                else
                {
                    DrawFlatBeam(g, radar.CurrentAzimuth, radar.BeamWidthRad);
                }
            }
            else
            {
                // Mechanical ground radar: simple line
                double a = engine.GetCurrentBeamAngle();
                float x2 = (float)(radarDisplayRadius * Math.Cos(a));
                float y2 = (float)(radarDisplayRadius * Math.Sin(a));
                g.DrawLine(sweepEdgePen, 0, 0, x2, y2);
            }

            if (debugMode) DrawDebugBeamOutline(g, radar);

            // ——— Tracks ———
            var tracks = engine.GetTracks();
            hoveredTrack = null;

            // compute mouse in world coords
            float invZoom = 1f / zoomFactor;
            float realMouseX = (mousePos.X - cx - panX) * invZoom;
            float realMouseY = (mousePos.Y - cy - panY) * invZoom;

            float closest = float.MaxValue;

            foreach (var trk in tracks)
            {
                if (trk.ExistenceProb < 0.20) continue;

                double x = trk.Filter.State[0];
                double y = trk.Filter.State[1];
                double vx = trk.Filter.State[3];
                double vy = trk.Filter.State[4];
                double headingRad = Math.Atan2(vy, vx);
                double headingDeg = headingRad * 180.0 / Math.PI;

                int tx = (int)(x * scale);
                int ty = (int)(y * scale);

                // hover hit
                float dx = tx - realMouseX;
                float dy = ty - realMouseY;
                float d = (float)Math.Sqrt(dx * dx + dy * dy);
                if (d < 16f && d < closest) { closest = d; hoveredTrack = trk; }

                // draw confirmed track as an orange dot with heading tick
                var gs = g.Save();
                g.TranslateTransform(tx, ty);
                g.RotateTransform((float)(90.0 - headingDeg));

                int dotRadius = 5; // dot radius (px)
                g.FillEllipse(confirmedTrackBrush, -dotRadius, -dotRadius, 2 * dotRadius, 2 * dotRadius);
                g.DrawLine(headingPen, 0, 0, 0, -(dotRadius + 8));
                g.Restore(gs);

                // highlight (fixed ring, no pulse)
                if (trk == hoveredTrack || trk == lockedTrack)
                    g.DrawEllipse(highlightPen, tx - 12, ty - 12, 24, 24);

                // condensed label: one line
                string snrInfo = "";
                if (!string.IsNullOrEmpty(trk.FlightName))
                {
                    var tgt = FindTargetByFlightName(trk.FlightName);
                    if (tgt != null)
                    {
                        double z = trk.Filter.State[2];
                        double r = Math.Sqrt(x * x + y * y + z * z);
                        double snr_dB = radar.GetSNR_dB(tgt.RCS, r);
                        snrInfo = $" SNR {snr_dB:F1} dB";
                    }
                }
                string label = $"T{trk.TrackId} {trk.FlightName}  P {trk.ExistenceProb:F2}  H {headingDeg:000}°{snrInfo}";
                g.DrawString(label, trackFont, labelBrush, tx + 14, ty - 14);
            }

            // ——— True positions (simple dots with optional tooltip) ———
            foreach (var tgt in engine.GetTargets())
            {
                double tx = tgt.State[0], ty = tgt.State[1], tz = tgt.State[2];
                int px = (int)(tx * scale);
                int py = (int)(ty * scale);

                g.FillEllipse(targetBrush, px - 3, py - 3, 6, 6);

                float dx2 = px - realMouseX;
                float dy2 = py - realMouseY;
                if (Math.Sqrt(dx2 * dx2 + dy2 * dy2) < 10.0)
                {
                    double dist = Math.Sqrt(tx * tx + ty * ty + tz * tz);
                    string tip = $"R {dist:F0} m  V {tgt.State[3]:F0} m/s  Alt {tz:F0} m";
                    g.DrawString(tip, trackFont, labelBrush, px + 10, py + 8);
                }
            }

            // ——— UI overlay (screen space) ———
            g.ResetTransform();
            DrawStatusPanel(g, radar, tracks.Count);

            // ——— Debug raw measurements ———
            if (debugMode)
            {
                var raw = engine.GetLastMeasurements();
                using var mp = new Pen(Color.IndianRed, 2);
                foreach (var m in raw)
                {
                    double mx = m.Range * Math.Cos(m.Azimuth);
                    double my = m.Range * Math.Sin(m.Azimuth);
                    int ix = (int)(cx + panX + zoomFactor * (mx * scale));
                    int iy = (int)(cy + panY + zoomFactor * (my * scale));
                    g.DrawEllipse(mp, ix - 3, iy - 3, 6, 6);
                }
            }
        }

        // ————— Helpers —————

        private void DrawFlatBeam(Graphics g, double centerAz, double bwRad)
        {
            float startDeg = (float)((centerAz - bwRad / 2.0) * 180.0 / Math.PI);
            float sweepDeg = (float)(bwRad * 180.0 / Math.PI);

            var rect = new RectangleF(-radarDisplayRadius, -radarDisplayRadius,
                                       radarDisplayRadius * 2, radarDisplayRadius * 2);

            g.FillPie(sweepFill, rect, startDeg, sweepDeg);
            g.DrawPie(sweepEdgePen, rect, startDeg, sweepDeg);
        }

        private void DrawDebugBeamOutline(Graphics g, AdvancedRadar radar)
        {
            // Outline only, no intensity shading
            if (radar.RadarType.Equals("aircraft", StringComparison.OrdinalIgnoreCase))
            {
                if (radar.UseAesaMode && radar.AesaBeams != null && radar.AesaBeams.Count > 0)
                {
                    foreach (var b in radar.AesaBeams) DrawFlatBeam(g, b.CurrentAzimuth, radar.BeamWidthRad);
                }
                else
                {
                    DrawFlatBeam(g, radar.CurrentAzimuth, radar.BeamWidthRad);
                }
            }
            else
            {
                double a = engine.GetCurrentBeamAngle();
                float startDeg = (float)((a - radar.BeamWidthRad / 2.0) * 180.0 / Math.PI);
                float sweepDeg = (float)(radar.BeamWidthRad * 180.0 / Math.PI);
                var rect = new RectangleF(-radarDisplayRadius, -radarDisplayRadius,
                                           radarDisplayRadius * 2, radarDisplayRadius * 2);
                g.DrawPie(sweepEdgePen, rect, startDeg, sweepDeg);
            }
        }

        private void DrawStatusPanel(Graphics g, AdvancedRadar radar, int trackCount)
        {
            string s =
                $"{(radar.UseAesaMode ? "AESA" : "Mechanical")}  " +
                $"R {engine.GetMaxRange() / 1000:F1} km  " +
                $"Zoom {zoomFactor:F2}×  " +
                $"Tracks {trackCount}  " +
                $"[D]ebug {(debugMode ? "ON" : "off")}  [+/-] Zoom  [U] Unlock  Drag to pan";

            SizeF sz = g.MeasureString(s, uiFont);
            var rect = new RectangleF(12, ClientSize.Height - sz.Height - 16, sz.Width + 16, sz.Height + 10);

            g.FillRectangle(panelBg, rect);
            g.DrawString(s, uiFont, labelBrush, rect.Left + 8, rect.Top + 5);
        }
    }
}
