// Mission panel in the webcast style (replaces the plain ImGui window). Translucent black plate with clipped
// corners, Bai Jamjuree type, white-on-dark labels, arc dials like the webcast gauges (drag or scroll to set),
// landing-mode cards, propellant bars, one big LAUNCH button. In flight it shrinks to the clock, phase, the
// booster / upper-stage lines, ABORT and an expandable event log. Ctrl+Shift+F shows / hides it.
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace SterlingFlight
{
    internal static class MissionPanel
    {
        const float W = 560f, Pad = 20f, Cut = 22f;
        static bool _log;
        static ImDrawListPtr _dl;
        static float2 _o;                       // content origin (screen)
        static float S = 1f;                    // UI scale (layout is in 1080p units) = screen scale x UserScale
        // player size: drag the grip in the bottom-right corner, or Ctrl + mouse wheel over the panel. Saved next to the mod.
        static float _user = float.NaN;
        static float UserScale
        {
            get
            {
                if (float.IsNaN(_user))
                {
                    _user = 1f;
                    try { if (File.Exists(ScalePath) && float.TryParse(File.ReadAllText(ScalePath).Trim(), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out var u)) _user = Math.Clamp(u, MinUser, MaxUser); }
                    catch { }
                }
                return _user;
            }
            set
            {
                _user = Math.Clamp(value, MinUser, MaxUser);
                _saveAt = DateTime.UtcNow.AddSeconds(0.5);
            }
        }
        const float MinUser = 0.5f, MaxUser = 2.0f;
        static DateTime _saveAt = DateTime.MaxValue;
        static string ScalePath => ModPaths.File("MissionPanelScale.txt");
        static void SaveScaleIfDue()
        {
            if (DateTime.UtcNow < _saveAt) return;
            _saveAt = DateTime.MaxValue;
            try { File.WriteAllText(ScalePath, _user.ToString("0.000", System.Globalization.CultureInfo.InvariantCulture)); } catch { }
        }
        static float2 P(float x, float y) => new(_o.X + x * S, _o.Y + y * S);

        static ImColor8 Col(float r, float g, float b, float a) => new((byte)(r * 255), (byte)(g * 255), (byte)(b * 255), (byte)(Math.Clamp(a, 0, 1) * 255));
        static ImColor8 Wh(float a) => Col(1, 1, 1, a);
        static readonly ImColor8 Red = new(0x58, 0x10, 0x26, 255);
        static readonly ImColor8 Warn = new(255, 120, 90, 255);
        static readonly ImColor8 Go = new(70, 220, 110, 255);

        static void Text(ImFontPtr f, float size, string s, float x, float y, ImColor8 c, int align = 0)
        {
            float w = align == 0 ? 0 : f.CalcTextSizeA(size, float.MaxValue, 0f, s).X;
            float px = align == 0 ? x : align == 1 ? x - w * 0.5f : x - w;
            _dl.AddText(f, size * S, P(px, y), c, s);
        }
        static float TextW(ImFontPtr f, float size, string s) => f.CalcTextSizeA(size, float.MaxValue, 0f, s).X;
        static void Label(string s, float x, float y, int align = 0) => Text(Webcast.FontSemi1, 11.5f, s.ToUpperInvariant(), x, y, Wh(0.5f), align);
        static void Line(float y) => _dl.AddLine(P(0, y), P(W - 2 * Pad, y), Wh(0.15f), 1f);

        static bool Hit(string id, float x, float y, float w, float h)
        {
            ImGui.SetCursorScreenPos(P(x, y));
            return ImGui.InvisibleButton(id, new float2(w * S, h * S));
        }

        static void Box(float x, float y, float w, float h, ImColor8 border, ImColor8? fill = null, float thick = 1f)
        {
            var a = P(x, y); var b = P(x + w, y + h);
            if (fill.HasValue) _dl.AddRectFilled(a, b, fill.Value);
            _dl.AddRect(a, b, border, 0f, ImDrawFlags.None, thick);
        }

        static void Bar(float x, float y, float w, double frac)
        {
            Box(x, y, w, 5, Wh(0f), Wh(0.15f));
            float f = (float)Math.Clamp(frac, 0, 1);
            if (f > 0) _dl.AddRectFilled(P(x, y), P(x + w * f, y + 5), Wh(1f));
        }

        /// arc dial like the webcast gauges; drag (up/right = more) or scroll while hovered to change the value
        static void Dial(string id, string label, string unit, ref float v, float min, float max, float step, string fmt, float cx, float y)
        {
            float R = 30f * S, Th = 4.5f * S;
            Hit(id, cx - 42, y, 84, 92);
            bool hov = ImGui.IsItemHovered(), act = ImGui.IsItemActive();
            var io = ImGui.GetIO();
            if (act) { v += (io.MouseDelta.X - io.MouseDelta.Y) * (max - min) / 300f; ImGui.SetMouseCursor(ImGuiMouseCursor.ResizeAll); }
            if (hov && io.MouseWheel != 0) v += io.MouseWheel * step;
            v = Math.Clamp((float)Math.Round(v / step) * step, min, max);
            var c = P(cx, y + 36);
            const float a0 = 159f, aIdeal = 339f, aEnd = 379f;
            void Arc(float from, float to, ImColor8 col)
            {
                _dl.PathArcTo(c, R, from * MathF.PI / 180f, to * MathF.PI / 180f, 0);
                _dl.PathStroke(col, ImDrawFlags.None, Th);
            }
            _dl.AddCircleFilled(c, R + 6 * S, Col(0, 0, 0, 0.35f), 40);
            Arc(a0, aIdeal, Wh(hov || act ? 0.65f : 0.4f));
            Arc(aIdeal, aEnd, Red);
            float frac = (v - min) / Math.Max(max - min, 1e-6f);
            Arc(a0, a0 + Math.Max(frac * (aIdeal - a0), 0.6f), Wh(1f));
            var s = v.ToString(fmt);
            var f = Webcast.FontSemi1;
            float sz = 19f * S, tw = TextW(f, sz, s);
            _dl.AddText(f, sz, new float2(c.X - tw / 2, c.Y - sz * 0.62f), Wh(1f), s);
            Label(label, cx, y + 72, 1);
            Text(Webcast.FontSemi1, 10.5f, unit.ToUpperInvariant(), cx, y + 85, Wh(0.35f), 1);
        }

        static bool Button(string id, string title, string sub, float x, float y, float w, float h, bool primary, ImColor8? accent = null)
        {
            bool click = Hit(id, x, y, w, h);
            bool hov = ImGui.IsItemHovered();
            var border = accent ?? (primary ? Wh(1f) : Wh(0.4f));
            Box(x, y, w, h, border, hov ? Wh(0.08f) : (ImColor8?)null, primary ? 1.5f : 1f);
            float ts = primary ? 18f : 14f;
            Text(Webcast.FontSemi1, ts, title, x + w / 2, y + (sub.Length > 0 ? h / 2 - ts * 0.85f : h / 2 - ts * 0.6f), accent ?? (primary ? Wh(1f) : Wh(0.8f)), 1);
            if (sub.Length > 0) Text(Webcast.FontSemi1, 10.5f, sub.ToUpperInvariant(), x + w / 2, y + h / 2 + 4, Wh(0.5f), 1);
            return click;
        }

        static string Clock()
        {
            double t = Webcast.MissionTime(), a = Math.Abs(t);
            return $"{(t < 0 ? "T-" : "T+")}{(int)(a / 3600):00}:{(int)(a / 60) % 60:00}:{(int)a % 60:00}";
        }

        public static void Draw(Vehicle? v, ref bool open)
        {
            var c = Autopilot.Cfg;
            ImGui.SetNextWindowPos(new float2(60, 120), ImGuiCond.FirstUseEver);
            var vps = ImGui.GetMainViewport().Size;
            S = Math.Max(Math.Min(vps.Y / 1080f, vps.X / 1920f), 0.5f) * UserScale;   // 1080p 1, 1440p 1.33, 4K 2 ...
            SaveScaleIfDue();
            ImGui.PushStyleVar(ImGuiStyleVar.WindowPadding, new float2(Pad * S, Pad * S));
            ImGui.PushStyleVar(ImGuiStyleVar.WindowRounding, 0f);
            ImGui.PushStyleVar(ImGuiStyleVar.WindowBorderSize, 0f);
            ImGui.PushStyleColor(ImGuiCol.WindowBg, new float4(0, 0, 0, 0));
            var flags = ImGuiWindowFlags.NoTitleBar | ImGuiWindowFlags.NoScrollbar | ImGuiWindowFlags.AlwaysAutoResize | ImGuiWindowFlags.NoCollapse;
            if (ImGui.Begin("Sparrow Mission Control", flags))
            {
                _dl = ImGui.GetWindowDrawList();
                var wp = ImGui.GetWindowPos(); var ws = ImGui.GetWindowSize();
                // plate with clipped corners (top-right, bottom-left) + hairline border
                float k = Cut * S;
                var poly = new[] { wp, new float2(wp.X + ws.X - k, wp.Y), new float2(wp.X + ws.X, wp.Y + k), new float2(wp.X + ws.X, wp.Y + ws.Y),
                                   new float2(wp.X + k, wp.Y + ws.Y), new float2(wp.X, wp.Y + ws.Y - k) };
                _dl.AddConvexPolyFilled(poly, Col(0.02f, 0.025f, 0.03f, 0.82f));
                _dl.AddPolyline(poly, Wh(0.16f), ImDrawFlags.Closed, 1f);
                _o = ImGui.GetCursorScreenPos();
                float cw = W - 2 * Pad, y = 0;

                // ---- header
                string name = v != null ? v.Id.Replace('_', ' ').ToUpperInvariant() : Webcast.MissionName;
                Label("Mission control", 0, y);
                Text(Webcast.FontSemi1, 24f, name, 0, y + 14, Wh(1f));
                Label("Status", cw - 22, y, 2);
                string status = Autopilot.AwaitingRecovery ? "MISSION COMPLETE"
                              : !Autopilot.Active ? (Autopilot.StackPhase == "ABORTED" ? "ABORTED" : "READY ON PAD") : Autopilot.StackPhase;
                Text(Webcast.FontSemi1, 15f, status, cw - 22, y + 16, Wh(1f), 2);
                if (Hit("close", cw - 14, y - 4, 18, 18)) open = false;
                Text(Webcast.FontSemi1, 15f, "X", cw - 5, y - 2, Wh(ImGui.IsItemHovered() ? 1f : 0.5f), 1);
                y += 50; Line(y); y += 14;

                if (!Autopilot.Active && !Autopilot.AwaitingRecovery)
                {
                    // ---- landing mode cards
                    Label("Landing", 0, y); y += 18;
                    float cwid = (cw - 10) / 2;
                    if (Hit("rtls", 0, y, cwid, 46)) c.Droneship = false;
                    Box(0, y, cwid, 46, c.Droneship ? Wh(0.2f) : Wh(1f), c.Droneship ? (ImColor8?)null : Wh(0.06f));
                    Text(Webcast.FontSemi1, 16f, "RTLS", 12, y + 6, Wh(1f)); Label("Return to launch site", 12, y + 27);
                    if (Hit("asds", cwid + 10, y, cwid, 46)) c.Droneship = true;
                    Box(cwid + 10, y, cwid, 46, c.Droneship ? Wh(1f) : Wh(0.2f), c.Droneship ? Wh(0.06f) : (ImColor8?)null);
                    Text(Webcast.FontSemi1, 16f, "DRONESHIP", cwid + 22, y + 6, Wh(1f)); Label(DroneShip.VehicleId, cwid + 22, y + 27);
                    y += 60;

                    // ---- dials
                    int n = c.Droneship ? 4 : 2;
                    float slot = cw / n;
                    Dial("d_orbit", "Orbit", "km", ref c.OrbitKm, 150f, 600f, 5f, "0", slot * 0.5f, y);
                    Dial("d_inc", "Inclination", "deg", ref c.InclinationDeg, 0f, 140f, 0.5f, "0.0", slot * 1.5f, y);
                    if (c.Droneship)
                    {
                        Dial("d_ship", "Ship", "km downrange", ref c.ShipKm, 250f, 700f, 5f, "0", slot * 2.5f, y);
                        Dial("d_over", "Overshoot", "km", ref c.AsdsOvershootKm, 0f, 30f, 0.5f, "0.0", slot * 3.5f, y);
                    }
                    y += 100;
                    Text(Webcast.FontSemi1, 10.5f, "DRAG OR SCROLL A DIAL TO SET IT", cw / 2, y, Wh(0.3f), 1);
                    y += 20;

                    // ---- vehicle readouts
                    if (v != null)
                    {
                        Autopilot.Measure(v);
                        double s1 = Autopilot.PropellantKg(v, Parts.S1Tank), s2 = Autopilot.PropellantKg(v, Parts.S2Tank);
                        double f1 = Autopilot.FillFraction(v, Parts.S1Tank), f2 = Autopilot.FillFraction(v, Parts.S2Tank);
                        float half = (cw - 20) / 2;
                        Label("Stage 1 propellant", 0, y); Label("Stage 2 propellant", half + 20, y);
                        Text(Webcast.FontMed1, 15f, f1 < 0 ? "MISSING" : $"{s1 / 1000:0.0} t  ·  {f1 * 100:0} %", 0, y + 14, Wh(1f));
                        Text(Webcast.FontMed1, 15f, f2 < 0 ? "MISSING" : $"{s2 / 1000:0.0} t  ·  {f2 * 100:0} %", half + 20, y + 14, f2 >= 0 && f2 < 0.9 ? Warn : Wh(1f));
                        Bar(0, y + 35, half, f1); Bar(half + 20, y + 35, half, f2);
                        y += 50;
                        Label("Lift-off mass", 0, y); Label("Launch azimuth", half + 20, y);
                        Text(Webcast.FontMed1, 15f, $"{v.Props.TotalMass / 1000:0.0} t", 0, y + 14, Wh(1f));
                        Text(Webcast.FontMed1, 15f, $"{c.HeadingDeg:000.0}°", half + 20, y + 14, Wh(1f));
                        float aw = TextW(Webcast.FontMed1, 15f, $"{c.HeadingDeg:000.0}°");
                        Label($"pad {c.LatitudeDeg:0.00}° N", half + 30 + aw, y + 17);
                        y += 40;
                        if (f2 >= 0 && f2 < 0.9) { Text(Webcast.FontMed1, 12.5f, "STAGE 2 TANK NOT FULL: ATTACH THE ANVIL VAC TO ITS BOTTOM ENGINE NODE", 0, y, Warn); y += 20; }
                        if (c.InclinationDeg < Math.Abs(c.LatitudeDeg)) { Text(Webcast.FontMed1, 12.5f, $"LOWEST INCLINATION FROM THIS PAD: {Math.Abs(c.LatitudeDeg):0.0}°", 0, y, Wh(0.6f)); y += 20; }
                    }

                    // ---- webcast toggle
                    Label("Webcast at launch", 0, y + 3);
                    if (Hit("wc", cw - 46, y, 46, 20)) Webcast.Enabled = !Webcast.Enabled;
                    Box(cw - 46, y, 46, 20, Webcast.Enabled ? Wh(1f) : Wh(0.3f), Webcast.Enabled ? Wh(0.1f) : (ImColor8?)null);
                    Text(Webcast.FontSemi1, 12f, Webcast.Enabled ? "ON" : "OFF", cw - 23, y + 3, Webcast.Enabled ? Wh(1f) : Wh(0.5f), 1);
                    y += 34;

                    // ---- launch / repeat
                    if (c.AtSupportedSite)
                    {
                        if (Button("launch", "LAUNCH  ·  " + (c.Droneship ? "DRONESHIP" : "RTLS"), Webcast.Enabled ? "T-10 countdown" : "T-3 countdown", 0, y, cw, 52, true))
                            Autopilot.WantSave = true;
                    }
                    else
                    {
                        // automated flights are KSC-only for now: no LAUNCH button elsewhere
                        Box(0, y, cw, 52, Warn);
                        Text(Webcast.FontSemi1, 15f, "AUTOMATED FLIGHTS: KSC ONLY FOR NOW", cw / 2, y + 9, Warn, 1);
                        Text(Webcast.FontSemi1, 10.5f, $"THIS PAD: {c.LatitudeDeg:0.00}°, {c.LongitudeDeg:0.00}°  ·  {c.SiteName.ToUpperInvariant()}", cw / 2, y + 31, Wh(0.5f), 1);
                    }
                    y += 52;
                }
                else
                {
                    // ---- in flight: clock, phases, telemetry, abort
                    Text(Webcast.FontMed1, 34f, Clock(), 0, y, Wh(1f));
                    Label(c.Droneship ? "Droneship landing" : "RTLS landing", cw, y + 6, 2);
                    Text(Webcast.FontSemi1, 13f, "S2  " + Autopilot.UpperPhase, cw, y + 22, Wh(0.8f), 2);
                    y += 50;
                    Label("Booster", 0, y);
                    _dl.AddText(Webcast.FontMed1, 13f * S, P(0, y + 15), Wh(0.9f), Autopilot.Telemetry, cw * S);
                    y += 15 + Webcast.FontMed1.CalcTextSizeA(13f, float.MaxValue, cw, Autopilot.Telemetry).Y + 10;
                    Label("Upper stage", 0, y);
                    _dl.AddText(Webcast.FontMed1, 13f * S, P(0, y + 15), Wh(0.9f), Autopilot.UpperTelemetry, cw * S);
                    y += 15 + Webcast.FontMed1.CalcTextSizeA(13f, float.MaxValue, cw, Autopilot.UpperTelemetry).Y + 14;
                    if (Autopilot.AwaitingRecovery)
                    {
                        // flight control is shut down: recover the landed booster (+ drone ship) and close, ready for the next flight
                        if (Button("recover", "RECOVER VEHICLE", c.Droneship ? "booster and drone ship" : "booster", 0, y, cw, 52, true, Go))
                        {
                            Autopilot.Recover();
                            open = false;
                        }
                        y += 62;
                    }
                    else
                    {
                        if (Button("abort", "ABORT", "", 0, y, cw, 40, false, Warn)) Autopilot.Abort("pilot");
                        y += 50;
                    }
                }

                // ---- event log (expandable)
                Line(y); y += 8;
                if (Hit("log", 0, y, cw, 18)) _log = !_log;
                Label((_log ? "- " : "+ ") + "Event log", 0, y + 2);
                y += 22;
                if (_log)
                {
                    lock (Autopilot.Events)
                    {
                        int start = Math.Max(0, Autopilot.Events.Count - 14);
                        for (int i = start; i < Autopilot.Events.Count; i++)
                        {
                            var e = Autopilot.Events[i];
                            _dl.AddText(Webcast.FontMed1, 12.5f * S, P(0, y), Wh(0.75f), e, cw * S);
                            y += Webcast.FontMed1.CalcTextSizeA(12.5f, float.MaxValue, cw, e).Y + 3;
                        }
                    }
                }
                // resize grip (bottom-right) + Ctrl+wheel anywhere over the panel
                float gs = 16f;
                Hit("resize", cw - gs + 6, y - 4, gs + 8, gs + 8);
                {
                    bool gh = ImGui.IsItemHovered(), ga = ImGui.IsItemActive();
                    var io = ImGui.GetIO();
                    if (ga)
                    {
                        // scale so the panel corner follows the mouse
                        var wp0 = ImGui.GetWindowPos();
                        float wantW = Math.Max(io.MousePos.X - wp0.X, 50f), baseS = S / UserScale;
                        UserScale = wantW / ((W) * baseS);
                        ImGui.SetMouseCursor(ImGuiMouseCursor.ResizeNWSE);
                    }
                    else if (gh) ImGui.SetMouseCursor(ImGuiMouseCursor.ResizeNWSE);
                    var col = Wh(ga ? 0.9f : gh ? 0.7f : 0.3f);
                    for (int i = 1; i <= 3; i++)
                        _dl.AddLine(P(cw + 6 - i * 5, y + gs - 4), P(cw + 6, y + gs - 4 - i * 5), col, 1.5f);
                }
                if (ImGui.IsWindowHovered(ImGuiHoveredFlags.ChildWindows) && ImGui.GetIO().KeyCtrl && ImGui.GetIO().MouseWheel != 0)
                    UserScale = UserScale * (1f + 0.08f * ImGui.GetIO().MouseWheel);
                y += gs - 4;
                ImGui.SetCursorScreenPos(_o);
                ImGui.Dummy(new float2(cw * S, y * S));
            }
            ImGui.End();
            ImGui.PopStyleColor();
            ImGui.PopStyleVar(3);
        }
    }
}
