// Launch webcast overlay - a broadcast-style HUD driven by the RTLS autopilot's live telemetry,
// laid out on a 1920x1080 reference frame in Bai Jamjuree type.
//
// Flow: LAUNCH / REPEAT -> the game's own UI fades out (ImGui windows via style alpha, gauge canvases via their
// render tint) -> Program.DrawUI is switched off -> the webcast animates in: event dome rises, the timeline arc
// draws outward from "now", markers pop in, telemetry gauges sweep up, the caption slides in. The timeline scrolls
// continuously with mission time (past events left, future right, compressed with distance). F2 (the game's
// toggle-UI key) or Ctrl+Shift+W leaves the webcast and fades the game UI back in.
//
// Hooks:
//   Program.OnDrawUiFrame prefix   - scale ImGui style alpha while the game UI fades (first UI draw of the frame)
//   Program.DrawFps postfix        - draws the webcast (runs every frame, even with DrawUI off), restores alpha
//   CanvasRenderer.RenderInImGuiPass / GaugeCanvas.DrawFittedText prefixes - fade the gauge canvases
//   FontManager.RegenerateFonts postfix - adds Bai Jamjuree from the mod's Fonts folder (falls back to the game font)
using System.Diagnostics;
using System.Reflection;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace SterlingFlight
{
    internal static class Webcast
    {
        // ------------------------------------------------------------------ settings
        public static bool Enabled = true;                 // panel checkbox: go to the webcast view on LAUNCH
        public static string MissionName = "SPARROW";      // set from the vehicle name when the mission starts
        public const float CountdownWithWebcast = 10f;     // seconds of T- countdown when the webcast is on

        // ------------------------------------------------------------------ state
        enum Mode { Off, FadeUi, Intro, Live, Outro }
        static Mode _mode = Mode.Off;
        static double _modeRt;                             // real time the mode started
        public static float UiAlpha = 1f;                  // game UI opacity (read by the gauge tint patches)
        static float _uiFadeFrom = 1f, _uiFadeTo = 1f;
        static double _uiFadeRt = -10;
        static bool _weHidUi;                              // we switched Program.DrawUI off (F2 turning it on = leave)
        static double _cdStart = double.NaN;               // sim time the countdown started
        static readonly Stopwatch Clock = Stopwatch.StartNew();
        static double Rt => Clock.Elapsed.TotalSeconds;

        // ------------------------------------------------------------------ fonts
        internal static ImFontPtr FontMed, FontSemi;
        internal static bool HaveFonts;
        internal static string FontDir => ModPaths.File("Fonts");
        static string FontMedPath => Path.Combine(FontDir, "BaiJamjuree-Medium.ttf");
        static string FontSemiPath => Path.Combine(FontDir, "BaiJamjuree-SemiBold.ttf");
        internal static bool FontFilesPresent => File.Exists(FontMedPath) && File.Exists(FontSemiPath);

        internal static void LoadFonts()
        {
            HaveFonts = false;
            if (!FontFilesPresent) return;
            try
            {
                var atlas = ImGui.GetIO().Fonts;
                float density = GameSettings.GetFontDensity() / 100f;
                // baked large so the biggest webcast text (40 design units = 80 px at 4K) stays sharp; drawn smaller it is downsampled
                FontMed = FontManager.LoadFont(FontMedPath, atlas, 80f, false, density);
                FontSemi = FontManager.LoadFont(FontSemiPath, atlas, 80f, false, density);
                HaveFonts = !FontMed.IsNull() && !FontSemi.IsNull();
                Log.Info("webcast fonts " + (HaveFonts ? "loaded" : "failed"));
            }
            catch (Exception e) { Log.Error("webcast fonts: " + e.Message); }
        }

        /// ask the game to rebuild its font atlas once (our RegenerateFonts postfix then adds Bai Jamjuree)
        internal static void RequestFontRebuild()
        {
            if (!FontFilesPresent) { Log.Info("webcast: Bai Jamjuree not found in " + FontDir + ", using the game font"); return; }
            try { AccessTools.Field(typeof(Program), "_rebuildFontNeeded")?.SetValue(null, true); }
            catch (Exception e) { Log.Error("webcast font rebuild request: " + e.Message); }
        }

        // ------------------------------------------------------------------ timeline
        sealed class Ev
        {
            public string Key = "", Label = "", Title = "", Info = "";
            public double Nominal;          // nominal time from liftoff (s)
            public int Chain;               // 0 = shared/booster, 1 = second stage
            public double Actual = double.NaN;
            public double DoneRt = -100;
            public double Est;              // this frame's estimate (mission time)
            public bool Done => !double.IsNaN(Actual);
        }

        static readonly List<Ev> Events = new();
        static Ev? _caption, _prevCaption;
        static double _captionRt = -100;

        static void BuildTimeline()
        {
            Events.Clear();
            lock (Pred) Pred.Clear();
            void Add(string key, string label, double t, int chain, string title, string info) =>
                Events.Add(new Ev { Key = key, Label = label, Nominal = t, Chain = chain, Title = title, Info = info });
            Add("GO", "GO FOR LAUNCH", -12, 0, "GO FOR LAUNCH", "THE FLIGHT COMPUTER HAS TAKEN CONTROL OF THE COUNTDOWN");
            Add("STARTUP", "STARTUP", -2, 0, "ENGINE STARTUP", "ALL NINE ANVIL ENGINES ARE IGNITING AND THROTTLING UP BEFORE RELEASE");
            Add("LIFTOFF", "LIFTOFF", 0, 0, "LIFTOFF", "LIFTOFF OF SPARROW - THE VEHICLE HAS CLEARED THE TOWER");
            Add("MAXQ", "MAX-Q", 59, 0, "MAX-Q", "MAXIMUM DYNAMIC PRESSURE - THIS IS THE LARGEST AMOUNT OF AERODYNAMIC STRESS EXERTED ON THE VEHICLE");
            Add("MECO", "MECO", 127, 0, "MECO", "MAIN ENGINE CUTOFF - THE FIRST STAGE ENGINES HAVE SHUT DOWN");
            Add("SEP", "STAGE SEP", 129, 0, "STAGE SEPARATION", "THE FIRST AND SECOND STAGES HAVE SEPARATED");
            Add("SES1", "SES-1", 135, 1, "SECOND ENGINE START", "THE ANVIL VAC ENGINE HAS IGNITED ON THE SECOND STAGE");
            bool ds = Autopilot.Cfg.Droneship;
            if (!ds) Add("BOOSTBACK", "BOOSTBACK", 142, 0, "BOOSTBACK BURN", "THE FIRST STAGE HAS FLIPPED AND IS BURNING TO RETURN TO THE LAUNCH SITE");
            Add("FAIRING", "FAIRING SEP", 180, 1, "FAIRING SEPARATION", "THE PAYLOAD FAIRING HALVES HAVE BEEN JETTISONED");
            Add("ENTRY", "ENTRY BURN", 380, 0, "ENTRY BURN", "THE FIRST STAGE IS SLOWING DOWN TO PROTECT ITSELF DURING REENTRY");
            Add("LANDBURN", "LANDING BURN", 445, 0, "LANDING BURN", "A SINGLE ANVIL ENGINE IS SLOWING THE FIRST STAGE FOR TOUCHDOWN");
            Add("LANDING", "LANDING", ds ? 500 : 470, 0, "THE SPARROW HAS LANDED",
                ds ? "THE FIRST STAGE HAS TOUCHED DOWN ON THE DRONE SHIP" : "THE FIRST STAGE HAS TOUCHED DOWN BACK AT THE LAUNCH SITE");
            Add("SECO", "SECO", 540, 1, "SECO-1", "SECOND ENGINE CUTOFF - THE SECOND STAGE HAS REACHED ORBIT");
            _caption = _prevCaption = null;
        }

        /// mission time relative to (actual or predicted) liftoff, seconds
        internal static double MissionTime()
        {
            if (!double.IsNaN(Autopilot.T0)) return Autopilot.Now - Autopilot.T0;
            if (!double.IsNaN(_cdStart)) return Autopilot.Now - (_cdStart + Autopilot.CountdownSeconds + 2.0);
            return -CountdownWithWebcast - 2;
        }

        // live predictions from the guidance: key -> (mission time, real time of the last update)
        static readonly Dictionary<string, (double T, double Rt)> Pred = new();

        /// guidance hook: event `key` is expected `inSeconds` from now (smoothed; stale after 3 s)
        public static void Predict(string key, double inSeconds)
        {
            if (double.IsNaN(inSeconds) || double.IsInfinity(inSeconds) || inSeconds < 0 || inSeconds > 3600) return;
            double t = MissionTime() + inSeconds, rt = Rt;
            lock (Pred)
            {
                if (Pred.TryGetValue(key, out var old) && rt - old.Rt < 3)
                    t = old.T + (t - old.T) * Math.Clamp((rt - old.Rt) * 1.5, 0.05, 1.0);
                Pred[key] = (t, rt);
            }
        }

        /// estimate every event for this frame. Live guidance predictions win; otherwise the nominal time shifted by
        /// the drift of the last finished event. Pending events keep a share of their nominal spacing to the event
        /// before them, so one late event pushes the later ones along instead of bunching them up at "now".
        static void ComputeEstimates(double now)
        {
            double SepNominal = Events.Find(x => x.Key == "SEP")?.Nominal ?? 129;
            var lastEst = new double[2]; var lastNom = new double[2]; var lastPending = new bool[2];
            var drift = new double[2];
            bool first = true;
            foreach (var e in Events)   // listed in nominal order
            {
                int c = e.Chain;
                if (e.Done) e.Est = e.Actual;
                else
                {
                    double est = e.Nominal + drift[c];
                    bool live = false;
                    lock (Pred)
                        if (Pred.TryGetValue(e.Key, out var p) && Rt - p.Rt < 3) { est = p.T; live = true; }
                    if (!first)
                    {
                        double gap = Math.Max(e.Nominal - lastNom[c], 0);
                        double keep = live ? 1.0 : (lastPending[c] ? 0.6 * gap : 0.25 * gap);
                        est = Math.Max(est, lastEst[c] + Math.Max(keep, 1.0));
                    }
                    e.Est = Math.Max(est, now + 1.0);
                }
                first = false;
                lastEst[c] = e.Est; lastNom[c] = e.Nominal; lastPending[c] = !e.Done;
                if (e.Done) drift[c] = e.Actual - e.Nominal;
                if (c == 0 && e.Nominal <= SepNominal)   // the second stage shares everything up to separation
                {
                    lastEst[1] = e.Est; lastNom[1] = e.Nominal; lastPending[1] = !e.Done;
                    if (e.Done) drift[1] = e.Actual - e.Nominal;
                }
            }
        }

        /// autopilot hook: a mission phase or event happened
        public static void OnPhase(string phase, bool upper = false)
        {
            string? key = upper ? phase switch
            {
                "SES-1" => "SES1",
                "FAIRING" => "FAIRING",
                _ when phase.StartsWith("SECO") => "SECO",
                _ => null
            } : phase switch
            {
                "COUNTDOWN" => "GO",
                "IGNITION" => "STARTUP",
                "ASCENT" => "LIFTOFF",
                "MAXQ" => "MAXQ",
                "MECO" => "MECO",
                "STAGE SEP" => "SEP",
                "BOOSTBACK" => "BOOSTBACK",
                "ENTRY BURN" => "ENTRY",
                "LANDING BURN" => "LANDBURN",
                "LANDED" => "LANDING",
                _ => null
            };
            if (phase == "COUNTDOWN" && !upper) _cdStart = Autopilot.Now;
            if (phase == "ABORTED")
            {
                SetCaption(new Ev { Key = "ABORT", Title = "MISSION ABORTED", Info = "THE FLIGHT COMPUTER HAS ENDED THE AUTOMATED SEQUENCE" });
                return;
            }
            if (key == null) return;
            var ev = Events.Find(x => x.Key == key);
            if (ev == null || ev.Done) return;
            ev.Actual = MissionTime();
            if (key == "LIFTOFF") ev.Actual = 0;
            ev.DoneRt = Rt;
            SetCaption(ev);
        }

        static void SetCaption(Ev e)
        {
            if (_caption == e) return;
            _prevCaption = _caption;
            _caption = e;
            _captionRt = Rt;
        }

        // ------------------------------------------------------------------ mode control
        /// called when a mission starts (LAUNCH or REPEAT)
        public static void Begin(Vehicle? v = null)
        {
            if (v != null && !string.IsNullOrWhiteSpace(v.Id)) MissionName = v.Id.Replace('_', ' ').Trim().ToUpperInvariant();
            BuildTimeline();
            _cdStart = Autopilot.Now;
            if (!Enabled) return;
            _disp1 = _disp2 = default;
            if (_mode == Mode.Live || _mode == Mode.Intro) { SetMode(Mode.Intro); return; }
            if (Program.DrawUI) { FadeUi(0f, 0.7f); SetMode(Mode.FadeUi); }
            else { _weHidUi = false; SetMode(Mode.Intro); }
        }

        public static void Leave()
        {
            if (_mode == Mode.Off || _mode == Mode.Outro) return;
            SetMode(Mode.Outro);
        }

        public static bool Showing => _mode != Mode.Off;

        static void SetMode(Mode m) { _mode = m; _modeRt = Rt; }

        static float _uiFadeDur = 0.7f;
        static void FadeUi(float to, float dur) { _uiFadeFrom = UiAlpha; _uiFadeTo = to; _uiFadeRt = Rt; _uiFadeDur = dur; }

        static void UpdateUiAlpha()
        {
            float k = (float)Math.Clamp((Rt - _uiFadeRt) / _uiFadeDur, 0, 1);
            UiAlpha = _uiFadeFrom + (_uiFadeTo - _uiFadeFrom) * Ease.InOutSine(k);
        }

        /// per frame, before any game UI is drawn
        internal static float _savedStyleAlpha = -1f;
        internal static void BeforeUi()
        {
            UpdateUiAlpha();
            if (UiAlpha >= 0.999f) return;
            var st = ImGui.GetStyle();
            _savedStyleAlpha = st.Alpha;
            st.Alpha = Math.Max(0.0001f, st.Alpha * UiAlpha);
        }

        /// per frame, after all game UI (Program.DrawFps postfix)
        internal static void Frame()
        {
            if (_savedStyleAlpha >= 0f) { ImGui.GetStyle().Alpha = _savedStyleAlpha; _savedStyleAlpha = -1f; }
            UpdateUiAlpha();

            var io = ImGui.GetIO();
            if (io.KeyCtrl && io.KeyShift && ImGui.IsKeyPressed(ImGuiKey.W, repeat: false))
            {
                if (_mode == Mode.Off) { if (Events.Count == 0) BuildTimeline(); if (Program.DrawUI) { FadeUi(0f, 0.7f); SetMode(Mode.FadeUi); } else SetMode(Mode.Intro); }
                else Leave();
            }

            switch (_mode)
            {
                case Mode.Off: return;
                case Mode.FadeUi:
                    if (Rt - _modeRt >= _uiFadeDur)
                    {
                        Program.DrawUI = false;
                        _weHidUi = true;
                        UiAlpha = 0f; _uiFadeFrom = _uiFadeTo = 0f;
                        SetMode(Mode.Intro);
                    }
                    return;
                case Mode.Intro:
                case Mode.Live:
                    if (_weHidUi && Program.DrawUI) { Leave(); break; }   // F2: the player wants the game UI back
                    if (_mode == Mode.Intro && Rt - _modeRt > 2.2) SetMode(Mode.Live);
                    break;
                case Mode.Outro:
                    if (Rt - _modeRt > 0.45)
                    {
                        if (!Program.DrawUI) Program.DrawUI = true;
                        _weHidUi = false;
                        UiAlpha = 0f; FadeUi(1f, 0.6f);
                        SetMode(Mode.Off);
                        return;
                    }
                    break;
            }
            Draw();
        }

        // ------------------------------------------------------------------ drawing
        static ImDrawListPtr _dl;
        static float _s;                       // pixels per design unit
        static float2 _vpPos, _vpSize;
        static float _ox;                      // x of the centred 1920-wide design frame
        static float _oy;                      // y of the design frame's top (frame pinned to the screen bottom)
        static float _a = 1f;                  // overall overlay opacity (outro)
        static double _intro;                  // seconds since the intro started (large when live)

        static float2 Pc(double x, double y) => new((float)(_ox + x * _s), (float)(_oy + y * _s));
        static float2 Pl(double x, double y) => new((float)(_vpPos.X + x * _s), (float)(_oy + y * _s));
        static float2 Pr(double x, double y) => new((float)(_vpPos.X + _vpSize.X - (1920 - x) * _s), (float)(_oy + y * _s));

        static ImColor8 Col(float r, float g, float b, float a) =>
            new((byte)(r * 255), (byte)(g * 255), (byte)(b * 255), (byte)(Math.Clamp(a * _a, 0f, 1f) * 255));
        static ImColor8 White(float a) => Col(1, 1, 1, a);
        static ImColor8 Black(float a) => Col(0, 0, 0, a);

        static float A(double start, double dur) => (float)Math.Clamp((_intro - start) / dur, 0, 1);

        static void Draw()
        {
            var vp = ImGui.GetMainViewport();
            _vpPos = vp.Pos; _vpSize = vp.Size;
            if (_vpSize.Y < 10) return;
            // 1920x1080 design frame scaled to fit any resolution / aspect (1080p = 1, 1440p = 1.33, 4K = 2; 16:10 and
            // narrower fit the width), centred horizontally and pinned to the bottom, where every webcast element lives
            _s = Math.Min(_vpSize.Y / 1080f, _vpSize.X / 1920f);
            _ox = _vpPos.X + (_vpSize.X - 1920f * _s) * 0.5f;
            _oy = _vpPos.Y + _vpSize.Y - 1080f * _s;
            _dl = ImGui.GetBackgroundDrawList(vp);
            _intro = _mode == Mode.Intro ? Rt - _modeRt : 100.0;
            _a = _mode == Mode.Outro ? 1f - Ease.InOutSine((float)Math.Clamp((Rt - _modeRt) / 0.45, 0, 1)) : 1f;

            double t = MissionTime();
            ComputeEstimates(t);
            DrawTimeline(t);
            DrawCountdown(t);
            DrawTelemetry();
            DrawCaption();
        }

        // --- the event dome + flow arc ----------------------------------------------------------------
        const double ArcCx = 954.0, ArcTop = 918.0, ArcR = 948.0;    // circle through the design's flow line
        const double ArcMaxDeg = 35.0;

        static double AngleFor(double dt)
        {
            double a = 0.3 * Math.Pow(Math.Abs(dt), 0.75);
            return Math.Sign(dt) * Math.Min(a, ArcMaxDeg + 5);
        }

        static float2 ArcPt(double deg, double radialOffset, float dy)
        {
            double th = deg * Math.PI / 180, r = ArcR + radialOffset;
            var p = Pc(ArcCx + r * Math.Sin(th), ArcTop + ArcR - r * Math.Cos(th));
            return new float2(p.X, p.Y + dy);
        }

        static void DrawTimeline(double t)
        {
            float aDome = Ease.OutCubic(A(0.0, 0.6));
            float dy = (1 - aDome) * 70 * _s;

            // dome overlay (black 15 %)
            var dome = new List<float2>();
            float2 O(double x, double y) { var p = Pc(325 + x, 874 + y); return new float2(p.X, p.Y + dy); }
            dome.Add(O(0.09, 206));
            Bez(dome, O(0.09, 206), O(-5.57, 206), O(266.75, 4.49), O(623.24, 0.07));
            Bez(dome, O(623.24, 0.07), O(979.73, -4.35), O(1257, 206), O(1257, 206));
            _dl.AddConvexPolyFilled(dome.ToArray(), Black(0.15f * aDome));

            // flow arc, drawn outward from "now"
            float reveal = Ease.OutCubic(A(0.3, 0.9));
            double half = reveal * ArcMaxDeg;
            if (half > 0.2)
            {
                var pts = new List<float2>();
                for (double d = -half; d <= half + 1e-6; d += 0.5) pts.Add(ArcPt(d, 0, dy));
                _dl.AddPolyline(pts.ToArray(), White(1f * aDome), ImDrawFlags.None, 2f * _s);
            }
            // "now" tick
            var tk0 = Pc(953.5, 911); var tk1 = Pc(953.5, 923);
            _dl.AddLine(new float2(tk0.X, tk0.Y + dy), new float2(tk1.X, tk1.Y + dy), White(aDome), 2f * _s);

            // events
            int i = 0;
            foreach (var e in Events)
            {
                bool above = (i++ & 1) == 0;
                double est = e.Est;
                double deg = AngleFor(est - t);
                if (Math.Abs(deg) > ArcMaxDeg) continue;
                // pop in as the arc reaches the marker
                float pop = (float)Math.Clamp((half - Math.Abs(deg)) / 3.0, 0, 1);
                if (pop <= 0) continue;
                float sc = Ease.OutBack(pop);
                float edge = (float)Math.Clamp((ArcMaxDeg - Math.Abs(deg)) / 5.0, 0, 1);   // fade at the arc ends
                float alpha = aDome * edge;
                DrawMarker(deg, above, e.Done, sc, alpha, dy, e.DoneRt);
                DrawArcLabel(e.Label, deg, above, e.Done ? 1f : 0.5f, alpha * pop, dy);
            }
        }

        static void DrawMarker(double deg, bool above, bool done, float scale, float alpha, float dy, double doneRt)
        {
            double sgn = above ? 1 : -1;
            var basePt = ArcPt(deg, 0, dy);
            var center = ArcPt(deg, sgn * 12 * scale, dy);
            var stemEnd = ArcPt(deg, sgn * 4.75 * scale, dy);
            float r = 7.25f * _s * scale;
            _dl.AddLine(stemEnd, basePt, White(alpha), 2f * _s);
            _dl.AddCircleFilled(center, r, Black(alpha), 24);
            _dl.AddCircle(center, r, White(alpha), 24, 1.5f * _s);
            if (done) _dl.AddCircleFilled(center, 3f * _s * scale, White(alpha), 16);
            // reached: expanding pulse ring
            double p = (Rt - doneRt) / 1.2;
            if (p >= 0 && p < 1)
            {
                float k = Ease.OutCubic((float)p);
                _dl.AddCircle(center, r + 16f * _s * k, White(alpha * (1 - k) * 0.9f), 32, 2f * _s);
            }
        }

        static void DrawArcLabel(string text, double deg, bool above, float bright, float alpha, float dy)
        {
            if (alpha <= 0.01f) return;
            float size = 14f * _s;
            var font = FontSemi1;
            var sz = font.CalcTextSizeA(size, float.MaxValue, 0f, text);
            var pivot = ArcPt(deg, (above ? 30.5 : -30.5), dy);
            int start = _dl.VtxBuffer.Count;
            _dl.AddText(font, size, new float2(pivot.X - sz.X * 0.5f, pivot.Y - sz.Y * 0.5f), White(bright * alpha), text);
            RotateVerts(start, pivot, (float)(deg * Math.PI / 180));
        }

        // --- clock --------------------------------------------------------------------------------------
        static void DrawCountdown(double t)
        {
            float a = Ease.OutCubic(A(0.15, 0.65));
            float dy = (1 - a) * 60 * _s;
            var ov = new List<float2>();
            float2 O(double x, double y) { var p = Pc(508 + x, 961 + y); return new float2(p.X, p.Y + dy); }
            var curve = new List<float2> { O(2.06, 119) };
            Bez(curve, O(2.06, 119), O(-1.94, 119), O(190.44, 4.55), O(442.28, 2.04));
            Bez(curve, O(442.28, 2.04), O(694.12, -0.47), O(890, 119), O(890, 119));
            ov.AddRange(curve);
            _dl.AddConvexPolyFilled(ov.ToArray(), Black(0.5f * a));
            _dl.AddPolyline(curve.ToArray(), White(0.15f * a), ImDrawFlags.None, 2f * _s);

            var at = Math.Abs(t);
            int hh = (int)(at / 3600), mm = (int)(at / 60) % 60, ss = (int)at % 60;
            string clock = $"{(t < 0 ? "T-" : "T+")}{hh:00}:{mm:00}:{ss:00}";
            float ta = Ease.OutCubic(A(0.35, 0.6));
            TextCentered(FontMed1, 43f, clock, 954, 977, White(ta), dy, false);
            TextCentered(FontSemi1, 16f, MissionName, 954, 1031, White(0.5f * ta), dy, false);
        }

        // --- telemetry ----------------------------------------------------------------------------------
        struct Disp { public double Spd, Alt; public bool Init; }
        static Disp _disp1, _disp2;

        static void DrawTelemetry()
        {
            var booster = Autopilot.Booster ?? Autopilot.Stack ?? Program.ControlledVehicle;   // preview: Ctrl+Shift+W before launch
            var upper = Autopilot.Upper;
            float a = Ease.OutCubic(A(0.5, 0.7));
            float sweep = Ease.InOutSine(A(0.75, 0.9));
            float slide = (1 - a) * 160 * _s;

            if (Read(booster, false, out double spd1, out double alt1))
            {
                Smooth(ref _disp1, spd1, alt1);
                TelemetryBlock(left: true, slide, a, sweep, _disp1, upper != null ? "STAGE 1 TELEMETRY" : "STAGE 1 TELEMETRY", 7000, 120);
            }
            if (upper != null && Read(upper, true, out double spd2, out double alt2))
            {
                if (!_disp2.Init) _upperShownRt = Rt;
                Smooth(ref _disp2, spd2, alt2);
                float b = Ease.OutCubic((float)Math.Clamp((Rt - _upperShownRt) / 0.7, 0, 1)) * a;
                float sw2 = Ease.InOutSine((float)Math.Clamp((Rt - _upperShownRt - 0.25) / 0.9, 0, 1)) * sweep;
                TelemetryBlock(left: false, (1 - b) * 160 * _s, b, sw2, _disp2, "STAGE 2 TELEMETRY", 28500, Autopilot.Cfg.OrbitKm + 60);
            }
            else _disp2 = default;
        }
        static double _upperShownRt;

        static bool Read(Vehicle? v, bool inertial, out double kmh, out double km)
        {
            kmh = km = 0;
            if (v?.Orbit?.Parent == null) return false;
            try
            {
                var n = new Nav(v);
                kmh = V.Len(inertial ? n.Vel : n.VSurf) * 3.6;
                km = Math.Max(0, n.Alt / 1000.0);
                return true;
            }
            catch { return false; }
        }

        static void Smooth(ref Disp d, double spd, double alt)
        {
            if (!d.Init) { d.Spd = spd; d.Alt = alt; d.Init = true; return; }
            double k = Math.Clamp(ImGui.GetIO().DeltaTime * 10.0, 0, 1);
            d.Spd += (spd - d.Spd) * k; d.Alt += (alt - d.Alt) * k;
        }

        static void TelemetryBlock(bool left, float slide, float a, float sweep, Disp d, string title, double spdFull, double altFull)
        {
            if (a <= 0.01f) return;
            float sx = left ? -slide : slide;
            // panel with radial falloff (design: radial gradient, black 50 % -> 0)
            Panel(left, sx, 898.5, a);
            Gauge(left ? 165 : 1920 - 327, 984, left, sx, a, sweep, d.Spd / spdFull, "SPEED", ((int)Math.Round(d.Spd)).ToString(), "KM/H");
            string alt = d.Alt < 100 ? d.Alt.ToString("0.0") : d.Alt.ToString("0");
            Gauge(left ? 327 : 1920 - 165, 984, left, sx, a, sweep, d.Alt / altFull, "ALTITUDE", alt, "KM");
            var c = left ? Pl(248, 1026) : Pr(1920 - 248, 1026);
            TextCenteredAt(FontSemi1, 17f, title, new float2(c.X + sx, c.Y), White(a));
        }

        /// the corner panel shape (design "Overlay" 623.5 x 181.5) filled with its radial gradient via a vertex grid
        static void Panel(bool left, float sx, double top, float a)
        {
            const int Rows = 18, Cols = 22;
            var flags = _dl.Flags; _dl.Flags &= ~ImDrawListFlags.AntiAliasedFill;
            for (int r = 0; r < Rows; r++)
                for (int c = 0; c < Cols; c++)
                {
                    double y0 = 181.5 * r / Rows, y1 = 181.5 * (r + 1) / Rows;
                    double x00 = RightEdge(y0) * c / Cols, x01 = RightEdge(y0) * (c + 1) / Cols;
                    double x10 = RightEdge(y1) * c / Cols, x11 = RightEdge(y1) * (c + 1) / Cols;
                    var q = new[] { PP(x00, y0), PP(x01, y0), PP(x11, y1), PP(x10, y1) };
                    int start = _dl.VtxBuffer.Count;
                    _dl.AddConvexPolyFilled(q, Black(0.5f * a));
                    var span = _dl.VtxBuffer.Span;
                    double[] lx = { x00, x01, x11, x10 }, ly = { y0, y0, y1, y1 };
                    for (int k = start, j = 0; k < span.Length && j < 4; k++, j++)
                    {
                        var vtx = span[k];
                        float m = RadialFall(lx[j], ly[j]);
                        uint al = vtx.col >> 24;
                        vtx.col = (vtx.col & 0x00FFFFFFu) | ((uint)(al * m) << 24);
                        span[k] = vtx;
                    }
                }
            _dl.Flags = flags;

            float2 PP(double x, double y)
            {
                var p = left ? Pl(x, top + y) : Pr(1920 - x, top + y);
                return new float2(p.X + sx, p.Y);
            }
        }

        static double RightEdge(double y) => y < 17
            ? 363.9 + (413.2 - 363.9) * Math.Sqrt(y / 17.0)
            : 413.2 + (y - 17) * (623.5 - 413.2) / (181.5 - 17);

        static float RadialFall(double x, double y)
        {
            double dx = x - 137, dy = y + 214, ang = 49.8684 * Math.PI / 180;
            double u = (dx * Math.Cos(ang) + dy * Math.Sin(ang)) / 449.929;
            double w = (-dx * Math.Sin(ang) + dy * Math.Cos(ang)) / 377.523;
            double tt = Math.Sqrt(u * u + w * w);
            return tt <= 0.203 ? 1f : (float)Math.Clamp(1 - (tt - 0.203) / 0.797, 0, 1);
        }

        static void Gauge(double cx, double cy, bool left, float sx, float a, float sweep, double frac, string label, string value, string unit)
        {
            var c0 = left ? Pl(cx, cy) : Pr(cx, cy);
            var c = new float2(c0.X + sx, c0.Y);
            // backdrop disc, black 50 % fading out downward
            int start = _dl.VtxBuffer.Count;
            _dl.AddCircleFilled(c, 75f * _s, Black(0.5f * a), 48);
            var span = _dl.VtxBuffer.Span;
            float top = c.Y - 75f * _s;
            for (int k = start; k < span.Length; k++)
            {
                var vtx = span[k];
                float ly = (vtx.pos.Y - top) / _s;
                float m = ly < 53 ? 1f : Math.Clamp(1f - (ly - 53f) / 65f, 0f, 1f);
                vtx.col = (vtx.col & 0x00FFFFFFu) | ((uint)((vtx.col >> 24) * m) << 24);
                span[k] = vtx;
            }
            // dial: 220 deg sweep from lower left over the top; last 40 deg is the red over-limit zone
            float rr = 67.5f * _s, th = 5.5f * _s;
            const float a0 = 159f, aIdeal = 339f, aEnd = 379f;
            Arc(c, rr, a0, aIdeal, White(0.5f * a), th);
            Arc(c, rr, aIdeal, aEnd, Col(0x58 / 255f, 0x10 / 255f, 0x26 / 255f, a), th);
            float cur = a0 + (float)Math.Clamp(frac * (aIdeal - a0), 0, aEnd - a0) * sweep;
            if (cur > a0 + 0.5f) Arc(c, rr, a0, cur, White(a), th);

            TextCenteredAt(FontSemi1, 14f, label, new float2(c.X, c.Y - 39.5f * _s + 9f * _s), White(0.5f * a));
            TextCenteredAt(FontSemi1, 40f, value, new float2(c.X, c.Y - 1.5f * _s), White(a));
            TextCenteredAt(FontSemi1, 14f, unit, new float2(c.X, c.Y + 30.5f * _s), White(0.5f * a));
        }

        static void Arc(float2 c, float r, float fromDeg, float toDeg, ImColor8 col, float thick)
        {
            _dl.PathArcTo(c, r, fromDeg * MathF.PI / 180f, toDeg * MathF.PI / 180f, 0);
            _dl.PathStroke(col, ImDrawFlags.None, thick);
        }

        // --- caption (bottom right; above stage 2 telemetry once that is shown) ------------------------
        static void DrawCaption()
        {
            float a = Ease.OutCubic(A(0.6, 0.7));
            float slide = (1 - a) * 160 * _s;
            bool twoBlocks = _disp2.Init;
            if (!twoBlocks) Panel(false, slide, 897, a);
            double baseY = twoBlocks ? 760 : 903;
            double since = Rt - _captionRt;
            if (_prevCaption != null && since < 0.3)
            {
                float k = Ease.InOutSine((float)(since / 0.3));
                CaptionText(_prevCaption, baseY, slide, a * (1 - k), -12 * k);
            }
            if (_caption != null)
            {
                float k = Ease.OutCubic((float)Math.Clamp((since - 0.15) / 0.5, 0, 1));
                CaptionText(_caption, baseY, slide + (1 - k) * 50 * _s, a * k, 0);
            }
        }

        static void CaptionText(Ev e, double baseY, float sx, float a, float dyUnits)
        {
            if (a <= 0.01f) return;
            var right = Pr(1822, baseY + dyUnits);
            float x = right.X + sx;
            var f = FontSemi1;
            float ts = 25f * _s;
            var tsz = f.CalcTextSizeA(ts, float.MaxValue, 0f, e.Title);
            _dl.AddText(f, ts, new float2(x - tsz.X, right.Y), White(a), e.Title);
            // description, right aligned, wrapped to 336 design units
            var fm = FontMed1;
            float bs = 16f * _s, maxW = 336f * _s;
            float y = right.Y + 30f * _s;
            foreach (var line in Wrap(fm, bs, e.Info, maxW))
            {
                var lsz = fm.CalcTextSizeA(bs, float.MaxValue, 0f, line);
                _dl.AddText(fm, bs, new float2(x - lsz.X, y), White(a), line);
                y += lsz.Y;
            }
        }

        static IEnumerable<string> Wrap(ImFontPtr f, float size, string text, float maxW)
        {
            var line = "";
            foreach (var w in text.Split(' ', StringSplitOptions.RemoveEmptyEntries))
            {
                var tryL = line.Length == 0 ? w : line + " " + w;
                if (line.Length > 0 && f.CalcTextSizeA(size, float.MaxValue, 0f, tryL).X > maxW) { yield return line; line = w; }
                else line = tryL;
            }
            if (line.Length > 0) yield return line;
        }

        // --- helpers ----------------------------------------------------------------------------------
        internal static ImFontPtr FontMed1 => HaveFonts ? FontMed : ImGui.GetFont();
        internal static ImFontPtr FontSemi1 => HaveFonts ? FontSemi : ImGui.GetFont();

        static void TextCentered(ImFontPtr f, float sizeUnits, string text, double cx, double top, ImColor8 col, float dy, bool _)
        {
            float size = sizeUnits * _s;
            var p = Pc(cx, top);
            var sz = f.CalcTextSizeA(size, float.MaxValue, 0f, text);
            _dl.AddText(f, size, new float2(p.X - sz.X * 0.5f, p.Y + dy), col, text);
        }

        static void TextCenteredAt(ImFontPtr f, float sizeUnits, string text, float2 center, ImColor8 col)
        {
            float size = sizeUnits * _s;
            var sz = f.CalcTextSizeA(size, float.MaxValue, 0f, text);
            _dl.AddText(f, size, new float2(center.X - sz.X * 0.5f, center.Y - sz.Y * 0.5f), col, text);
        }

        static void RotateVerts(int start, float2 pivot, float ang)
        {
            float c = MathF.Cos(ang), s = MathF.Sin(ang);
            var span = _dl.VtxBuffer.Span;
            for (int k = start; k < span.Length; k++)
            {
                var v = span[k];
                float dx = v.pos.X - pivot.X, dy = v.pos.Y - pivot.Y;
                v.pos = new float2(pivot.X + dx * c - dy * s, pivot.Y + dx * s + dy * c);
                span[k] = v;
            }
        }

        static void Bez(List<float2> pts, float2 p0, float2 p1, float2 p2, float2 p3, int n = 24)
        {
            for (int i = 1; i <= n; i++)
            {
                float t = i / (float)n, u = 1 - t;
                float b0 = u * u * u, b1 = 3 * u * u * t, b2 = 3 * u * t * t, b3 = t * t * t;
                pts.Add(new float2(b0 * p0.X + b1 * p1.X + b2 * p2.X + b3 * p3.X, b0 * p0.Y + b1 * p1.Y + b2 * p2.Y + b3 * p3.Y));
            }
        }
    }

    internal static class Ease
    {
        public static float OutCubic(float t) { t = Math.Clamp(t, 0f, 1f); float u = 1 - t; return 1 - u * u * u; }
        public static float InOutSine(float t) { t = Math.Clamp(t, 0f, 1f); return 0.5f - 0.5f * MathF.Cos(MathF.PI * t); }
        public static float OutBack(float t)
        {
            t = Math.Clamp(t, 0f, 1f);
            const float c1 = 1.70158f, c3 = c1 + 1;
            float u = t - 1;
            return 1 + c3 * u * u * u + c1 * u * u;
        }
    }

    // ====================================================================== hooks
    [HarmonyPatch(typeof(Program), "OnDrawUiFrame")]
    internal static class WebcastUiAlpha
    {
        [HarmonyPrefix, HarmonyPriority(Priority.First)]
        static void Prefix()
        {
            try { Webcast.BeforeUi(); } catch (Exception e) { Log.ErrorOnce("wc-alpha", e.ToString()); }
        }
    }

    [HarmonyPatch(typeof(Program), "DrawFps")]
    internal static class WebcastDraw
    {
        [HarmonyPostfix]
        static void Postfix()
        {
            try { Webcast.Frame(); } catch (Exception e) { Log.ErrorOnce("wc-draw", e.ToString()); }
        }
    }

    [HarmonyPatch(typeof(CanvasRenderer), "RenderInImGuiPass")]
    internal static class WebcastGaugeFade
    {
        [HarmonyPrefix]
        static void Prefix(ref float4 tint) { if (Webcast.UiAlpha < 0.999f) tint.W *= Webcast.UiAlpha; }
    }

    [HarmonyPatch]
    internal static class WebcastGaugeTextFade
    {
        static IEnumerable<MethodBase> TargetMethods() =>
            typeof(GaugeCanvas).GetMethods(BindingFlags.Static | BindingFlags.NonPublic | BindingFlags.Public)
                .Where(m => m.Name == "DrawFittedText" && m.GetParameters().Any(p => p.Name == "tint"));

        [HarmonyPrefix]
        static void Prefix(ref float4 tint) { if (Webcast.UiAlpha < 0.999f) tint.W *= Webcast.UiAlpha; }
    }

    /// gauge canvases punch an "opaque UI" hole into the 3D render (coverage mask). It is skipped by the game when the
    /// tint is translucent, so fading gauges must report their faded tint here too - otherwise they leave black boxes.
    [HarmonyPatch(typeof(GaugeCanvas), "RegisterOpaqueCoverage")]
    internal static class WebcastGaugeCoverage
    {
        [HarmonyPrefix]
        static void Prefix(ref float4 tint) { if (Webcast.UiAlpha < 0.999f) tint.W *= Webcast.UiAlpha; }
    }

    [HarmonyPatch(typeof(FontManager), nameof(FontManager.RegenerateFonts))]
    internal static class WebcastFonts
    {
        [HarmonyPostfix]
        static void Postfix()
        {
            try { Webcast.LoadFonts(); } catch (Exception e) { Log.ErrorOnce("wc-font", e.ToString()); }
        }
    }
}
