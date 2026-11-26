// Continuous soot fade: blends the S1 textures at the exact soot level on a background thread and swaps them onto
// the "live" materials (SparrowPartsAssets.xml: SparrowS1_*SootLive*), so the booster darkens smoothly instead of
// stepping through the ladder.
//
// Same maths as branding/make_s1_soot.py: per texel coverage m and axial position t come from
// Textures/sparrow_s1_{skin,details}_sootmask.png (R = m / 0.93, G = t); level L -> ml = m * clamp(1.3 L - 0.3 t),
// colour = clean * (1 - ml) + soot(ml) * ml; roughness -> 0.86 and metal -> 0 under the soot.
//
// GPU side uses the game's own texture path: each update builds a TextureAsset from a GenericTexture, uploads it
// with a fresh StagingPool (Dispose = submit + wait, as ModLibrary does at load), registers a NEW bindless slot and
// points the live TextureReference at it; the previous slot + texture are freed ~0.35 s later, well after every
// frame in flight that could still sample them. Nothing is written into a slot that a pending frame uses.
using System.Diagnostics;
using Brutal.Numerics;
using Brutal.TextureApi;
using Brutal.TextureApi.Abstractions;
using KSA;
using RenderCore;
using Brutal.VulkanApi.Abstractions;

namespace SterlingFlight
{
    internal static class SootLive
    {
        static readonly float[] Soot = { 0.072f, 0.068f, 0.064f }, Warm = { 0.16f, 0.13f, 0.10f };

        sealed class Channel
        {
            public required string TexId, Clean, Mask; public required int Size; public required bool Rma;
            public byte[]? Src;            // clean texels, RGB, at Size
            public TextureReference? Ref;
            public SimpleVkTexture? Pending; // uploaded, not yet swapped in
            public bool Shown;
            public SimpleVkTexture? LastTex; // texture currently behind Ref (ours); the placeholder's own slot is never freed
        }

        static readonly Channel SkinDiff = new() { TexId = "SparrowS1_SkinSootLive_Diffuse", Clean = "sparrow_s1_skin_diffuse.png", Mask = "skin", Size = 2048, Rma = false };
        static readonly Channel PlainDiff = new() { TexId = "SparrowS1_SkinPlainSootLive_Diffuse", Clean = "sparrow_s1_skin_plain_diffuse.png", Mask = "skin", Size = 2048, Rma = false };
        static readonly Channel DetDiff = new() { TexId = "SparrowS1_DetailsSootLive_Diffuse", Clean = "sparrow_s1_details_diffuse.png", Mask = "details", Size = 2048, Rma = false };
        static readonly Channel SkinRma = new() { TexId = "SparrowS1_SkinSootLive_AoRoughMetal", Clean = "sparrow_s1_skin_roughmetalao.png", Mask = "skin", Size = 1024, Rma = true };
        static readonly Channel DetRma = new() { TexId = "SparrowS1_DetailsSootLive_AoRoughMetal", Clean = "sparrow_s1_details_roughmetalao.png", Mask = "details", Size = 1024, Rma = true };

        static readonly Dictionary<string, (byte[] M, byte[] T, int Size)> Masks = new();
        static Task? _job;
        static float _jobLevel = -1, _shownLevel = -1;
        static bool _jobPlain, _shownPlain;
        static bool _failed;
        static int _seq;
    static long _lastStart;
        static readonly List<(SimpleVkTexture Tex, int Handle, long Due)> ToFree = new();
        static System.Reflection.MethodInfo? _setHandle;

        static Channel[] For(bool plain) => new[] { plain ? PlainDiff : SkinDiff, DetDiff, SkinRma, DetRma };

        public static bool Failed => _failed;
        /// live textures are up for this livery at (about) this level
        public static bool ReadyFor(bool plain, float level) => !_failed && _shownLevel >= 0 && _shownPlain == plain && MathF.Abs(_shownLevel - level) < 0.03f;
        public static float ShownLevel => _shownLevel;

        static string TexDir()
        {
            var m = ModLibrary.Find(ModPaths.ModId);
            if (m == null || string.IsNullOrEmpty(m.DirectoryPath)) throw new InvalidOperationException(ModPaths.ModId + " mod not found");
            return System.IO.Path.Combine(m.DirectoryPath, "Textures");
        }

        /// (main thread) ask for the live textures at this level; ignored while a job runs or when already close
        public static void Request(float level, bool plain)
        {
            if (_failed || _job != null) return;
            if (_shownPlain == plain && _shownLevel >= 0 && MathF.Abs(level - _shownLevel) < 0.004f) return;
            long now = Stopwatch.GetTimestamp();
            if (_shownLevel >= 0 && now - _lastStart < Stopwatch.Frequency / 4) return;   // at most 4 updates a second
            _lastStart = now;
            _jobLevel = level; _jobPlain = plain;
            string dir;
            try { dir = TexDir(); } catch (Exception e) { Fail("tex dir: " + e.Message); return; }
            var chans = For(plain);
            _job = Task.Run(() =>
            {
                // same path as ModLibrary.Bind at load: build + upload on a worker with its own staging pool
                foreach (var c in chans) { LoadSources(dir, c); c.Pending = Build(c, Blend(c, level)); }
            });
        }

        static void Fail(string why) { _failed = true; Log.Error("soot live: " + why + " (falling back to the step ladder)"); }

        static byte[] LoadRgb(string path, int size, out int srcSize)
        {
            Brutal.TextureApi.ITexture tex = TextureLoader.LoadFromFile(path);
            int w = tex.Extent.X, h = tex.Extent.Y; srcSize = w;
            var data = tex.Data; int bpp = data.Length / (w * h);
            var outp = new byte[size * size * 3];
            for (int y = 0; y < size; y++)
            {
                int sy = y * h / size;
                for (int x = 0; x < size; x++)
                {
                    int sx = x * w / size, si = (sy * w + sx) * bpp, di = (y * size + x) * 3;
                    outp[di] = data[si]; outp[di + 1] = data[si + Math.Min(1, bpp - 1)]; outp[di + 2] = data[si + Math.Min(2, bpp - 1)];
                }
            }
            if (tex is IDisposable d) d.Dispose();
            return outp;
        }

        static void LoadSources(string dir, Channel c)
        {
            c.Src ??= LoadRgb(System.IO.Path.Combine(dir, c.Clean), c.Size, out _);
            lock (Masks)
            {
                string key = c.Mask + ":" + c.Size;
                if (Masks.ContainsKey(key)) return;
                var mk = LoadRgb(System.IO.Path.Combine(dir, $"sparrow_s1_{c.Mask}_sootmask.png"), c.Size, out _);
                var m = new byte[c.Size * c.Size]; var t = new byte[c.Size * c.Size];
                for (int i = 0; i < m.Length; i++) { m[i] = mk[i * 3]; t[i] = mk[i * 3 + 1]; }
                Masks[key] = (m, t, c.Size);
            }
        }

        static byte[] Blend(Channel c, float L)
        {
            (byte[] M, byte[] T, int Size) mk; lock (Masks) mk = Masks[c.Mask + ":" + c.Size];
            int n = c.Size; var src = c.Src!; var outp = new byte[n * n * 4];
            Parallel.For(0, n, y =>
            {
                for (int x = 0; x < n; x++)
                {
                    int i = y * n + x;
                    float m = mk.M[i] / 255f * 0.93f, t = mk.T[i] / 255f;
                    float ml = m * Math.Clamp(L * 1.3f - 0.3f * t, 0f, 1f);
                    int si = i * 3, di = i * 4;
                    if (c.Rma)
                    {
                        outp[di] = src[si];
                        outp[di + 1] = (byte)Math.Clamp(src[si + 1] * (1 - ml) + 0.86f * 255f * ml + 0.5f, 0, 255);
                        outp[di + 2] = (byte)Math.Clamp(src[si + 2] * (1 - ml) + 0.5f, 0, 255);
                    }
                    else
                    {
                        float a = Math.Clamp(ml / 0.6f, 0f, 1f);
                        for (int k = 0; k < 3; k++)
                        {
                            float col = Warm[k] + (Soot[k] - Warm[k]) * a;
                            float v = src[si + k] / 255f * (1 - ml) + col * ml;
                            outp[di + k] = (byte)Math.Clamp(v * 255f + 0.5f, 0, 255);
                        }
                    }
                    outp[di + 3] = 255;
                }
            });
            return outp;
        }

        /// (main thread, once per frame) finish a done job: upload + swap; free retired textures
        public static void Tick()
        {
            long now = Stopwatch.GetTimestamp();
            for (int i = ToFree.Count - 1; i >= 0; i--)
            {
                if (now < ToFree[i].Due) continue;
                try { Program.Instance.BindlessTextures.FreeTexture(ToFree[i].Handle); ToFree[i].Tex.Dispose(); }
                catch (Exception e) { Log.ErrorOnce("soot-free", e.ToString()); }
                ToFree.RemoveAt(i);
            }
            if (_job == null || !_job.IsCompleted) return;
            var job = _job; _job = null;
            if (job.IsFaulted) { Fail("blend: " + job.Exception?.GetBaseException().Message); return; }
            try
            {
                foreach (var c in For(_jobPlain)) { Swap(c); c.Pending = null; }
                _shownLevel = _jobLevel; _shownPlain = _jobPlain;
            }
            catch (Exception e) { Fail("upload: " + e); }
        }

        static SimpleVkTexture Build(Channel c, byte[] rgba)
        {
            var renderer = Program.GetRenderer();
            int n = c.Size;
            var gt = GenericTexture.Defaults.RGBA8UNorm(new int2(n, n));
            try
            {
                rgba.AsSpan().CopyTo(gt.Data);
                var asset = new TextureAsset(gt, $"SootLive_{c.TexId}_{Interlocked.Increment(ref _seq)}");
                using var sp = renderer.Allocator.CreateStagingPool(renderer.Graphics, 1);
                return new SimpleVkTexture(renderer.Allocator, sp, asset, new SimpleVkTexture.CreateOptions(true));
            }
            finally { gt.Destroy(); }
        }

        /// (main thread) register the new texture in a NEW bindless slot and point the live reference at it
        static void Swap(Channel c)
        {
            if (c.Pending == null) return;
            if (c.Ref == null)
            {
                if (!ModLibrary.TryGet<TextureReference>(c.TexId, out var r) || r == null) throw new InvalidOperationException("texture " + c.TexId + " not found");
                c.Ref = r;
            }
            _setHandle ??= typeof(TextureReference).GetProperty("BindlessHandle")!.GetSetMethod(true)
                           ?? throw new InvalidOperationException("BindlessHandle setter not found");
            int handle = Program.Instance.BindlessTextures.AddTexture(c.Pending.VkImageView);
            int old = c.Ref.BindlessHandle;
            _setHandle.Invoke(c.Ref, new object[] { handle });
            if (c.LastTex != null) ToFree.Add((c.LastTex, old, Stopwatch.GetTimestamp() + Stopwatch.Frequency * 35 / 100));
            c.LastTex = c.Pending; c.Shown = true;
        }
    }
}
