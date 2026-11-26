// "Sterling Orbital" menu in the game's top menu bar (after File / Universe / View / HUD), with a toggle for the Sparrow
// mission control panel. The game calls an empty Program.DrawProgramMenusHook() at that spot; an empty method can be
// inlined by the JIT, so instead of patching it we rewrite DrawMenuBar's call to it into a call to Draw() below.
using System.Reflection;
using System.Reflection.Emit;
using Brutal.ImGuiApi;
using HarmonyLib;
using KSA;

namespace SterlingFlight
{
    [HarmonyPatch(typeof(Program), nameof(Program.DrawMenuBar))]
    internal static class SterlingMenu
    {
        static readonly MethodInfo HookMethod = AccessTools.Method(typeof(Program), nameof(Program.DrawProgramMenusHook));
        static readonly MethodInfo DrawMethod = AccessTools.Method(typeof(SterlingMenu), nameof(Draw));

        [HarmonyTranspiler]
        static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> code)
        {
            bool done = false;
            foreach (var ins in code)
            {
                if (!done && (ins.opcode == OpCodes.Call || ins.opcode == OpCodes.Callvirt) && ins.operand as MethodInfo == HookMethod)
                {
                    // stack holds 'this' (Program): add the viewport (arg 1) and call Draw(Program, IGameViewport)
                    yield return new CodeInstruction(OpCodes.Ldarg_1).WithLabels(ins.ExtractLabels());
                    yield return new CodeInstruction(OpCodes.Call, DrawMethod);
                    done = true;
                    continue;
                }
                yield return ins;
            }
            if (!done) Log.Error("Sterling Orbital menu: DrawProgramMenusHook call not found in Program.DrawMenuBar");
        }

        static void Draw(Program self, IGameViewport viewport)
        {
            try { self.DrawProgramMenusHook(); } catch { }   // keep whatever else the game or other mods put there
            try
            {
                if (!ImGuiHelper.BeginMenu("Sterling Orbital"u8)) return;
                ((IGameViewportLifecycle)viewport).SetMenuBarInUse(inValue: true);   // clicks in the menu don't reach the world
                if (ImGui.MenuItem("Sparrow Mission Control"u8, "Ctrl+Shift+F"u8, AutopilotWindow.Open))
                    AutopilotWindow.Open = !AutopilotWindow.Open;
                if (ImGui.IsItemHovered())
                    ImGui.SetTooltip("Shows while you control a Sparrow (a vehicle with the octaweb)."u8);
                ImGui.EndMenu();
            }
            catch (Exception e) { Log.ErrorOnce("sterlingmenu", e.ToString()); }
        }
    }
}
