# SPDX-License-Identifier: Apache-2.0
"""Headless deterministic scenario runner + renderer.

  python tools/sim.py stand  --duration 4 --seed 0
  python tools/sim.py push   --force 150 --dt 0.1 --dir 1 0 --controller pd --out media/push.mp4
  python tools/sim.py sweep  --controller ankle --fmax 400 --step 25
  python tools/sim.py media  # regenerate all README media + media/metrics.json
Rendering uses MuJoCo's offscreen renderer (MUJOCO_GL=egl, falls back to osmesa).
"""
import argparse, json, os, sys
os.environ.setdefault("MUJOCO_GL", "egl")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); os.chdir(ROOT); sys.path.insert(0, ROOT)
import numpy as np, mujoco
from PIL import Image, ImageDraw
from ziggy.scenarios import run_stance, push_sweep
from ziggy.analysis import load, solve_ik, HOUSEHOLD_TARGETS

def label(frame, lines, color=(255, 255, 255)):
    im = Image.fromarray(frame); dr = ImageDraw.Draw(im)
    dr.rectangle((0, 0, im.width, 14 * len(lines) + 6), fill=(0, 0, 0))
    for i, t in enumerate(lines): dr.text((6, 3 + 14 * i), t, fill=color)
    return np.asarray(im)

def save_video(frames, path, fps=30):
    import subprocess, tempfile
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    if path.endswith(".gif"):
        ims = [Image.fromarray(f).convert("P", palette=Image.ADAPTIVE, colors=128) for f in frames[::2]]
        ims[0].save(path, save_all=True, append_images=ims[1:], duration=int(2000 / fps), loop=0, optimize=True)
    else:
        h, w = frames[0].shape[:2]
        p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}",
                              "-r", str(fps), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "28", path], stdin=subprocess.PIPE)
        for f in frames: p.stdin.write(np.ascontiguousarray(f).tobytes())
        p.stdin.close(); p.wait()
    print("wrote", path)

def push_clip(force, controller, dt=0.1, duration=4.0, direction=(1, 0), seed=0):
    fr = []; r = run_stance(duration, force, direction, push_dt=dt, controller=controller, seed=seed, record=fr)
    st = ("FELL at %.2fs" % r["t_fall_s"] if r["t_fall_s"] else "FALLING (CoM outside support)") if r["fell"] else ("RECOVERED" if r["recovered"] else "SURVIVED, not recovered")
    T = np.arange(len(fr)) / 30.0
    out = [label(f, [f"PHYSICS sim | controller={controller} | push {force:.0f} N x {dt}s ({force*dt:.0f} N*s) at t=1.0s",
                     f"t={t:4.2f}s  result: {st}  maxCoM {r['max_com_disp_mm']:.0f} mm"],
                 (255, 120, 120) if r["fell"] else ((120, 255, 120) if r["recovered"] else (255, 230, 120)))
           for f, t in zip(fr, T)]
    return out, r

def overview(path):
    m, d = load(); mujoco.mj_forward(m, d); ren = mujoco.Renderer(m, 480, 640); imgs = []
    for az, name in ((180, "front"), (90, "side"), (135, "iso")):
        cam = mujoco.MjvCamera(); cam.lookat[:] = (0, 0, 0.85); cam.distance = 2.6; cam.azimuth = az; cam.elevation = -8
        opt = mujoco.MjvOption(); opt.frame = mujoco.mjtFrame.mjFRAME_NONE
        ren.update_scene(d, cam, opt); imgs.append(label(ren.render(), [f"ZIGGY v0.1 MuJoCo model - {name} (capsule/box proxies, not CAD)"]))
    Image.fromarray(np.hstack(imgs)).save(path); print("wrote", path)

def ik_demo(path_mp4, path_gif, path_png):
    m, d = load(); ren = mujoco.Renderer(m, 480, 640); frames = []; rows = {}
    cam = mujoco.MjvCamera(); cam.lookat[:] = (0.2, 0.1, 0.9); cam.distance = 2.2; cam.azimuth = 150; cam.elevation = -15
    sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "l_grasp"); stills = []
    for name, tgt in HOUSEHOLD_TARGETS.items():
        d.qpos[:] = m.qpos0
        best = None
        for ww in (False, True):
            dd = mujoco.MjData(m); r = solve_ik(m, dd, "l_grasp", tgt, "l", with_waist=ww, q0=m.qpos0.copy(), iters=400)
            if best is None or r["error_m"] < best[0]["error_m"]: best = (r, ww)
        r, ww = best; rows[name] = {"error_mm": round(r["error_m"] * 1e3, 2), "reached": r["error_m"] < 5e-3, "used_waist": ww,
                                    "within_limits": r["within_limits"]}
        # replay: interpolate home -> solution (kinematic, for visualization only)
        from ziggy.analysis import arm_dofs
        _, qa, _, _ = arm_dofs(m, "l", ww); q1 = r["q"]
        for a in np.linspace(0, 1, 30):
            d.qpos[:] = m.qpos0; d.qpos[qa] = a * q1; mujoco.mj_kinematics(m, d)
            ren.update_scene(d, cam); s = ren.scene
            for pos, col in ((tgt, (0.1, 0.9, 0.1, 0.6)), (d.site_xpos[sid], (0.9, 0.1, 0.1, 0.9))):
                mujoco.mjv_initGeom(s.geoms[s.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.025, 0, 0]), np.array(pos, float),
                                    np.eye(3).ravel(), np.array(col, np.float32)); s.ngeom += 1
            err = np.linalg.norm(np.array(tgt) - d.site_xpos[sid]) * 1e3
            ok = rows[name]["reached"]
            f = label(ren.render(), [f"KINEMATIC IK (DLS, no dynamics) target {name} green=target red=achieved",
                                     f"target {tuple(round(v,2) for v in tgt)} m  error now {err:6.1f} mm  final {rows[name]['error_mm']} mm "
                                     + ("REACHED" if ok else "UNREACHABLE standing")], (120, 255, 120) if ok else (255, 120, 120))
            frames.append(f)
        frames += [frames[-1]] * 15; stills.append(frames[-1])
    save_video(frames, path_mp4); save_video(frames, path_gif)
    grid = np.vstack([np.hstack(stills[0:3]), np.hstack(stills[3:6])]); Image.fromarray(grid).save(path_png); print("wrote", path_png)
    return rows

def plots(sweeps, clips, path_sweep, path_trace):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    for s in sweeps:
        F = [r["push_n"] for r in s["rows"]]; D = [min(r["max_com_disp_mm"], 400) for r in s["rows"]]
        ax[0].plot(F, D, "o-", label=s["controller"])
        for r in s["rows"]:
            mk = "x" if r["fell"] else ("o" if r["recovered"] else "s")
            ax[1].scatter(r["push_n"], {"pd": 0, "ankle": 1}[s["controller"]], marker=mk,
                          c="r" if r["fell"] else ("g" if r["recovered"] else "orange"))
    ax[0].set_xlabel("push force N (0.1 s at pelvis, +x)"); ax[0].set_ylabel("max CoM displacement mm (clipped 400)"); ax[0].legend(); ax[0].grid()
    ax[1].set_yticks([0, 1], ["pd", "ankle"]); ax[1].set_xlabel("push force N"); ax[1].set_title("green o recovered / orange s survived / red x fell"); ax[1].grid()
    fig.tight_layout(); fig.savefig(path_sweep, dpi=110); plt.close(fig); print("wrote", path_sweep)
    fig, ax = plt.subplots(figsize=(8, 4))
    for name, r in clips.items():
        ax.plot(r["log"]["t"], np.array(r["log"]["com_x"]) * 1e3, label=name)
    ax.axvspan(1.0, 1.1, color="r", alpha=0.2, label="push window"); ax.set_ylim(-150, 400)
    ax.set_xlabel("t s"); ax.set_ylabel("CoM x displacement mm"); ax.legend(); ax.grid(); fig.tight_layout(); fig.savefig(path_trace, dpi=110); plt.close(fig)
    print("wrote", path_trace)

def media():
    os.makedirs("media", exist_ok=True)
    overview("media/sim_overview.png")
    ik = ik_demo("media/ik_tracking.mp4", "media/ik_tracking.gif", "media/ik_tracking.png")
    cases = [("pd_150N", 150, "pd"), ("ankle_250N", 250, "ankle"), ("ankle_300N_fall", 300, "ankle")]
    vids, clips = [], {}
    for name, f, c in cases:
        fr, r = push_clip(f, c); vids.append(fr); clips[name] = r
    n = min(len(v) for v in vids); comp = [np.hstack([v[i] for v in vids]) for i in range(n)]
    save_video(comp, "media/push_compare.mp4")
    small = [np.asarray(Image.fromarray(f).resize((f.shape[1] // 2, f.shape[0] // 2))) for f in comp]
    save_video(small, "media/push_compare.gif")
    Image.fromarray(np.vstack([comp[int(1.05 * 30)], comp[int(2.0 * 30)], comp[-1]])).save("media/push_compare.png")
    sweeps = [push_sweep(c, range(0, 401, 25)) for c in ("pd", "ankle")]
    lat = [push_sweep(c, range(0, 401, 50), direction=dv) for c in ("pd", "ankle") for dv in ((0, 1), (-1, 0))]
    long = [push_sweep(c, range(0, 301, 25), push_dt=0.2, direction=(1, 0)) for c in ("pd", "ankle")]
    plots(sweeps, clips, "media/push_sweep.png", "media/push_com_trace.png")
    strip = lambda r: {k: v for k, v in r.items() if k != "log"}
    met = {"ik_household": ik, "push_clips": {k: strip(v) for k, v in clips.items()},
           "sweep_sagittal": [{k: v for k, v in s.items() if k != "rows"} | {"rows": s["rows"]} for s in sweeps],
           "sweep_lateral_and_backward": [{k: v for k, v in s.items() if k != "rows"} for s in lat],
           "sweep_sagittal_0.2s_holdout": [{k: v for k, v in s.items() if k != "rows"} for s in long],
           "stand_noise_seeds": [strip(run_stance(4.0, 0.0, seed=s, noise_deg=1.0)) for s in range(5)]}
    json.dump(met, open("media/metrics.json", "w"), indent=1); print("wrote media/metrics.json")

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("scenario", choices=["stand", "push", "sweep", "media"])
    ap.add_argument("--force", type=float, default=150); ap.add_argument("--dt", type=float, default=0.1)
    ap.add_argument("--dir", type=float, nargs=2, default=(1, 0)); ap.add_argument("--controller", default="pd", choices=["pd", "ankle"])
    ap.add_argument("--duration", type=float, default=4.0); ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--noise-deg", type=float, default=0.0); ap.add_argument("--fmax", type=float, default=400); ap.add_argument("--step", type=float, default=25)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    if a.scenario == "media": return media()
    if a.scenario == "sweep":
        s = push_sweep(a.controller, np.arange(0, a.fmax + 1e-9, a.step), a.dt, a.duration, tuple(a.dir))
        print(json.dumps({k: v for k, v in s.items() if k != "rows"}, indent=1)); return
    f = 0.0 if a.scenario == "stand" else a.force
    if a.out:
        fr, r = push_clip(f, a.controller, a.dt, a.duration, tuple(a.dir), a.seed); save_video(fr, a.out)
    else:
        r = run_stance(a.duration, f, tuple(a.dir), push_dt=a.dt, controller=a.controller, seed=a.seed, noise_deg=a.noise_deg)
    r.pop("log", None); print(json.dumps(r, indent=1))

if __name__ == "__main__":
    main()
