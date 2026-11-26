"""CLI regression with independent pinhole and equidistant projections (stdlib only)."""
import json
import math
import re
import subprocess
import sys
import tempfile
from pathlib import Path

CAMERAS = [(0, 0, 0), (1.5, 0, 0), (-0.8, -0.1, 0.14)]
BASELINE = 1.5
HEIGHT = 1.63
FLOOR_Y = 0.95
LENS = ["pinhole", "fisheye", "pinhole"]
K = [765, 0, 640, 0, 750, 480, 0, 0, 1]


def project(cam, xyz):
    x, y, z = [a - b for a, b in zip(xyz, CAMERAS[cam])]
    x, y = x / z, y / z
    r = math.hypot(x, y)
    if LENS[cam] == "fisheye" and r:
        factor = math.atan(r) / r
        x, y = x * factor, y * factor
    return [765 * x + 640, 750 * y + 480]


def matrix(rows, cols, values, indent=""):
    return f"!!opencv-matrix\n{indent}   rows: {rows}\n{indent}   cols: {cols}\n{indent}   dt: d\n{indent}   data: {json.dumps(values)}\n"


def scene_inputs(path, cam2_seed_dy=0):
    header = "%YAML:1.0\n---\n"
    intr = header + 'schema: fitra_calibration_v1\nunit: m\ncoordinate_system: "test"\nintrinsics:\n'
    scene = header + "schema: fitra_markerless_background_v1\n"
    for c, (x, y, z) in enumerate(CAMERAS):
        intr += f"   cam{c}:\n      width: 1280\n      height: 960\n      distortion_model: {LENS[c]}\n"
        intr += "      K: " + matrix(3, 3, K, "      ")
        intr += "      dist: " + matrix(1, 4, [0] * 4, "      ")
        scene += f"id{c}: cam{c}\nlens{c}: {LENS[c]}\nwidth{c}: 1280\nheight{c}: 960\n"
        scene += f"K{c}: " + matrix(3, 3, K)
        scene += f"D{c}: " + matrix(1, 4, [0] * 4)
        seed_y = -y / BASELINE + (cam2_seed_dy if c == 2 else 0)
        scene += f"T{c}: " + matrix(4, 4, [
            1, 0, 0, -x/BASELINE, 0, 1, 0, seed_y, 0, 0, 1, -z/BASELINE, 0, 0, 0, 1
        ])
    tracks = []
    for row in range(8):
        for col in range(10):
            point = ((col - 4.5) * .22, (row - 3.5) * .25, 3.9 + (col + row) % 4 * .21)
            tracks.extend(v for c in range(3) for v in project(c, point))
    scene += "background_tracks: " + matrix(80, 6, tracks)
    (path/"intr.yml").write_text(intr)
    (path/"scene.yml").write_text(scene)


def body_file(path, indices, *, fixed_feet=False, foot_shift=0):
    rows = [{"schema": "fitra_markerless_observations_v1", "keypoint_format": "halpe26"}]
    for n in indices:
        x = 0 if fixed_feet else .46 * math.sin(n * .52)
        z = 4 if fixed_feet else 4 + .35 * math.cos(n * .71)
        joints = {17: (x, FLOOR_Y - HEIGHT, z)}
        for side, sign in enumerate([-1, 1]):
            for k,dx,y,dz in [(5,.20,-1.34,0),(7,.33,-1.03,.04),(11,.15,-.86,0),
                               (13,.15,-.44,.02),(15,.15,-.07,0),(20,.14,0,.07),
                               (22,.20,0,.05),(24,.15,0,-.09)]:
                joints[k+side] = (x+dx*sign, FLOOR_Y+y, z+dz)
        frames=[]
        for c in range(3):
            keypoints=[]
            for k,(jx,jy,jz) in joints.items():
                pos=project(c,(jx,jy+(foot_shift if k>=20 else 0),jz))
                keypoints.append([k,*pos,.9])
            frames.append(dict(camera_id=f"cam{c}",source_seq=n+1,stream_epoch=1,
                               capture_mono_ns=1_000_000_000+n*200_000_000+c*1_000_000,
                               timestamp_kind="synthetic_monotonic", width=1280,height=960,
                               person_count=1,keypoints=keypoints))
        rows.append(dict(sample_id=n+1,continuity_id="fixture",frames=frames))
    path.write_text("".join(json.dumps(row)+"\n" for row in rows))


def main(binary):
    with tempfile.TemporaryDirectory(prefix="fitra-hybrid-test-") as work:
        p=Path(work)
        scene_inputs(p)
        body_file(p/"fit",range(32))
        body_file(p/"check",range(39,58))
        def run(output, *, fit="fit", check="check", scene="scene.yml", height="1.63", accept=False):
            result=subprocess.run([binary,p/"intr.yml",p/scene,p/fit,p/check,height,p/output],
                                  text=True,capture_output=True)
            assert (result.returncode == 0) == accept, (result.stdout,result.stderr)
        # A YAML map may use a different order from the background triplets.
        intr_file = p / "intr.yml"
        top, camera_nodes = intr_file.read_text().split("intrinsics:\n")
        entries = re.findall(r"   cam[0-9]:\n.*?(?=   cam[0-9]:\n|\Z)", camera_nodes, re.S)
        assert len(entries) == 3
        intr_file.write_text(top + "intrinsics:\n" + "".join(reversed(entries)))
        run("valid",accept=True)
        camera_yaml=(p/"valid/calibration.yaml").read_bytes()
        report=json.loads((p/"valid/report.json").read_text())
        assert abs(report["baseline01_m"]-BASELINE)<1e-4
        assert report["validation_foot_p90_m"]<1e-4 and report["validation_stature_p90_m"]<1e-4
        assert report["body_extrinsic_correction"] == 0
        centers=re.findall(r"camera_center_w:.*?data:\s*\[([^\]]+)\]",camera_yaml.decode(),re.S)
        assert len(centers)==3
        for coords,(_,cy,_) in zip(centers,CAMERAS):
            assert abs(float(coords.split(",")[2])-(FLOOR_Y-cy))<1e-4
        scene_inputs(p, cam2_seed_dy=.012)
        run("corrected", accept=True)
        corrected = json.loads((p/"corrected/report.json").read_text())
        assert corrected["body_extrinsic_correction"] == 1
        assert corrected["body_candidate_p90_px"] < corrected["body_fixed_p90_px"]
        assert corrected["background_candidate_p90_px"] < corrected["background_fixed_p90_px"]
        scene_inputs(p)
        run("valid")
        assert (p/"valid/calibration.yaml").read_bytes()==camera_yaml
        body_file(p/"fixed",range(32),fixed_feet=True)
        body_file(p/"floating",range(39,58),foot_shift=-.2)
        run("nofloor",fit="fixed")
        run("feet",check="floating")
        run("overlap",check="fit")
        run("badheight",height="nan")
        (p/"swapped").write_text((p/"scene.yml").read_text().replace("id1: cam1","id1: cam2"))
        run("badcam",scene="swapped")
        assert all(not (p/x).exists() for x in ("nofloor","feet","overlap","badheight","badcam"))


if __name__ == "__main__":
    main(sys.argv[1])
