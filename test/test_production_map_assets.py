import os
import json
import hashlib
import yaml
import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROD_MAP_DIR = os.path.join(REPO_ROOT, "ros2", "maps", "production")
MANIFEST_PATH = os.path.join(PROD_MAP_DIR, "MANIFEST.json")

EXPECTED_FILES = [
    "house_slam_2026-09-23_candidate_hallway.yaml",
    "house_slam_2026-09-23_candidate_hallway.pgm",
    "house_slam_2026-09-23_candidate_hallway.data",
    "house_slam_2026-09-23_candidate_hallway.posegraph",
    "active_map_path.txt",
    "home_pose_slam_2026-09-23_candidate_hallway.json",
    "home_pose_slam.json",
    "MANIFEST.json"
]

def test_production_directory_and_files_exist():
    assert os.path.isdir(PROD_MAP_DIR), f"Production map directory missing: {PROD_MAP_DIR}"
    for f in EXPECTED_FILES:
        fp = os.path.join(PROD_MAP_DIR, f)
        assert os.path.isfile(fp), f"Required production map asset missing: {f}"
        assert os.path.getsize(fp) > 0, f"Production map asset {f} is empty!"

def test_manifest_checksums_and_integrity():
    with open(MANIFEST_PATH, "r", encoding="utf-8") as fp:
        manifest = json.load(fp)

    assert manifest.get("status") == "PRODUCTION_AUTHORITATIVE"
    assert manifest.get("map_name") == "house_slam_2026-09-23_candidate_hallway"

    expected_hashes = manifest.get("files", {})
    for filename, exp_hash in expected_hashes.items():
        fp = os.path.join(PROD_MAP_DIR, filename)
        assert os.path.isfile(fp), f"File in manifest missing from disk: {filename}"
        with open(fp, "rb") as f_data:
            data_bytes = f_data.read()
            if filename.endswith((".yaml", ".json", ".txt")):
                data_bytes = data_bytes.replace(b"\r\n", b"\n")
            actual_hash = hashlib.sha256(data_bytes).hexdigest()
        assert actual_hash == exp_hash, f"Checksum mismatch for {filename}: expected {exp_hash}, got {actual_hash}"

def test_production_yaml_format():
    yaml_path = os.path.join(PROD_MAP_DIR, "house_slam_2026-09-23_candidate_hallway.yaml")
    with open(yaml_path, "r", encoding="utf-8") as fp:
        cfg = yaml.safe_load(fp)

    assert cfg.get("resolution") == 0.05, f"Unexpected resolution: {cfg.get('resolution')}"
    origin = cfg.get("origin")
    assert origin is not None and len(origin) == 3, f"Invalid origin: {origin}"
    assert abs(origin[0] - (-4.202)) < 1e-4, f"Unexpected origin X: {origin[0]}"
    assert abs(origin[1] - (-5.100)) < 1e-4, f"Unexpected origin Y: {origin[1]}"
    assert cfg.get("image") == "house_slam_2026-09-23_candidate_hallway.pgm"

def test_production_pgm_dimensions_and_payload():
    pgm_path = os.path.join(PROD_MAP_DIR, "house_slam_2026-09-23_candidate_hallway.pgm")
    with open(pgm_path, "rb") as fp:
        buf = fp.read()

    assert buf[:2] == b"P5", "Invalid PGM magic number (must be P5 binary)"
    
    header_end = 0
    newlines_seen = 0
    for i in range(len(buf)):
        if buf[i] == 10:  # \n
            if not buf[:i].strip().startswith(b"#"):
                newlines_seen += 1
            if newlines_seen == 3:
                header_end = i + 1
                break

    lines = [l.strip() for l in buf[:header_end].decode("ascii", errors="ignore").splitlines() if l.strip() and not l.startswith("#")]
    w, h = map(int, lines[1].split())
    assert w == 340, f"Expected map width 340, got {w}"
    assert h == 132, f"Expected map height 132, got {h}"

    pixel_data = buf[header_end:]
    assert len(pixel_data) == 340 * 132, f"Payload size mismatch: expected {340*132} bytes, got {len(pixel_data)}"

def test_home_pose_data_integrity():
    for f in ["home_pose_slam_2026-09-23_candidate_hallway.json", "home_pose_slam.json"]:
        p = os.path.join(PROD_MAP_DIR, f)
        with open(p, "r", encoding="utf-8") as fp:
            data = json.load(fp)

        pos = data["pose"]["position"]
        ori = data["pose"]["orientation"]
        assert abs(pos["x"] - 1.193853) < 1e-4, f"HOME X deviation: {pos['x']}"
        assert abs(pos["y"] - (-0.045221)) < 1e-4, f"HOME Y deviation: {pos['y']}"
        assert abs(data["pose"]["yaw_deg"] - (-5.047)) < 1e-2, f"HOME yaw deviation: {data['pose']['yaw_deg']}"
        assert abs(ori["z"] - (-0.044029)) < 1e-4, f"HOME qz deviation: {ori['z']}"
        assert abs(ori["w"] - 0.999030) < 1e-4, f"HOME qw deviation: {ori['w']}"
        assert data.get("frame_id") == "map"

def test_active_map_pointer():
    p = os.path.join(PROD_MAP_DIR, "active_map_path.txt")
    with open(p, "r", encoding="utf-8") as fp:
        val = fp.read().strip()
    assert val == "/ros2_ws/maps/house_slam_2026-09-23_candidate_hallway.yaml"

def test_startup_executes_bringup_launch():
    compose_path = os.path.join(REPO_ROOT, "ros2", "compose.yaml")
    with open(compose_path, "r", encoding="utf-8") as fp:
        c = fp.read()
    assert "bringup.launch.py" in c, "compose.yaml must launch bringup.launch.py to include navigation automatically"
