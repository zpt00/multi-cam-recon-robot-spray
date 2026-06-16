# -*- coding: utf-8 -*-

import io
import os
import glob
import time
import socket
import threading
import shutil
from dataclasses import dataclass
from typing import Tuple, Optional, List

import numpy as np
import open3d as o3d
from ftplib import FTP, all_errors

from scipy.spatial import KDTree
from scipy.interpolate import splprep, splev
from scipy.spatial.transform import Rotation as SciRot


# ======================
# UDP Config
# ======================
PC_BIND_IP = "YOUR_ROBOT_IP"
PC_BIND_PORT = 5005

PLC_ACK_PORT_DEFAULT = 4000
REPLY_TO_SOURCE_PORT = False   # True: 回源端口；False: 固定发 PLC_ACK_PORT_DEFAULT

PAYLOAD_LEN = 50
UDP_BUF_SIZE = 4096

# PLC -> PC: Start 位（byte0 bit1）
START_BYTE_INDEX = 0
START_BIT_INDEX = 1

# PC -> PLC: GetReady/Done/Error（byte0 bit1/2/3）
READY_BYTE_INDEX = 0
READY_BIT_INDEX = 1
DONE_BYTE_INDEX = 0
DONE_BIT_INDEX = 2
ERR_BYTE_INDEX = 0
ERR_BIT_INDEX = 3

# 若PLC希望回包除状态位以外也“保留/回显”原包内容，可开启
REPLY_ECHO_RX_BYTES = False


# ======================
# Cleanup Config
# ======================
DELETE_DIRS_AFTER_SUCCESS = True      # 上传成功后是否清空目录内容
DELETE_INPUT_DIR_FULIN = True         # 清空 FULIN
DELETE_WORK_DIR = True                # 清空 ls_work
DELETE_ONLY_OLDER_THAN_JOB_TS = True  


# ======================
# FANUC FTP Config
# ======================
FANUC_HOST = "YOUR_ROBOT_IP"
FANUC_USER = "admin"
FANUC_PASS = "123456"
FANUC_REMOTE_DIR = "md:/"
FANUC_PASSIVE = False
FANUC_DEBUGLEVEL = 2


# ======================
# PCD -> LS Pipeline Config
# ======================
PCD_INPUT_DIR = r"C:\Users\admin\Desktop\FULIN"   # PCD输入目录
PCD_PATTERN = "*.pcd"
WORK_DIR = r"C:\Users\admin\Desktop\ls_work"      # 中间文件输出目录

REMOTE_LS_FILENAME = "test20250910wk2.ls"         # 上传到FANUC控制器的文件名


# ======================
# PCD Filter Defaults (NO visualization)
# ======================
FILT_ZMIN = 1080.0
FILT_ZMAX = 1450.0
FILT_VOXEL = 1.0
FILT_NB_NEIGHBORS = 30
FILT_STD_RATIO = 2.0
FILT_RADIUS = 3.0
FILT_MIN_NB = 6
FILT_DBSCAN_EPS = 5.0
FILT_DBSCAN_MIN_POINTS = 30
FILT_MIN_CLUSTER_POINTS = 200
FILT_THICKNESS_TH = 6.0
FILT_ASPECT_TH = 10.0
FILT_LINEARITY_TH = 0.85
FILT_MAX_LEN_TH = None  # Optional[float]

PITCH_OFFSET_DEG = 180.0     # P整体+180°
WORLD_X_OFFSET_MM = 200.0    # 世界坐标+X推进200mm


# ======================
# Trajectory Preview Config
# ======================
SHOW_TRAJ_PREVIEW = True
PREVIEW_SECONDS = 10.0


# ======================
# Orientation / FANUC Euler Config
# ======================
# FANUC 常见约定是 fixed XYZ（W,P,R 对应绕固定 X/Y/Z 的欧拉角）
# -> SciPy 用小写 'xyz' 表示 extrinsic (global) XYZ
FANUC_EULER_SEQ_EXTRINSIC = "xyz"

# 避免蛇形路径每换行就 180° 翻面（绕到背面）
KEEP_X_CONTINUITY = True  # 若检测 X 与上一帧相反，则同时翻转 X&Y（Z不变）


# ======================
# Mesh params (TXT->Mesh)
# ======================
OUTLIER_NB_NEIGHBORS = 10
OUTLIER_STD_RATIO = 2.0

DBSCAN_EPS = 0.03
DBSCAN_MIN_POINTS = 20

UPSAMPLE_VOXEL = 0.005
UPSAMPLE_TARGET_NUM = 200000

NORMAL_RADIUS = 0.1
NORMAL_MAX_NN = 20
NORMAL_ORIENT_K = 30

POISSON_DEPTH = 9
POISSON_SCALE = 1.1

MESH_SIMPLIFY_VOXEL = 0.003
MESH_SMOOTH_ITERS = 2


# ======================
# Mesh->XYZWPR params
# ======================
PATH_SAMPLE_NUM = 5000
PATH_STEP = 150.0
PATH_STANDOFF = 80.0
PATH_MIN_DIST = 10.0
BSPLINE_SMOOTH = 5.0
MAX_ROT_STEP_DEG = 12.0
WORLD_DOWN = np.array([0.0, 0.0, -1.0])   # 如世界“下”是+Z，改成 [0,0,1]


# ======================
# XYZWPR->LS params
# ======================
PROG_NAME  = "TEST20250910WK2\t  Process"
UFRAME_NUM = 1
UTOOL_NUM  = 1
CNT_VALUE  = 100
CONFIG_STR = "F U T, 0, 0, 0"

BASE_LINEAR_SPEED = 200.0
ANG_SPEED_LIMIT   = 70.0
MIN_LINEAR_SPEED  = 40.0
SPEED_ROUND       = 5.0
SMOOTH_WINDOW     = 5
PRESET_COUNT      = 40


# ======================
# Bit helpers
# ======================
def get_bit(b: int, bit: int) -> bool:
    return ((b >> bit) & 1) == 1

def set_bit(buf: bytearray, byte_index: int, bit: int, value: bool):
    if value:
        buf[byte_index] |= (1 << bit)
    else:
        buf[byte_index] &= ~(1 << bit)

def parse_start(pkt: bytes) -> bool:
    return get_bit(pkt[START_BYTE_INDEX], START_BIT_INDEX)

def build_reply(template_pkt: Optional[bytes], get_ready: bool, done: bool, error: bool) -> bytes:
    if REPLY_ECHO_RX_BYTES and template_pkt is not None and len(template_pkt) >= PAYLOAD_LEN:
        buf = bytearray(template_pkt[:PAYLOAD_LEN])
    else:
        buf = bytearray(PAYLOAD_LEN)

    set_bit(buf, READY_BYTE_INDEX, READY_BIT_INDEX, get_ready)
    set_bit(buf, DONE_BYTE_INDEX, DONE_BIT_INDEX, done)
    set_bit(buf, ERR_BYTE_INDEX, ERR_BIT_INDEX, error)
    return bytes(buf)


# ======================
# Cleanup helpers
# ======================
def purge_dir_contents(dir_path: str, older_than_ts: Optional[float] = None) -> Tuple[int, int]:
    """
    Delete ALL contents under dir_path (files + subdirs), but keep the directory itself.

    If older_than_ts is not None:
        only delete items whose mtime <= older_than_ts (safer against deleting new incoming files).

    Returns: (deleted_files, deleted_dirs)
    """
    deleted_files = 0
    deleted_dirs = 0

    if not dir_path or not os.path.isdir(dir_path):
        return (0, 0)

    for entry in os.scandir(dir_path):
        try:
            path = entry.path

            if older_than_ts is not None:
                try:
                    mtime = os.path.getmtime(path)
                    if mtime > older_than_ts:
                        continue
                except Exception:
                    continue

            if entry.is_file() or entry.is_symlink():
                try:
                    os.remove(path)
                    deleted_files += 1
                except Exception as e:
                    print(f"[CLEAN][WARN] remove file failed: {path} err={repr(e)}", flush=True)

            elif entry.is_dir():
                try:
                    shutil.rmtree(path, ignore_errors=False)
                    deleted_dirs += 1
                except Exception as e:
                    print(f"[CLEAN][WARN] rmtree failed: {path} err={repr(e)}", flush=True)

        except Exception as e:
            print(f"[CLEAN][WARN] scandir entry failed: {repr(e)}", flush=True)

    return (deleted_files, deleted_dirs)


def cleanup_after_success(job_ts: Optional[float]):
    if not DELETE_DIRS_AFTER_SUCCESS:
        return

    older_ts = job_ts if (DELETE_ONLY_OLDER_THAN_JOB_TS and job_ts is not None) else None

    if DELETE_INPUT_DIR_FULIN:
        f, d = purge_dir_contents(PCD_INPUT_DIR, older_than_ts=older_ts)
        print(f"[CLEAN] FULIN cleared: files={f} dirs={d} (older_than={older_ts})", flush=True)

    if DELETE_WORK_DIR:
        f, d = purge_dir_contents(WORK_DIR, older_than_ts=older_ts)
        print(f"[CLEAN] ls_work cleared: files={f} dirs={d} (older_than={older_ts})", flush=True)


# ======================
# FANUC FTP upload
# ======================
def upload_ls_to_fanuc(host, user, password,
                       local_ls_path,
                       remote_dir="md:/",
                       remote_filename="test20250910wk2.ls",
                       port=21,
                       timeout=15.0,
                       passive=True,
                       debuglevel=2):

    if not os.path.isfile(local_ls_path):
        raise FileNotFoundError(local_ls_path)

    ftp = FTP()
    ftp.encoding = "utf-8"
    ftp.connect(host=host, port=port, timeout=timeout)
    ftp.set_debuglevel(debuglevel)
    ftp.login(user=user, passwd=password)
    ftp.set_pasv(passive)

    ftp.cwd(remote_dir)
    print("[FTP] Remote PWD =", ftp.pwd(), flush=True)

    ftp.voidcmd("TYPE A")  # ASCII

    with open(local_ls_path, "rb") as f:
        raw = f.read()

    raw = raw.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    if not raw.endswith(b"\n"):
        raw += b"\n"

    bio = io.BytesIO(raw)
    ftp.storlines(f"STOR {remote_filename}", bio)

    try:
        names = ftp.nlst()
        ok = any(n.upper() == remote_filename.upper() for n in names)
        print(f"[FTP] NLST contains {remote_filename}? {ok}", flush=True)
    except all_errors as e:
        print("[FTP][WARN] NLST failed:", repr(e), flush=True)

    ftp.quit()
    print("[FTP][SUCCESS] Upload finished.", flush=True)


# ======================
# Open3D preview helper (auto close after N seconds)
# ======================
def preview_geometries_for_seconds(geoms: List[o3d.geometry.Geometry],
                                   seconds: float = 5.0,
                                   window_name: str = "Preview",
                                   width: int = 1280,
                                   height: int = 720):
    if seconds is None or seconds <= 0:
        return
    vis = o3d.visualization.Visualizer()
    try:
        vis.create_window(window_name=window_name, width=width, height=height, visible=True)
        for g in geoms:
            vis.add_geometry(g)

        vis.poll_events()
        vis.update_renderer()

        t0 = time.time()
        while (time.time() - t0) < float(seconds):
            vis.poll_events()
            vis.update_renderer()
            time.sleep(0.01)

    except Exception as e:
        print("[VIS][WARN] preview failed:", repr(e), flush=True)
    finally:
        try:
            vis.destroy_window()
        except Exception:
            pass


# ======================
# PCD filter (embedded, defaults, NO visualization)
# ======================
def crop_by_z(pcd: o3d.geometry.PointCloud, zmin: float, zmax: float) -> o3d.geometry.PointCloud:
    pts = np.asarray(pcd.points)
    if pts.size == 0:
        return pcd
    mask = (pts[:, 2] >= float(zmin)) & (pts[:, 2] <= float(zmax))
    idx = np.where(mask)[0]
    return pcd.select_by_index(idx.tolist())

def pca_eigenvalues(points_xyz: np.ndarray) -> np.ndarray:
    if points_xyz.shape[0] < 10:
        return np.array([0.0, 0.0, 0.0], dtype=float)
    X = points_xyz - points_xyz.mean(axis=0, keepdims=True)
    C = (X.T @ X) / max(points_xyz.shape[0] - 1, 1)
    w, _ = np.linalg.eig(C)
    w = np.real(w)
    w.sort()
    return w[::-1]

def filter_clusters_by_shape(
    pcd: o3d.geometry.PointCloud,
    labels: np.ndarray,
    min_cluster_points: int,
    thickness_th: float,
    aspect_th: float,
    max_len_th: Optional[float],
    linearity_th: Optional[float],
):
    pts = np.asarray(pcd.points)
    keep_chunks = []

    max_label = int(labels.max())
    for lb in range(max_label + 1):
        idx = np.where(labels == lb)[0]
        if idx.size < int(min_cluster_points):
            continue

        cluster = pcd.select_by_index(idx.tolist())
        aabb = cluster.get_axis_aligned_bounding_box()
        ext = np.array(aabb.get_extent(), dtype=float)

        ext_sorted = np.sort(ext)
        thickness = float(ext_sorted[0])
        length = float(ext_sorted[2])
        aspect = float(length / max(thickness, 1e-9))

        eigen = pca_eigenvalues(pts[idx])
        if eigen[0] > 1e-12:
            linearity = float((eigen[0] - eigen[1]) / eigen[0])
        else:
            linearity = 0.0

        is_thin_and_long = (thickness < float(thickness_th)) and (aspect > float(aspect_th))

        if max_len_th is not None:
            is_thin_and_long = is_thin_and_long and (length < float(max_len_th))

        if linearity_th is not None:
            is_thin_and_long = is_thin_and_long and (linearity > float(linearity_th))

        if is_thin_and_long:
            continue

        keep_chunks.append(idx)

    if len(keep_chunks) == 0:
        return pcd

    keep_indices = np.concatenate(keep_chunks)
    return pcd.select_by_index(keep_indices.tolist())

def filter_pcd_default(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    if pcd.is_empty():
        return pcd

    n0 = len(pcd.points)

    # 1) Z裁剪
    pcd_roi = crop_by_z(pcd, FILT_ZMIN, FILT_ZMAX)

    # 2) 体素下采样
    if FILT_VOXEL and FILT_VOXEL > 0:
        pcd_roi = pcd_roi.voxel_down_sample(float(FILT_VOXEL))

    # 3) 统计离群
    pcd_roi, _ = pcd_roi.remove_statistical_outlier(
        nb_neighbors=int(FILT_NB_NEIGHBORS),
        std_ratio=float(FILT_STD_RATIO)
    )

    # 4) 半径离群
    pcd_roi, _ = pcd_roi.remove_radius_outlier(
        nb_points=int(FILT_MIN_NB),
        radius=float(FILT_RADIUS)
    )

    if pcd_roi.is_empty():
        raise RuntimeError("PCD filter result is empty. Check filter thresholds (z/voxel/radius/std_ratio...).")

    # 5) DBSCAN聚类 + 形状过滤
    labels = np.array(pcd_roi.cluster_dbscan(
        eps=float(FILT_DBSCAN_EPS),
        min_points=int(FILT_DBSCAN_MIN_POINTS),
        print_progress=False
    ))
    if labels.size > 0 and labels.max() >= 0:
        pcd_roi = filter_clusters_by_shape(
            pcd_roi, labels,
            min_cluster_points=int(FILT_MIN_CLUSTER_POINTS),
            thickness_th=float(FILT_THICKNESS_TH),
            aspect_th=float(FILT_ASPECT_TH),
            max_len_th=FILT_MAX_LEN_TH,
            linearity_th=float(FILT_LINEARITY_TH) if (FILT_LINEARITY_TH is not None and FILT_LINEARITY_TH > 0) else None
        )

    n1 = len(pcd_roi.points)
    print(f"[PCD_FILTER] points: {n0} -> {n1} (z[{FILT_ZMIN},{FILT_ZMAX}] voxel={FILT_VOXEL})", flush=True)
    return pcd_roi


# ======================
# Pipeline: PCD -> TXT (xyzrgb)
# ======================
def pcd_to_txt_xyzrgb(in_pcd: str, out_txt: str) -> None:
    pcd = o3d.io.read_point_cloud(in_pcd)
    if pcd.is_empty():
        raise RuntimeError("PCD is empty")

    # ---- NEW: filter stage (defaults, no vis) ----
    pcd = filter_pcd_default(pcd)

    pts = np.asarray(pcd.points)
    if pts.size == 0:
        raise RuntimeError("PCD is empty after filtering")

    if pcd.has_colors():
        cols = np.asarray(pcd.colors)
        if cols.size == 0:
            cols = np.ones_like(pts)
        elif cols.max() > 1.5:
            cols = cols / 255.0
    else:
        cols = np.ones_like(pts)

    arr = np.hstack([pts, cols]).astype(np.float64)
    os.makedirs(os.path.dirname(out_txt), exist_ok=True)
    np.savetxt(out_txt, arr, fmt="%.6f")
    print(f"[PIPE] PCD->TXT OK: {in_pcd} -> {out_txt} shape={arr.shape}", flush=True)


# ======================
# Pipeline: TXT -> Mesh (Poisson)
# ======================
def _load_and_preprocess(txt_path: str,
                         nb_neighbors=10, std_ratio=2.0) -> o3d.geometry.PointCloud:
    data = np.loadtxt(txt_path)
    if data.ndim == 1:
        data = data[None, :]
    xyz = data[:, :3]
    rgb = data[:, 3:6] if data.shape[1] >= 6 else np.ones_like(xyz)
    if rgb.max() > 1.5:
        rgb = rgb / 255.0

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.colors = o3d.utility.Vector3dVector(rgb)

    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=int(nb_neighbors), std_ratio=float(std_ratio))
    print(f"[PIPE] Outlier removed -> {len(pcd.points)} pts", flush=True)
    return pcd

def _cluster_and_keep_largest(pcd: o3d.geometry.PointCloud,
                             eps=0.03, min_points=20) -> o3d.geometry.PointCloud:
    labels = np.array(pcd.cluster_dbscan(eps=float(eps), min_points=int(min_points), print_progress=False))
    num_clusters = int(labels.max() + 1)
    print(f"[PIPE] DBSCAN clusters = {num_clusters}", flush=True)

    if num_clusters <= 0:
        print("[PIPE] No valid cluster -> use whole pcd", flush=True)
        return pcd

    counts = np.bincount(labels[labels >= 0])
    main_label = int(counts.argmax())
    idxs = np.where(labels == main_label)[0]
    pcd_main = pcd.select_by_index(idxs.tolist())
    print(f"[PIPE] Main cluster pts = {len(pcd_main.points)}", flush=True)
    return pcd_main

def _upsample_pcd(pcd: o3d.geometry.PointCloud,
                  voxel_size=0.005, target_num=200000) -> o3d.geometry.PointCloud:
    pcd = pcd.voxel_down_sample(float(voxel_size))
    try:
        pcd = pcd.fisher_uniform_resample(npoints=int(target_num))
        print(f"[PIPE] Resample -> {len(pcd.points)} pts", flush=True)
    except Exception:
        print("[PIPE] fisher_uniform_resample not supported, skip", flush=True)
    return pcd

def _estimate_normals(pcd: o3d.geometry.PointCloud,
                      search_radius=0.1, max_nn=20, orient_k=30) -> o3d.geometry.PointCloud:
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=float(search_radius), max_nn=int(max_nn)))
    pcd.orient_normals_consistent_tangent_plane(k=int(orient_k))
    return pcd

def _poisson_reconstruct(pcd: o3d.geometry.PointCloud, depth=9, scale=1.1, linear_fit=False) -> o3d.geometry.TriangleMesh:
    print(f"[PIPE] Poisson reconstruct depth={depth}", flush=True)
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=int(depth), scale=float(scale), linear_fit=bool(linear_fit)
    )
    densities = np.asarray(densities)
    thresh = np.quantile(densities, 0.01)
    mask = densities < thresh
    mesh.remove_vertices_by_mask(mask)
    mesh.compute_vertex_normals()
    print(f"[PIPE] Mesh triangles kept = {len(mesh.triangles)}", flush=True)
    return mesh

def _clean_and_smooth(mesh: o3d.geometry.TriangleMesh,
                      voxel_size=0.003, smooth_iterations=2) -> o3d.geometry.TriangleMesh:
    mesh.remove_unreferenced_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_non_manifold_edges()
    mesh = mesh.simplify_vertex_clustering(voxel_size=float(voxel_size))
    mesh = mesh.filter_smooth_simple(number_of_iterations=int(smooth_iterations))
    mesh.compute_vertex_normals()
    return mesh

def txt_to_mesh_poisson(txt_path: str, out_mesh_path: str) -> None:
    pcd_raw = _load_and_preprocess(txt_path, OUTLIER_NB_NEIGHBORS, OUTLIER_STD_RATIO)
    pcd_main = _cluster_and_keep_largest(pcd_raw, DBSCAN_EPS, DBSCAN_MIN_POINTS)
    pcd_main = _upsample_pcd(pcd_main, UPSAMPLE_VOXEL, UPSAMPLE_TARGET_NUM)
    pcd_main = _estimate_normals(pcd_main, NORMAL_RADIUS, NORMAL_MAX_NN, NORMAL_ORIENT_K)
    mesh = _poisson_reconstruct(pcd_main, POISSON_DEPTH, POISSON_SCALE, linear_fit=False)
    mesh = _clean_and_smooth(mesh, MESH_SIMPLIFY_VOXEL, MESH_SMOOTH_ITERS)

    os.makedirs(os.path.dirname(out_mesh_path), exist_ok=True)
    o3d.io.write_triangle_mesh(out_mesh_path, mesh)
    print(f"[PIPE] TXT->MESH OK: {out_mesh_path}", flush=True)


# ======================
# Pipeline: Mesh -> XYZWPR
# ======================
def _safe_normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v if n < 1e-12 else (v / n)

def _normalize_rows(M: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(M, axis=1, keepdims=True)
    n = np.clip(n, 1e-12, None)
    return M / n

def _load_mesh_with_normals(mesh_path: str) -> o3d.geometry.TriangleMesh:
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    return mesh

def _sample_points_and_normals_on_mesh(mesh: o3d.geometry.TriangleMesh, num_points=8000):
    pcd = mesh.sample_points_poisson_disk(int(num_points), init_factor=5)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    return points, normals, pcd

def _mesh_bounding_box_grid(points: np.ndarray, step: float) -> np.ndarray:
    min_xyz = points.min(axis=0)
    max_xyz = points.max(axis=0)
    extent = max_xyz - min_xyz

    step_use = float(step)
    if extent.max() > 1e-9 and step_use > extent.max():
        step_use = extent.max() / 50.0

    x_vals = np.arange(min_xyz[0], max_xyz[0], step_use)
    y_vals = np.arange(min_xyz[1], max_xyz[1], step_use)
    if len(x_vals) == 0 or len(y_vals) == 0:
        raise RuntimeError(f"Grid empty: extent={extent}, step={step} -> step_use={step_use}")

    grid_pts = []
    for i, x in enumerate(x_vals):
        y_range = y_vals if i % 2 == 0 else y_vals[::-1]
        for y in y_range:
            dist = np.sqrt((points[:, 0] - x) ** 2 + (points[:, 1] - y) ** 2)
            min_idx = int(np.argmin(dist))
            if dist[min_idx] < step_use * 1.2:
                grid_pts.append(min_idx)
    if len(grid_pts) < 2:
        raise RuntimeError("Grid path too short")
    return np.array(grid_pts, dtype=np.int64)

def _remove_short_jumps(path: np.ndarray, min_dist: float) -> np.ndarray:
    new_path = [path[0]]
    for pt in path[1:]:
        if np.linalg.norm(pt - new_path[-1]) > float(min_dist):
            new_path.append(pt)
    return np.array(new_path)

def _project_points_to_mesh_if_far(mesh: o3d.geometry.TriangleMesh,
                                   offset_path: np.ndarray,
                                   offset_dist: float,
                                   threshold_ratio=1.5) -> np.ndarray:
    mesh_points = np.asarray(mesh.vertices)
    kdtree = KDTree(mesh_points)
    dist, idx = kdtree.query(offset_path)
    mask = dist > (float(offset_dist) * float(threshold_ratio))
    offset_path[mask] = mesh_points[idx[mask]]
    return offset_path

def _smooth_bspline_curve(points: np.ndarray, smoothing: float, num: int) -> np.ndarray:
    if len(points) < 4:
        return points
    tck, _ = splprep(points.T, s=float(smoothing))
    u_fine = np.linspace(0, 1, int(num))
    curve = np.array(splev(u_fine, tck)).T
    return curve

def _make_path_lineset(path: np.ndarray) -> o3d.geometry.LineSet:
    pts = o3d.utility.Vector3dVector(path.astype(np.float64))
    lines = [[i, i + 1] for i in range(len(path) - 1)]
    ls = o3d.geometry.LineSet(points=pts, lines=o3d.utility.Vector2iVector(lines))
    ls.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in lines])
    return ls

def orientations_y_tangent_z_down(path: np.ndarray,
                                  world_down=WORLD_DOWN,
                                  keep_hemisphere=True,
                                  z_down_lock=True,
                                  max_rot_step_deg=12.0,
                                  keep_x_continuity=True):
    N = len(path)
    if N < 2:
        raise ValueError("path length must >= 2")

    Rs = []
    prev_R = None
    prev_Z = None
    prev_X = None

    for i in range(N):
        # Y = tangent
        if i < N - 1:
            t = path[i + 1] - path[i]
        else:
            t = path[i] - path[i - 1]
        Y = _safe_normalize(t)

        # Z = projection of down onto plane orthogonal to Y
        z_raw = world_down - np.dot(world_down, Y) * Y
        if np.linalg.norm(z_raw) < 1e-9:
            ref = np.array([1.0, 0.0, 0.0])
            if abs(np.dot(ref, Y)) > 0.9:
                ref = np.array([0.0, 1.0, 0.0])
            z_raw = ref - np.dot(ref, Y) * Y
        Z = _safe_normalize(z_raw)

        if keep_hemisphere and prev_Z is not None and np.dot(Z, prev_Z) < 0.0:
            Z = -Z
        if z_down_lock and np.dot(Z, world_down) < 0.0:
            Z = -Z

        X = _safe_normalize(np.cross(Y, Z))
        Z = _safe_normalize(np.cross(X, Y))

        # ---- NEW: avoid 180deg flips for serpentine path ----
        if keep_x_continuity and prev_X is not None and np.dot(X, prev_X) < 0.0:
            X = -X
            Y = -Y
            # Z unchanged; still right-handed because (-X)×(-Y)=X×Y=Z

        R_now = np.stack([X, Y, Z], axis=1)

        if prev_R is not None and max_rot_step_deg is not None and max_rot_step_deg > 0:
            dR = SciRot.from_matrix(prev_R).inv() * SciRot.from_matrix(R_now)
            ang = np.degrees(np.linalg.norm(dR.as_rotvec()))
            if ang > max_rot_step_deg:
                scale = float(max_rot_step_deg) / (ang + 1e-9)
                limited = SciRot.from_matrix(prev_R) * SciRot.from_rotvec(dR.as_rotvec() * scale)
                R_now = limited.as_matrix()

        Rs.append(R_now)
        prev_R = R_now
        prev_Z = R_now[:, 2]
        prev_X = R_now[:, 0]

    Rs = np.stack(Rs, axis=0)

    # ---- IMPORTANT: export as FANUC WPR with extrinsic XYZ ----
    WPR = SciRot.from_matrix(Rs).as_euler(FANUC_EULER_SEQ_EXTRINSIC, degrees=True)  # [W, P, R]
    return WPR

def mesh_to_xyzwpr(mesh_path: str, out_xyzwpr_txt: str) -> None:
    mesh = _load_mesh_with_normals(mesh_path)
    points, normals, pcd_surf = _sample_points_and_normals_on_mesh(mesh, num_points=PATH_SAMPLE_NUM)

    idxs = _mesh_bounding_box_grid(points, step=PATH_STEP)
    path_on_surface = points[idxs]
    path_normals = _normalize_rows(normals[idxs])

    offset_path = path_on_surface + path_normals * PATH_STANDOFF
    offset_path = _project_points_to_mesh_if_far(mesh, offset_path, PATH_STANDOFF, threshold_ratio=1.2)

    path_final = _remove_short_jumps(offset_path, PATH_MIN_DIST)
    target_num = max(60, min(180, len(path_final)))
    path_smooth = _smooth_bspline_curve(path_final, BSPLINE_SMOOTH, target_num)

    # ---- NEW: preview for N seconds, then continue ----
    if SHOW_TRAJ_PREVIEW:
        try:
            mesh_vis = mesh.paint_uniform_color([0.7, 0.7, 0.7])
            pcd_vis = pcd_surf.paint_uniform_color([0.2, 0.6, 1.0])
            path_ls = _make_path_lineset(path_smooth)
            preview_geometries_for_seconds([mesh_vis, pcd_vis, path_ls], seconds=PREVIEW_SECONDS,
                                           window_name=f"Trajectory Preview ({PREVIEW_SECONDS}s)")
        except Exception as e:
            print("[VIS][WARN] trajectory preview exception:", repr(e), flush=True)

    WPR = orientations_y_tangent_z_down(
        path_smooth,
        world_down=WORLD_DOWN,
        keep_hemisphere=True,
        z_down_lock=True,
        max_rot_step_deg=MAX_ROT_STEP_DEG,
        keep_x_continuity=KEEP_X_CONTINUITY,
    )

    out = np.hstack([path_smooth, WPR])
    os.makedirs(os.path.dirname(out_xyzwpr_txt), exist_ok=True)
    np.savetxt(out_xyzwpr_txt, out, fmt="%.6f", header="x y z W P R")
    print(f"[PIPE] MESH->XYZWPR OK: {out_xyzwpr_txt}  N={len(out)}", flush=True)


# ======================
# Pipeline: XYZWPR -> LS
# ======================
def _auto_scale_to_mm(pts: np.ndarray) -> np.ndarray:
    m = float(np.nanmax(np.linalg.norm(pts, axis=1)))
    if m < 20.0:
        return pts * 1000.0
    return pts

def xyzwpr_to_ls(xyzwpr_path: str, out_ls_path: str) -> None:
    data = np.loadtxt(xyzwpr_path, comments='#')
    if data.ndim == 1:
        data = data[None, :]
    if data.shape[1] < 6:
        raise RuntimeError("XYZWPR file must have 6 columns")

    pts = data[:, :3].astype(float)
    wpr = data[:, 3:6].astype(float)

    pts = _auto_scale_to_mm(pts)

    wpr[:, 1] = (wpr[:, 1] + PITCH_OFFSET_DEG + 180.0) % 360.0 - 180.0  # wrap到(-180,180]

    pts[:, 0] += WORLD_X_OFFSET_MM

    N = len(pts)
    if N < 2:
        raise RuntimeError("Not enough points for LS")

    # ---- IMPORTANT: match FANUC WPR convention ----
    R_list = SciRot.from_euler(FANUC_EULER_SEQ_EXTRINSIC, wpr, degrees=True)

    ang_deg = np.zeros(N)
    dist_mm = np.zeros(N)
    for i in range(1, N):
        dR = R_list[i-1].inv() * R_list[i]
        ang_deg[i] = np.degrees(np.linalg.norm(dR.as_rotvec()))
        dist_mm[i] = np.linalg.norm(pts[i] - pts[i-1])

    eps = 1e-6
    deg_per_mm = np.zeros(N)
    mask = dist_mm > eps
    deg_per_mm[mask] = ang_deg[mask] / dist_mm[mask]

    speeds = np.full(N, BASE_LINEAR_SPEED, dtype=float)
    for i in range(1, N):
        if deg_per_mm[i] <= eps:
            speeds[i] = BASE_LINEAR_SPEED
        else:
            cap = ANG_SPEED_LIMIT / deg_per_mm[i]
            speeds[i] = min(BASE_LINEAR_SPEED, max(MIN_LINEAR_SPEED, cap))

    if SMOOTH_WINDOW > 0 and N > 2:
        half = SMOOTH_WINDOW // 2
        smoothed = speeds.copy()
        for i in range(1, N):
            i0 = max(1, i - half)
            i1 = min(N, i + half + 1)
            avg = float(np.mean(speeds[i0:i1]))
            smoothed[i] = min(speeds[i], avg)
        speeds = smoothed

    if SPEED_ROUND > 0:
        speeds = np.round(speeds / SPEED_ROUND) * SPEED_ROUND

    speeds[0] = speeds[1] if N >= 2 else max(MIN_LINEAR_SPEED, min(BASE_LINEAR_SPEED, speeds[0]))

    appl_lines = []
    appl_lines.append("PAINT_PROCESS;")
    appl_lines.append("  LAST_CYCLE_TIME\t: 0.0 sec;")
    appl_lines.append("  LAST_GUN_ON_TIME\t: 0.0 sec;")
    appl_lines.append(f"  DEFAULT_USER_FRAME\t: {UFRAME_NUM};")
    appl_lines.append(f"  DEFAULT_TOOL_FRAME\t: {UTOOL_NUM};")
    appl_lines.append("  START_DELAY\t\t: 0;")
    appl_lines.append("  LAST_GUN_OFF_LINE\t: 0;")
    appl_lines.append("  LAST_PROCESSED_DATE\t: DATE 25-06-29 TIME 12:00:00;")
    appl_lines.append("  ")
    for k in range(1, PRESET_COUNT + 1):
        if k < 10:
            appl_lines.append(f"  PRESET_#{k}_GUN_ON_TIME   : 0.000 min;")
        else:
            appl_lines.append(f"  PRESET_#{k}_GUN_ON_TIME  : 0.000 min;")
    appl_block = "\n".join(appl_lines)

    header = f"""/PROG  {PROG_NAME}


/ATTR
OWNER\t\t= MNEDITOR;
COMMENT\t\t= "";
PROG_SIZE\t= 10000;
CREATE\t\t= DATE 25-06-29  TIME 12:00:00;
MODIFIED\t= DATE 25-06-29  TIME 12:00:00;
FILE_NAME\t= {PROG_NAME};
VERSION\t\t= 0;
LINE_COUNT\t= 0;
MEMORY_SIZE\t= 10000;
PROTECT\t\t= READ_WRITE;
STORAGE\t\t= SHADOW ONDEMAND;
TCD:  STACK_SIZE\t= 0,
      TASK_PRIORITY\t= 50,
      TIME_SLICE\t= 0,
      BUSY_LAMP_OFF\t= 0,
      ABORT_REQUEST\t= 0,
      PAUSE_REQUEST\t= 0;
DEFAULT_GROUP\t= 1,*,*,*,*;
CONTROL_CODE\t= 00000000 00000000;
/APPL

{appl_block}
/MN
"""

    mn_lines = []
    for i in range(N):
        line_no = i + 1
        v = int(round(float(speeds[i])))
        mn_lines.append(f"   {line_no}:L P[{line_no}] {v}mm/sec CNT{CNT_VALUE};")

    pos_lines = []
    for i in range(N):
        x, y, z = pts[i]
        W, P, Rr = wpr[i]
        pos_lines.append(f"""
P[{i+1}] {{
   GP1:
    UF : {UFRAME_NUM}, UT : {UTOOL_NUM},     CONFIG : '{CONFIG_STR}',
    X = {x:.3f} mm,    Y = {y:.3f} mm,    Z = {z:.3f} mm,
    W = {W:.3f} deg,    P = {P:.3f} deg,    R = {Rr:.3f} deg
}};
""")

    ls_content = header + "\n".join(mn_lines) + "\n/POS\n" + "\n".join(pos_lines) + "\n/END\n"
    os.makedirs(os.path.dirname(out_ls_path), exist_ok=True)
    with open(out_ls_path, "w", encoding="utf-8") as f:
        f.write(ls_content)

    print(f"[PIPE] XYZWPR->LS OK: {out_ls_path}", flush=True)


def find_latest_pcd(input_dir: str, pattern: str) -> str:
    files = glob.glob(os.path.join(input_dir, pattern))
    if not files:
        raise FileNotFoundError(f"No PCD files in {input_dir} pattern={pattern}")
    files.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return files[0]


def generate_ls_from_latest_pcd(run_dir: str) -> str:
    os.makedirs(run_dir, exist_ok=True)

    pcd_path = find_latest_pcd(PCD_INPUT_DIR, PCD_PATTERN)
    txt_path = os.path.join(run_dir, "filtered_output.txt")
    mesh_path = os.path.join(run_dir, "final_poisson2.ply")
    xyzwpr_path = os.path.join(run_dir, "surface_XYZWPR.txt")
    ls_path = os.path.join(run_dir, REMOTE_LS_FILENAME)

    print(f"[PIPE] Using latest PCD: {pcd_path}", flush=True)
    pcd_to_txt_xyzrgb(pcd_path, txt_path)
    txt_to_mesh_poisson(txt_path, mesh_path)
    mesh_to_xyzwpr(mesh_path, xyzwpr_path)
    xyzwpr_to_ls(xyzwpr_path, ls_path)

    return ls_path


# ======================
# State + Threads
# ======================
@dataclass
class SharedState:
    get_ready: bool = True
    done: bool = False
    error: bool = False
    busy: bool = False
    last_start: bool = False
    last_target: Optional[Tuple[str, int]] = None
    last_rx_template: Optional[bytes] = None

state = SharedState()
state_lock = threading.Lock()

start_queue = []
start_cv = threading.Condition()


def worker_thread(sock: socket.socket):
    while True:
        with start_cv:
            while not start_queue:
                start_cv.wait()
            job_ts = start_queue.pop(0)

        run_dir = os.path.join(WORK_DIR)
        ok = False
        err_msg = ""

        try:
            ls_local_path = generate_ls_from_latest_pcd(run_dir)
            upload_ls_to_fanuc(
                host=FANUC_HOST,
                user=FANUC_USER,
                password=FANUC_PASS,
                local_ls_path=ls_local_path,
                remote_dir=FANUC_REMOTE_DIR,
                remote_filename=REMOTE_LS_FILENAME,
                passive=FANUC_PASSIVE,
                debuglevel=FANUC_DEBUGLEVEL,
            )
            ok = True
        except Exception as e:
            ok = False
            err_msg = repr(e)

        if ok:
            try:
                cleanup_after_success(job_ts)
            except Exception as e:
                print("[CLEAN][WARN] cleanup_after_success failed:", repr(e), flush=True)

        with state_lock:
            state.done = bool(ok)
            state.error = (not ok)
            state.busy = False
            target = state.last_target
            tpl = state.last_rx_template

        if target is not None:
            payload = build_reply(tpl, get_ready=True, done=state.done, error=state.error)
            sock.sendto(payload, target)
            print(f"[TX] Final status -> {target} done={state.done} error={state.error}", flush=True)
        else:
            print("[TX][WARN] No PLC target known yet", flush=True)

        if not ok:
            print(f"[FLOW] FAILED: {err_msg}", flush=True)
        else:
            print("[FLOW] SUCCESS", flush=True)


def main():
    os.makedirs(WORK_DIR, exist_ok=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((PC_BIND_IP, PC_BIND_PORT))
    print(f"[UDP] Listening on {PC_BIND_IP}:{PC_BIND_PORT}", flush=True)

    t = threading.Thread(target=worker_thread, args=(sock,), daemon=True)
    t.start()

    while True:
        data, addr = sock.recvfrom(UDP_BUF_SIZE)
        if len(data) < PAYLOAD_LEN:
            print(f"[RX][WARN] short packet from {addr}: {len(data)}", flush=True)
            continue

        pkt = data[:PAYLOAD_LEN]
        start = parse_start(pkt)
        target = addr if REPLY_TO_SOURCE_PORT else (addr[0], PLC_ACK_PORT_DEFAULT)

        with state_lock:
            start_edge = bool(start and (not state.last_start))
            state.last_start = bool(start)

            state.last_target = target
            state.last_rx_template = pkt

            if start_edge:
                state.done = False
                state.error = False
                state.busy = True

        if start_edge:
            payload = build_reply(pkt, get_ready=True, done=False, error=False)
            sock.sendto(payload, target)
            print(f"[FLOW] Start EDGE from {addr} -> clear done/error, busy=1 -> {target}", flush=True)

            with start_cv:
                start_queue.append(time.time())
                start_cv.notify()
        else:
            with state_lock:
                payload = build_reply(pkt, get_ready=True, done=state.done, error=state.error)
                busy = state.busy
                done = state.done
                err = state.error
            sock.sendto(payload, target)
            print(f"[RX] {addr} start={start} edge={start_edge} busy={busy} -> reply(done={done}, error={err}) to {target}", flush=True)


if __name__ == "__main__":
    main()
