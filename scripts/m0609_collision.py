#!/usr/bin/env python3
"""M0609 충돌 메시(dsr_description2 m0609_collision/*.dae) ↔ 상자(설치물) 거리 — 대기 자세 최적화의 거치대 충돌 검사 (#208, 10/14).

URDF 의 각 링크 collision 메시(.dae, mm 단위·노드 행렬 포함)를 읽어 꼭짓점을 링크 프레임에 두고, 관절값 q 에서
순기구학(ejt_wrench.Chain 과 같은 프레임)으로 base_link 로 옮겨 상자까지 거리를 잰다. 꼭짓점 기준이라 상자가
메시 면 사이로 들어가는 경우는 놓칠 수 있으나 메시가 촘촘(링크당 수백~수천 점)해 cm 단위 여유 판단에는 충분.
그리퍼(RG2)·손목 카메라·패드는 URDF 에 없으므로 플랜지 → TCP 선분 + 반지름(캡슐)과 패드 원판으로 근사.

    from m0609_collision import RobotBody, Box
    body = RobotBody()
    rack = Box((0.25, -0.25, 0.0), (0.30, -0.05, 0.17))     # 패드 거치대 (패드 없음), base [m]
    d, who = body.clearance(q_rad, [rack])                    # 최소 여유 [m] (음수 = 겹침), 가장 가까운 부위
"""
from __future__ import annotations

import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ejt_wrench import Chain  # noqa: E402

NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


@dataclass
class Box:
    lo: tuple
    hi: tuple
    name: str = "rack"

    def dist(self, pts: np.ndarray) -> np.ndarray:
        """점들 → 상자 거리 [m]. 안쪽은 음수(가장 가까운 면까지 깊이)."""
        lo, hi = np.asarray(self.lo), np.asarray(self.hi)
        d_out = np.maximum(np.maximum(lo - pts, pts - hi), 0.0)
        outside = np.linalg.norm(d_out, axis=1)
        inside = np.min(np.minimum(pts - lo, hi - pts), axis=1)
        return np.where(np.all((pts >= lo) & (pts <= hi), axis=1), -inside, outside)


def _load_dae(path: Path, scale: float) -> np.ndarray:
    """.dae 의 모든 메시 꼭짓점을 (노드 행렬 적용 후) 링크 프레임 [m] 으로."""
    root = ET.parse(path).getroot()
    geo = {}
    for g in root.iterfind(".//c:library_geometries/c:geometry", NS):
        arr = g.find(".//c:source/c:float_array", NS)              # 첫 source = positions
        for src in g.iterfind(".//c:source", NS):
            if "position" in src.get("id", "").lower():
                arr = src.find("c:float_array", NS)
                break
        geo[g.get("id")] = np.array(arr.text.split(), float).reshape(-1, 3)
    pts = []

    def walk(node, T):
        m = node.find("c:matrix", NS)
        if m is not None:
            T = T @ np.array(m.text.split(), float).reshape(4, 4)
        for ig in node.findall("c:instance_geometry", NS):
            v = geo.get(ig.get("url", "").lstrip("#"))
            if v is not None:
                pts.append((np.c_[v, np.ones(len(v))] @ T.T)[:, :3])
        for ch in node.findall("c:node", NS):
            walk(ch, T)

    for scene in root.iterfind(".//c:library_visual_scenes/c:visual_scene", NS):
        for n in scene.findall("c:node", NS):
            walk(n, np.eye(4))
    return np.vstack(pts) * scale if pts else np.zeros((0, 3))


class RobotBody:
    """링크별 충돌 꼭짓점 + 그리퍼 캡슐 + (선택) 패드 원판."""

    def __init__(self, tcp_offset_m=(0.0008, -0.0014, 0.2259), gripper_r=0.04, pad_offset_tool_m=None, pad_r=0.075,
                 pad_normal_tool=(-0.7071, 0.0, 0.7071), max_pts_per_link=1500):
        from ament_index_python.packages import get_package_share_directory
        from urdf_parser_py.urdf import URDF
        share = Path(get_package_share_directory("dsr_description2"))
        urdf = URDF.from_xml_file(str(share / "urdf" / "m0609.urdf"))
        self.chain = Chain()
        self.links = {}                                     # 이름 → (관절 수: 0=base_link, i=link_i, 꼭짓점)
        rng = np.random.default_rng(0)
        for link in urdf.links:
            m = re.fullmatch(r"(base_link|link_(\d))", link.name)
            if not m:
                continue
            idx = 0 if link.name == "base_link" else int(m.group(2))
            vs = []
            for col in link.collisions or ([link.collision] if link.collision else []):
                g = col.geometry
                fn = getattr(g, "filename", None)
                if not fn:
                    continue
                p = share / fn.split("dsr_description2/", 1)[1]
                v = _load_dae(p, float((g.scale or [1, 1, 1])[0]))
                o = col.origin
                if o is not None and (any(o.xyz) or any(o.rpy)):
                    raise NotImplementedError(f"{link.name} collision origin {o.xyz} {o.rpy}")
                vs.append(v)
            if vs:
                v = np.vstack(vs)
                if len(v) > max_pts_per_link:
                    v = v[rng.choice(len(v), max_pts_per_link, replace=False)]
                self.links[link.name] = (idx, v)
        self.tcp_off = np.asarray(tcp_offset_m, float)
        self.gripper_r = gripper_r
        self.pad_off = None if pad_offset_tool_m is None else np.asarray(pad_offset_tool_m, float)
        self.pad_r, self.pad_n_tool = pad_r, np.asarray(pad_normal_tool, float)

    def link_points(self, q):
        """base 좌표 꼭짓점 {이름: (N,3)} + 그리퍼 캡슐 축 표본 + 패드 원판 둘레 표본."""
        frames, Tf = self.chain.fk_frames(q)
        out = {}
        for name, (idx, v) in self.links.items():
            if idx == 0:
                T = np.eye(4)
            else:
                T = frames[idx - 1].copy()
                c, s = np.cos(q[idx - 1]), np.sin(q[idx - 1])
                Rz = np.eye(4); Rz[:2, :2] = [[c, -s], [s, c]]
                T = T @ Rz
            out[name] = v @ T[:3, :3].T + T[:3, 3]
        R, p = Tf[:3, :3], Tf[:3, 3]
        tcp = p + R @ self.tcp_off
        out["gripper"] = np.linspace(p, tcp, 12)            # 캡슐 축 — 거리에서 gripper_r 를 뺀다
        if self.pad_off is not None:
            c = tcp + R @ self.pad_off
            n = R @ self.pad_n_tool
            a = np.cross(n, [0, 0, 1.0]); a = a / np.linalg.norm(a) if np.linalg.norm(a) > 1e-6 else np.array([1.0, 0, 0])
            b = np.cross(n, a)
            t = np.linspace(0, 2 * np.pi, 24, endpoint=False)
            out["pad"] = np.vstack([c, c + self.pad_r * (np.outer(np.cos(t), a) + np.outer(np.sin(t), b))])
        return out

    def clearance(self, q, boxes, skip=("base_link", "link_1")):
        """상자들까지 최소 여유 [m] 와 그 부위. skip: 상자와 원래 붙어 있을 수 있는 고정부(베이스·J1 몸통)는 따로 볼 때만."""
        best, who = np.inf, ""
        for name, pts in self.link_points(q).items():
            if name in skip:
                continue
            r = self.gripper_r if name == "gripper" else 0.0
            for bx in boxes:
                d = float(bx.dist(pts).min()) - r
                if d < best:
                    best, who = d, f"{name}↔{bx.name}"
        return best, who
