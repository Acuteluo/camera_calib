#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把标定结果渲染成 ACE27 的相机配置 JSON（JSONC）。

ACE27 的 `config/cameras/*.json` 是 JSONC（允许 `//` 行注释），字段语义以该目录为权威。
本模块**不做 JSON 解析后重排**——那样会丢注释与键序——而是以某个现有相机配置为模板做
「文本手术」，只替换内参相关字段：

    image_width / image_height
    camera_matrix            （3x3 行主序）
    distortion_model
    distortion_coefficients  （rows / cols / data）
    projection_matrix        （3x4 行主序）
    文件头「内参来源」注释块

其余内容（安装外参 `odom2camera`、采集参数 `camera_config`、全部注释与缩进）逐字节保持
模板原样，保证「格式和参数与模板一模一样，只有内参不同」。

用法（库）：
    import ace_json
    text = ace_json.build(template_path, K, D, P, (w, h), model, header_lines)
"""

import json
import os
import re

import numpy as np

def default_template():
    """默认 ACE 相机配置模板。

    优先环境变量 ACE_TEMPLATE；否则找常见的 ACE27 相机配置（本工具默认配套的机器人项目）。
    模板只是"格式样板"：除了内参字段，其余内容（安装外参、采集参数、注释、缩进）都会
    原样复制，所以换成任意一份同结构的相机配置都可以。
    """
    env = os.environ.get("ACE_TEMPLATE")
    if env:
        return os.path.abspath(os.path.expanduser(env))
    return os.path.join(os.path.expanduser("~"), "ACE27", "config", "cameras", "uav.json")


DEFAULT_TEMPLATE = default_template()

# 定点位数：与模板保持一致（矩阵 5 位、畸变 6 位）
PREC_MATRIX = 5
PREC_DISTORTION = 6

# 替换说明（写进文件头注释）
DEFAULT_NOTE = "本次仅替换内参、畸变与投影矩阵；安装外参与采集参数保持模板原值。"


# --------------------------------------------------------------------------- #
# 模板定位：括号配对 / 键定位（跳过字符串与注释）
# --------------------------------------------------------------------------- #
def _scan_close(text, i):
    """text[i] 是 '{' 或 '['；返回配对收尾字符的下标（跳过字符串与注释）。"""
    start = i
    open_ch = text[i]
    close_ch = "}" if open_ch == "{" else "]"
    depth = 0
    in_str = False
    esc = False
    line_c = False
    block_c = False
    while i < len(text):
        ch = text[i]
        nxt = text[i + 1] if i + 1 < len(text) else ""
        if line_c:
            if ch == "\n":
                line_c = False
            i += 1
            continue
        if block_c:
            if ch == "*" and nxt == "/":
                block_c = False
                i += 2
                continue
            i += 1
            continue
        if in_str:
            if esc:
                esc = False
            elif ch == "\\":
                esc = True
            elif ch == '"':
                in_str = False
            i += 1
            continue
        if ch == '"':
            in_str = True
            i += 1
            continue
        if ch == "/" and nxt == "/":
            line_c = True
            i += 2
            continue
        if ch == "/" and nxt == "*":
            block_c = True
            i += 2
            continue
        if ch == open_ch:
            depth += 1
        elif ch == close_ch:
            depth -= 1
            if depth == 0:
                return i
        i += 1
    raise ValueError("模板 JSON 括号不配对（位置 %d 起）" % start)


def _find_object(text, key):
    """定位 `"key": { ... }`，返回 (起点, 终点+1)。"""
    m = re.search(r'"%s"\s*:\s*\{' % re.escape(key), text)
    if not m:
        raise KeyError("模板里找不到对象键 %r" % key)
    start = text.index("{", m.start())
    return start, _scan_close(text, start) + 1


def _find_array_in(text, start, end, key="data"):
    """在 [start,end) 内定位 `"key": [ ... ]`，返回 (数组起点, 数组终点+1)。"""
    seg = text[start:end]
    m = re.search(r'"%s"\s*:\s*\[' % re.escape(key), seg)
    if not m:
        raise KeyError("模板里找不到数组键 %r" % key)
    a = start + seg.index("[", m.start())
    return a, _scan_close(text, a) + 1


# --------------------------------------------------------------------------- #
# 数值与数组格式化（贴合模板风格）
# --------------------------------------------------------------------------- #
def _fmt(value, prec):
    """模板风格：精确 0/1 写成 `0.0`/`1.0`，其余用定点小数。"""
    v = float(value)
    if v == 0.0:
        return "0.0"
    if v == 1.0:
        return "1.0"
    return "%.*f" % (prec, v)


def _replace_array(text, key, values, prec):
    """把 `"key": { ... "data": [ ... ] ... }` 里的 data 换成模板风格的多行数组。"""
    start, end = _find_object(text, key)
    a, b = _find_array_in(text, start, end)
    seg = text[a:b]
    if "\n" not in seg:                                   # 单行数组：保持单行
        new = "[" + ", ".join(_fmt(v, prec) for v in values) + "]"
    else:
        m = re.search(r"\n([ \t]*)\S", seg)               # 取值行的缩进
        indent = m.group(1) if m else " " * 4
        last_nl = seg.rfind("\n")
        close_indent = seg[last_nl + 1:-1]                # 收尾 `]` 前的缩进
        items = []
        for i, v in enumerate(values):
            items.append("%s%s%s" % (indent, _fmt(v, prec),
                                    "," if i < len(values) - 1 else ""))
        new = "[\n" + "\n".join(items) + "\n" + close_indent + "]"
    return text[:a] + new + text[b:]


def _replace_number(text, key, literal):
    """替换 `"key": <数字>` 的取值。"""
    pat = re.compile(r'("%s"\s*:\s*)(-?[0-9][0-9.eE+-]*)' % re.escape(key))
    new, n = pat.subn(lambda m: m.group(1) + literal, text, count=1)
    if n != 1:
        raise KeyError("模板里找不到数值键 %r" % key)
    return new


def _replace_block_number(text, key, sub_key, literal):
    """替换对象 `key` 内部 `sub_key` 的数值（例如畸变系数的 rows/cols）。"""
    start, end = _find_object(text, key)
    seg = text[start:end]
    pat = re.compile(r'("%s"\s*:\s*)(-?[0-9][0-9.eE+-]*)' % re.escape(sub_key))
    new_seg, n = pat.subn(lambda m: m.group(1) + literal, seg, count=1)
    if n != 1:
        raise KeyError("模板 %r 里找不到数值键 %r" % (key, sub_key))
    return text[:start] + new_seg + text[end:]


def _replace_string(text, key, value):
    """替换 `"key": "..."` 的取值。"""
    pat = re.compile(r'("%s"\s*:\s*)("(?:[^"\\]|\\.)*")' % re.escape(key))
    new, n = pat.subn(lambda m: m.group(1) + json.dumps(value, ensure_ascii=False),
                      text, count=1)
    if n != 1:
        raise KeyError("模板里找不到字符串键 %r" % key)
    return new


# --------------------------------------------------------------------------- #
# 文件头注释
# --------------------------------------------------------------------------- #
def _trim_num(v):
    """方格边长显示风格：25.0 -> '25.0'，24.70 -> '24.7'，24.75 -> '24.75'。"""
    txt = "%.2f" % float(v)
    if txt.endswith("0"):
        txt = txt[:-1]
    return txt


def build_header(when, driver, pattern, square_mm, size, n_views, rms,
                 uv=None, session=None, note=DEFAULT_NOTE):
    """生成「内参来源」注释正文（与模板同风格的多行文本，不含 `//` 前缀）。

    @param when     标定时刻，如 "2026-09-23 01:23:24"
    @param driver   驱动/相机名，如 "海康 MV-CS016-10UC"
    @param pattern  内角点数 (cols, rows)
    @param square_mm 方格边长（毫米）
    @param size     图像尺寸 (w, h)
    @param n_views  有效视图数
    @param rms      重投影误差（像素）
    @param uv       可选，(u_min, u_max, v_min, v_max) 角点覆盖范围
    @param session  可选，结果目录（相对路径更佳）
    @return         注释正文行列表
    """
    cols, rows = pattern
    line1 = "内参来源：%s %s 单目标定（%dx%d 内角点 / %s mm 方格 /" % (
        when, driver, cols, rows, _trim_num(square_mm))
    cover = ""
    if uv is not None:
        cover = " / 角点覆盖 u[%d,%d] v[%d,%d]" % (uv[0], uv[1], uv[2], uv[3])
    line2 = "%dx%d / %d 视图%s）。RMS %.4f px。结果目录" % (
        size[0], size[1], n_views, cover, rms)
    line3 = ("%s。%s" % (session, note)) if session else note
    return [line1, line2, line3]


def _replace_header(text, header_lines):
    """替换文件头里以「内参来源」开头的连续 `//` 注释块。"""
    lines = text.split("\n")
    idx = None
    for i, ln in enumerate(lines):
        if ln.lstrip().startswith("//") and "内参来源" in ln:
            idx = i
            break
    if idx is None:                       # 模板没有该段，保持原样
        return text
    stripped = lines[idx].lstrip()
    indent = lines[idx][:len(lines[idx]) - len(stripped)]
    j = idx
    while j < len(lines) and lines[j].lstrip().startswith("//"):
        j += 1
    new = [indent + "// " + h for h in header_lines]
    return "\n".join(lines[:idx] + new + lines[j:])


# --------------------------------------------------------------------------- #
# 渲染
# --------------------------------------------------------------------------- #
def render(template_text, camera_matrix, distortion, projection, size,
           distortion_model="plumb_bob", header_lines=None):
    """在模板文本上替换内参字段，返回新的 JSONC 文本。

    @param template_text  模板文件全文（JSONC）
    @param camera_matrix  3x3 内参（行主序可展平为 9 个）
    @param distortion     畸变系数（长度 4/5/8/12/14）
    @param projection     3x4 投影矩阵（行主序 12 个）
    @param size           图像尺寸 (w, h)
    @param distortion_model 波形的模型名，写入 distortion_model
    @param header_lines   可选，文件头「内参来源」注释正文
    @return               替换后的 JSONC 文本
    """
    cm = np.asarray(camera_matrix, dtype=float).ravel().tolist()
    dc = np.asarray(distortion, dtype=float).ravel().tolist()
    pm = np.asarray(projection, dtype=float).ravel().tolist()
    if len(cm) != 9:
        raise ValueError("camera_matrix 需要 9 个数，收到 %d" % len(cm))
    if len(pm) != 12:
        raise ValueError("projection_matrix 需要 12 个数，收到 %d" % len(pm))
    if len(dc) not in (4, 5, 8, 12, 14):
        raise ValueError("畸变系数长度须为 4/5/8/12/14，收到 %d" % len(dc))

    out = template_text
    if header_lines:
        out = _replace_header(out, header_lines)
    out = _replace_number(out, "image_width", "%d" % int(size[0]))
    out = _replace_number(out, "image_height", "%d" % int(size[1]))
    out = _replace_string(out, "distortion_model", distortion_model)
    out = _replace_block_number(out, "distortion_coefficients", "rows", "1")
    out = _replace_block_number(out, "distortion_coefficients", "cols", "%d" % len(dc))
    out = _replace_array(out, "camera_matrix", cm, PREC_MATRIX)
    out = _replace_array(out, "distortion_coefficients", dc, PREC_DISTORTION)
    out = _replace_array(out, "projection_matrix", pm, PREC_MATRIX)
    return out


def build(template_path, camera_matrix, distortion, projection, size,
          distortion_model="plumb_bob", header_lines=None):
    """读模板文件并渲染（参数同 render）。"""
    with open(template_path, encoding="utf-8") as f:
        return render(f.read(), camera_matrix, distortion, projection, size,
                      distortion_model, header_lines)


def default_output_name(template_path):
    """输出文件名默认与模板同名（uav.json → uav.json，可直接替换回 config/cameras）。"""
    return os.path.basename(template_path)


def rel_to_home(path):
    """把绝对路径尽量写成相对家目录的形式（写进注释里更短、可读）。"""
    try:
        rel = os.path.relpath(os.path.abspath(path), os.path.expanduser("~"))
    except ValueError:
        return os.path.abspath(path)
    return os.path.abspath(path) if rel.startswith("..") else rel


# --------------------------------------------------------------------------- #
# 自检：输出必须是合法 JSONC，且除内参外的参数逐项等于模板
# --------------------------------------------------------------------------- #
def strip_comments(text):
    """去掉 JSONC 的 // 与 /* */ 注释（保留字符串内容），便于标准 JSON 解析。"""
    out = []
    i = 0
    in_str = False
    esc = False
    while i < len(text):
        ch = text[i]
        nxt = text[i + 1] if i + 1 < len(text) else ""
        if in_str:
            out.append(ch)
            if esc:
                esc = False
            elif ch == "\\":
                esc = True
            elif ch == '"':
                in_str = False
            i += 1
            continue
        if ch == '"':
            in_str = True
            out.append(ch)
            i += 1
            continue
        if ch == "/" and nxt == "/":
            while i < len(text) and text[i] != "\n":
                i += 1
            continue
        if ch == "/" and nxt == "*":
            i += 2
            while i < len(text) and not (text[i] == "*" and
                                         i + 1 < len(text) and text[i + 1] == "/"):
                i += 1
            i += 2
            continue
        out.append(ch)
        i += 1
    return "".join(out)


# 内参相关字段：这些允许与模板不同
INTRINSIC_PATHS = (
    ("image_width",), ("image_height",), ("distortion_model",),
    ("camera_matrix",), ("distortion_coefficients",), ("projection_matrix",),
)


def _leaf_diffs(a, b, path=()):
    """递归比较两个 JSON 树，返回差异的键路径列表。"""
    diffs = []
    if isinstance(a, dict) and isinstance(b, dict):
        for k in set(a) | set(b):
            if k not in a or k not in b:
                diffs.append(path + (k,))
            else:
                diffs.extend(_leaf_diffs(a[k], b[k], path + (k,)))
    elif isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            diffs.append(path)
        else:
            for i, (x, y) in enumerate(zip(a, b)):
                diffs.extend(_leaf_diffs(x, y, path + (str(i),)))
    else:
        if a != b:
            diffs.append(path)
    return diffs


def self_test(template_path=None):
    """自检：渲染后除内参字段外，模板参数必须逐项不变，且输出是合法 JSONC。"""
    tpl_path = template_path or DEFAULT_TEMPLATE
    with open(tpl_path, encoding="utf-8") as f:
        tpl = f.read()
    tpl_obj = json.loads(strip_comments(tpl))

    # 找一个 cameras.camera_N 节点，用它的当前内参做"假标定"（数值整体偏移）
    cam_key = list(tpl_obj["cameras"].keys())[0]
    cam = tpl_obj["cameras"][cam_key]
    K = np.array(cam["camera_matrix"]["data"], float).reshape(3, 3)
    D = np.array(cam["distortion_coefficients"]["data"], float)
    P = np.array(cam["projection_matrix"]["data"], float).reshape(3, 4)
    size = (int(cam["image_width"]), int(cam["image_height"]))

    K2 = K.copy()
    K2[0, 0] += 12.5
    K2[1, 1] += 11.25
    D2 = D.copy()
    if len(D2):
        D2[0] += 0.001
    P2 = P.copy()
    P2[0, 0] += 12.5

    header = build_header("2026-01-02 03:04", "自检相机", (11, 8), 25.0, size, 103,
                          0.1234, uv=(39, 1339, 31, 993),
                          session="camera_calib/标定结果/self_test")
    out = render(tpl, K2, D2, P2, size, cam.get("distortion_model", "plumb_bob"), header)
    out_obj = json.loads(strip_comments(out))

    checks = []
    diffs = _leaf_diffs(tpl_obj, out_obj)
    allowed = set()
    for p in INTRINSIC_PATHS:
        allowed.add(("cameras", cam_key) + p)
    bad = []
    for d in diffs:
        if d in allowed or (len(d) > 3 and ("cameras", cam_key, d[2]) in allowed):
            continue
        bad.append(d)
    checks.append(("除内参外参数逐项一致", not bad,
                   "多出的差异: %s" % bad if bad else "%d 处内参差异，符合预期" % len(diffs)))
    checks.append(("输出是合法 JSONC", True, "%d 字节" % len(out)))
    checks.append(("camera_matrix 已替换",
                   abs(out_obj["cameras"][cam_key]["camera_matrix"]["data"][0]
                       - K2[0, 0]) < 1e-4, "fx=%.5f" % K2[0, 0]))
    checks.append(("注释与键序保持模板风格", out.count("//") >= tpl.count("//") - 10,
                   "模板 %d 条注释，输出 %d 条" % (tpl.count("//"), out.count("//"))))

    ok = True
    for name, passed, detail in checks:
        print("  [%s] %s —— %s" % ("通过" if passed else "失败", name, detail))
        ok = ok and passed
    return ok


def main():
    import argparse
    ap = argparse.ArgumentParser(description="ACE 相机配置 JSON 渲染库自检")
    ap.add_argument("--self-test", action="store_true", help="用模板跑一遍渲染自检")
    ap.add_argument("--template", default=DEFAULT_TEMPLATE, help="模板 JSONC 路径")
    args = ap.parse_args()
    if args.self_test:
        print("==> ACE JSON 渲染自检（模板 %s）" % args.template)
        ok = self_test(args.template)
        print("==> %s" % ("全部通过" if ok else "存在失败"))
        return 0 if ok else 1
    ap.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
