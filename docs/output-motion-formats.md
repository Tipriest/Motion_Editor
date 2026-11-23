# 动作输出格式标准

核对日期：2026-11-23。本文描述应用当前导出的内容，不承诺保留原文件全部字段或满足所有外部训练框架的版本要求。

相关文档：[动作输入格式](input-motion-formats.md) · [模型与固定关节/刚体顺序](model-formats.md)。`T` 为输出帧数，`D` 为当前动作关节列数；位置单位与模型一致，推荐米，转动关节为弧度。

## 1. 输出能力与界面限制

当前编辑/导出基于机器人 `MotionClip`。BVH、SMPL 播放不提供这些格式的直接导出，也不导出 URDF、人体模型、OBJ、STL、DAE。

| 输出 | 扩展名 | 核心内容 | 约束 |
| --- | --- | --- | --- |
| CSV | `.csv` | 每帧根 XYZ + XYZW + D 个关节 | 当前 CSV 原格式导出；无表头、无 FPS |
| GMR PKL | `.pkl` | `fps/root_pos/root_rot/dof_pos/link_body_list/local_body_pos` | 当前 GMR 原格式导出；D 为动作 schema 大小 |
| MimicKit PKL | `.pkl` | `frames/fps` | 当前 MimicKit 原格式导出；旋转向量 |
| GMR NPZ / Robot-State | `.npz` | `fps/root_pos/root_rot/dof_pos` | 固定 Tiangong3 29 顺序，不含头部 |
| UFO Training / MotionLib | `.pkl` | 动作键 → 完整 record | 固定 Tiangong3 29 DOF、30 bodies；需模型前向运动学 |
| UFO Reference | `.npz` | 关节、刚体姿态与速度 | Tiangong3 29 DOF、30 bodies；需前向运动学 |
| DEX_MOSAIC | `.npz` | 同上，DEX 布局 | DEX EVT 29 DOF、39 bodies；需前向运动学 |
| 批量 GMR | `.zip` | 多个独立 GMR PKL | Motion Browser 的批量出口；导入前先解压 |
| 批量 UFO Training | `.pkl` | 一个字典，包含多条 MotionLib record | Motion Browser 的批量出口 |

### 1.1 格式菜单不是任意转换器

依据：[MotionFormatModes.ts](../src/app/MotionFormatModes.ts)。

| 识别到的模型类别 | 除当前源格式外的可选输出 |
| --- | --- |
| Tiangong3 | GMR NPZ、UFO Training PKL、UFO Reference NPZ、DEX_MOSAIC NPZ |
| DEX EVT / EVT2 / Tiangong2dex | GMR NPZ、UFO Reference NPZ |
| 其他机器人 | 仅当前源格式对应的输出 |

模型类别通过 robot name/URDF 路径中的名称标识推断，不会验证或补造所有需要的 link。菜单可选也必须满足该格式的关节名、body 名和采样要求；缺少必需 link 会导致导出失败。

“当前源格式”映射：CSV→CSV；MimicKit→MimicKit PKL；GMR→GMR PKL；UFO Training→UFO Training PKL；UFO Reference→UFO Reference NPZ；Robot-State→GMR NPZ。

DEX 模型选 UFO Reference NPZ 时仍走 **30-body Tiangong3** 输出路径；它不自动跟随源文件变成 39-body。39-body 必须是 `DEX_MOSAIC` 路径。GMR NPZ 则无论源关节排序如何，都按名称重排到 Tiangong3 29 顺序。

## 2. 导出前变换

依据：[App.ts](../src/app/App.ts)、[MotionClipResampling.ts](../src/io/motion/MotionClipResampling.ts)。

单段导出按以下顺序执行：

1. 选择起止帧：界面为 **1 开始、两端均包含**，有效范围 `1 <= start <= end <= N`。
2. 根据输出 FPS 与速度倍率重采样；速度允许 `0.1x` 到 `8x`。
3. 可选左右镜像。
4. 可选倒放。
5. 编码成选定文件结构。

设区间有 N 帧、源帧率 F、速度 s、输出帧率 R，则 N>1 时输出帧数为：

```text
T = max(2, round((N - 1) * R / (F * s)) + 1)
```

此处 round 对应实现的 JavaScript `Math.round`。单帧保留一帧。根平移和关节值线性插值，根四元数用短弧 SLERP；输出存储的 FPS 为 R。速度不是只改文件名。

镜像按机器人 X-Z 平面反射：Y 变号，左右关节交换，符号结合 URDF axis 或 pitch/roll/yaw 名称推断；要求左右配对关节完整。倒放反转帧顺序，并整体平移根轨迹，使倒放首帧根位置等于原区间首帧根位置。它不是仅反转数组而保持绝对位置不变。

导出使用的是当前编辑结果与变换后的数据；输入中的多余列、未使用字段、原始速度等不会自动透传。不要把“原格式导出”理解为无损 round-trip。

## 3. CSV 输出

依据：[CsvMotionService.toCsv](../src/io/motion/CsvMotionService.ts)、`AppController.getRobotHeight()`。

- UTF-8 数值文本，一帧一行，逗号分隔。
- **没有表头、时间戳、FPS、关节名或模型名**。
- 列顺序固定为 `[x,y,z,qx,qy,qz,qw,joint_0,...,joint_D_minus_1]`，共 `7+D` 项；关节按当前动作 schema 顺序。
- 重新导入会按当前 URDF 顺序解释，并默认 30 FPS；接收方必须另行获知实际输出 FPS 和关节顺序。
- 当前界面导出额外执行 **`z_out = z_clip + robotHeight`**。robotHeight 由预览场景中根 Y 位置减机器人包围盒最低 Y 计算，不能当作固定 URDF 常数；无法取得时为 0。
- 上述 Z 修正是现有实现行为，GMR/MimicKit/NPZ 不执行这一额外 CSV 修正，跨格式比较轨迹时必须考虑。

```text
0,0,1,0,0,0,1,0.1
0.01,0,1,0,0,0,1,0.2
```

示例只有一个关节。CSV 的数值精度来自内存 Float32Array 的十进制字符串表达，不固定小数位数。

## 4. GMR PKL 输出

依据：[GmrMotionService.toGmrPkl](../src/io/motion/GmrMotionService.ts)。

根对象为 Python 字典，轨迹字段为 **嵌套 Python list**，不是 NumPy ndarray：

| 字段 | Python 类型 | 形状/内容 |
| --- | --- | --- |
| `fps` | int/float | 输出帧率 |
| `root_pos` | `list[list[float]]` | `[T,3]`，XYZ |
| `root_rot` | `list[list[float]]` | `[T,4]`，**XYZW** |
| `dof_pos` | `list[list[float]]` | `[T,D]`，当前动作 schema 顺序 |
| `link_body_list` | `list[str]` | D 项关节名，与 dof_pos 列对应 |
| `local_body_pos` | `None` | 恒为 None，不输出刚体位置数组 |

使用 `picklefriend` 编码普通 pickle 字典。下游若要求 ndarray，应在 Python 中显式 `np.asarray(..., dtype=np.float32)`，不要假定此 PKL 的数组类型与 UFO Training 相同。

31-DOF 动作会输出 31 列；此路径不会固定截成 29，也不会保留输入文件中已被忽略的字段。

## 5. MimicKit PKL 输出

依据：[MimicKitMotionService.toMimicKitPkl](../src/io/motion/MimicKitMotionService.ts)。

| 字段 | 类型/形状 | 内容 |
| --- | --- | --- |
| `fps` | 数值标量 | 输出帧率 |
| `frames` | 嵌套 Python list `[T,6+D]` | `[x,y,z,rx,ry,rz,joint_0,...]` |

根旋转从内存 XYZW 四元数转为旋转向量，前三项为根平移；旋转向量长度为角度（弧度），不是欧拉角。当前转换使用 `2*acos(qw)`，应保证输入四元数有效且单位化。

不输出关节名称、身体轨迹或模型标识；接收方必须使用相同关节顺序。

## 6. GMR NPZ / Robot-State 输出

依据：[GmrNpzService.ts](../src/io/motion/GmrNpzService.ts)。

| 字段 | dtype | 形状 | 内容 |
| --- | --- | --- | --- |
| `fps` | `<f4` | `()` | 输出帧率 |
| `root_pos` | `<f4` | `[T,3]` | 根 XYZ |
| `root_rot` | `<f4` | `[T,4]` | **XYZW** |
| `dof_pos` | `<f4` | `[T,29]` | 固定 Tiangong3 29 顺序 |

从当前动作按关节名提取 29 列；缺一项即失败。头部关节不写出；DEX EVT 源顺序也会重排为 Tiangong3 顺序。

ZIP 内包含四个同名 `.npy`，不带 `joint_names`、`body_names`、速度或 local_body_pos。可由[Robot-State 输入协议](input-motion-formats.md)重新读取，但没有逐刚体世界姿态，不能替代完整 UFO Reference NPZ。

## 7. UFO Reference / DEX_MOSAIC NPZ 输出

依据：[UfoReferenceNpzService.ts](../src/io/motion/UfoReferenceNpzService.ts)。

| 字段 | dtype | 形状 | 含义/单位 |
| --- | --- | --- | --- |
| `fps` | `<f4` | `()` | 输出 FPS |
| `joint_pos` | `<f4` | `[T,29]` | 关节位置，转动关节弧度 |
| `joint_vel` | `<f4` | `[T,29]` | 关节速度，rad/s |
| `body_pos_w` | `<f4` | `[T,B,3]` | 刚体位置，推荐米 |
| `body_quat_w` | `<f4` | `[T,B,4]` | **WXYZ** 单位四元数 |
| `body_lin_vel_w` | `<f4` | `[T,B,3]` | 线速度，m/s |
| `body_ang_vel_w` | `<f4` | `[T,B,3]` | 角速度，rad/s |

- UFO Reference：B=30，关节/刚体按 Tiangong3 固定表。
- DEX_MOSAIC：B=39，关节/刚体按 DEX EVT 固定表。**29 列的顺序也不同**，不是只多了 9 个 body。
- 输出无名称数组；必须携带或引用[固定索引表](model-formats.md#joint-layouts)。

上述顺序是本应用导出器的明确写出约定。一个外部 NPZ 即使具有相同字段与形状，也不能由此确认采用同一顺序；导入器按约定赋名不是自动识别或语义验证。若源动作导入时已使用错误的顺序假设，重新导出也不会自动纠正该错误。

### 坐标与采样

`body_*_w` 来自当前 URDF 的逐帧前向运动学。导出时计算 `inverse(robot.parent.matrixWorld) * link.matrixWorld`，去掉 viewer 外层 model-root 的显示变换，再分解位置与旋转。因此文件中的“world”是模型原始运动坐标系，不是截图里的 Three.js 显示坐标。

每个必需关节必须在动作 schema 中存在，每个必需 body 必须在当前 robot.links 中存在并具有有效 matrixWorld。不能只凭 root_pos/dof_pos，在没有正确模型的情况下生成真实 body 轨迹。

四元数会归一化并保持相邻帧符号连续。平移与关节速度：内部帧用中心差分，首尾用单边差分；单帧速度全为 0。角速度通过 `q_to * inverse(q_from)` 的短弧旋转除采样时间求得，表达在相应 world/model 坐标中。写出前检查这些数组的有限值。

### NPZ 物理编码

两种完整 NPZ 和 GMR NPZ 都使用 [NpzWriter.ts](../src/io/motion/NpzWriter.ts)：ZIP Store、NPY v1.0、C 顺序、小端 float32。`fps` 是真正的 0 维数组。可用 `np.load(path, allow_pickle=False)` 读取。

<a id="ufo-training"></a>
## 8. UFO Training / MotionLib PKL 输出

依据：[UfoTrainingPklService.ts](../src/io/motion/UfoTrainingPklService.ts)、[NumpyPickleWriter.ts](../src/io/motion/NumpyPickleWriter.ts)。

文件为 **pickle protocol 4** 字典：`{motion_key: record}`。单段也是这一层嵌套，不是直接把 record 放在根对象。

| record 字段 | Python 类型/dtype | 形状 | 内容 |
| --- | --- | --- | --- |
| `root_trans_offset` | ndarray float32 | `[T,3]` | FK pelvis 根位置 |
| `root_rot` | ndarray float32 | `[T,4]` | FK pelvis 根旋转，**XYZW** |
| `dof_pos` | ndarray float32 | `[T,29]` | Tiangong3 固定 29 顺序 |
| `pose_aa` | ndarray float32 | `[T,30,3]` | 根全局轴角 + 29 个单轴关节局部旋转向量 |
| `pose_quat_global` | ndarray float32 | `[T,30,4]` | 30 个刚体的全局 **XYZW** 单位四元数 |
| `fps` | Python int/float | 标量 | 输出帧率 |
| `joint_names` | `list[str]` | 29 项 | Tiangong3 固定关节表 |
| `body_names` | `list[str]` | 30 项 | Tiangong3 固定 body 表 |
| `motion_key` | `str` | 标量 | 与外层键一致 |

`pose_aa[:,0,:]` 是 pelvis 全局旋转向量；后 29 项根据关节名字将角度填入对应轴：roll→X、pitch→Y、yaw→Z。它们**不是 30 个 body 的全局轴角**，也不是 SMPL 人体姿态。

body 的 FK 采样和去除 viewer 外层变换的规则与 UFO Reference 一致；不输出额外速度字段。需要完整 29 个关节和 30 个 link。新导出写 `dof_pos`，不写旧字段 `dof`。

单段 motion key 去掉扩展名，把非英文字母/数字/下划线/连字符替换为下划线，再移除首尾下划线；空结果回退 `modified_motion`。全局四元数会做时间符号连续处理。

## 9. 批量导出

Motion Browser 可把兼容的 CSV、GMR/MimicKit/UFO Training PKL、UFO Reference/Robot-State NPZ 读成机器人动作。BVH 不提供机器人 DOF，SMPL NPZ 也不属于该批量转换链路。

浏览器检查先要求已加载 URDF。批量 **GMR PKL** 接受成功解析为机器人 MotionClip 的条目，不额外要求 Tiangong3；批量 **UFO Training** 还要求当前模型被识别为 Tiangong3，且动作包含全部 Tiangong3 29 项关节。两种批量导出都会先按选定 FPS 与速度重采样；不能把 UFO 的兼容检查要求套到 GMR ZIP 上。

### GMR PKL ZIP

- 一个 ZIP 内每段动作对应一个独立的 GMR PKL，内容与第 4 节相同。
- ZIP 不压缩，不包含额外 manifest；不是 NPZ。
- 路径分隔符先转成 `__`，再按导出文件命名规则生成条目名；撞名时追加 `_2`、`_3` 等。
- 下载名：`motion_batch_{数量}_{速度}x_gmr.zip`。
- 不能把整个 ZIP 直接当动作导入，应先解压并选目标 PKL。

### UFO Training 单 PKL

- 一个 PKL 中存多条完整记录：`{key_a: record_a, key_b: record_b, ...}`；每条结构与第 8 节一致。
- key 基于完整相对路径去扩展名、把 `/` 替换成 `__`，再清理为允许字符。清理后重复会报错，不会覆盖。
- 下载名：`motion_batch_{数量}_{速度}x_ufo_training.pkl`。
- 单条记录只描述自己的帧数；不同记录的 T 可以不同。

## 10. 单段文件命名

依据：[ExportFileName.ts](../src/io/motion/ExportFileName.ts)。

```text
{name}_{speed}x[_mirror][_reverse][_{format}].{extension}
```

`name` 取原名字的文件名部分并去扩展名，非 `[a-zA-Z0-9_-]` 替换为 `_`，空结果为 `motion`。速度最多保留四位小数并去掉多余尾零。格式后缀：CSV 无后缀；其余为 `gmr`、`mimickit`、`ufo_training`、`ufo_29dof`、`dex_mosaic`。

示例：`walk_1x_gmr.npz`、`walk_0.5x_mirror_reverse_ufo_training.pkl`。文件名不编码输出 FPS，接收方必须读取文件字段或单独约定 CSV 帧率。

## 11. 接收端核验示例

下面分别核验一份 GMR NPZ 与一份完整 UFO Training PKL。文件名需替换为自己的实际导出路径。

```python
import pickle
import numpy as np

with np.load("walk_1x_gmr.npz", allow_pickle=False) as z:
    assert set(z.files) == {"fps", "root_pos", "root_rot", "dof_pos"}
    T = z["root_pos"].shape[0]
    assert T > 0 and z["fps"].shape == () and float(z["fps"]) > 0
    assert z["root_pos"].shape == (T, 3)
    assert z["root_rot"].shape == (T, 4)  # XYZW
    assert z["dof_pos"].shape == (T, 29)  # Tiangong3 order
    assert all(np.isfinite(z[k]).all() for k in z.files)

with open("walk_1x_ufo_training.pkl", "rb") as f:
    motions = pickle.load(f)
for key, record in motions.items():
    T = record["dof_pos"].shape[0]
    assert T > 0 and record["motion_key"] == key
    assert record["root_trans_offset"].shape == (T, 3)
    assert record["root_rot"].shape == (T, 4)  # XYZW
    assert record["dof_pos"].shape == (T, 29)
    assert record["pose_aa"].shape == (T, 30, 3)
    assert record["pose_quat_global"].shape == (T, 30, 4)  # XYZW
    assert len(record["joint_names"]) == 29 and len(record["body_names"]) == 30
    assert float(record["fps"]) > 0
    for field in ("root_trans_offset", "root_rot", "dof_pos", "pose_aa", "pose_quat_global"):
        assert record[field].dtype == np.float32
        assert np.isfinite(record[field]).all()
```

检查形状之外，还必须比较**名称及顺序**、四元数模长、模型版本和单位。长度为 29 的数组不代表顺序正确。

## 12. 相关测试

- [GMR NPZ round-trip](../src/io/motion/GmrNpzService.test.ts)。
- [UFO Reference 输出](../src/io/motion/UfoReferenceNpzService.test.ts)。
- [UFO Training 输出](../src/io/motion/UfoTrainingPklService.test.ts)。
- [NumPy Pickle 编码](../src/io/motion/NumpyPickleWriter.test.ts)。
- [NPZ 编码](../src/io/motion/NpzWriter.test.ts)、[导出格式菜单](../src/app/MotionFormatModes.test.ts)。
- [重采样](../src/io/motion/MotionClipResampling.test.ts)、[镜像](../src/io/motion/MotionClipSagittalMirror.test.ts)、[倒放](../src/io/motion/MotionClipTimeReverse.test.ts)。
