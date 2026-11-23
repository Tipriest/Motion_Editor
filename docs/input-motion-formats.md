# 动作输入格式标准

核对日期：2026-11-23。本文描述当前工作区代码实际支持的输入协议，不代表同名上游项目的所有版本都兼容。

相关文档：[动作输出格式](output-motion-formats.md) · [模型与资源格式](model-formats.md)。

## 1. 使用约定与格式识别

- `T`：帧数，正整数；`D`：关节自由度列数；`J`：人体姿态关节数。
- 表中 `float32/float64` 指数值数组；推荐使用有限数值、C 连续存储的 `float32`。
- “必需”是当前解析要求；“推荐”是生成新文件应遵守的更严格约定。兼容补值、截断不等于数据语义正确。
- 机器人位置推荐单位米，转动关节为弧度，移动关节为米；机器人运动使用与 URDF 一致的坐标，当前交互约定为 X 前、Y 左、Z 上。BVH 的角度与单位规则单独说明。
- 同一文件内所有轨迹推荐有相同 `T`，不能通过修改扩展名完成格式转换。

| 文件/格式 | 内容识别依据 | 前置模型 | 根旋转 |
| --- | --- | --- | --- |
| CSV `.csv` | 有表头或定序数值列 | 已加载 URDF | XYZW 四元数 |
| GMR `.pkl` | `fps`, `root_pos`, `root_rot`, `dof_pos` | 已加载 URDF | XYZW 四元数 |
| MimicKit `.pkl` | `frames` | 已加载 URDF | 旋转向量/expmap，弧度 |
| UFO Training / MotionLib `.pkl` | 一条记录或记录字典，记录有 `root_trans_offset`, `root_rot`, `dof_pos`/`dof`, `fps` | 已加载 URDF；按关节名绑定 | XYZW 四元数 |
| Robot-State / GMR `.npz` | `fps`, `root_pos`, `root_rot`, `dof_pos` | 已加载兼容 URDF | XYZW 四元数 |
| UFO Reference / DEX EVT `.npz` | `fps`, `joint_pos`, `body_pos_w`, `body_quat_w` | 已加载兼容 URDF | body 四元数为 **WXYZ** |
| SMPL 动作 `.npz` | `poses`, `trans` | SMPL/SMPL-H/SMPL-X 模型 | 每关节三维轴角/旋转向量 |
| BVH `.bvh` | `HIERARCHY` + `MOTION` 文本 | 自带骨架 | 按 `CHANNELS` 指定的欧拉角，度 |

### 拖放路由

主界面依次检查：URDF → CSV → BVH → UFO Reference NPZ → Robot-State NPZ → 动作 PKL → SMPL 模型/动作 → OBJ。命中分支通常会结束本次处理。

因此，机器人模型与动作应分两次加载：先 URDF 文件夹，再动作文件。把 CSV、BVH 和多种 NPZ 混在一个拖放包里，不会自动把全部动作同时导入。

动作 PKL 的主界面尝试顺序是 UFO Training → MimicKit → GMR；Motion Browser 检查 PKL 时是 UFO Training → GMR → MimicKit。新文件只使用一个明确的结构，不要混合多套识别字段。

多个候选文件可能自动选择其中一个，具体路径会进入提示。需要确定性行为时只导入目标文件，或在界面明确选择目标。

## 2. NPZ 与 PKL 容器要求

### NPZ

NPZ 是 ZIP 容器，字段 `root_pos` 对应 ZIP 内的 `root_pos.npy`。字段名称及大小写必须准确，推荐直接使用 `np.savez(path, root_pos=array, ...)`，不要在 ZIP 内再包一层目录。

- 读取器支持 ZIP Store 和 Deflate；Deflate 依赖运行环境的解压能力。推荐普通 `np.savez`，兼容性最直接。
- 支持 NPY 主版本 1、2、3；数值类型为 `float32/64`、`int8/16/32/64`、`uint8/16/32/64`。64 位整数转为 JavaScript Number 后可能失去大整数精度。
- C/Fortran 数组会转为 C 顺序；推荐 C 顺序、小端格式。
- 数值字段不支持 `object`、结构化 dtype、complex、float16；不要把 Python 字典直接塞成 object array。
- SMPL 文本元数据使用 **0 维** Unicode/bytes 字符串数组，例如 `np.array("male")`，不是 `np.array(["male"])`。
- `mocap_framerate`/`mocap_frame_rate` 必须是 0 维数值；机器人 NPZ 的 `fps` 读取第一个数值，推荐同样写为 0 维。
- 独立 `.npy` 不是主界面动作输入格式；其支持仅限作为 NPZ 内部字段。

### PKL

- 使用 Python pickle 的字典、字符串、数字及下文指定的数组。不是 JSON 文本，也不是任意 Python 对象的通用反序列化接口。
- 推荐 Python `pickle.dump(..., protocol=4)`；NumPy 数组必须是当前读取器支持的 ndarray 序列化结构，不能假定任意协议或 out-of-band buffer 都兼容。
- GMR、MimicKit 同时接受规则的二维 Python list；UFO Training 的三个核心轨迹字段要求 NumPy ndarray，**不能直接换成二维 list**。
- PKL ndarray 解码支持 `float32/64`、`int8/16/32`、`uint8/16/32`、bool，支持 C/Fortran 顺序；不要使用 int64、float16、complex 或 object 数组。
- 读取器还兼容部分旧版、未压缩 Joblib `NumpyArrayWrapper` 布局；这不等于支持任意 Joblib 压缩文件。新文件推荐普通 pickle。
- `.p`、`.pickle`、`.joblib` 不会自动作为 `.pkl` 路由。转换内容并使用受支持扩展名后再导入。

## 3. CSV：机器人关节轨迹

依据：[CsvMotionService.ts](../src/io/motion/CsvMotionService.ts)、[URDF schema 生成](../src/io/urdf/UrdfLoadService.ts)。

### 3.1 定序模式

每行一帧，严格推荐 `7 + D` 列：

```text
root_x,root_y,root_z,root_qx,root_qy,root_qz,root_qw,joint_0,...,joint_D_minus_1
```

上述一行只是列顺序说明；**无表头文件实际应从数值行开始**。关节列按当前 URDF 动作 schema 的顺序排列：排除 `fixed` 和 `floating` 后的关节枚举顺序，不能只凭 DOF 数量猜测。

第一行可多于 `7 + D` 列，多余尾列会被忽略并产生提示；所有后续数据行仍必须与首行总列数相同。少列报错。

### 3.2 表头模式

根字段支持以下别名，大小写不敏感：

| 标准名 | 简写 |
| --- | --- |
| `root_x`, `root_y`, `root_z` | `x`, `y`, `z` |
| `root_qx`, `root_qy`, `root_qz`, `root_qw` | `qx`, `qy`, `qz`, `qw` |

还必须包含当前 URDF 中所有非固定、非浮动关节的名称。按名称重排列；多余列被忽略并提示，缺少根字段或任一必需关节会失败。表头可任意排序，推荐名称唯一。

```csv
root_x,root_y,root_z,root_qx,root_qy,root_qz,root_qw,knee
0,0,1,0,0,0,1,0.1
0.01,0,1,0,0,0,1,0.2
```

此示例只适用于动作 schema 恰好包含 `knee` 一个关节的 URDF。

### 3.3 兼容边界

- 第一条非空行含任意不能转为有限数值的 token，即被当作表头。数据首行包含 NaN/Infinity 时可能因此被误识别为表头。
- 仅支持简单逗号分隔，不支持带逗号的引号字段、注释行、分号或制表符分隔。
- 空白行跳过；空单元格经 `Number("")` 当前会变成 0。**生成文件时禁止空单元格**，避免静默补零。
- 文件中不读取时间戳或 FPS 字段。导入默认 **30 FPS**；在界面调整播放帧率。输出 CSV 同样不携带帧率。
- 推荐根四元数已单位化且非零。CSV 解析本身不会据此拒绝或修复整份文件。

## 4. GMR PKL

依据：[GmrMotionService.ts](../src/io/motion/GmrMotionService.ts)。根对象必须为单段动作字典。

| 字段 | 必需 | 推荐类型/形状 | 含义及当前兼容行为 |
| --- | --- | --- | --- |
| `fps` | 是 | 正数标量 | 缺失报错；存在但无效时提示并回退 30 |
| `root_pos` | 是 | 浮点 ndarray 或二维 list `[T,3]` | 根 XYZ 位置 |
| `root_rot` | 是 | 浮点 ndarray 或二维 list `[T,4]` | **XYZW**；导入会归一化；接近零的四元数替换为 `[0,0,0,1]` 并提示 |
| `dof_pos` | 是 | 浮点 ndarray 或二维 list `[T,D]` | 关节位置；三组轨迹帧数必须一致 |
| `link_body_list` | 否，强烈推荐 | `list[str]`，长度 D | 此应用把它视作 **关节列名**，不是刚体列表 |
| `local_body_pos` | 否 | 任意/`None` | 当前播放不读取；不能替代 `root_pos` |

`link_body_list` 缺失/None 会提示；代码还接受逗号分隔字符串，但新文件应使用唯一、非空的字符串列表。

关节映射规则：

1. D 与模型一致且名称全部匹配时，按名称重排。
2. D 与模型一致但名称不能匹配时，仍可能导入，并提示映射无法确认；不能据“导入成功”认定姿态正确。
3. D 不同，只有可按有效名称映射时才接受；标准 Tiangong3 29 ↔ 31 还支持无名称推断。29 → 31 将两项头部关节置 0；31 → 29 丢弃头部两项。
4. 一般模型不接受任意缺关节；Tiangong3 头部兼容规则不是通用补零规则。

Tiangong3 的 29/31 顺序见[固定索引表](model-formats.md#joint-layouts)。

最小生成示例，仍需匹配只有 `knee` 一个动作关节的 URDF：

```python
import pickle

data = {
    "fps": 50,
    "root_pos": [[0.0, 0.0, 1.0], [0.01, 0.0, 1.0]],
    "root_rot": [[0.0, 0.0, 0.0, 1.0]] * 2,
    "dof_pos": [[0.1], [0.2]],
    "link_body_list": ["knee"],
    "local_body_pos": None,
}
with open("example_gmr.pkl", "wb") as f:
    pickle.dump(data, f, protocol=4)
```

## 5. MimicKit PKL

依据：[MimicKitMotionService.ts](../src/io/motion/MimicKitMotionService.ts)。

| 字段 | 必需 | 类型/形状 | 要求 |
| --- | --- | --- | --- |
| `frames` | 是 | 浮点 ndarray 或规则二维 list `[T,6+D]` | T>0，D>0，D 必须等于当前 URDF 动作关节数 |
| `fps` | 推荐 | 正数标量 | 缺失或无效时 30 FPS，并提示 |

每帧顺序为 `[x,y,z,rx,ry,rz,joint_0,...]`。`[rx,ry,rz]` 是**旋转向量**：方向为旋转轴，长度为旋转角度（弧度），不是 Euler XYZ，也不是四元数。

关节没有名称映射元数据；严格按当前 URDF schema 的顺序解释。Tiangong3 的 GMR 29/31 自动适配不适用于 MimicKit。

```python
import pickle

data = {"fps": 50, "frames": [
    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.1],
    [0.01, 0.0, 1.0, 0.0, 0.0, 0.0, 0.2],
]}
with open("example_mimickit.pkl", "wb") as f:
    pickle.dump(data, f, protocol=4)
```

该示例同样只有一个关节，不是 29/31-DOF 模型的完整示例。

## 6. UFO Training / MotionLib PKL

依据：[UfoTrainingPklMotionService.ts](../src/io/motion/UfoTrainingPklMotionService.ts)。推荐外层为动作键字典：

```text
{"walk": record_1, "run": record_2, ...}
```

也接受根对象直接为一条 record。多记录默认选第一条可识别记录；可在动作目录选择具体记录。当前选择器请求的 motion key 不存在时会回退第一条，调用方应检查实际选择结果。

| record 字段 | 播放必需 | 类型/形状 | 要求 |
| --- | --- | --- | --- |
| `root_trans_offset` | 是 | NumPy ndarray `[T,3]` | 根平移；当前直接作为根位置，不再额外加一个 offset |
| `root_rot` | 是 | NumPy ndarray `[T,4]` | **XYZW**；非零、有限；导入归一化 |
| `dof_pos` | 是，或旧字段 `dof` | NumPy ndarray `[T,D]` | 同一 T；旧 `dof` 会提示 |
| `fps` | 是 | 正数；推荐 Python int/float | 无效报错，不回退 30 |
| `joint_names` | 条件必需 | 唯一非空的 `list[str]`，长度 D | 缺失时 D 必须为 29，并按 Tiangong3 29 顺序解释 |
| `motion_key` | 否 | 字符串 | 单记录命名信息 |
| `pose_aa`, `pose_quat_global`, `body_names` | 否 | 见[完整导出协议](output-motion-formats.md#ufo-training) | 当前编辑播放只使用根姿态与 DOF，不读取这些姿态字段 |

提供有效 `joint_names` 时解析器可接受非 29 的 D，但仍须能绑定当前 URDF。这与“能够导出标准 UFO 训练包”是不同要求；标准输出固定 Tiangong3 29 DOF/30 bodies。

```python
import pickle
import numpy as np

T = 2
record = {
    "fps": 50,
    "root_trans_offset": np.zeros((T, 3), dtype=np.float32),
    "root_rot": np.tile(np.array([0, 0, 0, 1], np.float32), (T, 1)),
    "dof_pos": np.zeros((T, 29), dtype=np.float32),
    # 未提供 joint_names：本示例明确采用 Tiangong3 29 顺序。
}
with open("example_ufo_training.pkl", "wb") as f:
    pickle.dump({"walk": record}, f, protocol=4)
```

此为本应用**可播放的最小子集**，不能当作下游训练程序要求的完整记录。

## 7. Robot-State / GMR NPZ

依据：[RobotStateNpzMotionService.ts](../src/io/motion/RobotStateNpzMotionService.ts)。四项缺一不可：

| 字段 | 推荐 dtype | 形状 | 含义 |
| --- | --- | --- | --- |
| `fps` | float32 | `()`，当前也读非空数组的首值 | 正帧率 |
| `root_pos` | float32 | `[T,3]` | 根位置 |
| `root_rot` | float32 | `[T,4]` | **XYZW**，不得为零四元数 |
| `dof_pos` | float32 | `[T,29]` | 固定 Tiangong3 29 顺序 |

所有数值必须有限。根四元数模长小于 `1e-8` 会报错，非单位但有效的四元数会归一化。纯 `dof_pos` NPZ、任意 D 的 GMR NPZ、DEX EVT 列顺序直接装入此结构，都不满足本协议。

```python
import numpy as np

T = 2
np.savez("example_robot_state.npz",
         fps=np.array(50, dtype=np.float32),
         root_pos=np.zeros((T, 3), dtype=np.float32),
         root_rot=np.tile(np.array([0, 0, 0, 1], np.float32), (T, 1)),
         dof_pos=np.zeros((T, 29), dtype=np.float32))
```

## 8. UFO Reference / DEX EVT NPZ

依据：[UfoReferenceMotionService.ts](../src/io/motion/UfoReferenceMotionService.ts)。

| 字段 | 播放必需 | 形状 | 含义 |
| --- | --- | --- | --- |
| `fps` | 是 | 推荐 `()` | 正帧率，读取首个数值 |
| `joint_pos` | 是 | `[T,29]` | 关节位置 |
| `body_pos_w` | 是 | `[T,B,3]` | 世界/模型原始坐标中的刚体位置 |
| `body_quat_w` | 是 | `[T,B,4]` | **WXYZ**，不是 XYZW |
| `joint_vel` | 否 | `[T,29]` | 当前播放忽略；完整输出包含 |
| `body_lin_vel_w`, `body_ang_vel_w` | 否 | `[T,B,3]` | 当前播放忽略；完整输出包含 |

`B` 必须是 30 或 39，当前解析器据此**选择假定的 joint_pos 顺序**，不能据此验证文件真实的列语义：

- B=30：Tiangong3 固定 29 关节顺序；使用 Tiangong3 30-body 表。
- B=39：DEX EVT 29 个关节的广度优先约定；使用 DEX EVT 39-body 表。

完整顺序见[模型固定索引表](model-formats.md#joint-layouts)。不能只修改 B 或数组形状来转换格式。

**协议约定不是文件自证。** 当前读取器不会读取或核验 `joint_names/body_names` 元数据，而是直接给数值列赋予上述固定名称。即使额外添加这两个字段，也不会自动按它们重排。来自本应用且源动作映射正确的导出，可依据导出器确认其顺序；外部文件必须核对生成脚本、数据集约定或明确的索引映射。没有这些依据时，真实顺序应标记为“未验证”，不能因为结构可导入而认定符合约定。

例如当前 `assets/left_punch.npz` 有 `fps=50`、`joint_pos [150,29]`、`body_pos_w [150,30,3]`、`body_quat_w [150,30,4]` 及速度字段，但没有名称元数据。程序因此按 30-body Tiangong3 布局解释它；这些字段和形状本身不能证明真实关节顺序、body 顺序或四元数分量顺序正确。文件名也不是证据。

解析器检查四个必需字段的形状、T、有限值；取 **body 索引 0 的 pelvis** 作为根位置与根四元数，并用关节位置驱动当前 URDF。其余 body 轨迹不会逐刚体强制覆盖 URDF 的前向运动学；只有 pelvis 的四元数执行非零校验/归一化。

以下是可播放的 30-body 最小结构示例。body 的零值仅作结构占位，真实导出/训练数据必须有正确的前向运动学：

```python
import numpy as np

T, B = 2, 30
q = np.zeros((T, B, 4), dtype=np.float32)
q[..., 0] = 1  # WXYZ identity
np.savez("example_ufo_reference.npz",
         fps=np.array(50, dtype=np.float32),
         joint_pos=np.zeros((T, 29), dtype=np.float32),
         body_pos_w=np.zeros((T, B, 3), dtype=np.float32),
         body_quat_w=q)
```

## 9. SMPL / AMASS 动作 NPZ

依据：[SmplMotionService.ts](../src/io/motion/SmplMotionService.ts)、[SmplMotionPlayer.ts](../src/motion/SmplMotionPlayer.ts)。模型字段见[模型文档](model-formats.md#smpl-models)。

| 字段 | 必需 | 推荐形状/类型 | 行为 |
| --- | --- | --- | --- |
| `poses` | 是 | 浮点 `[T,3J]` | 完整逐关节轴角，包含第 0 项根关节旋转；不是四元数，也不是手部 PCA 系数 |
| `trans` | 是 | 浮点 `[T,3]` | 根平移 |
| `betas` | 否 | 浮点 `[K]` | 一段动作的一组体型系数；缺失用 0 |
| `mocap_framerate` | 否 | 正数 0 维标量 | 优先于下项；缺失时看下项 |
| `mocap_frame_rate` | 否 | 正数 0 维标量 | 两者都缺失时 30 FPS |
| `gender` | 否 | 0 维字符串 | `male/female/neutral`，用于模型选择与性别检查 |
| `seq_name`, `obj_name` | 否 | 0 维字符串 | 序列名、物体名；还用于物体模型匹配 |
| `trans2joint` | 否 | 浮点 `[3]` | 物体对齐偏移，兼容 `[1,>=3]` / `[>=3,1]` |

推荐 `poses.shape[1] % 3 == 0`，且 J 与模型关节序一致。代码按 `floor(width/3)` 计算关节数，并只动画模型/动作共同拥有的前若干关节；这不是跨 SMPL/SMPL-H/SMPL-X 的自动重定向。

当前兼容行为：

- `trans` 可多于 3 列，只取 XYZ 并提示；poses/trans 的 T 不同会截断到较短者并提示。
- `betas` 展平成一维，只取模型支持的前 K 项；不足补零。不要用 `[T,K]` 表示逐帧体型。
- 存在的帧率字段通过 `max(1, value)` 使用；不要依赖此行为修复 NaN/Infinity。新数据必须主动检查有限、正数。
- 男/女明确不匹配会失败；neutral 或未知性别有兼容/提示。模型性别还依赖文件名/路径，应保留明确命名。
- 播放时减去首帧 `trans.x/y`，Z 保留；原始 XY 绝对位置不会原样呈现。
- 只有动作 NPZ 时必须已激活 SMPL 模型；也可把模型和动作一起导入。

```python
import numpy as np

T, J = 2, 24  # 必须改成目标模型的真实关节布局
np.savez("example_smpl_motion.npz",
         poses=np.zeros((T, 3 * J), dtype=np.float32),
         trans=np.zeros((T, 3), dtype=np.float32),
         betas=np.zeros(10, dtype=np.float32),
         mocap_framerate=np.array(30, dtype=np.float32),
         gender=np.array("neutral"))
```

### 9.1 SMPL 主物体轨迹扩展

必须先存在 `obj_trans` 才会构造主物体轨迹。OBJ 几何模型另行提供，见[物体资源](model-formats.md#obj-models)。

| 字段 | 推荐形状 | 解释及兼容 |
| --- | --- | --- |
| `obj_trans` | `[T,3]` | 也支持 `[T,3,1]`；二维多余列忽略 |
| `obj_rot_mat` | `[T,3,3]` | 可选；行优先 3×3 旋转矩阵，也接受 `[T,>=9]` 的前 9 项 |
| `obj_scale` | `[T]` 或 `[T,1]` | 可选；每帧统一比例，二维只读首列；不是三轴独立缩放 |
| `obj_bottom_trans`, `obj_bottom_rot_mat`, `obj_bottom_scale` | 不参与播放 | 会提示：当前仅播放主物体轨迹 |

物体有效 T 取人/物体各已有轨迹的最小值，之后沿用最后一帧。`trans2joint` 不存在时，疑似 OMOMO 序列可能使用内置性别相关偏移并提示；需要精确复现时应提供真实偏移。

## 10. BVH

依据：[BvhMotionService.ts](../src/io/motion/BvhMotionService.ts) 和当前 Three.js BVHLoader。

必须是包含以下结构的 BVH 文本：

```text
HIERARCHY
ROOT Hips
{
  OFFSET 0 0 0
  CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation
  End Site
  {
    OFFSET 0 1 0
  }
}
MOTION
Frames: 2
Frame Time: 0.0333333333
0 0 0 0 0 0
0.1 0 0 0 0 0
```

- `ROOT/JOINT` 声明层级与 OFFSET；每帧列数及遍历顺序必须对应所有声明的 CHANNELS。
- position 的长度单位由界面 **BVH Unit** 指定：`m`→1，`dm`→0.1，`cm`→0.01，`inch`→0.0254，`feet`→0.3048；默认 `m`。不是自动单位识别。
- 旋转通道是度，顺序依照文件 CHANNELS；帧率通常为 `1 / Frame Time`。缺少可靠采样间隔时内部可能回退 30。
- 预览使用 +Y 上；根轨迹减去首帧 X/Z，保留 Y 高度。更换单位会影响骨架整体比例。
- 自带骨架，只用于 BVH 播放；不会自动转换成机器人关节轨迹或 SMPL，也没有 BVH 原格式导出入口。

## 11. 提交文件前的检查清单

1. 明确选择一种内容结构，不只声明“这是 PKL/NPZ”。
2. 写出单位、轴方向、四元数顺序和关节/刚体索引表。
3. 验证 T、D/J、字段维度，确认没有 NaN/Infinity、零四元数或空 CSV 单元格。
4. 使用匹配的模型；缺少模型时先按[模型文档](model-formats.md)加载。
5. NPZ 数值字段不要用 object dtype；PKL 的 list/ndarray 选择遵守对应小节。
6. 导入后点“数据检查”，核对文件选择、默认帧率、补零、截断、关节映射提示。绿色只说明导入成功。

不支持直接输入 FBX、MJCF/XML 动作、通用 JSON 动作、原始 OMOMO `.p`、独立 NPY、视频或 ZIP 动作包。目录中存在这类资源不代表浏览器已实现对应解析器。

## 12. 实现与测试索引

- 入口与路由：[App.ts](../src/app/App.ts)。
- 容器读取：[NumpyIO.ts](../src/io/motion/NumpyIO.ts)、[PythonPickleIO.ts](../src/io/motion/PythonPickleIO.ts)。
- 机器人解析测试：`src/io/motion/{CsvMotionService,GmrMotionService,MimicKitMotionService,UfoTrainingPklMotionService,UfoReferenceMotionService,RobotStateNpzMotionService}.test.ts`。
- 骨架测试：[BvhMotionService.test.ts](../src/io/motion/BvhMotionService.test.ts)。
- 数据检查：[ImportDiagnostics.ts](../src/app/ImportDiagnostics.ts)、[输入集成测试](../src/app/App.importDiagnostics.test.ts)。
