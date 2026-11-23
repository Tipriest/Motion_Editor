# 模型与配套资源格式标准

核对日期：2026-11-23。本文以当前代码为准，定义可加载的模型、资源组织及机器人固定索引表。

相关文档：[动作输入格式](input-motion-formats.md) · [动作输出格式](output-motion-formats.md)。

## 1. 模型能力总表

| 模型/资源 | 入口扩展名 | 主要内容 | 用途 |
| --- | --- | --- | --- |
| 机器人 | `.urdf` | URDF XML，link/joint/几何引用 | CSV、GMR、MimicKit、UFO、Robot-State 动作 |
| URDF 引用网格 | `.stl`, `.obj`, `.dae` | 网格几何 | URDF 的 visual/collision；不是独立骨架动作模型入口 |
| SMPL/SMPL-H/SMPL-X | `.npz` | 六项必需 NumPy 模型数组 | 人体模型预览和 SMPL 动作播放 |
| 旧版 smpl_webuser | `basicmodel_*_lbs_*.pkl` | 旧版模型字典，部分 Chumpy/稀疏数据兼容 | 同上 |
| 独立物体/交互物体 | `.obj` | 有面几何的 OBJ | 单独预览或 SMPL 物体轨迹配套 |
| BVH 骨架 | `.bvh` | 自带层级和动作 | 独立骨架预览，不配 URDF/SMPL |
| 预设清单 | `public/presets/presets.json` | 资源路径与输入类型配置 | 启动目录与预设加载；不是用户动作文件 |

没有模型导出入口，也没有 MJCF/XML、USD、FBX、GLTF/GLB、通用人体 PKL 或独立 STL/DAE 的顶层模型导入入口。不能因为资源目录包含某扩展名，就视为已实现解析。

## 2. URDF 机器人

依据：[UrdfLoadService.ts](../src/io/urdf/UrdfLoadService.ts)、[pathResolver.ts](../src/io/urdf/pathResolver.ts)。

### 2.1 XML 与动作 schema

使用有效 URDF XML：`<robot name="...">`，有效的 link/joint 名称，明确的 parent/child、origin、axis 与几何。推荐使用米、弧度，机器人运动约定为 X 前、Y 左、Z 上。

动作 schema 并非“URDF 所有关节”：

1. 遍历加载后的 `robot.joints`。
2. 排除 `fixed`。
3. 排除 `floating`，第一个 floating 关节用作根关节名。
4. 其余关节名按枚举顺序收集，每项对应一个动作标量。
5. 未找到 floating 时根名回退 `floating_base_joint`，根运动可能通过 robot transform fallback 应用。

因此，UI 显示的模型 joint 总数可能包含固定关节，**不等于动作 D**。普通标量关节使用 revolute/continuous（弧度）、prismatic（米）；不能认为每个非 fixed 类型的任意多自由度关节都可用一个标量正确驱动。

CSV 定序模式和 MimicKit 必须按当前 schema 顺序生成。GMR 推荐提供对应名称。UFO/Robot-State 的固定顺序见本文第 6 节。根姿态属于单独 7 个数值，不计入 D。

没有 floating root 不一定失败；缺少任何可应用根变换的目标时会提示根运动被忽略。导入成功仍需确认模型和动作语义一致。

### 2.2 网格与路径

推荐拖入整个资源文件夹，保留相对目录：

```text
robot_package/
  robot.urdf
  meshes/
    pelvis.STL
    thigh.obj
    foot.dae
```

URDF 里使用例如 `meshes/pelvis.STL` 或可解析的 `package://robot_package/meshes/pelvis.STL`。

- 网格仅实现 STL、OBJ、DAE，扩展名比较不区分大小写；DAE 场景内的光源会移除。
- 同时解析 visual 与 collision；内置 URDF box/cylinder/sphere 等基础几何交给 URDFLoader。
- 路径优先相对 URDF 目录或完整文件 map 查找，再尝试同名文件回退；文件 map 查找有大小写兼容。
- 同名 basename 出现在多个目录时回退可能歧义，应保留准确相对路径。
- 不要依赖本机绝对路径、ROS 安装路径或操作系统 package 搜索。浏览器只能访问实际拖入或服务器可取到的资源。
- 缺网格、网格加载失败或不支持的网格格式通常产生 warning 并放入空 Group；**有骨架不等于模型几何完整**。
- 独立 OBJ 没有 MTLLoader 集成；不要把材质/纹理完整复现当成此协议保证。URDF 的 OBJ 分支同样只调用 OBJLoader；DAE 相关纹理仍需路径可达。

多个 URDF 通常按路径深度、再按名称自动选一个。想选择特定版本时，缩小拖放范围或从 Models 中选择。

### 2.3 最小结构示例

以下是只有一个活动关节 `knee` 的演示模型，适合[输入文档](input-motion-formats.md)中的单关节示例；它不是训练用真实机器人：

```xml
<?xml version="1.0"?>
<robot name="single_joint_demo">
  <link name="pelvis">
    <visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual>
  </link>
  <link name="leg">
    <visual>
      <origin xyz="0 0 -0.2"/>
      <geometry><box size="0.1 0.1 0.4"/></geometry>
    </visual>
  </link>
  <joint name="knee" type="revolute">
    <parent link="pelvis"/>
    <child link="leg"/>
    <origin xyz="0 0 -0.1"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2" upper="2" effort="10" velocity="5"/>
  </joint>
</robot>
```

<a id="smpl-models"></a>
## 3. SMPL / SMPL-H / SMPL-X 模型

依据：[SmplMotionService.ts](../src/io/motion/SmplMotionService.ts)、[PythonPickleIO.ts](../src/io/motion/PythonPickleIO.ts)。`V` 为顶点数，`J` 为模型关节数，`K` 为体型基数，`F` 为三角形数，均应为正整数。

### 3.1 NPZ 必需字段

| 字段名 | ZIP 内部条目 | 推荐类型/形状 | 含义 |
| --- | --- | --- | --- |
| `v_template` | `v_template.npy` | float32/64 `[V,3]` | 模板顶点 |
| `shapedirs` | `shapedirs.npy` | float32/64 `[V,3,K]` | 体型线性基；不能直接写成 `[3V,K]` |
| `weights` | `weights.npy` | float32/64 `[V,J]` | 关节蒙皮权重 |
| `kintree_table` | `kintree_table.npy` | 整数 `[2,J]` | 第一行 parent，第二行 child ID |
| `J_regressor` | `J_regressor.npy` | **稠密** float32/64 `[J,V]` | 由顶点回归关节 |
| `f` | `f.npy` | 非负整数 `[F,3]` | **0 开始**的三角形顶点索引 |

六项必须同时存在，且大小写准确。NPZ 的 object/稀疏矩阵包装不受支持；不要把 SciPy sparse 对象直接作为 NPZ 字段值。

当前检查顶点、权重、回归器维度一致，K>0、J>0、F>0；kintree_table 第二维实际容许 `>=J`，但生成新文件应严格为 `[2,J]`。parent/child 应使用与关节顺序一致的 0..J-1 索引，根 parent 推荐为 -1，根关节索引为 0。

不要依赖整数转换对错误数据“修复”：读取器对浮点索引截断，对 f 的负值会夹到 0。应预先验证所有 face 索引 `<V`，层级无环、权重有限且非负、每顶点权重总和约为 1。

### 3.2 旧版模型 PKL

自动识别要求 basename：以 `basicmodel_` 开头、包含 `_lbs_`、以 `.pkl` 结束，比较大小写不敏感。例如 `basicmodel_m_lbs_10_207_0_v1.0.0.pkl`。

仅把任意模型改名为该形式不够；根对象必须包含与上表对应的六个字段：

- `v_template`, `weights`, `kintree_table`, `f`：可解码的 NumPy ndarray。
- `shapedirs`：NumPy ndarray，或兼容 Chumpy 对象中的 `.x` ndarray。
- `J_regressor`：稠密 ndarray，或带 `_shape/data/indices/indptr` 的兼容稀疏布局；当前稀疏解码按 **CSC 列压缩**解释，不保证任意 CSR 布局正确。

PKL ndarray 的 dtype/序列化限制见[输入容器规则](input-motion-formats.md)。泛化 SMPL-X `.pkl`（例如任意名字的 `SMPLX_*.pkl`）不是本入口支持的模型包，优先使用规范的 NPZ。

### 3.3 人体模型与动作匹配

- 从 weights 得出模型 J，从动作 `poses.shape[1] // 3` 得出动作 J；匹配按前缀索引，不自动做语义重定向。
- `betas` 从动作文件提供；按模型 K 取前若干值，缺失/不足的系数用 0，多余忽略；模型单独预览使用零 betas。
- 模型名称/路径用于推断 smpl/smplh/smplx 及 male/female/neutral。保留明确名称，避免“unknown gender”及错误匹配。
- 明确的 male/female 模型与动作性别冲突会拒绝；多个模型可能按性别自动选择，neutral 可作兼容候选。
- 当前人体渲染使用体型基、关节回归、骨架和蒙皮；每顶点选最大 4 项蒙皮权重并归一化。
- `posedirs`、表情、手部 PCA 等扩展不是必需字段，也没有完整官方 SMPL pose-corrective/表情求值保证。应提供完整轴角姿态，不要假定所有官方模型扩展都参与渲染。
- 模型单独预览使用 +Y 上；加载 SMPL 动作时场景改用 +Z 上。程序没有通用的源坐标/长度单位自动识别，应使用与动作一致的模型版本和原始坐标。

NPZ 的 NPY 版本、数值 dtype、标量字符串与压缩兼容规则同[输入文档](input-motion-formats.md)。不应通过随机生成或补零六个字段来伪造可用人体模型，应从真实模型数组转换。

<a id="obj-models"></a>
## 4. OBJ 物体与 SMPL 配套

依据：[ObjLoadService.ts](../src/io/object/ObjLoadService.ts)。

- 使用 OBJ 文本，包含可生成 Mesh 的顶点与面几何，边界盒非空；只有点或线不满足当前物体加载要求。
- 推荐单位米、与动作中的物体坐标、原点及尺度一致；程序不从 OBJ 自动推断单位。
- 独立预览默认对原始几何执行：减去包围盒中心 X/Z、减去最低 Y；这是加载器的局部坐标归一化。外层场景另有轴变换，不能把独立预览摆放当成原始运动坐标。
- SMPL 配对物体使用 `normalizeToGround:false`，保留几何原点以匹配 obj_trans/obj_rot_mat；不要预先用独立预览的归一化位移修改原文件。
- 模型统一替换为应用的物体材质并重新计算法线；不加载 MTL/纹理，不保留源材质作为输出保证。
- 多个 OBJ 通常自动选一个；SMPL 的 `obj_name`、`seq_name` 或文件名可用于匹配目录中的物体，必要时在 Objects 中明确选择。

最小几何示例：

```obj
o triangle
v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
```

有物体几何不等于有物体动画；动画字段应在 SMPL 动作 NPZ 中提供。当前只播放一个主物体，不播放 obj_bottom_* 的第二套轨迹。

## 5. BVH 骨架与预设清单

### BVH

BVH 自带 ROOT/JOINT 层级、OFFSET、CHANNELS 与动作帧，不需要独立 URDF。骨架与角度/长度单位见[动作输入文档](input-motion-formats.md)。不能把 BVH 骨架当成 SMPL 蒙皮模型或机器人驱动模型。

### 预设 JSON

依据：`App.ts` 的 preset parser；真实清单：[presets.json](../public/presets/presets.json)。

```json
{
  "capturedObjects": ["presets/objects/table.obj"],
  "presets": [
    {
      "id": "robot-walk",
      "label": "Robot + Walk",
      "model": {"urdfPath": "assets/robots/example/robot.urdf"},
      "motion": {"kind": "gmr", "path": "assets/motions/walk.pkl"}
    }
  ]
}
```

以上为结构示例，路径需替换为实际部署资源。

- 根 `presets` 必须为数组；每项 `id/label` 为非空字符串，可选 description；至少有 model 或 motion。
- `model`：提供 `urdfPath`，或非空 `files`；可选 `selectedUrdfPath`。
- `motion.kind` 目前只接受 `csv | mimickit | gmr | bvh | smpl`；提供 `path` 或非空 `files`，可选 `selectedMotionPath`。主界面支持 UFO/Robot-State 不代表 manifest 可直接写同名 kind。
- `files` 项可以是路径字符串，或 `{"path":"服务器资源路径","mapAs":"导入文件map路径"}`。模型文件集合适用于 SMPL 等组合资产。
- `capturedObjects` 可省略；结构同 files 项。
- 路径相对于应用部署根/文档 URL 可获取，保持正斜杠；清单不会把开发机任意磁盘文件自动暴露给浏览器。
- 不提供任意用户 JSON 拖放模型入口；这是项目预设配置文件。

<a id="joint-layouts"></a>
## 6. Tiangong3 / DEX EVT 固定索引表

下列列表都按 **0 开始索引**。机器学习张量应严格按列表顺序写列；关节顺序不能依据机器人外观、文件名或“都是 29 DOF”推断。

这些表定义本应用支持的协议约定，不证明任何未携带名称、来源不明的数组实际符合约定。尤其 UFO NPZ 当前根据 body 数量选择固定布局，再给关节列赋名；这一步没有从数值中识别真实关节，也没有与源文件名称元数据核验。外部数据应先由生成代码或索引映射确认来源顺序。

来源：[RobotTrackingLayouts.ts](../src/io/motion/RobotTrackingLayouts.ts)、[UfoReferenceNpzService.ts](../src/io/motion/UfoReferenceNpzService.ts)。

### 6.1 Tiangong3 29 关节与 31 关节

用于 Robot-State/GMR NPZ、30-body UFO Reference、UFO Training，以及标准 Tiangong3 GMR 适配。

```python
TIANGONG3_JOINT_NAMES_29 = [
    "hip_pitch_l_joint", "hip_roll_l_joint", "hip_yaw_l_joint",
    "knee_pitch_l_joint", "ankle_pitch_l_joint", "ankle_roll_l_joint",
    "hip_pitch_r_joint", "hip_roll_r_joint", "hip_yaw_r_joint",
    "knee_pitch_r_joint", "ankle_pitch_r_joint", "ankle_roll_r_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "shoulder_pitch_l_joint", "shoulder_roll_l_joint", "shoulder_yaw_l_joint",
    "elbow_pitch_l_joint", "elbow_yaw_l_joint", "wrist_pitch_l_joint", "wrist_roll_l_joint",
    "shoulder_pitch_r_joint", "shoulder_roll_r_joint", "shoulder_yaw_r_joint",
    "elbow_pitch_r_joint", "elbow_yaw_r_joint", "wrist_pitch_r_joint", "wrist_roll_r_joint",
]
TIANGONG3_JOINT_NAMES_31 = (
    TIANGONG3_JOINT_NAMES_29[:15]
    + ["head_yaw_joint", "head_pitch_joint"]
    + TIANGONG3_JOINT_NAMES_29[15:]
)
```

31 项布局的头部位于 **索引 15、16**，不是数组尾部。GMR 的标准 29→31 自动适配在此处插入两个 0；其他依赖固定 29 的输出不携带这两个值。

### 6.2 DEX EVT 29 关节

用于 39-body UFO Reference 输入及 DEX_MOSAIC 输出：

```python
DEX_EVT_JOINT_NAMES_29 = [
    "hip_pitch_l_joint", "hip_pitch_r_joint", "waist_yaw_joint",
    "hip_roll_l_joint", "hip_roll_r_joint", "waist_roll_joint",
    "hip_yaw_l_joint", "hip_yaw_r_joint", "waist_pitch_joint",
    "knee_pitch_l_joint", "knee_pitch_r_joint",
    "shoulder_pitch_l_joint", "shoulder_pitch_r_joint",
    "ankle_pitch_l_joint", "ankle_pitch_r_joint",
    "shoulder_roll_l_joint", "shoulder_roll_r_joint",
    "ankle_roll_l_joint", "ankle_roll_r_joint",
    "shoulder_yaw_l_joint", "shoulder_yaw_r_joint",
    "elbow_pitch_l_joint", "elbow_pitch_r_joint",
    "elbow_yaw_l_joint", "elbow_yaw_r_joint",
    "wrist_pitch_l_joint", "wrist_pitch_r_joint",
    "wrist_roll_l_joint", "wrist_roll_r_joint",
]
```

### 6.3 Tiangong3 30 bodies

用于 30-body UFO Reference、UFO Training。索引 0 是 pelvis；随后每项对应 29 关节表的一项 body：

```python
TIANGONG3_BODY_NAMES_30 = [
    "pelvis",
    "hip_pitch_l_link", "hip_roll_l_link", "hip_yaw_l_link",
    "knee_pitch_l_link", "ankle_pitch_l_link", "ankle_roll_l_link",
    "hip_pitch_r_link", "hip_roll_r_link", "hip_yaw_r_link",
    "knee_pitch_r_link", "ankle_pitch_r_link", "ankle_roll_r_link",
    "waist_yaw_link", "waist_roll_link", "waist_pitch_link",
    "shoulder_pitch_l_link", "shoulder_roll_l_link", "shoulder_yaw_l_link",
    "elbow_pitch_l_link", "elbow_yaw_l_link", "wrist_pitch_l_link", "wrist_roll_l_link",
    "shoulder_pitch_r_link", "shoulder_roll_r_link", "shoulder_yaw_r_link",
    "elbow_pitch_r_link", "elbow_yaw_r_link", "wrist_pitch_r_link", "wrist_roll_r_link",
]
```

### 6.4 DEX EVT 39 bodies

包含头部、IMU、相机及 TCP 等固定 link；固定 link 需要 body 轨迹，但不因此增加 joint_pos 的 29 列：

```python
DEX_EVT_BODY_NAMES_39 = [
    "pelvis",
    "hip_pitch_l_link", "hip_pitch_r_link", "waist_yaw_link", "imu_waist_link",
    "hip_roll_l_link", "hip_roll_r_link", "waist_roll_link",
    "hip_yaw_l_link", "hip_yaw_r_link", "waist_pitch_link",
    "knee_pitch_l_link", "knee_pitch_r_link",
    "head_yaw_link", "radar_head_link", "imu_head_link",
    "shoulder_pitch_l_link", "shoulder_pitch_r_link", "camera_body_front_link",
    "ankle_pitch_l_link", "ankle_pitch_r_link", "head_pitch_link",
    "shoulder_roll_l_link", "shoulder_roll_r_link",
    "ankle_roll_l_link", "ankle_roll_r_link", "camera_head_link",
    "shoulder_yaw_l_link", "shoulder_yaw_r_link",
    "elbow_pitch_l_link", "elbow_pitch_r_link", "elbow_yaw_l_link", "elbow_yaw_r_link",
    "wrist_pitch_l_link", "wrist_pitch_r_link", "wrist_roll_l_link", "wrist_roll_r_link",
    "left_tcp_link", "right_tcp_link",
]
```

使用这些表转换顺序时应按名称建立索引映射，不能只 reshape：

```python
import numpy as np

def reorder_columns(values, source_names, target_names):
    if len(set(source_names)) != len(source_names):
        raise ValueError("source joint names must be unique")
    values = np.asarray(values)
    if values.ndim != 2 or values.shape[1] != len(source_names):
        raise ValueError("joint array and source names disagree")
    index = {name: i for i, name in enumerate(source_names)}
    return values[:, [index[name] for name in target_names]]
```

此函数遇到缺失目标关节直接失败，不自动补齐；31→29 可按名称提取，29→31 则需要明确的头部补值策略。

## 7. 资源与排查

此工作区未提供仓库说明中提到的 `ref/`、`models/`、`motions/` 数据根；实际可核对的资源示例有：

- `assets/robots/tiangong3/tiangong3.urdf`。
- `assets/robots/EVT2/urdf/tiangong2dex_31dof.urdf`。
- `assets/motions/left_punch.pkl`。
- `public/presets/LAFAN1_Retargeting_Dataset/robot_description/` 下的 G1/H1/H1-2 URDF 与网格。
- `public/presets/SMPL-X/SMPLX_MALE.npz`。
- `public/presets/AMASS/ACCAD/SMPL-X G/Male2MartialArtsExtended_c3d/Extended_3_stageii.npz`。
- `public/presets/LAFAN1/bvh/dance1_subject1.bvh`。

这些路径用于说明当前资产组织，不是格式定义的一部分；换用自己的文件时以字段、索引表和模型语义为准。

| 现象 | 应检查 |
| --- | --- |
| 模型有骨架但部分外观消失 | mesh 路径、资源是否全部导入、网格扩展名、Missing resource 警告 |
| 模型能加载，动作明显扭曲 | 关节顺序、左右命名、旋转单位、根四元数顺序 |
| 29/31 列不匹配 | 是否为标准 Tiangong3 布局；头部插入位置是否正确；格式是否支持该适配 |
| 人体形状/手部不正确 | 模型 family、性别、betas、完整姿态轴角和关节索引是否匹配 |
| 物体悬浮或相对人体偏移 | OBJ 原点/尺度、obj_trans、trans2joint、是否误用了独立预览归一化 |
| UFO/DEX 导出缺 body 报错 | robot.links 是否含目标固定 body 表中的所有 link，模型版本是否一致 |

新数据请先用[动作输入检查清单](input-motion-formats.md)核验，再通过界面的“数据检查”查看解析器提示。
