# VINS + LightGlue VIO Engineering Project

Tags: #engineering/vio #domain/drone #domain/slam

## 目标

这个工程用于把 **LightGlue learned feature matching** 接入 **VINS-Fusion / VINS-Mono** 风格的视觉惯性里程计流程，并在公开 visual-inertial 数据集上做可复现实验。

核心问题：

- learned matcher 是否能提升 VINS 在弱纹理、低光、运动模糊、重复纹理、动态干扰下的跟踪稳定性？
- LightGlue 前端带来的计算开销是否适合无人机 onboard 或近实时离线处理？
- 与原始 VINS KLT/feature_tracker 相比，ATE/RPE、tracking failure、特征 track length、IMU preintegration reset 次数是否改善？

## 当前工程形态

这是一个面向 VINS-Fusion 的实验工程。早期路线包含离线前端评测和外部 ROS1 learned frontend；完整 EuRoC 结果显示，直接替换 VINS frontend 会破坏原始 track/id/camodocal 语义，精度弱于 baseline。

当前主线已改为 **C++ 内部 recovery bridge**：

1. 原始 VINS C++ `FeatureTracker` 仍负责 KLT 高频跟踪、GFTT refill、camodocal/MEI normalization、stereo LK、`ids`、`track_cnt` 和 velocity。
2. LightGlue 只作为 C++ recovery candidate provider，向 `/feature_tracker/recovery_candidates` 发布少量候选匹配。
3. VINS C++ 端只接纳靠近 KLT lost track 的候选点，并复用原 track id；接纳后继续走原始 VINS normalization、velocity 和 stereo 路径。

## 目录

```text
engineering/vins-lightglue-vio/
├── README.md
├── pyproject.toml
├── .gitignore
├── configs/
│   ├── datasets/
│   │   ├── euroc.yaml
│   │   └── tum_vi.yaml
│   └── experiments/
│       └── euroc_mh01_lightglue.yaml
├── docs/
│   ├── datasets.md
│   ├── vins_fusion_integration.md
│   └── experiment_protocol.md
├── scripts/
│   ├── run_lightglue_matches.ps1
│   └── ros1/
│       └── lightglue_feature_tracker_node.py
├── src/
│   └── vins_lightglue/
│       ├── __init__.py
│       ├── cli.py
│       ├── dataset.py
│       ├── lightglue_frontend.py
│       └── vins_export.py
└── tests/
    └── test_dataset.py
```

## 依赖

建议在 Linux / WSL2 / Ubuntu ROS 环境运行 VINS 联调；Windows 下可以先做离线匹配评测。

Python 依赖：

```powershell
cd F:\research\research\engineering\vins-lightglue-vio
python -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install -U pip
pip install -e .[dev]
```

LightGlue 官方包通常需要从 GitHub 安装：

```powershell
pip install git+https://github.com/cvg/LightGlue.git
```

GPU 版本 PyTorch 需要按本机 CUDA 单独安装，见 PyTorch 官方安装页。

## 数据集

优先支持：

- **EuRoC MAV**：无人机/室内 VIO 标准数据集，适合作为 VINS baseline。
- **TUM-VI**：鱼眼相机 + IMU，包含更长序列和更复杂运动。

配置文件在：

- `configs/datasets/euroc.yaml`
- `configs/datasets/tum_vi.yaml`

## 最小离线测试

假设已有 EuRoC `MH_01_easy` 解压目录：

```powershell
python -m vins_lightglue.cli match `
  --dataset-config configs/datasets/euroc.yaml `
  --sequence-root D:\datasets\EuRoC\MH_01_easy\mav0 `
  --camera cam0 `
  --max-pairs 50 `
  --output runs\euroc_mh01_lightglue\matches.csv
```

输出：

- `matches.csv`：相邻帧匹配统计和可用于 debug 的 match counts。
- 后续可扩展为 `tracks.csv`，再通过 ROS node 或 VINS backend adapter 使用。

## 已完成的公开样本测试

已在公开 EuRoC-format 小样本 `sample_euroc_MH05` 上完成前端 smoke test：

- SuperPoint + LightGlue CPU：20 对相邻帧，平均 786.0 matches，平均 2355.8 ms/对。
- SuperPoint + LightGlue GPU k1024：平均 786.95 matches，去掉首对 warm-up 后约 56.1 ms/对，约 17.8 Hz。
- ORB + BF ratio matching：20 对相邻帧，平均 550.8 matches，CPU 平均 30.8 ms/对。

结论：LightGlue 在该样本上给出更多匹配；GPU 后速度从不可用的 CPU 离线速度提升到接近 VIO 前端可测范围，但仍慢于 ORB。实时 VINS 需要 GPU、降频、关键帧触发或更深的 tracker/cache 优化。

详见 [docs/results_public_sample_euroc_mh05.md](docs/results_public_sample_euroc_mh05.md)。

## WSL20 EuRoC MH_01 VINS 对比

已在 WSL Ubuntu 20.04 / ROS Noetic 上完成同一个 `MH_01_easy.bag` 的两条链路测试：

- 原始 VINS-Fusion frontend baseline 完整跑通，Leica Sim(3) aligned RMSE 为 `0.193380 m`。
- LightGlue external frontend 已能接入 `/feature_tracker/feature` 并让 VINS 初始化。
- 已通过 Python 3.9 + `torch==2.8.0+cu128` 解决 RTX 5060 / `sm_120` 支持问题，并完成完整 `MH_01_easy.bag` GPU 重跑。
- LightGlue GPU k256 完整 RMSE 为 `0.829410 m`；k512 完整 RMSE 为 `0.612067 m`。当前仍弱于原始 VINS frontend，因此直接替换 `/feature_tracker/feature` 不再作为主线。
- 已新增 KLT + LightGlue recovery hybrid 外部前端。30 秒短测 RMSE `0.051580 m`，但完整 bag RMSE `0.763128 m`，说明外部 Python KLT 并不等价于原始 VINS C++/camodocal frontend。
- 已在原始 VINS C++ `FeatureTracker` 内部加入 LightGlue recovery candidate bridge。无候选输入时 30 秒 smoke test RMSE `0.050014 m`，验证启用 bridge 不会破坏原始 VINS frontend。
- 已新增 C++ `lightglue_recovery_candidate_node`。`fake_grid` 后端用于验证 candidate topic 和 C++ 接纳路径：30 秒 RMSE `0.050240 m`，完整 `MH_01_easy` RMSE `0.204532 m`；这不是 LightGlue 算法结果，只是 bridge 压测。
- 已接入 ONNX Runtime C++ + CUDA12 pipeline 模型。固定频率 real-time `MH_01_easy` 发布 6640 个 ONNX candidates 但接纳 0 条，轨迹 RMSE `0.193381 m`，等同原始 baseline。
- 已把 ONNX recovery 改为 `FeatureTracker` health-triggered request，并按 lost-track proximity 对 LightGlue 候选重排。完整 real-time `MH_01_easy`：accepted total `995`，RMSE `0.183657 m`，比原始 VINS baseline `0.193380 m` 略好；当前只能说明调度/候选选择方向有效，还不能声称 LightGlue 在所有序列上稳定提升。

详见 [docs/results_vins_lightglue_euroc_mh01_wsl20.md](docs/results_vins_lightglue_euroc_mh01_wsl20.md)。
Hybrid 结果详见 [docs/results_hybrid_klt_lightglue_euroc_mh01_wsl20.md](docs/results_hybrid_klt_lightglue_euroc_mh01_wsl20.md)。
C++ 内部桥接详见 [docs/cpp_featuretracker_lightglue_recovery_bridge.md](docs/cpp_featuretracker_lightglue_recovery_bridge.md)。
C++ candidate node 与 fake-grid 压测详见 [docs/results_cpp_recovery_bridge_euroc_mh01_wsl20.md](docs/results_cpp_recovery_bridge_euroc_mh01_wsl20.md)。

## Degradation-Aware Recovery

针对高速运动、运动模糊和光强突变，本工程不把 LightGlue 作为全量 frontend 替代，而是扩展 `FeatureTracker` health trigger：

- 高速：用成功 KLT tracks 的平均像素位移 `mean_flow` 触发 recovery request。
- 模糊：用 Laplacian variance `blur_score` 识别低纹理/运动模糊退化。
- 光强突变：用相邻帧灰度均值差 `brightness_delta` 识别曝光变化。

新增参数默认关闭，避免改变已记录实验：

```bash
RECOVERY_REQUEST_MAX_MEAN_FLOW=35
RECOVERY_REQUEST_MIN_BLUR_SCORE=20
RECOVERY_REQUEST_BRIGHTNESS_DELTA=30
```

详细计划见 [docs/degradation_aware_recovery_plan.md](docs/degradation_aware_recovery_plan.md)，sweep 脚本见 [scripts/wsl20/run_degradation_recovery_sweep.sh](scripts/wsl20/run_degradation_recovery_sweep.sh)。

首轮 60 秒合成退化测试见 [docs/results_synthetic_degradation_mh01.md](docs/results_synthetic_degradation_mh01.md)。当前结论：退化触发有效，frame-skip 高速 proxy 略有改善；blur/exposure 下接纳过多导致 RMSE/P95 略变差，下一步需要几何 gate 和更保守参数。

## VINS-Fusion 联调路线

推荐保留 VINS-Fusion 后端和原始 C++ frontend 主路径不动：

1. 启动 VINS-Fusion estimator，保持 `_external_feature_only` 为默认 `false`。
2. 启用 `_enable_lightglue_recovery_bridge:=true`。
3. 运行 C++ `lightglue_recovery_candidate_node`，订阅左目图像并发布 `/feature_tracker/recovery_candidates`。
4. VINS C++ `FeatureTracker` 只从候选中接纳少量 lost-track recovery，不让 LightGlue 替代 KLT 高频 tracking。

详细见 [docs/vins_fusion_integration.md](docs/vins_fusion_integration.md)。

## 评测指标

- VIO trajectory: ATE、RPE、scale drift。
- Frontend: matched keypoints、inlier ratio、track length、lost track count。
- Robustness: tracking failure、relocalization/loop closure success、低光/模糊/快速旋转片段的退化曲线。
- Runtime: feature extraction time、matching time、VINS backend time、end-to-end Hz。

## 近期 TODO

- [ ] 在 WSL Ubuntu 22.04 跑完整 EuRoC 前端统计，见 [docs/wsl_ubuntu22_full_euroc_plan.md](docs/wsl_ubuntu22_full_euroc_plan.md)。
- [x] 在 Linux/ROS1 环境 clone VINS-Fusion 并跑通 EuRoC 原始 baseline；优先 Ubuntu 20.04 WSL，见 [docs/wsl20_vins_fusion_noetic_plan.md](docs/wsl20_vins_fusion_noetic_plan.md)。
- [x] 完成 ROS node 与 VINS estimator topic 对齐。
- [x] 更换支持 RTX 5060 / `sm_120` 的 PyTorch，重跑完整 EuRoC LightGlue frontend。
- [x] 完成外部 KLT + LightGlue recovery hybrid prototype 并跑完整 EuRoC。
- [x] 在原始 C++ `FeatureTracker` 内部接入 LightGlue recovery candidate bridge，而不是用外部 Python KLT 替代。
- [x] 实现 C++ `lightglue_recovery_candidate_node` 框架和 `fake_grid` dry-run backend。
- [x] 对 C++ bridge 做 fake-candidate dry run，确认日志出现 `LightGlue recovery bridge accepted X tracks`。
- [x] 跑完整 EuRoC MH_01 C++ bridge fake-grid 压测，并与原始 VINS baseline `0.193380 m` 对比。
- [x] 安装或 vendor ONNX Runtime C++，准备 SuperPoint/LightGlue ONNX 模型，接入真实 LightGlue ONNX backend。
- [x] 把 ONNX recovery 从固定频率发布改为 `FeatureTracker` health-triggered request，并按 lost-track proximity 重排候选。
- [ ] 在更难 EuRoC 序列上重跑 request-driven ONNX recovery，并加入几何一致性过滤/更保守接纳策略。
- [x] 为高速运动、运动模糊和光强突变加入退化感知 request trigger 与实验计划。
- [x] 基于 `MH_01_easy` 生成 60 秒 motion blur / exposure jump / frame-skip 合成退化 bag 并完成首轮 baseline 对比。
- [ ] 基于 `run_degradation_recovery_sweep.sh` 跑 MH_04/MH_05/V1_03/V2_03 并调参。
- [ ] 为 blur/exposure recovery 加入几何一致性 gate，降低错误 recovery 接纳。
- [ ] 加入 ORB/KLT baseline 的相同统计导出。
- [ ] 加入 TUM-VI fisheye undistortion/camera model 处理。
