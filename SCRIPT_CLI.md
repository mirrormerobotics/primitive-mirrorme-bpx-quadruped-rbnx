# 用户脚本命令行参数

新机器无真机开发见同级 Robot 仓库的 `OFFLINE_DEVELOPMENT.md`。
其 `scripts/verify_offline.py` 统一执行两仓库测试与 Docker fake/replay 验收。
`odom_acceptance compare` 在缺少 SDK 或机器人版本记录时保留观测值，但将报告标为
`insufficient_evidence`，不会把双方都缺失版本误判为相同基线。

所有 `scripts/*.sh` 通过命令行参数配置，并支持 `--help`，
查看帮助不调用 Docker、不连接真机。Shell 包装层选项支持 `--name value` 和
`--name=value`；路径含空格时必须加引号。wheel／输出路径使用位置参数。

## 只读采集及手动运动探针

以下脚本统一支持 `--image IMAGE`、`--network-mode host|bridge`、`--package-root PATH`：

- `run-read-only-probe.sh WHEEL [探针参数]`
- `record-sdk-replay.sh WHEEL OUTPUT [录制参数]`
- `record-odom-stage.sh WHEEL OUTPUT_DIR STAGE [采集参数]`
- `run-posture-cycle-probe.sh WHEEL [动作参数]`
- `run-yaw-circle-probe.sh WHEEL [动作参数]`

默认镜像为 `primitive-mirrorme-bpx-quadruped-rbnx:dev`，默认网络为 `host`。
两个运动探针还支持 `--container-name NAME`，默认容器名分别为
`bpx_motion_level_posture_probe`／`bpx_motion_level_yaw_circle_probe`。
所选镜像同时用于 wheel 校验和实际探针。
指定 wheel 会强制替换镜像中已有的同名同版本 SDK，使实际运行制品与校验结果一致。

`--robot-ip`、`--robot-state-port`、`--duration-s` 等传给对应 Python 工具。
Python 完整选项可在仓库中运行对应模块的 `--help` 查看，例如：

```bash
python3 -m bpx_quadruped.odom_acceptance capture --help
python3 -m bpx_quadruped.posture_cycle_probe --help
```

Odom 采集示例（输出目录需存在，每个阶段禁止覆盖）：

```bash
mkdir -p /tmp/bpx-odom-origin-manual
bash scripts/record-odom-stage.sh \
  /absolute/path/to/bpx_sdk_open-1.0.8-cp310-cp310-manylinux_2_24_x86_64.manylinux_2_28_x86_64.whl \
  /tmp/bpx-odom-origin-manual reconnect_before \
  --network-mode bridge \
  --image primitive-mirrorme-bpx-quadruped-rbnx:dev \
  --robot-ip 192.168.1.237 --robot-state-port 9873 \
  --state-rate-hz 10 --duration-s 10 --sample-period-s 0.1 \
  --note 'P1/H1; stationary before reconnect'
```

运动探针分别要求 `--confirm-physical-motion`、`--confirm-physical-yaw-circle`，
缺少确认时在 Docker 启动之前拒绝。

### 本地 1.0.8 wheel 的只读测试

`verify-sdk-wheel.sh`、`run-read-only-probe.sh`、`record-sdk-replay.sh` 和
`record-odom-stage.sh` 支持 `--sdk-profile release|local-v1.0.8`，默认 `release`。
`local-v1.0.8` 仅接受 `sdk/SHA256SUMS.local-v1.0.8` 中固定的本地文件：

```text
/root/workspace/c2/bpx_sdk_open_out/v1.0.8/wheelhouse/bpx_sdk_open-1.0.8-cp310-cp310-manylinux_2_24_x86_64.manylinux_2_28_x86_64.whl
SHA256: 6a0bcb708c55605dfbfeead6c84141f474505f1c01efbdb564e3dfa4549889c7
```

该文件离线版本及 API 检查通过，但不是 `cab478a` 发布制品，包内缺少
`LICENSE`、`NOTICE`、`THIRD_PARTY_NOTICES.md`。它只供本地只读调试，不用于
发布 Deployment 或运动测试。默认发布校验不变，不提供跳过 checksum 的选项。
Odom JSONL 会保存实际 wheel SHA256，测试证据不得归入发布制品的验收结果。

## 构建及工具验证

| 脚本 | 可配置参数 |
| --- | --- |
| `build.sh`、`build-container.sh` | `--image`、`--test-image`、`--clean`、`--use-tuna-mirror 0\|1`、`--package-root` |
| `verify-sdk-wheel.sh WHEEL` | `--image`、`--package-root`、`--posture` 或 `--yaw-circle` |
| `verify-rbnx.sh [SOURCE]` | `--robonix-source`、`--expected-commit`、`--tool-image`、`--tool-cache`、`--target-cache`、`--cargo-cache`、`--config-cache`、`--package-root` |
| `test.sh` | `--package-root` |

`--clean` 同时传给 codegen 和 Docker 无缓存构建。`--use-tuna-mirror` 默认 `1`。
Robonix commit 默认固定为 `f6cacfc4b402eb57c75b553d375c6cfaba11dce7`；
改参数不代表其他版本已兼容。缓存默认位于本仓库 `.artifacts/rbnx-tool`，
子缓存的显式参数优先于 `--tool-cache` 基目录。
相对缓存路径按调用脚本时的工作目录解析；创建目录后转换为绝对路径再挂载，
已有符号链接解析到其目标目录。

## 生命周期入口

`start.sh` 和所有 `start-*.sh` 支持：

- `--sdk-wheel PATH`、`--image IMAGE`、`--container-name NAME`、`--instance-name NAME`；
- `--network-mode host|bridge`、`--robot-state-port PORT`、`--docker-host-address HOST`；
- `--atlas-address HOST:PORT`、`--provider-bind-host HOST`、`--advertise-host HOST`；
- `--ros-domain-id ID`、`--rmw-implementation NAME`、`--zenoh-router VALUE`、`--zenoh-mode VALUE`；
- `--stop-timeout-s SECONDS`（默认 30）、`--package-root PATH`。

`start-posture-test.sh` 额外要求 `--confirm-physical-posture-service`；SDK 类入口
要求 wheel。每种入口的 backend 和能力限制由包装脚本固定，不提供绕过参数。
`stop.sh --container-name NAME --stop-timeout-s 30` 停止指定容器，并校验归属标签。
启动脚本清理时使用启动时指定的容器名。

这些是供 Robonix 调用的底层入口；直接启动它们并不等于完整 Deployment。
手动真机部署使用 Robot 仓库的 `start-sdk-read-only-docker.sh`，由它配置
Atlas／Soma、机器人 IP 和 backend 等参数。不向上游 `rbnx` 命令传本仓库的选项。

Robonix 内部环境变量传值机制不变。
