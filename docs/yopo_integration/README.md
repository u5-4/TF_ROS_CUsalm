# YOPO 实机集成合同

本目录是 YOPO、定位适配、SO3、MAVROS 与 PX4 实机集成阶段的主合同。
合同基线标识为 `yopo_integration_20260723_v1`。

当前状态是**需求已确认、文档内部一致性检查已完成、等待提交冻结、实现尚未开始**。
本目录不构成飞行授权，也不表示任何目标接口已经通过实机验收。

## 文档导航

| 文档 | 责任 |
| --- | --- |
| [PROJECT_REQUIREMENTS.md](PROJECT_REQUIREMENTS.md) | 最终目标、系统范围、功能和非功能需求 |
| [INTERFACE_CONTRACTS.md](INTERFACE_CONTRACTS.md) | frame、变换、topic、语义和发布权 |
| [SAFETY_CONSTRAINTS.md](SAFETY_CONSTRAINTS.md) | 安全边界、飞行限制、故障处理和禁止事项 |
| [ACCEPTANCE_CRITERIA.md](ACCEPTANCE_CRITERIA.md) | 各阶段通过条件和必须保留的证据 |
| [TASKFLOW.md](TASKFLOW.md) | YOPO 阶段唯一任务状态来源 |
| [DECISION_LOG.md](DECISION_LOG.md) | 需求访谈中确认的架构决策与理由 |

## 权威顺序

发生冲突时按以下顺序处理：

1. `SAFETY_CONSTRAINTS.md` 的禁止项和门禁；
2. `INTERFACE_CONTRACTS.md` 的运行语义；
3. `PROJECT_REQUIREMENTS.md` 的范围和目标；
4. `ACCEPTANCE_CRITERIA.md` 的验收方法；
5. `TASKFLOW.md` 的当前执行状态。

`DECISION_LOG.md` 记录为什么这样设计，但不能覆盖现行安全和接口合同。

## 变更规则

- `TASKFLOW.md` 是唯一任务状态来源，其他文档不得维护平行待办列表。
- 坐标、外参、主定位模式、飞行限制或控制权限的变更必须新增决策记录。
- 任何放宽安全门禁的变更必须单独评审，不能夹带在普通实现提交中。
- rosbag、模型权重和其他大型二进制证据不提交 Git；Git 只保存版本、摘要和 SHA-256。
- 目标接口和已验收事实必须分开陈述，禁止用设计文档代替运行证据。

## 首个执行动作

在当前定位证据分支之上创建 `feat/yopo-integration-v1`，然后按照
`TASKFLOW.md` 从文档复核、深度流验证和 MAVROS 外部视觉接口审计开始推进。
