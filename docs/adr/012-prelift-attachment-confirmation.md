# ADR 012: 抬升前确认附着，分离夹持与抬升诊断

- 状态：已决定
- 日期：2026-10-03
- 范围：bridge 附着确认、task executor 的 CLOSE/LIFT 阶段门
- 关联：[ADR 010](010-grasp-conditioned-object-state.md)、[ADR 011](011-vision-object-pose-contract-tightening.md)

## 背景

简化状态机后，CLOSE 必须等 bridge 的 ATTACHED 才进入 LIFT，但 bridge 使用 `classifyGrasp()` 的成功结果确认附着。该分类要求物体中心 z >0.26 m；物体在桌面被双指夹住时 z≈0.24 m，因此永远无法启动抬升。oracle 不消费视觉拒绝，仍受这条共享循环依赖影响，最终超时重试。

## 决策

新增纯函数 `confirmsAttachment()`，仅检查实际宽度与盒宽的差小于容差、双指接触物体、物体到 TCP 的 XY 距离在半径内；宽度/距离有限且距离非负。不要求物体高度，不依赖视觉位姿。bridge 使用该函数确认并锁存 ATTACHED，保留实际张爪释放与 reset 清零行为。

`classifyGrasp()` 保留原高度判据及诊断语义，不驱动附着。executor 中仅用于 CLOSE 超时原因分类；桌面夹持时的 SLIP 诊断不等于已经掉落。LIFT 的成功退出同时要求 ATTACHED、机械臂位置/速度到达抬升目标和最小 settle 时间。

## 权衡与替代方案

独立确认避免把“已夹住”与“已抬起”混为一谈，并保留旧分类器的诊断含义。直接全局删除分类器高度条件会改变已有诊断；将所有 SLIP 视作附着也不正确，因为 TCP 距离失败同样可得到 SLIP。

该方案仍依赖仿真真值 XY 和接触，并且确认后的锁存可能掩盖闭爪滑落；机械臂到位也不能独立证明物体一直随臂运输。真机附着检测、持续滑落观测及完整视觉放置验收需另外设计验证。

## 验证

修复前真实 MuJoCo oracle episode 在 CLOSE 超时：宽度 0.0387 m、物体 z=0.2400 m、TCP 平面距离 0.0001 m、双指接触均为真，却没有 ATTACHED。

新增单元测试覆盖桌面夹持确认、单指/缺接触、错误宽度、距离及非有限输入；FSM 覆盖已附着但未到抬升目标时继续等待与超时。真实运行探针 `src/task_executor/test/oracle_attachment_probe.py` 持续注入匹配生命周期/序号的 REJECTED/CANDIDATE_INVALID，检查 oracle 成功、零重试、桌面确认后真实抬升、释放和 reset。最终实测结果记录在 [Week 4 Stage 5](../../Job_guides/my_study/week4.md#581-排查记录oracle-停在-close)，不将此 oracle 实验等同于视觉抓放验收。
