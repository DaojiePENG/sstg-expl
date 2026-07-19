# SSTG-Explorer：RAL 论文公式、算法与写作参考

本文档给出与当前实现一致的数学表述。投稿时应压缩为主文需要的公式，不能保留代码中不存在的虚构目标项。

## 1. 符号与问题定义

令二维工作空间为 \(\Omega\subset\mathbb{R}^2\)，占据区域为 \(\mathcal O\)，栅格分辨率为 \(\rho\)。机器人半径为 \(r_r\)，安全裕量为 \(d_s\)。规划障碍和安全自由空间为

\[
\mathcal O^+ = \mathcal O \oplus \mathcal B(r_r+d_s),
\qquad
\mathcal F_s = \Omega\setminus\mathcal O^+,
\tag{1}
\]

其中 \(\mathcal B(r)\) 是半径为 \(r\) 的闭圆盘。时刻 \(t\) 已接受的观测节点序列为

\[
\mathcal V_t=\{\mathbf v_0,\ldots,\mathbf v_t\},
\qquad \mathbf v_i=(x_i,y_i)\in\mathcal F_s.
\tag{2}
\]

圆形视野近似为 \(\mathcal B(\mathbf v_i,r_v)\)。自由空间覆盖集合和覆盖率为

\[
\mathcal C(\mathcal V_t)
=\mathcal F\cap\bigcup_{\mathbf v_i\in\mathcal V_t}
\mathcal B(\mathbf v_i,r_v),
\qquad
R_C(t)=\frac{\mu(\mathcal C(\mathcal V_t))}{\mu(\mathcal F)},
\tag{3}
\]

其中 \(\mu(\cdot)\) 为面积测度。当前实现不建模障碍遮挡后的真实相机可见多边形，因此论文应把它称为 disk-coverage proxy。给定重叠参数 \(o\)，期望排斥间距为

\[
d_{\mathrm{rep}}=r_v-o.
\tag{4}
\]

目标是寻找有限序列 \(\mathcal V_T\)，使 \(R_C(T)\ge \tau_C\)，同时控制测地旅行距离、节点数和安全性：

\[
\min_{\mathcal V_T}
\left[L_G(\mathcal V_T),|\mathcal V_T|,-Q_{\mathrm{safe}},-Q_{\mathrm{uniform}}\right]
\quad\mathrm{s.t.}\quad
R_C(T)\ge\tau_C,\;\mathbf v_i\in\mathcal F_s.
\tag{5}
\]

这里是多目标评价，不应声称代码直接求解 Eq. (5) 的全局 Pareto optimum。

## 2. 角向候选生成

从当前接受节点 \(\mathbf v_t\) 以基础角分辨率 \(\Delta\theta\) 生成

\[
\theta_k=k\Delta\theta,
\qquad
\mathbf f_{t,k}=\mathbf v_t+r_v
\begin{bmatrix}\cos\theta_k\\ \sin\theta_k\end{bmatrix},
\quad k=0,\ldots,\left\lfloor\frac{2\pi}{\Delta\theta}\right\rfloor-1.
\tag{6}
\]

最终 SSTG-Explorer 使用固定 \(\Delta\theta=30^\circ\)。我们还评估了一个窄通道检测器变体，它根据局部距离场 \(D_{\mathcal O}(\mathbf x)\) 减小角间隔：

\[
\Delta\theta(\mathbf v_t)=
\begin{cases}
\Delta\theta_{\min}, & 2D_{\mathcal O}(\mathbf v_t)<w_{\mathrm{narrow}},\\
\Delta\theta, & \text{otherwise}.
\end{cases}
\tag{7}
\]

## 3. 碰撞类型与探索强度

令候选到最近已探索节点的距离为

\[
d_V(\mathbf f)=\min_{\mathbf v\in\mathcal V_t}\|\mathbf f-\mathbf v\|_2.
\tag{8}
\]

当前实现的候选探索强度为

\[
s(\mathbf f)=
\begin{cases}
0, & \mathbf f\notin\mathcal F_s,\\
\frac{1}{2}\frac{d_V(\mathbf f)}{r_v}, & d_V(\mathbf f)<d_{\mathrm{rep}},\\
\frac{d_V(\mathbf f)}{r_v}, & d_{\mathrm{rep}}\le d_V(\mathbf f)<r_v,\\
1, & d_V(\mathbf f)\ge r_v.
\end{cases}
\tag{9}
\]

相应状态为 hard obstacle、soft/overlap candidate 或 free candidate。若 \(s(\mathbf f)<s_{\min}\)，候选被 strength pruning 拒绝。所有状态都写入逐步 trace。

## 4. 测地优先级

在按 (r_r+d_s) 膨胀后的统一安全栅格上定义测地距离

\[
d_G(\mathbf x,\mathbf y)=
\min_{\pi:\mathbf x\leadsto\mathbf y}\int_\pi 1_{\mathcal F_s}(\mathbf p)\,\mathrm d\ell,
\tag{10}
\]

不可达时 \(d_G=+\infty\)。实现对每个接受节点计算一次 one-to-all cost map，避免把隔墙但欧氏距离很近的候选当作廉价目标。

定义候选的归一化障碍净空

\[
q_S(\mathbf f)=\min\!\left(\frac{D_{\mathcal O}(\mathbf f)}{r_v},1\right).
\]

最终优先级显式联合新颖性、测地旅行代价与安全视点偏好：

\[
P_t(\mathbf f)=s(\mathbf f)
\exp\!\left(-\beta\frac{d_G(\mathbf v_t,\mathbf f)}{r_v}\right)
\left[1+w_Sq_S(\mathbf f)\right]b(\rho_O),
\tag{11}
\]

\[
b(\rho_O)=
\begin{cases}
1,&\rho_O\le\rho_0,\\
1+\min\!\left(\frac{\rho_O-\rho_0}{0.1},2\right),&\rho_O>\rho_0,
\end{cases}
\qquad \rho_O=\frac{|\mathcal O|}{|\Omega|}.
\tag{12}
\]

全局队列每次选择

\[
\mathbf f_t^*=\arg\max_{\mathbf f\in\mathcal Q_t}P_t(\mathbf f).
\tag{13}
\]

正式实验使用 \(\beta=1.0\)、\(w_S=2.0\)。\(w_S\) 不是硬安全约束：硬约束已由 \(r_r+d_s=0.5\,\mathrm m\) 的膨胀栅格保证；它用于在均可行的候选中偏向净空更大的语义视点。`No clearance utility` 消融令 \(w_S=0\)。

队列更新必须使旧 heap entry 失效。实现通过对象身份而非仅检查 frontier ID 来判定条目是否为当前版本；旧实现会弹出 stale priority。

## 5. A* 可达性与执行代价

A* 在安全栅格上使用

\[
F(n)=G(n)+H(n),
\qquad H(n)=\rho\|n-n_g\|_2.
\tag{14}
\]

搜索上限至少为地图栅格数 \(|M|\)，避免大地图上把“迭代预算耗尽”误判为不可达。接受路径 \(\pi_t^*\) 后

\[
L_{t+1}=L_t+\sum_{j=0}^{|\pi_t^*|-2}\|\pi_{t,j+1}^*-\pi_{t,j}^*\|_2.
\tag{15}
\]

## 6. 全局覆盖缺口恢复

局部队列为空且 \(R_C(t)<\tau_C\) 时，定义未覆盖自由空间

\[
\mathcal U_t=\mathcal F\setminus\mathcal C(\mathcal V_t).
\tag{16}
\]

去除面积小于 \(A_{\min}\) 的连通分量，在每个有效区域中取未覆盖距离变换的局部极大值：

\[
\hat{\mathbf g}_j\in\arg\max_{\mathbf x\in\mathcal U_t\cap\mathcal F_s}
D_{\mathcal U_t^c}(\mathbf x).
\tag{17}
\]

只保留 A* 可达候选。其恢复优先级为

\[
P_R(\mathbf g)=
w_I\min(\hat I(\mathbf g),1)
+w_C\min\!\left(\frac{D_{\mathcal O}(\mathbf g)}{r_v},1\right)
{}-w_L\min\!\left(\frac{d_G(\mathbf v_t,\mathbf g)}{10r_v},1\right),
\tag{18}
\]

其中 \(\hat I\) 是视野邻域内未覆盖栅格比例。恢复候选以 `global_recovery` 类型插回同一队列，使局部生成和全局恢复共享选择/执行过程。

## 7. 终止条件

算法在以下任一条件成立时停止：

\[
R_C(t)\ge\tau_C\land \max_{\mathbf f\in\mathcal Q_t}P_t(\mathbf f)<P_{\mathrm{stop}},
\tag{19a}
\]

\[
\mathcal Q_t=\varnothing\land\text{Recovery}(\mathcal U_t)=\varnothing,
\tag{19b}
\]

\[
t\ge T_{\max}.
\tag{19c}
\]

输出必须记录 `termination_reason`，不能只给一个含义不明的 success 布尔量。

## 8. Algorithm 1 伪代码

```text
Input: occupancy grid M, start v0, view radius rv, target τC
Inflate obstacles; compute clearance field; initialize V={v0}, global queue Q=∅
Compute one-to-all geodesic costs from v0
GENERATE-AND-TRACE(v0, Q)
while |V| < Tmax:
    if Q is empty:
        Q ← reachable maxima of uncovered-space distance transform
        if Q is empty: break
    refresh all P(f) using geodesic cost and viewpoint clearance
    f* ← argmax Q
    π* ← A*(current, f*)
    if π* does not exist:
        trace UNREACHABLE(f*); continue
    append f* to V; accumulate length(π*)
    update coverage, geodesic map, and collision KD-tree
    GENERATE-AND-TRACE(f*, Q)
    prune covered/duplicate/low-priority frontiers
    if target coverage and queue priority is low: break
return V, paths, complete decision trace, metrics
```

## 9. 复杂度

令栅格数为 \(N\)，每步方向数为 \(K\)，活动 frontier 数为 \(F_t\)，接受节点数为 \(T\)。one-to-all Dijkstra/MCP 为 \(O(N\log N)\)，全队列重排为 \(O(F_t\log T)\)，单次 A* 最坏为 \(O(N\log N)\)。总体主项为

\[
O\!\left(T\,[N\log N+K\log T+F_t\log T]\right).
\tag{20}
\]

论文应同时报告墙钟时间，因为渐近复杂度无法反映 Python 实现与栅格分辨率常数。

## 10. 评价指标

\[
L=\sum_{t=0}^{T-1}\operatorname{len}(\pi_t),
\qquad \eta_C=\frac{R_C(T)}{L}.
\tag{21}
\]

\[
Q_{\mathrm{safe}}=\frac{1}{T+1}\sum_{i=0}^{T}D_{\mathcal O}(\mathbf v_i),
\quad Q_{\min}=\min_iD_{\mathcal O}(\mathbf v_i).
\tag{22}
\]

\[
d_i^{NN}=\min_{j\ne i}\|\mathbf v_i-\mathbf v_j\|_2,
\qquad
Q_{\mathrm{uniform}}=\max\!\left(0,1-\frac{\sigma(d^{NN})}{\mu(d^{NN})}\right).
\tag{23}
\]

令矩形地图边界为 \(\partial\Omega\)，视点与执行路径的安全指标为

\[
Q_{\partial}=\frac{1}{T+1}\sum_{i=0}^{T}d(\mathbf v_i,\partial\Omega),
\qquad
R_{V}^{\mathrm{safe}}=\frac{1}{T+1}\sum_{i=0}^{T}
\mathbb 1[D_{\mathcal O}(\mathbf v_i)\ge r_r+d_s],
\tag{24}
\]

\[
Q_{\pi}=\frac{1}{|\Pi|}\sum_{\mathbf p\in\Pi}D_{\mathcal O}(\mathbf p),
\qquad
R_{\pi}^{\mathrm{safe}}=\frac{1}{|\Pi|}\sum_{\mathbf p\in\Pi}
\mathbb 1[D_{\mathcal O}(\mathbf p)\ge r_r+d_s],
\tag{25}
\]

其中 \(\Pi\) 以地图分辨率对所有实际 A* 路径段采样。必须分别报告均值和最小值；地图外围墙虽计入障碍距离，仍单列 \(Q_{\partial}\) 以区分“远离内部障碍”和“远离边界”。

另报告 success rate、runtime、节点数、恢复次数、候选总数与各拒绝原因计数。

## 11. 必做消融

| 变体 | 关闭/替换内容 | 要回答的问题 |
|---|---|---|
| Full SSTG-Explorer | 无 | 最终性能 |
| Euclidean priority | \(d_G\to\|x-y\|_2\) | 隔墙候选是否导致绕行 |
| No recovery | 关闭 Eq. (16–18) | 局部队列耗尽是否复现 |
| Local priority update | 只更新附近 frontier | 全局重排是否减少回访 |
| Adaptive angular | 启用 Eq. (7)，窄通道降至 \(15^\circ\) | 增密是否真的优于最终固定 30° |
| No aggressive pruning | 关闭 strength/duplicate pruning | 速度–候选质量权衡 |
| No clearance utility | 令 \(w_S=0\) | 安全偏好对净空、覆盖和路程的影响 |

旧 priority queue bug 是实现错误，应修复并通过单测保证，不应把修 bug 包装成贡献。

## 12. 学习型基线的正确表述

`ANS-Global (adapted)` 加载 Chaplot et al. 发布的 global-policy checkpoint。保留 CNN、orientation embedding 与连续 goal head；本项目以 observed occupancy/explored map 构造其 8-channel 输入，用共同 A* 执行目标。没有使用原论文的 RGB Neural-SLAM mapper 和 learned local controller。因此：

- 可作为“公开预训练 learned global goal policy”对比；
- 不能声称复现完整 Active Neural SLAM 数字；
- 表格脚注和正文首次出现处都要写 `(adapted)`；
- checkpoint URL、SHA-256、PyTorch 版本写入复现材料。

完整 ANS、DRL-Graph 和 Exploring Exploration 的输入/数据集不同，适合作为 related work 或第二套 Habitat 协议，不能悄悄并入已知栅格主表。

## 13. 写作禁区

- 不写 “optimal”，除非给出理论最优性或界。
- 不用单 seed 结果声称显著提升。
- 不把仿真圆盘覆盖等同于真实相机语义可见性。
- 不忽略 dense、multiple rooms 或 narrow 的失败历史；应展示修复前后消融。
- 不把 runtime 跨硬件比较。
- 不把过程 trace 的候选数量当作独立统计样本。

## 14. 当前可直接引用的实验事实

数据源：`outputs/benchmark_runs/20260719_043528/results.json`（270 runs）与 `outputs/ablation_runs/20260719_043544/results.json`（315 runs）。二者的 experiment-source SHA-256 均为 `efa1828c62bac7ca0f33849449e1363a3ff14bd0079a5e9668764b8ade7e8642`，ANS checkpoint SHA-256 为 `616fd1485e1f0ba9673db08340d586c050f001f171890d966809c0b9f0320314`。

- SSTG-Explorer 宏平均 coverage 为 98.52%，Frontier 为 96.92%。环境 cluster bootstrap 差值为 `+1.61 pp [0.90, 2.45]`，环境级 Wilcoxon 经 Holm 校正 `p=0.0234`。
- SSTG 平均实际路径 48.13 m，Frontier 45.28 m；差值 `+2.85 m [-2.80, 8.92]`，Holm `p=0.652`，因此写 comparable travel distance，不写更短。
- 相对 ANS-Global (adapted)，SSTG coverage 差 `+2.05 pp [1.41, 2.75]`，distance 差 `-31.36 m [-51.98, -15.01]`；二者 Holm `p=0.0195`。
- 相对 NBV/RRT/Uniform，coverage 分别高 `2.42/2.20/1.60 pp`，实际路径分别短 `41.83/166.43/12.10 m`，相应 cluster CI 均不跨 0。
- SSTG 在 multiple_rooms/dense_obstacles/narrow_passages 分别达到 97.11%/99.59%/98.00%；修复前历史值约为 34%/31%/37%。前后地图合法性和代码均变化，旧值只能作为 failure history，不能当 controlled ablation。
- SSTG 平均视点净空 1.160 m，仅低于 RRT 1.188 m；相对 Frontier 高 `0.097 m [0.032, 0.157]`，Holm `p=0.0469`。每-run 最小路径净空的宏平均 0.597 m 为六方法最高点估计，但相应 Holm 检验不显著。
- 去掉 clearance utility 后，视点净空下降 `0.140 m [0.074, 0.213]`，Holm `p=0.0469`，coverage 下降 1.56 pp；去掉 recovery 后 success 从 100% 降到 88.9%；关闭 pruning 使时间从 2.98 s 增到 5.68 s。
- fixed-30° 相对 adaptive-15° 的 coverage/distance 宏平均为 98.52%/48.13 m vs 98.31%/49.54 m，但差异 CI 跨 0；只能称为多指标 point-estimate 选择。

建议摘要报告 coverage 显著性、相对 Frontier 的净空收益及相对学习基线的路径差；正文同时承认 RRT 的平均视点净空略高、Frontier 的路径点估计略短。
