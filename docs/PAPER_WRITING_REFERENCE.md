# SSTG-Explorer：RAL 论文公式、算法与写作参考

本文档给出与当前实现一致的三类证据数学表述：Sec. 1–10 对应 `known_static_disk` 结构参考；Sec. 11 对应论文主算法 `unknown_static_grid_joint_topological_coverage`；Sec. 12 保留冻结 `sensor-only unknown` 的诊断公式。三者共享部分图结构与测地执行，但 coverage、终止和可用信息不同，投稿时绝不能混作同一统计变量。公式可压缩，不能保留代码中不存在的虚构目标项。

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

在按 \(r_r+d_s\) 膨胀后的统一安全栅格上定义测地距离

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

## 11. Joint unknown-map SSTG-Explorer（论文主算法）

### 11.1 两种覆盖 construct

令真实自由栅格集合为

\[
\mathcal F^\star=\{c\in\mathcal C\mid M^\star(c)=0\}.
\tag{26}
\]

Occlusion-aware sensor coverage 是被 ray observation 正确写为 free 的比例：

\[
C_t^I=
\frac{|\{c\in\mathcal F^\star\mid B_t(c)=0\}|}
{|\mathcal F^\star|}.
\tag{27}
\]

令空间拓扑节点集合为 \(V_t=\{v_i\}_{i=0}^{N_t}\)，节点位置为 \(p_i\)，任务有效半径为 \(r_v\)。Topology coverage 定义为

\[
C_t^T=
\frac{\left|\left\{c\in\mathcal F^\star\middle|
\min_{v_i\in V_t}\|x_c-p_i\|_2\le r_v\right\}\right|}
{|\mathcal F^\star|}.
\tag{28}
\]

正式实验 \(r_v=2\) m，与 known-map disk benchmark 完全相同。它是 task-distance proxy，不包含 wall occlusion、incidence angle、pixel resolution 或 recognition uncertainty。部署时应把 Euclidean disk \(\mathcal D(p_i,r_v)\) 替换为 calibrated task-valid observation set。

保守 joint scalar 与真正终止条件分别为

\[
C_t^J=\min(C_t^I,C_t^T),
\tag{29}
\]

\[
\text{success}
\Longleftrightarrow
C_t^I\ge\tau_I\ \land\ C_t^T\ge\tau_T,
\qquad \tau_I=\tau_T=0.95.
\tag{30}
\]

不能只对 \(C^J\) 写一个阈值而隐藏两个 component。表、图、JSON 和摘要都分别报告 \(C^I,C^T\)。

### 11.2 Truth–belief 分离与观测更新

策略 belief 为

\[
B_t:\mathcal C\rightarrow\{-1,0,100\},
\qquad B_0(c)=-1.
\tag{31}
\]

对位姿 \(q_t=(p_t,\theta_t)\)，FOV \(\phi\)、量程 \(R\) 的 first-obstacle visible set 记作 \(\mathcal Z(q_t;M^\star)\)。更新：

\[
B_{t+1}(c)=
\begin{cases}
M^\star(c), & c\in\mathcal Z(q_t;M^\star),\\
B_t(c), & \text{otherwise}.
\end{cases}
\tag{32}
\]

\(M^\star\) 只在 Eq. (32) 的 ray sensor 与 Eqs. (27–28) 的 evaluator 中出现。Candidate generator、gain predictor、ANS adapter、cost map 和 A* 的函数签名都只接收 \(B_t\)。

### 11.3 Known-free safety 与可达性

机器人中心的硬可行域要求半径 \(r_r=0.3\) m footprint 全部已知 free：

\[
\mathcal F_t^K=
\left\{x\middle|
B_t(c)=0,\ \forall c\in\mathcal B(x,r_r)
\right\}.
\tag{33}
\]

四连通 start component 定义 reachable set \(\mathfrak R_t\subseteq\mathcal F_t^K\)。One-to-all geodesic \(d_G(p_t,x)\) 与执行 A* 共用该 mask，且禁止 diagonal corner cutting。Preferred clearance \(d_{pref}=0.5\) m 只进入排序和报告，不扩大硬机器人尺寸。

### 11.4 在线拓扑缺口

已接受节点在当前 belief 的已知自由空间上产生 disk union：

\[
\widehat{\mathcal C}_t^T=
\left(\bigcup_{v_i\in V_t}\mathcal D(p_i,r_v)\right)
\cap\{c\mid B_t(c)=0\}.
\tag{34}
\]

Policy-visible、reachable 的未覆盖集合为

\[
\widehat{\mathcal U}_t^T=
\left(\mathfrak R_t\cap\{c\mid B_t(c)=0\}\right)
\setminus\widehat{\mathcal C}_t^T.
\tag{35}
\]

关键性质：当 \(\mathcal U_t=\{B_t=-1\}=\varnothing\) 时，Eq. (35) 仍可能非空。因此 sensor gain 为零以后，算法继续从当前 known-free gap 生成候选，而不是读取 truth uniform grid。

Gap source 同时保留：

- \(\widehat{\mathcal U}_t^T\) 上 nearest reachable local gap；
- 确定性 FPS representatives；
- frontier band；
- full reachable-space topological vantages；
- \(\phi<360^\circ\) 时的 current-position rotations。

FPS 为

\[
f_1=\arg\max_{x\in\mathcal S}
\left[d_G(p_t,x)+0.25D_O^t(x)\right],
\qquad
f_k=\arg\max_{x\in\mathcal S}
\min_{j<k}\|x-f_j\|_2,
\tag{36}
\]

下一候选与集合距离小于 \(d_{rep}=2\) m 时停止。

### 11.5 双增益与联合效用

候选 \(f=(x,\theta)\) 的 belief-only information gain：

\[
G_t^I(f)=
|\widehat{\mathcal Z}(f;B_t)\cap\mathcal U_t|,
\qquad \mathcal U_t=\{c\mid B_t(c)=-1\}.
\tag{37}
\]

边际拓扑增益：

\[
G_t^T(f)=
|\mathcal D(x,r_v)\cap\widehat{\mathcal U}_t^T|.
\tag{38}
\]

候选只要 \(G_t^I\ge g_{min}^I\) 或 \(G_t^T\ge g_{min}^T\) 就可保持 active。这样 Eq. (37) 为零不会把 topology-only action 错误剪掉。

在 active set \(\mathcal Q_t\) 内分别 max-normalize：

\[
\bar G_t^k(f)=
\frac{G_t^k(f)}{\max_{h\in\mathcal Q_t}G_t^k(h)},
\qquad k\in\{I,T\},
\tag{39}
\]

若分母为零则取 1。Joint task gain：

\[
\bar g_t(f)=0.4\bar G_t^I(f)+0.6\bar G_t^T(f).
\tag{40}
\]

归一化 cost 与 clearance：

\[
\bar\ell_t(f)=\frac{d_G(p_t,x)}{r_v},
\qquad
\bar c_t(f)=\min\left(\frac{D_O^t(x)}{d_{pref}},1\right).
\tag{41}
\]

候选相对已有空间节点的截断间距为

\[
\bar s_t(f)=\min\!\left(
\frac{\min_{v_i\in V_t}\|x-p_i\|_2}{d_{rep}},1
\right),\qquad d_{rep}=2\text{ m}.
\]

原地旋转不创建空间分离，代码使用常数 0.35，避免完全压制窄 FOV 下必要的 heading action。

最终 SSTG-Explorer joint utility 与实现一致：

\[
U_t(f)=
\frac{1.20\bar g_t(f)}{1+0.60\bar\ell_t(f)}
+0.20w_c\bar c_t(f)+w_s\bar s_t(f),
\qquad w_c=1.5,\quad w_s=0.30.
\tag{42}
\]

即 clearance 项系数为 0.30。分母式 travel penalty 避免远端 absolute gain 造成跨地图振荡；spacing 项在候选层直接偏好非冗余空间动作。54-cluster 全矩阵配对选择实验中，加入该项没有可检测的 coverage、success 或 clearance 损失，同时相对无该项版本减少 2.00 m 路程、0.17 个节点和 0.33 个动作；因此最终配置采用 \(w_s=0.30\)。该比较属于开发阶段的配置选择而非独立 confirmatory test。

### 11.6 空间节点与朝向动作

感知动作 \(a_k=(p_k,\theta_k)\)。它创建新 node 当且仅当

\[
\min_{v_i\in V_t}\|p_k-p_i\|_2>\delta_m,
\qquad \delta_m=0.25\text{ m}.
\tag{43}
\]

否则 action 关联到最近节点。分别报告：

\[
N_T=|V_T|,\qquad A_T=|\{a_k\}|,
\qquad N_T\le A_T.
\tag{44}
\]

按空间节点顺序的 redundancy：

\[
R_{red}=\frac{1}{N_T-1}
\sum_{i=1}^{N_T-1}
\mathbb 1\left[
\min_{0\le j<i}\|p_i-p_j\|_2<1\text{ m}
\right].
\tag{45}
\]

Rotation 不进入 Eq. (45)，而进入 \(A_T\)、`in_place_rotations` 和 `total_rotation_deg`。这是最终算法与旧文档最重要的语义修正之一。

### 11.7 精确增益预算与已执行剪枝

候选先用 range/FOV optimistic count 和局部 disk gain 排序。精确 ray gain shortlist 包含：

- joint preliminary top 18；
- top information 6；
- top topology 8；
- nearest 6；
- 所有 rotation；
- ANS predicted goal 最近候选。

所有候选仍进入 trace。`execution_key=(row,col,rounded_heading,kind)` 已执行后，后续状态为 `pruned_executed`，不会被重复选择。

### 11.8 Algorithm 2：最终在线闭环

```text
Input: hidden M* (sensor/evaluator only), all-unknown B0, start q0
OBSERVE(q0); V ← {start node}; A ← {start action}
while CI < τI or CT < τT, and decisions < Tmax:
    FK ← erode(known_free(B), robot footprint)
    R  ← start-connected(FK); compute dG and clearance
    UT ← reachable known-free minus union of node disks
    C  ← multi-frontier ∪ gap FPS ∪ vantages ∪ rotations/samples
    trace optimistic GI, GT, cost, clearance, ID and lifecycle
    shortlist C; compute exact belief-ray GI; retain if GI or GT is useful
    f* ← argmax Eq. (42)
    π* ← known-free A*(q, f*)
    execute π*; observe every 1 m and at final heading
    associate action to a node using Eq. (43)
    update B, CI, CT; store updates, scans, path, node and action
return B, V, A, executed trajectory, complete trace
```

### 11.9 复杂度

令栅格数 \(N\)，raw candidates \(C_t\)，exact candidates \(C_e\)，每候选射线 \(K=\lceil\phi/\delta_\alpha\rceil\)，每束 ray steps \(S=\lceil R/(\rho/2)\rceil\)，disk window cells \(W=O((r_v/\rho)^2)\)。主项：

\[
O\left(
T\left[N\log N+C_tN+C_tW+C_eKS\right]
\right).
\tag{46}
\]

\(C_tN\) 来自当前 deterministic FPS worst case；实现通过 local-window disk gain 避免对每个候选复制全图 coverage map。

### 11.10 Post-sensor gap 指标

令 \(t_I=\min\{t:C_t^I\ge0.95\}\)。报告：

\[
\Delta C_{post}^T=C_T^T-C_{t_I}^T,
\qquad
A_{post}=|\{a_k:k>t_I\}|.
\tag{47}
\]

正式 SSTG 为 \(C_{t_I}^T=48.31\%\)、\(\Delta C_{post}^T=47.83\) pp、\(A_{post}=10.76\)。其中 7.09 个是 selected `coverage_gap`，7.13 个 `new_observed_count=0`。这些是 topology-building actions，不是 sensor stagnation。

## 12. Sensor-only unknown 诊断公式（冻结旧证据）

### 12.1 Truth–belief 分离与遮挡观测

令静态 ground-truth 占据栅格为 \(M^\star:\mathcal C\rightarrow\{0,100\}\)，策略可读 belief 为

\[
B_t:\mathcal C\rightarrow\{-1,0,100\},
\qquad B_0(c)=-1,
\tag{26}
\]

其中 \(-1\) 表示未知。\(M^\star\) 只由传感器仿真器和评价器持有，candidate generator、utility、frontier detector、geodesic planner 和学习型 global policy 都只能接收 \(B_t\)。地图边界尺寸已知，内容未知。

在位姿 \(\mathbf q_t=(x_t,y_t,\theta_t)\) 处，水平 FOV 为 \(\phi\)、量程为 \(R\)、角分辨率为 \(\delta_\alpha\) 的射线角集合为

\[
\mathcal A_t=\left\{\theta_t-\frac{\phi}{2}+k\delta_\alpha
\;\middle|\;k=0,\ldots,
\left\lceil\frac{\phi}{\delta_\alpha}\right\rceil\right\}.
\tag{27}
\]

沿角度 \(\alpha\) 以栅格步长 \(\delta_r=\rho/2\) 采样 \(c_{\alpha,k}\)。第一障碍索引与单束可见集为

\[
k_\alpha^\star=
\min\{k\mid M^\star(c_{\alpha,k})=100\},
\qquad
\mathcal Z_\alpha(\mathbf q_t)=
\{c_{\alpha,k}\mid k\le k_\alpha^\star, k\delta_r\le R\}.
\tag{28}
\]

若量程内没有障碍，则 \(k_\alpha^\star\) 取最后一个有效采样。障碍表面可见而墙后不可见。整帧可见集与 belief 更新为

\[
\mathcal Z(\mathbf q_t)=\bigcup_{\alpha\in\mathcal A_t}
\mathcal Z_\alpha(\mathbf q_t),
\qquad
B_{t+1}(c)=
\begin{cases}
M^\star(c),&c\in\mathcal Z(\mathbf q_t),\\
B_t(c),&\text{otherwise}.
\end{cases}
\tag{29}
\]

实现使用理想静态传感器与完美位姿；没有把定位漂移、动态障碍或量测噪声藏在“未知地图”名称中。机器人沿每条实际 A* 路径每 \(d_{scan}=1\,\mathrm m\) 继续观测，并在目标朝向补一帧，而不是只在离散终点画覆盖圆。

未知协议的 observed-free coverage 为

\[
R_C^{obs}(t)=
\frac{\sum_{c\in\mathcal C}
\mathbb 1[M^\star(c)=0\land B_t(c)=0]}
{\sum_{c\in\mathcal C}\mathbb 1[M^\star(c)=0]}.
\tag{30}
\]

`known_ratio` 和障碍召回率分别为

\[
R_K(t)=\frac{1}{|\mathcal C|}\sum_c\mathbb 1[B_t(c)\ne-1],
\qquad
R_O(t)=\frac{\sum_c\mathbb 1[M^\star(c)=100\land B_t(c)=100]}
{\sum_c\mathbb 1[M^\star(c)=100]}.
\tag{31}
\]

### 12.2 已知安全区、frontier band 与拓扑候选

未知障碍不能被当成 free。机器人中心的硬可行域要求整个 footprint 已观测为自由：

\[
\mathcal F_t^K=
\{\mathbf x\mid B_t(c)=0,
\ \forall c\in\mathcal B(\mathbf x,r_r)\}.
\tag{32}
\]

当前实现用 \(r_r=0.3\,\mathrm m\) 作硬碰撞约束；\(d_{pref}=0.5\,\mathrm m\) 是报告和排序使用的偏好净空，不再错误地扩大机器人几何尺寸。四连通 start component 定义 reachable set \(\mathfrak R_t\subseteq\mathcal F_t^K\)，从而 cost map 与禁止斜穿墙角的 A* 具有一致可达性。

令未知栅格集合 \(\mathcal U_t=\{c\mid B_t(c)=-1\}\)，SSTG 的 frontier band 为

\[
\mathcal R_t=
\{\mathbf x\in\mathfrak R_t\mid
d(\mathbf x,\mathcal U_t)\le d_{band}\},
\qquad d_{band}=r_r+2\rho.
\tag{33}
\]

大 frontier 往往是单个连通边界，只取一个 centroid 会漏掉门洞和货架 aisle。实现因此用确定性 farthest-point sampling 从 \(\mathfrak R_t\) 和 \(\mathfrak R_t\cap\mathcal R_t\) 分别保留拓扑视点：

\[
\mathbf f_1=\arg\max_{\mathbf x\in\mathcal S}
\left[d_G(\mathbf q_t,\mathbf x)+\lambda_D D_O^t(\mathbf x)\right],
\qquad
\mathbf f_k=\arg\max_{\mathbf x\in\mathcal S}
\min_{j<k}\|\mathbf x-\mathbf f_j\|_2,
\tag{34}
\]

并在最近间距小于 \(d_{rep}=2\,\mathrm m\) 时停止。这里 \(\mathcal S\) 完全来自 belief 的已知安全区域，不是预先读取 truth 的 uniform grid。

### 12.3 遮挡预测增益与 sensor-only 优先级

候选 \(f=(\mathbf x,\theta)\) 的 optimistic gain 只把 belief 中已知 occupied cell 当作射线终点：

\[
G_t(f)=|\widehat{\mathcal Z}(f;B_t)\cap\mathcal U_t|.
\tag{35}
\]

这不是从 truth 偷看遮挡；未知障碍会在真实执行观测后截断射线。候选先按无射线的 range/FOV optimistic count 与 travel cost 预筛，最多对 18 个高潜力点、6 个近点和所有原地旋转计算 Eq. (35)，其余标记为 `pruned_evaluation_budget`。

令

\[
\bar g_t(f)=\frac{G_t(f)}{\max_{h\in\mathcal Q_t}G_t(h)},\quad
\bar \ell_t(f)=\frac{d_G(\mathbf q_t,\mathbf x)}{R},\quad
\bar c_t(f)=\min\!\left(\frac{D_O^t(\mathbf x)}{d_{pref}},1\right),
\tag{36}
\]

\[
\bar s_t(f)=\min\!\left(
\frac{\min_{\mathbf v\in\mathcal V_t}\|\mathbf x-\mathbf v\|_2}
{d_{rep}},1\right).
\tag{37}
\]

冻结的旧 sensor-only SSTG 版本使用如下加性归一化效用

\[
P_t^{U}(f)=
1.20\bar g_t(f)-0.15\bar\ell_t(f)
+0.20w_S\bar c_t(f),
\qquad w_S=1.5.
\tag{38}
\]

加性形式避免远端高增益门洞被乘性 travel decay 压成近零。空间离散性由 Eq. (34) 的 FPS 候选层实现，\(\bar s_t\) 仍写入每个 candidate trace 并用于 Eq. (39–40) 的结果解释。旧 sensor-only 消融给 Eq. (38) 额外加入 \(+0.30\bar s_t\) 后，coverage、distance、nodes、clearance、NN distance 和 redundancy 的宏平均点估计均未改善；这个结论只解释冻结协议，不替代 Sec. 13 的 joint 变体选择。原地换朝向不增加平移路程，但仍作为 oriented viewpoint 计入节点数、旋转量和空间冗余率。

### 12.4 冻结旧结果的 oriented-view 指标

除 Eq. (23) 的 nearest-neighbor 统计外，按决策顺序定义阈值 \(d_{red}=1\,\mathrm m\) 的回访冗余率

\[
R_{red}=\frac{1}{T}
\sum_{i=1}^{T}
\mathbb 1\!\left[
\min_{0\le j<i}\|\mathbf v_i-\mathbf v_j\|_2<d_{red}
\right].
\tag{39}
\]

原地换朝向在空间上确实重复，因此 Eq. (39) 计为 redundant；同时必须报告 `in_place_rotations` 与 `total_rotation_deg`，以区分无意义回访和窄 FOV 下必要的多朝向观测。信息/视点效率为

\[
\eta_V=\frac{R_C^{obs}(T)}{|\mathcal V_T|}.
\tag{40}
\]

论文应联合报告 \(R_C^{obs}\)、实际平移距离、节点数、mean/median/min \(d^{NN}\)、\(R_{red}\)、\(\eta_V\) 和成功率。少量远隔点也会产生很大的 NN distance，因此“间距更大”本身不能证明探索更好。

### 12.5 Sensor-only 在线闭环

```text
Input: hidden truth M* (sensor/evaluator only), all-unknown B0, start q0
OBSERVE(M*, B0, q0); trace all changed cells
while observed-free coverage < τ and decisions < Tmax:
    FK ← erode(known-free(Bt), robot footprint)
    R  ← start-connected component of FK; compute geodesic costs
    C  ← multi-representative frontier band ∪ topological vantages
    if FOV < 360°: C ← C ∪ current-pose orientation candidates
    for f in C:
        trace optimistic gain, cost, clearance, spacing and candidate ID
    shortlist C; ray-cast predicted gain using Bt; trace every pruning state
    f* ← argmax Eq. (38); π* ← known-free A*(qt, f*)
    if π* is unreachable: trace it and continue
    execute π*; observe every 1 m and at f*; store compact belief updates
    append oriented viewpoint, actual path, scans and coverage delta
return belief, oriented viewpoints, actual trajectory, complete decision trace
```

若候选数为 \(C_t\)，每个候选的射线数为 \(K=\lceil\phi/\delta_\alpha\rceil\)，每束最大步数为 \(S=\lceil R/(\rho/2)\rceil\)，只对 \(C_e\le18+6+C_{rot}\) 个候选做精确预测。未知算法的主项为

\[
O\!\left(T\left[N\log N+C_tN+C_eKS\right]\right),
\tag{41}
\]

其中当前 farthest-point implementation 对候选 mask 的最坏项写为 \(C_tN\)。论文还应报告实际 runtime，而不能只给大 \(O\)。

## 13. 消融与配置选择

### 13.1 Joint 主任务单因素消融

| 变体 | 关闭/替换内容 | 要回答的问题 |
|---|---|---|
| Full SSTG-Explorer | 无 | 最终性能 |
| Single frontier centroid | 每个连通 frontier 只留一个代表点 | warehouse/dense 的遮挡短板是否复现 |
| Known-obstacle-only safety | 不要求整个 footprint 已知 free | 是否出现新观测后被障碍困住 |
| No topology vantages | 只保留 frontier band | aisle/doorway 的视点是否丢失 |
| No spacing utility | Eq. (42) 令 \(w_s=0\) | 显式间距偏好是否减少路程、节点和冗余 |

正式 joint hard set 为 4 scenes × 3 sensors × 3 seeds × 5 variants = 180 runs。12 个 sensor–environment clusters 的校正检验统计功效有限，必须把它当模块行为与 failure-mode 证据；若一个变体可能改变最终选择，应补跑 54-cluster 全矩阵，不能用 hard-set 点估计直接命名“最佳”。配置选择属于 development evidence，独立外部验证仍需预注册。

### 13.2 Known-map 支撑消融

已知地图 315-run 研究保留 Euclidean priority、no recovery、local priority update、adaptive angular、no aggressive pruning 和 no clearance utility，用于隔离 geodesic queue、recovery、pruning 和安全偏好。它们的 coverage 定义和 joint 主任务不同，只能作机制支撑，不能与在线 joint 值混检。

旧 priority queue bug 是实现错误，应修复并通过单测保证，不应把修 bug 包装成贡献。

## 14. 学习型基线的正确表述

`ANS-Global (adapted)` 加载 Chaplot et al. 发布的 global-policy checkpoint。保留 CNN、orientation embedding 与连续 goal head；本项目以 observed occupancy/explored map 构造其 8-channel 输入，用共同 A* 执行目标。没有使用原论文的 RGB Neural-SLAM mapper 和 learned local controller。因此：

- 可作为“公开预训练 learned global goal policy”对比；
- 不能声称复现完整 Active Neural SLAM 数字；
- 表格脚注和正文首次出现处都要写 `(adapted)`；
- checkpoint URL、SHA-256、PyTorch 版本写入复现材料。

完整 ANS、DRL-Graph 和 Exploring Exploration 的输入/数据集不同，适合作为 related work 或第二套 Habitat 协议，不能悄悄并入已知栅格主表。

## 15. 写作禁区

- 不写 “optimal”，除非给出理论最优性或界。
- 不用单 seed 结果声称显著提升。
- 不把仿真圆盘覆盖等同于真实相机语义可见性。
- 不忽略 dense、multiple rooms 或 narrow 的失败历史；应展示修复前后消融。
- 不把 runtime 跨硬件比较。
- 不把过程 trace 的候选数量当作独立统计样本。

## 16. 当前可直接引用的实验事实

### 16.1 Joint unknown 主证据

最终目录 `outputs/joint_benchmark_selected/20260719_223630/`：

- SSTG sensor/topology 为 99.99% / 96.14%，162/162 成功；15.76 nodes、17.30 actions、2.15 m mean NN、0.07% redundant nodes、0.95 m clearance、63.99 m travel。
- 相对 ANS/Frontier/NBV/RRT，redundancy 低 4.38/29.56/14.96/3.41 pp，95% CI 均不跨 0，Holm `p<=1.80e-5`。
- Nodes 相对 ANS/Frontier/NBV 少 3.24/16.26/1.70，均显著；相对 RRT 多 0.94，显著。
- Actions 相对 ANS/Frontier/NBV 少 4.11/22.17/2.99，均显著；相对 RRT 多 0.64，显著。
- Clearance 相对 ANS/Frontier 高 0.227/0.155 m，显著；相对 NBV/RRT 低 0.045/0.037 m，但 Holm `p=0.130`，不能写显著。
- Travel 相对 Frontier/NBV/RRT 长 9.91/17.22/6.36 m，均显著；相对 ANS +0.79 m 不显著。
- Success 相对 NBV +9.26 pp，Holm `p=0.0266`；与 RRT 同为 100%。
- Terminal topology 相对 Frontier 低 0.89 pp 且显著，原因需与 Frontier 的 32.02 nodes、29.6% redundancy、96.3% success 和 threshold overshoot 联合解释；不能隐藏。
- 多房间/dense/warehouse SSTG topology 为 95.47/96.16/95.61%，54/54 成功；travel 为 90.87/55.60/110.27 m。
- 审计：810 JSON/belief/每类 CSV，6,270 PNG、270 GIF、270 MP4，零 replay/truth/media/html/log 错误；selected SSTG 162/162 normalized result 与配置选择 run 精确匹配，冻结基线另有 20-run normalized JSON 精确复现。
- 180-run 单因素 hard-scene 消融中，Full/No-spacing/Single/No-vantage/Known-obstacle-footprint 的 topology 为 95.75/95.65/95.82/95.75/83.14%，success 为 100/100/100/100/75%。12-cluster Holm 检验无模块差异显著，不能把 hard-set 点估计写成模块排名。
- 由于 single-centroid 在 hard set 点估计有利，已升格到 54-cluster 全矩阵。最终 multi-frontier 相对 `single + spacing` topology 高 0.361 pp，CI [0.108, 0.634]，development-family-adjusted `p=0.0487`，代价为 +0.46 nodes / +0.70 actions；success 和 redundancy 相同。最终保留 multi-frontier。
- Spacing 相对 `w_s=0` 不改变 coverage/success/clearance，travel 少 2.00 m、actions 少 0.33，factor-three adjusted `p=0.0494/0.00836`；最终使用 `spacing_weight=0.30`。这些是配置选择证据，不是独立 confirmatory evidence。

摘要建议使用 redundancy、success 和明确 RRT trade-off；不要把 threshold-compressed terminal coverage 排名写成主胜负。

### 16.2 Frozen sensor-only 诊断证据

数据源：`outputs/unknown_benchmark_runs/20260719_141122/`（810 runs）与 `outputs/unknown_ablation_runs/20260719_141132/`（180 runs）。两者均为 commit `502afcd`、source SHA-256 `2385e87afd5f5bb601de27a73fd88620d7422c7131ed03f965081e9c3bb4b5a4`，且自动 audit 全部通过。

- SSTG coverage 98.38%、distance 15.73 m、4.57 oriented views、3.69 m mean NN、26.5% redundant views、1.45 m clearance、100% success。
- 相对 Frontier，coverage 为 `+4.12 pp [2.13, 6.41]`、Holm `p=0.00023`；distance `-1.45 m [-3.10, 0.23]` 不显著。
- 相对 NBV/RRT，coverage 均高约 0.64 pp，cluster CI 不跨 0，Holm 均 `p=0.0169`；distance 分别为 `+0.97/+0.46 m`，CI 均跨 0。
- 相对 ANS-Global Unknown (adapted)，coverage 为 `+1.28 pp [0.09, 2.92]`，但 Holm `p=0.0756`；不能把未校正 CI 不跨 0 写成多重比较后显著。
- SSTG 的 NN distance 相对 Frontier/NBV/RRT/ANS 高 `2.18/1.37/1.57/1.44 m`，回访率低 `28.16/2.32/4.35/6.39 pp`；同时节点数最低且 coverage 最高，因此不是失败 run 造成的虚假大间距。
- SSTG 视点净空 1.45 m 低于 NBV/RRT 的 1.53/1.48 m，但高于 ANS/Frontier 的 1.40/1.14 m。不要写 safety 全面最佳。
- 在 90° 条件，SSTG coverage 97.87%、distance 20.19 m、7.67 views、59.45% spatial redundancy；同点多朝向属于窄 FOV 的必要代价，必须同时报告旋转。360°×12 m 则为 98.78%、13.48 m、3.22 views、0% redundancy。
- 旧 sensor-only hard-set 中，single-centroid coverage/success 为 96.43%/91.7%，Full 为 98.32%/100%；额外 spacing score 的点估计没有改善。该结论只属于冻结诊断协议，不能直接决定 joint 最终配置。

### 16.3 Known static disk 受控证据

数据源：`outputs/benchmark_runs/20260719_043528/results.json`（270 runs）与 `outputs/ablation_runs/20260719_043544/results.json`（315 runs）。二者的 experiment-source SHA-256 均为 `efa1828c62bac7ca0f33849449e1363a3ff14bd0079a5e9668764b8ade7e8642`，ANS checkpoint SHA-256 为 `616fd1485e1f0ba9673db08340d586c050f001f171890d966809c0b9f0320314`。

- SSTG-Explorer 宏平均 coverage 为 98.52%，Frontier 为 96.92%。环境 cluster bootstrap 差值为 `+1.61 pp [0.90, 2.45]`，环境级 Wilcoxon 经 Holm 校正 `p=0.0234`。
- SSTG 平均实际路径 48.13 m，Frontier 45.28 m；差值 `+2.85 m [-2.80, 8.92]`，Holm `p=0.652`，因此写 comparable travel distance，不写更短。
- 相对 ANS-Global (adapted)，SSTG coverage 差 `+2.05 pp [1.41, 2.75]`，distance 差 `-31.36 m [-51.98, -15.01]`；二者 Holm `p=0.0195`。
- 相对 NBV/RRT/Uniform，coverage 分别高 `2.42/2.20/1.60 pp`，实际路径分别短 `41.83/166.43/12.10 m`，相应 cluster CI 均不跨 0。
- SSTG 在 multiple_rooms/dense_obstacles/narrow_passages 分别达到 97.11%/99.59%/98.00%；修复前历史值约为 34%/31%/37%。前后地图合法性和代码均变化，旧值只能作为 failure history，不能当 controlled ablation。
- SSTG 平均视点净空 1.160 m，仅低于 RRT 1.188 m；相对 Frontier 高 `0.097 m [0.032, 0.157]`，Holm `p=0.0469`。每-run 最小路径净空的宏平均 0.597 m 为六方法最高点估计，但相应 Holm 检验不显著。
- 去掉 clearance utility 后，视点净空下降 `0.140 m [0.074, 0.213]`，Holm `p=0.0469`，coverage 下降 1.56 pp；去掉 recovery 后 success 从 100% 降到 88.9%；关闭 pruning 使时间从 2.98 s 增到 5.68 s。
- fixed-30° 相对 adaptive-15° 的 coverage/distance 宏平均为 98.52%/48.13 m vs 98.31%/49.54 m，但差异 CI 跨 0；只能称为多指标 point-estimate 选择。

这些 known-map 数字只作为结构参考或模块支持，不再替代 joint 主结果进入摘要。摘要主张按 Sec. 16.1 的 redundancy、success、clearance 与 travel trade-off 书写。
