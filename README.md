# 网格寻路与博弈搜索

本项目实现网格最短路、带进入代价的最小成本路径、博弈树求值和两人轮流争夺宝藏，比较启发函数与子节点访问顺序对搜索量的影响。核心算法、校验器和实验入口仅依赖 Python 标准库（Python 3.10及以上），示例输入已包含在代码中。

## 运行

在项目根目录使用 Windows PowerShell。首次运行可用 `py -3 -m venv .venv` 创建虚拟环境，无需安装第三方包：

```powershell
.\.venv\Scripts\python.exe experiments.py test
.\.venv\Scripts\python.exe experiments.py all
.\.venv\Scripts\python.exe experiments.py diagnose
```

`test` 实际执行断言与独立 BFS/Dijkstra 校验；`all` 运行正确性和全部对照；`diagnose` 单独完整搜索每个首步分支，输出精确值及逐回合轨迹，其节点数不计入效率实验。三条命令均已实际执行并通过；环境与命令保存在 `results/execution_*.json`。重复运行覆盖对应结果。时间为单次辅助测量，不作性能排名。

如需保留已有结果，可在命令后添加 `--output-dir <其他目录>`，将本次输出保存到其他位置。

## 方法与接口

博弈树接口 `alpha_beta(root)` 接收调用者创建的 `TreeNode`，直接返回最佳得分。根为MAX，按层交替MAX/MIN，从左到右访问。例如：

```python
from search_lab import TreeNode, alpha_beta

root = TreeNode(children=[TreeNode(3), TreeNode(5)])
print(alpha_beta(root))  # 5
```

叶子用 `TreeNode(value=分数)`，内部节点用 `TreeNode(children=[...])`。`tree_from_json` 将官方完整JSON转为相同节点结构；公开接口与官方实验均复用 `tree_search`。需要实验统计时可调用 `tree_search(prune=True, root=root)`，返回得分、节点数、访问叶子及剪枝轨迹；默认不剪枝即为完整Minimax。`test` 还验证单叶树（-7）及不平衡树（手算与两种算法均为6）。

`search_lab.py` 中 `maze_path(grid,start,goal)` 返回包含端点的坐标路径，无解为 `[]`；`weighted_cost` 返回代价，无解为 `-1`。`astar` 同时返回路径、有效扩展数与扩展轨迹。队列首先按 g+h 排序，默认按入队序号解决同分，邻居顺序上、右、下、左；改善 g 时重新入队并允许重开，过期条目跳过，仅终点有效出队后结束。扩展数包含终点，不包含过期条目。

加权寻路从0计费，进入E为0。安全启发为 `m*max(曼哈顿距离-1,0)`，m取所有可进入非终点格代价的最小值，允许m=0。它为至少需要进入的非终点格数提供成本下界。宝藏地图中的特殊格进入代价为1，因此使用普通曼哈顿。

博弈树示例按完整JSON建树，从左到右，根为MAX。宝藏博弈状态保存双方位置、未领取宝藏、双方累计得分、当前玩家、剩余深度；仅目标宝藏被领取。双方均以当前 `Score_A-Score_B` 为终止评价。Minimax与Alpha-beta共享状态转换及评价函数，严格改善才替换选择，同值取遍历顺序中的第一个。原顺序为T1到T4。反序直接反转顺序；收益优先仅用当前玩家的宝藏价值减移动代价排序，不先求解子树。

节点数计每次递归进入，含根与叶；叶评估数计终止评价次数。剪枝数仅计确实跳过尚未访问后续子分支的事件。完整轨迹、窗口及跳过分支均保存在结果中。

## 结果导航

|文件（results/）|内容|
|---|---|
|official_tests.json|四题官方结果，边界测试，路径与逐格计费，题3访问叶与具体剪枝|
|tree_interface_tests.json|通用树接口的官方JSON、单叶、不平衡树测试，手算值及Minimax核对|
|astar_comparison.json|官方、开阔、绕路图的启发对照；开阔图同分规则对照|
|weighted_comparison.json|三种启发、真实最优代价、高估见证、扩展示例反例地图及完整轨迹|
|pruning_comparison.json|两道博弈题的节点、叶评估、实际剪枝、排序与辅助时间|
|distance_matrix.json / .csv|6×6矩阵；JSON附36项Dijkstra核对及路径|
|first_move_diagnostics.json|每个首步独立完整Minimax精确值、最优后续及完整状态|
|diagnostic_notes.md|逐回合表格及T2/T3排名反转的具体行动解释|

已运行结果：题1最短7步；题2最小代价7；题3根值5；题4 H=2 为T2/-1，H=4 为T3/3。各首步H=2值为[-4,-1,-2,-4]，H=4为[-5,1,3,-7]。独立校验器不调用A*的邻居、计费或队列实现。

启发现象：官方迷宫h=0/曼哈顿扩展16/14；开阔图FIFO均225，曼哈顿改为同分优先较大g后为29；绕路图均211。说明启发改善搜索量依赖地图与同分规则。

加权官方图终点相邻格(4,3)真实剩余代价为0，原始曼哈顿为1，已发生高估，但本次官方输入仍返回最优7。额外零代价格地图中原始曼哈顿实际返回2而最优为0，地图和轨迹已保存。该扩展反例含0代价，不代表官方正整数地形上已发现次优反例。安全启发在该图退化为h=0。

题4 H=4普通Minimax/原序剪枝/反序剪枝/即时收益优先的节点数分别为65/47/46/37；不据单次耗时判断优劣。排名反转源于T3首步的最优续局中，B最后从T1到T4花12、净收益-5；T2首步续局中B从T3到T4只花6、净收益1。详见逐回合证据。
