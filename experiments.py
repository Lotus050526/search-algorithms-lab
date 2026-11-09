"""Independent oracles and reproducible experiment runner (standard library only)."""
import argparse
import csv
import heapq
import json
import platform
import sys
from collections import deque
from pathlib import Path
from search_lab import *

OUT=Path('results')

def oracle(grid,start,goal,weighted=False):
    # Independent adjacency, cost handling and frontier, no calls to A* helpers.
    dist={start:0}; queue=[(0,start)] if weighted else deque([start])
    while queue:
        if weighted:
            d,p=heapq.heappop(queue)
            if d!=dist[p]: continue
        else: p=queue.popleft(); d=dist[p]
        if p==goal: return d
        for dr,dc in ((1,0),(-1,0),(0,-1),(0,1)):
            q=p[0]+dr,p[1]+dc
            if not (0<=q[0]<len(grid) and 0<=q[1]<len(grid[0])): continue
            cell=grid[q[0]][q[1]]
            if cell=='X': continue
            w=(int(cell) if str(cell).isdigit() else 0 if cell=='E' else 1) if weighted else 1
            nd=d+w
            if nd<dist.get(q,float('inf')):
                dist[q]=nd
                if weighted: heapq.heappush(queue,(nd,q))
                else: queue.append(q)
    return -1

def check_path(grid,start,goal,result,weighted=False):
    expected=oracle(grid,start,goal,weighted)
    assert result['cost']==expected,(result,expected)
    path=result['path']
    if expected==-1: assert path==[]; return []
    assert path[0]==start and path[-1]==goal
    bills=[]; total=0
    for i,p in enumerate(path):
        assert 0<=p[0]<len(grid) and 0<=p[1]<len(grid[0]) and grid[p[0]][p[1]]!='X'
        if i: assert sum(abs(a-b) for a,b in zip(path[i-1],p))==1
        cost=(terrain(grid,p) if weighted else 1) if i else 0
        total+=cost; bills.append(dict(position=p,cell=grid[p[0]][p[1]],entry_cost=cost,cumulative=total))
    assert total==expected
    return bills

def save(name,data):
    OUT.mkdir(exist_ok=True)
    (OUT/name).write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')

def matrix_build():
    matrix={}; checks=[]
    for name,p in POINTS.items():
        matrix[name]={}
        for target,q in POINTS.items():
            r=astar(GAME_MAP,p,q,'manhattan',True)
            check_path(GAME_MAP,p,q,r,True)
            matrix[name][target]=r['cost']
            checks.append(dict(source=name,target=target,dijkstra=oracle(GAME_MAP,p,q,True),**r))
    return matrix,checks

def correctness():
    cases=[('official',MAZE,(0,0),(3,4)),('no_solution',[list('SXE')],(0,0),(0,2)),
           ('same_start_end',[list('S')],(0,0),(0,0)),('multiple_shortest',[list('S.'),list('.E')],(0,0),(1,1))]
    maze=[]
    for name,g,s,e in cases:
        r=astar(g,s,e); check_path(g,s,e,r); maze.append(dict(name=name,grid=g,**r))
    weighted=astar(WEIGHTED,(0,0),(4,4),'safe',True)
    weighted['billing']=check_path(WEIGHTED,(0,0),(4,4),weighted,True)
    assert weighted_cost(WEIGHTED,(0,0),(4,4))==7
    assert weighted_cost([list('SXE')],(0,0),(0,2))==-1
    assert weighted_cost([list('E')],(0,0),(0,0))==0
    trees={name:tree_search(p,r) for name,p,r in [('minimax',False,False),('alpha_beta',True,False),('reverse',True,True)]}
    assert all(r['value']==5 for r in trees.values())
    # Mixed leaf depths verify alternation by level rather than node metadata.
    unbalanced = TreeNode(children=[
        TreeNode(children=[TreeNode(8), TreeNode(children=[TreeNode(2), TreeNode(6)])]),
        TreeNode(children=[TreeNode(4), TreeNode(children=[
            TreeNode(-3), TreeNode(children=[TreeNode(7), TreeNode(1)])])])])
    interface_tests=[]
    for name,root,expected,calculation in [
        ('official_json',tree_from_json(TREE),5,'max(min(max(3,5),max(6,9)),min(max(6,9),max(1,2))) = 5'),
        ('single_leaf',TreeNode(-7),-7,'single leaf = -7'),
        ('unbalanced',unbalanced,6,'max(min(8,max(2,6)),min(4,max(-3,min(7,1)))) = max(6,1) = 6')]:
        mm=tree_search(root=root)
        ab=tree_search(prune=True,root=root)
        public_value=alpha_beta(root)
        assert public_value==mm['value']==ab['value']==expected,(name,public_value,mm,ab)
        interface_tests.append(dict(name=name,expected=expected,hand_calculation=calculation,
                                    public_value=public_value,minimax=mm,alpha_beta=ab))
    save('tree_interface_tests.json',dict(status='passed',cases=interface_tests))
    matrix,checks=matrix_build(); games={}
    for depth,target,value in [(2,'T2',-1),(4,'T3',3)]:
        games[str(depth)]={}
        for prune in [False,True]:
            r=game_search(matrix,depth,prune)
            games[str(depth)]['alpha_beta' if prune else 'minimax']=r
            assert (r['best_target'],r['value'])==(target,value),r
    save('official_tests.json',dict(status='passed',maze=maze,weighted=weighted,weighted_edge_cases='passed',tree_json=TREE,trees=trees,games=games))
    save('distance_matrix.json',dict(labels=list(POINTS),grid=GAME_MAP,values=VALUES,matrix=matrix,checks=checks))
    OUT.mkdir(exist_ok=True)
    with (OUT/'distance_matrix.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.writer(f); w.writerow(['from/to',*POINTS]); w.writerows([[k,*row.values()] for k,row in matrix.items()])
    return matrix

def comparisons(matrix):
    open_grid=[list('.'*15) for _ in range(15)]
    detour=[list('.'*15) for _ in range(15)]
    for r in range(14): detour[r][7]='X'
    mazes=[]
    for name,g,s,e in [('official',MAZE,(0,0),(3,4)),('open',open_grid,(0,0),(14,14)),('detour',detour,(0,0),(0,14))]:
        for h,tie in [('zero','fifo'),('manhattan','fifo')]+([('manhattan','larger_g'),('zero','larger_g')] if name=='open' else []):
            r=astar(g,s,e,h,False,tie); check_path(g,s,e,r)
            mazes.append(dict(map=name,grid=g,heuristic=h,tie=tie,**r))
    save('astar_comparison.json',dict(expansion_definition='有效出队计一次扩展，包含终点；过期条目不计；邻居上右下左；FIFO同分按入队序号；larger_g同分优先较大g再按序号',runs=mazes))
    weighted=[]
    # A zero-cost traversable non-goal makes raw Manhattan substantially overestimate.
    counterexample=[['S',2,'E'],[0,0,0]]
    for name,g,s,e in [('official',WEIGHTED,(0,0),(4,4)),('zero_cost_counterexample',counterexample,(0,0),(0,2))]:
        exact=oracle(g,s,e,True)
        m=min(terrain(g,(r,c)) for r in range(len(g)) for c in range(len(g[0])) if g[r][c]!='X' and (r,c)!=e)
        for h in ['zero','manhattan','safe']:
            r=astar(g,s,e,h,True)
            bills=[]; total=0
            for i,p in enumerate(r['path']):
                cost=terrain(g,p) if i else 0; total+=cost
                bills.append(dict(position=p,entry_cost=cost,cumulative=total))
            weighted.append(dict(map=name,grid=g,start=s,goal=e,m=m,heuristic=h,dijkstra=exact,suboptimal=r['cost']>exact,billing=bills,**r))
    save('weighted_comparison.json',dict(runs=weighted,adjacent_witness=dict(position=[4,3],goal=[4,4],true_remaining_cost=0,raw_manhattan=1,safe_heuristic=0),interpretation='官方图已观察到曼哈顿高估，但本次仍返回最优7；零代价格扩展反例实际返回次优2而Dijkstra为0。该反例含0代价，不声称官方正整数地形存在次优反例。'))
    games={str(h):{name:game_search(matrix,h,p,o) for name,p,o in [('minimax',False,'original'),('alpha_beta',True,'original'),('reverse',True,'reverse'),('gain',True,'gain')]} for h in [2,4]}
    for runs in games.values(): assert len({r['value'] for r in runs.values()})==1
    save('pruning_comparison.json',dict(tree={n:tree_search(p,r) for n,p,r in [('minimax',False,False),('alpha_beta',True,False),('reverse',True,True)]},game=games,definitions=dict(nodes='每次递归进入，包括根与叶',leaf_evaluations='深度耗尽或无剩余宝藏时评价当前U',cutoffs='alpha>=beta且确有尚未访问的后续子分支时计一次',gain_order='当前玩家 value-distance 降序，稳定同分按T1到T4；不调用Minimax排序',time='perf_counter单次测量，仅辅助')))
    diagnostics(matrix)
    return mazes,games

def diagnostics(matrix):
    diagnostic={}
    for h in [2,4]:
        diagnostic[str(h)]={}
        for target in VALUES:
            ns,event=transition(State(depth=h),target,matrix)
            result=game_search(matrix,state=ns,prune=False)
            diagnostic[str(h)][target]=dict(exact_value=result['value'],line=[event]+result['line'])
    save('first_move_diagnostics.json',dict(tie_rule='原顺序T1,T2,T3,T4；严格改善才更新，故并列取第一个。每个首步独立完整Minimax；计数不混入效率实验。',branches=diagnostic))
    lines=['首步精确值与行动轨迹（独立完整 Minimax）','', '|首步|H=2|H=4|','|---|---:|---:|']
    for t in VALUES: lines.append(f"|{t}|{diagnostic['2'][t]['exact_value']}|{diagnostic['4'][t]['exact_value']}|")
    for h,branches in diagnostic.items():
        for t,r in branches.items():
            lines.extend(['',f'H={h}，首步 {t}，精确值 {r["exact_value"]}','',
                          '|玩家|起点|目标|移动代价|宝藏价值|净收益|累计U|','|---|---|---|---:|---:|---:|---:|'])
            for e in r['line']: lines.append('|'+ '|'.join(str(e[k]) for k in ['player','origin','target','movement_cost','value','gain','cumulative_u'])+'|')
    lines.extend(['','原顺序 T1→T2→T3→T4，严格改善才更新；并列取第一个。',
        'H=2：T2路线 A→T2 获2、B→T3 获3，U=-1；T3路线 A→T3 获-1、B→T1 获1，U=-2。',
        'H=4：T2路线继续 A:T2→T1 获3、B:T3→T4 获1，最终U=1。',
        'T3路线继续 A:T3→T2 获0、B:T1→T4 获-5，最终U=3。具体反转来自第四回合：B从T1去T4成本12，被迫获得负收益-5；T2路线中B从T3去T4只花6，仍获1。后两回合分别使U增加2与5，足以反转前两回合的差距。'])
    (OUT/'diagnostic_notes.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    return diagnostic

def main():
    global OUT
    parser=argparse.ArgumentParser()
    parser.add_argument('mode',choices=['test','all','diagnose'])
    parser.add_argument('--output-dir',type=Path,default=Path('results'),
                        help='Result directory (default: results)')
    args=parser.parse_args()
    OUT=args.output_dir
    if args.mode=='diagnose':
        matrix,_=matrix_build(); diagnostics(matrix)
    else:
        matrix=correctness()
        if args.mode=='all': comparisons(matrix)
    command=f'.\\.venv\\Scripts\\python.exe experiments.py {args.mode}'
    if OUT != Path('results'):
        command += f' --output-dir "{OUT}"'
    save('execution_'+args.mode+'.json',dict(command=command,status='passed',python=sys.version,executable=sys.executable,platform=platform.platform()))
    print(f'{args.mode}: PASS; '+('independent exact first-move Minimax diagnostics saved' if args.mode=='diagnose' else 'official cases, edge cases, independent BFS/Dijkstra, 36 matrix entries, tree/game assertions passed'))

if __name__=='__main__': main()
