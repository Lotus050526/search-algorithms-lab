"""Search algorithms. Coordinates are (row, column); no PDF/runtime dependencies."""
import heapq
import itertools
from dataclasses import dataclass, asdict
from time import perf_counter

DIRS = ((-1, 0), (0, 1), (1, 0), (0, -1))

def neighbors(grid, p):
    for dr, dc in DIRS:
        q = p[0] + dr, p[1] + dc
        if 0 <= q[0] < len(grid) and 0 <= q[1] < len(grid[0]) and grid[q[0]][q[1]] != 'X':
            yield q

def terrain(grid, p):
    v = grid[p[0]][p[1]]
    return int(v) if str(v).isdigit() else (0 if v == 'E' else 1)

def astar(grid, start, goal, heuristic='manhattan', weighted=False, tie='fifo'):
    """Expansion = valid queue pop, including goal. Reopen improved states."""
    start, goal = tuple(start), tuple(goal)
    m = min((terrain(grid, (r,c)) for r in range(len(grid)) for c in range(len(grid[0]))
             if grid[r][c] != 'X' and (r,c) != goal), default=0)
    def h(p):
        d = abs(p[0]-goal[0])+abs(p[1]-goal[1])
        return 0 if heuristic == 'zero' else m*max(d-1,0) if heuristic == 'safe' else d
    seq = itertools.count()
    queue, dist, parent, expanded = [], {start:0}, {}, []
    def push(p,g):
        heapq.heappush(queue,(g+h(p), -g if tie == 'larger_g' else 0, next(seq), g,p))
    push(start,0)
    while queue:
        _,_,_,g,p = heapq.heappop(queue)
        if g != dist[p]:
            continue
        expanded.append(p)
        if p == goal:
            path = [p]
            while p in parent:
                p = parent[p]; path.append(p)
            path.reverse()
            return dict(cost=g,path=path,expanded=len(expanded),expansion_trace=expanded)
        for q in neighbors(grid,p):
            ng = g + (terrain(grid,q) if weighted else 1)
            if ng < dist.get(q,float('inf')):
                dist[q]=ng; parent[q]=p; push(q,ng)
    return dict(cost=-1,path=[],expanded=len(expanded),expansion_trace=expanded)

def maze_path(grid,start,goal):
    return astar(grid,start,goal)['path']

def weighted_cost(grid,start,goal):
    return astar(grid,start,goal,'safe',True)['cost']

TREE = {'tree_structure': {'is_maximizing_player':True,'children':[
    {'is_maximizing_player':False,'children':[
        {'is_maximizing_player':True,'children':[{'value':3},{'value':5}]},
        {'is_maximizing_player':True,'children':[{'value':6},{'value':9}]}]},
    {'is_maximizing_player':False,'children':[
        {'is_maximizing_player':True,'children':[{'value':6},{'value':9}]},
        {'is_maximizing_player':True,'children':[{'value':1},{'value':2}]}]}]}}

@dataclass
class TreeNode:
    """A leaf has value; an internal node has ordered children."""
    value: float | None = None
    children: list | None = None

    def __post_init__(self):
        self.children = list(self.children) if self.children is not None else []
        if not self.children and self.value is None:
            raise ValueError('A leaf must have a value')
        if self.children and self.value is not None:
            raise ValueError('An internal node cannot also have a leaf value')


def tree_from_json(node):
    """Convert the official JSON; turns follow depth, starting with MAX."""
    if 'tree_structure' in node:
        node = node['tree_structure']
    if 'value' in node:
        return TreeNode(value=node['value'])
    return TreeNode(children=[tree_from_json(child) for child in node['children']])


def alpha_beta(root):
    """Return the caller's tree value, MAX at root, alternating left to right."""
    return tree_search(prune=True, root=root)['value']


def tree_search(prune=False, reverse=False, root=None):
    """Generic Minimax/Alpha-beta with experiment counters and traces."""
    if root is None:
        root = tree_from_json(TREE)
    stats = dict(nodes=0,leaf_evaluations=0,cutoffs=0,leaves=[],pruning_events=[])
    def visit(node,path,a,b,maximizing):
        stats['nodes']+=1
        if not node.children:
            stats['leaf_evaluations']+=1
            stats['leaves'].append(dict(path=path,value=node.value))
            return node.value
        value = -float('inf') if maximizing else float('inf')
        children=list(enumerate(node.children))
        if reverse: children.reverse()
        for k,(i,child) in enumerate(children):
            v=visit(child,path+[i],a,b,not maximizing)
            value=max(value,v) if maximizing else min(value,v)
            if maximizing: a=max(a,value)
            else: b=min(b,value)
            if prune and a>=b and k+1<len(children):
                stats['cutoffs']+=1
                stats['pruning_events'].append(dict(path=path,alpha=a,beta=b,skipped=[j for j,_ in children[k+1:]]))
                break
        return value
    t=perf_counter(); value=visit(root,[],-float('inf'),float('inf'),True)
    return dict(value=value,seconds=perf_counter()-t,**stats)

MAZE = [list(s) for s in ['S..X.','.X...','.X.X.','...XE','XX...']]
WEIGHTED = [['S',1,1,8,1],[1,8,3,1,1],[1,8,1,8,1],[1,1,1,8,1],[8,8,1,1,'E']]
GAME_MAP = [['A',1,1,'X',2,1,'T1'],[1,'X',6,'X',1,'X',1],[1,1,'T2',1,1,1,1],
            ['X',1,'X',8,'X',1,'X'],[1,1,1,1,'T3',1,1],[1,'X',9,'X',1,'X',1],['T4',1,1,'X',1,1,'B']]
POINTS = dict(A=(0,0),B=(6,6),T1=(0,6),T2=(2,2),T3=(4,4),T4=(6,0))
VALUES = dict(T1=9,T2=6,T3=7,T4=7)

@dataclass(frozen=True)
class State:
    a: str = 'A'
    b: str = 'B'
    remaining: tuple = tuple(VALUES)
    score_a: int = 0
    score_b: int = 0
    player: str = 'A'
    depth: int = 4

def transition(s,target,matrix):
    origin=s.a if s.player=='A' else s.b
    cost=matrix[origin][target]; gain=VALUES[target]-cost
    ns=State(target if s.player=='A' else s.a,target if s.player=='B' else s.b,
             tuple(t for t in s.remaining if t!=target),s.score_a+(gain if s.player=='A' else 0),
             s.score_b+(gain if s.player=='B' else 0),'B' if s.player=='A' else 'A',s.depth-1)
    event=dict(player=s.player,origin=origin,target=target,movement_cost=cost,value=VALUES[target],gain=gain,
               cumulative_u=ns.score_a-ns.score_b,state_after=asdict(ns))
    return ns,event

def game_search(matrix,depth=4,prune=False,order='original',state=None):
    stats=dict(nodes=0,leaf_evaluations=0,cutoffs=0,pruning_events=[])
    def visit(s,a,b):
        stats['nodes']+=1
        if s.depth==0 or not s.remaining:
            stats['leaf_evaluations']+=1
            return s.score_a-s.score_b,[]
        targets=list(s.remaining)
        if order=='reverse': targets.reverse()
        if order=='gain':
            origin=s.a if s.player=='A' else s.b
            targets.sort(key=lambda t: -(VALUES[t]-matrix[origin][t]))
        best=-float('inf') if s.player=='A' else float('inf'); line=[]
        for i,t in enumerate(targets):
            ns,event=transition(s,t,matrix); v,tail=visit(ns,a,b)
            if (s.player=='A' and v>best) or (s.player=='B' and v<best): best,line=v,[event]+tail
            if s.player=='A': a=max(a,best)
            else: b=min(b,best)
            if prune and a>=b and i+1<len(targets):
                stats['cutoffs']+=1
                stats['pruning_events'].append(dict(state=asdict(s),alpha=a,beta=b,skipped=targets[i+1:]))
                break
        return best,line
    t=perf_counter(); value,line=visit(state or State(depth=depth),-float('inf'),float('inf'))
    return dict(value=value,best_target=line[0]['target'] if line else None,line=line,seconds=perf_counter()-t,**stats)
