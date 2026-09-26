"""Lane graph (traffic rules) shared by the swarm agents and the mission generator.

Loads the route_server geojson written by OpenAMR's sim/scripts/generate_route_graph.py: nodes carry
metadata {name, kind}, edges are directed (startid -> endid). Pure Python, no ROS, so it is unit-testable.
"""
import heapq
import json
import math


class LaneGraph:
    def __init__(self, geojson_path):
        with open(geojson_path) as f:
            features = json.load(f)['features']
        self.pos, self.name, self.kind, self.by_name = {}, {}, {}, {}
        self.succ, self.edge_kind, self.edge_id = {}, {}, {}
        self._dist_from = {}
        for ft in features:
            p = ft['properties']
            if ft['geometry']['type'] == 'Point':
                nid = p['id']
                self.pos[nid] = tuple(ft['geometry']['coordinates'])
                meta = p.get('metadata', {})
                self.name[nid] = meta.get('name', str(nid))
                self.kind[nid] = meta.get('kind', '')
                self.by_name[self.name[nid]] = nid
                self.succ.setdefault(nid, [])
        for ft in features:
            p = ft['properties']
            if ft['geometry']['type'] != 'Point':
                self.succ.setdefault(p['startid'], []).append(p['endid'])
                self.edge_kind[(p['startid'], p['endid'])] = p.get('metadata', {}).get('kind', 'street')
                self.edge_id[(p['startid'], p['endid'])] = p['id']

    def id(self, name_or_id):
        return name_or_id if isinstance(name_or_id, int) else self.by_name[name_or_id]

    def dist(self, a, b):
        (x0, y0), (x1, y1) = self.pos[a], self.pos[b]
        return math.hypot(x1 - x0, y1 - y0)

    def nearest(self, x, y, kinds=None):
        cands = [n for n in self.pos if kinds is None or self.kind[n] in kinds]
        return min(cands, key=lambda n: math.hypot(self.pos[n][0] - x, self.pos[n][1] - y))

    def heading(self, a, b):
        (x0, y0), (x1, y1) = self.pos[a], self.pos[b]
        return math.atan2(y1 - y0, x1 - x0)

    def route_length(self, nodes):
        return sum(self.dist(a, b) for a, b in zip(nodes, nodes[1:]))

    def of_kind(self, kind):
        return sorted(n for n, k in self.kind.items() if k == kind)

    def travel(self, a, b):
        """Length of the shortest directed route a -> b along the lanes (inf if unreachable). Local Dijkstra,
        cached per source: cheap estimates (energy budgets, charger choice) without a route_server round trip."""
        if a not in self._dist_from:
            d, pq = {a: 0.0}, [(0.0, a)]
            while pq:
                du, u = heapq.heappop(pq)
                if du > d[u]:
                    continue
                for v in self.succ.get(u, []):
                    dv = du + self.dist(u, v)
                    if dv < d.get(v, math.inf):
                        d[v] = dv
                        heapq.heappush(pq, (dv, v))
            self._dist_from[a] = d
        return self._dist_from[a].get(b, math.inf)

    def network(self, blocked):
        """The lane network left when `blocked` nodes (e.g. where a silent robot stands) are removed:
        (main, out) where main = the largest strongly connected set (every node in it can reach and be reached from
        every other) and out = the nodes that can still drive into main."""
        blocked = set(blocked)
        nodes = [n for n in self.pos if n not in blocked]
        succ = {n: [m for m in self.succ.get(n, []) if m not in blocked] for n in nodes}
        pred = {n: [] for n in nodes}
        for n in nodes:
            for m in succ[n]:
                pred[m].append(n)
        # Kosaraju, iterative: finish order on succ, then components on pred
        order, seen = [], set()
        for r in nodes:
            if r in seen:
                continue
            seen.add(r)
            stack = [(r, iter(succ[r]))]
            while stack:
                n, it = stack[-1]
                nxt = next((m for m in it if m not in seen), None)
                if nxt is None:
                    stack.pop(); order.append(n)
                else:
                    seen.add(nxt); stack.append((nxt, iter(succ[nxt])))
        comp, main = {}, set()
        for r in reversed(order):
            if r in comp:
                continue
            cur, stack = {r}, [r]
            comp[r] = r
            while stack:
                for m in pred[stack.pop()]:
                    if m not in comp:
                        comp[m] = r; cur.add(m); stack.append(m)
            if len(cur) > len(main):
                main = cur
        out, stack = set(main), list(main)
        while stack:
            for m in pred[stack.pop()]:
                if m not in out:
                    out.add(m); stack.append(m)
        return main, out

    def trapped(self, blocked):
        """Nodes (not blocked) from which the lane network can no longer be reached: on one-way lanes a stopped robot
        turns the stretch behind it into a dead end — a robot in there can't get out."""
        if not blocked:
            return set()
        _, out = self.network(blocked)
        return {n for n in self.pos if n not in blocked and n not in out}

    def cut_off(self, blocked):
        """Nodes (not blocked) outside the lane network: can't be driven out of (trapped) or can't be driven into
        (e.g. an aisle whose only entrance a silent robot blocks). Nobody should be sent there."""
        if not blocked:
            return set()
        main, _ = self.network(blocked)
        return {n for n in self.pos if n not in blocked and n not in main}
