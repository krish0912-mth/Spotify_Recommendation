#include <bits/stdc++.h>
using namespace std;

const long double EPS = 1e-9;

struct Point {
    long double x, y;
};

long double distSq(const Point &a, const Point &b) {
    long double dx = a.x - b.x;
    long double dy = a.y - b.y;
    return dx*dx + dy*dy;
}

long double dist(const Point &a, const Point &b) {
    return sqrt(distSq(a,b));
}

long double cross(const Point &a, const Point &b, const Point &c) {
    // cross of (b-a) x (c-a)
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

long double dot(const Point &a, const Point &b, const Point &c) {
    // (b-a) . (c-a)
    return (b.x - a.x) * (c.x - a.x) + (b.y - a.y) * (c.y - a.y);
}

long double distSqPointSegment(const Point &p, const Point &a, const Point &b) {
    long double ab2 = distSq(a,b);
    if (ab2 < EPS) return distSq(p, a);
    long double t = dot(a,b,p) / ab2;
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    Point proj = { a.x + t*(b.x - a.x), a.y + t*(b.y - a.y) };
    return distSq(p, proj);
}

bool onSegment(const Point &a, const Point &b, const Point &p) {
    if (min(a.x,b.x) - EPS <= p.x && p.x <= max(a.x,b.x) + EPS &&
        min(a.y,b.y) - EPS <= p.y && p.y <= max(a.y,b.y) + EPS)
        return fabsl(cross(a,b,p)) < EPS;
    return false;
}

int orientation(const Point &a, const Point &b, const Point &c) {
    long double val = cross(a,b,c);
    if (fabsl(val) < EPS) return 0; // collinear
    return (val > 0) ? 1 : 2; // 1 -> ccw, 2 -> cw
}

bool segmentsIntersect(const Point &p1, const Point &q1, const Point &p2, const Point &q2) {
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4) return true;

    // special collinear cases (endpoints touching or overlapping)
    if (o1 == 0 && onSegment(p1, q1, p2)) return true;
    if (o2 == 0 && onSegment(p1, q1, q2)) return true;
    if (o3 == 0 && onSegment(p2, q2, p1)) return true;
    if (o4 == 0 && onSegment(p2, q2, q1)) return true;

    return false;
}

int S;
struct Building { Point center; long double r; };
struct Vehicle { Point center; long double r; };

Vehicle vehicle;
Point destinationPoint;
int N;
vector<Building> buildings;
vector<pair<Point,Point>> tax_segments;

bool is_pos_valid(const Point &p, long double r) {
    if (p.x < r - EPS || p.x > S - r + EPS || p.y < r - EPS || p.y > S - r + EPS) return false;
    for (const auto &b : buildings) {
        if (distSq(p, b.center) < (r + b.r) * (r + b.r) - EPS) return false;
    }
    return true;
}

// returns -1 if path is invalid (hits building or outside boundary)
// otherwise returns number of tax-line crossings (>= 0) as cost
int path_cost(const Point &p1, const Point &p2) {
    // boundary check: entire segment must stay within [r, S-r] box's projection
    if (min(p1.x, p2.x) < vehicle.r - EPS || max(p1.x, p2.x) > S - vehicle.r + EPS ||
        min(p1.y, p2.y) < vehicle.r - EPS || max(p1.y, p2.y) > S - vehicle.r + EPS)
        return -1;

    // building clearance
    for (const auto &b : buildings) {
        long double d2 = distSqPointSegment(b.center, p1, p2);
        long double R = vehicle.r + b.r;
        if (d2 < R*R - EPS) return -1;
    }

    // count tax crossings
    int cnt = 0;
    for (const auto &seg : tax_segments) {
        if (segmentsIntersect(p1, p2, seg.first, seg.second)) cnt++;
    }
    return cnt;
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    cin >> S;
    cin >> vehicle.center.x >> vehicle.center.y >> vehicle.r;
    cin >> destinationPoint.x >> destinationPoint.y;
    cin >> N;
    buildings.resize(N);
    for (int i = 0; i < N; ++i) {
        cin >> buildings[i].center.x >> buildings[i].center.y >> buildings[i].r;
    }
    int T;
    cin >> T;
    for (int i = 0; i < T; ++i) {
        int u, v;
        cin >> u >> v;

        // Robustly handle either 0-based or 1-based indices per pair
        int uu = -1, vv = -1;
        bool looks_one_based = (1 <= u && u <= N && 1 <= v && v <= N);
        bool looks_zero_based = (0 <= u && u < N && 0 <= v && v < N);
        if (looks_one_based && !looks_zero_based) {
            uu = u - 1;
            vv = v - 1;
        } else if (looks_zero_based && !looks_one_based) {
            uu = u;
            vv = v;
        } else if (looks_one_based && looks_zero_based) {
            // Ambiguous (only possible when N==1). Prefer 0-based as it keeps 0 valid.
            uu = u;
            vv = v;
        } else {
            // Out of range; skip
            continue;
        }

        if (uu >= 0 && uu < N && vv >= 0 && vv < N) {
            tax_segments.push_back({ buildings[uu].center, buildings[vv].center });
        }
    }

    // positions valid?
    if (!is_pos_valid(vehicle.center, vehicle.r) || !is_pos_valid(destinationPoint, vehicle.r)) {
        cout << "impossible\n";
        return 0;
    }

    vector<Point> points;
    for (int i = 0; i < N; ++i) points.push_back(buildings[i].center);
    points.push_back(vehicle.center);      // index N
    points.push_back(destinationPoint);    // index N+1

    // Heuristic checkpoints between buildings (corridor midpoints between inflated circles)
    for (int i = 0; i < N; ++i) {
        for (int j = i+1; j < N; ++j) {
            Point c1 = buildings[i].center;
            Point c2 = buildings[j].center;
            long double r1 = buildings[i].r + vehicle.r;
            long double r2 = buildings[j].r + vehicle.r;
            long double d = dist(c1, c2);
            if (d > r1 + r2 + EPS) {
                Point dir = { c2.x - c1.x, c2.y - c1.y };
                Point p1_on = { c1.x + dir.x * (r1 / d), c1.y + dir.y * (r1 / d) };
                Point p2_on = { c2.x - dir.x * (r2 / d), c2.y - dir.y * (r2 / d) };
                Point mid = { (p1_on.x + p2_on.x) / 2.0L, (p1_on.y + p2_on.y) / 2.0L };
                if (is_pos_valid(mid, vehicle.r)) points.push_back(mid);
            }
        }
    }

    int num = (int)points.size();
    int src = N;
    int dest = N + 1;

    vector<vector<pair<int,int>>> adj(num);
    for (int i = 0; i < num; ++i) {
        for (int j = i+1; j < num; ++j) {
            int w = path_cost(points[i], points[j]);
            if (w >= 0) {
                adj[i].push_back({j, w});
                adj[j].push_back({i, w});
            }
        }
    }

    // Dijkstra (weights >= 0)
    const int INF_INT = 1e9;
    vector<int> distv(num, INF_INT);
    priority_queue<pair<int,int>, vector<pair<int,int>>, greater<pair<int,int>>> pq;
    distv[src] = 0;
    pq.push({0, src});

    while (!pq.empty()) {
        auto [d,u] = pq.top(); pq.pop();
        if (d != distv[u]) continue;
        if (u == dest) break;
        for (auto &pr : adj[u]) {
            int v = pr.first, w = pr.second;
            if (distv[v] > d + w) {
                distv[v] = d + w;
                pq.push({distv[v], v});
            }
        }
    }

    if (distv[dest] == INF_INT) {
        cout << "impossible\n";
    } else {
        cout << distv[dest] << '\n';
    }

    return 0;
}

