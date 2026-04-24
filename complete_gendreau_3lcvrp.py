import argparse
import math
import random
import copy
import sys
import os
import csv
import time
from typing import List, Tuple, Dict, Optional

# ==============================================================================
# 1. DATA STRUCTURES & CONFIGURATION
# ==============================================================================

class GlobalState:
    VEHICLE_W = 25   
    VEHICLE_H = 30
    VEHICLE_L = 60
    VEHICLE_CAPACITY = 200
    SUPPORT_ALPHA = 0.75

class Item:
    def __init__(self, id: int, client_id: int, type_id: str, w: int, h: int, l: int, weight: float, fragile: bool):
        self.id = id
        self.client_id = client_id
        self.type_id = type_id
        # Dimensions: w=x-axis, h=y-axis, l=z-axis (length)
        self.orig_w, self.orig_h, self.orig_l = w, h, l
        self.weight = weight
        self.fragile = fragile
        self.volume = w * h * l
        
        # State coordinates (Bottom-Left-Back corner)
        self.x = 0
        self.y = 0
        self.z = 0
        # Current dimensions (after possible rotation)
        self.w = w
        self.h = h
        self.l = l

class Client:
    def __init__(self, id: int, x: float, y: float, items: List[Item]):
        self.id = id
        self.x = x
        self.y = y
        self.items = items
        self.total_weight = sum(i.weight for i in items)
        self.total_volume = sum(i.volume for i in items)

class Vehicle:
    def __init__(self, id: int):
        self.id = id
        self.route: List[Client] = [] 
        self.packed_items: List[Item] = [] # 3D coordinates of items
        self.total_weight = 0.0
        self.route_length = 0.0
        self.loading_length = 0.0 
        self.is_feasible = True

# ==============================================================================
# 2. 3D PACKING ENGINE
# ==============================================================================

class PackingEngine:
    @staticmethod
    def check_feasibility(item: Item, x: int, y: int, z: int, w: int, l: int, h: int, 
                          packed_items: List[Item], client_visit_order: Dict[int, int]) -> bool:
        # 1. Boundary Check
        if x + w > GlobalState.VEHICLE_W or y + h > GlobalState.VEHICLE_H or z + l > GlobalState.VEHICLE_L:
            return False

        # 2. Overlap Check
        item_rect = (x, y, z, x+w, y+h, z+l)
        for other in packed_items:
            other_rect = (other.x, other.y, other.z, other.x+other.w, other.y+other.h, other.z+other.l)
            if PackingEngine._intersect(item_rect, other_rect):
                return False

        # 3. Fragility Check
        if not item.fragile:
            for other in packed_items:
                if other.fragile:
                    if (PackingEngine._rect_overlap(x, z, w, l, other.x, other.z, other.w, other.l) and 
                        y == other.y + other.h):
                        return False

        # 4. Support Area Check
        if y > 0:
            supported_area = 0.0
            base_area = w * l
            for other in packed_items:
                if other.y + other.h == y:
                    overlap = PackingEngine._rect_intersection_area(
                        x, z, w, l, other.x, other.z, other.w, other.l
                    )
                    supported_area += overlap
            if supported_area < (GlobalState.SUPPORT_ALPHA * base_area):
                return False

        # 5. LIFO Policy
        for other in packed_items:
            if other.client_id != item.client_id:
                xy_overlap = PackingEngine._rect_overlap(x, y, w, h, other.x, other.y, other.w, other.h)
                if xy_overlap:
                    if item.z < other.z:
                         return False
        return True

    @staticmethod
    def solve_single_vehicle(vehicle: Vehicle, persist_results: bool = False) -> float:
        if not vehicle.route:
            if persist_results: vehicle.packed_items = []
            return 0.0
        
        total_vol = sum(c.total_volume for c in vehicle.route)
        max_vol = GlobalState.VEHICLE_W * GlobalState.VEHICLE_H * GlobalState.VEHICLE_L
        
        if vehicle.total_weight > GlobalState.VEHICLE_CAPACITY or total_vol > max_vol:
            if persist_results: vehicle.packed_items = [] 
            return GlobalState.VEHICLE_L * 2

        packing_sequence = []
        visit_order_map = {client.id: idx for idx, client in enumerate(vehicle.route)}
        
        for client in reversed(vehicle.route):
            c_items = sorted(client.items, key=lambda x: (x.fragile, -x.volume))
            packing_sequence.extend(c_items)

        length_bl, packed_items_res = PackingEngine._run_heuristic_bl(packing_sequence, visit_order_map)
        
        if persist_results:
            vehicle.packed_items = packed_items_res
            
        return length_bl

    @staticmethod
    def _run_heuristic_bl(items: List[Item], visit_order_map: Dict[int, int]) -> Tuple[float, List[Item]]:
        packed_items: List[Item] = []
        max_z_used = 0.0

        for item in items:
            # Corner Point Strategy
            candidate_xs = sorted(list(set([0] + [p.x + p.w for p in packed_items if p.x + p.w <= GlobalState.VEHICLE_W])))
            candidate_ys = sorted(list(set([0] + [p.y + p.h for p in packed_items if p.y + p.h <= GlobalState.VEHICLE_H])))
            candidate_zs = sorted(list(set([0] + [p.z + p.l for p in packed_items if p.z + p.l <= GlobalState.VEHICLE_L])))

            found = False
            for x in candidate_xs:
                for z in candidate_zs:
                    for y in candidate_ys:
                        orientations = [
                            (item.orig_w, item.orig_l, item.orig_h),
                            (item.orig_l, item.orig_w, item.orig_h)
                        ]
                        for (w, l, h) in orientations:
                            if PackingEngine.check_feasibility(
                                item, x, y, z, w, l, h, packed_items, visit_order_map
                            ):
                                packed_item = copy.copy(item)
                                packed_item.x, packed_item.y, packed_item.z = x, y, z
                                packed_item.w, packed_item.h, packed_item.l = w, h, l
                                packed_items.append(packed_item)
                                max_z_used = max(max_z_used, z + l)
                                found = True
                                break 
                        if found: break
                    if found: break
                if found: break
            
            if not found:
                return GlobalState.VEHICLE_L * 2, []

        return max_z_used, packed_items

    @staticmethod
    def _intersect(r1, r2):
        return (r1[0] < r2[3] and r1[3] > r2[0] and
                r1[1] < r2[4] and r1[4] > r2[1] and
                r1[2] < r2[5] and r1[5] > r2[2])

    @staticmethod
    def _rect_overlap(x1, y1, w1, h1, x2, y2, w2, h2):
        return (x1 < x2 + w2 and x1 + w1 > x2 and
                y1 < y2 + h2 and y1 + h1 > y2)

    @staticmethod
    def _rect_intersection_area(x1, y1, w1, h1, x2, y2, w2, h2):
        ix = max(0, min(x1+w1, x2+w2) - max(x1, x2))
        iy = max(0, min(y1+h1, y2+h2) - max(y1, y2))
        return ix * iy

# ==============================================================================
# 3. ROUTING ENGINE
# ==============================================================================

class RoutingEngine:
    @staticmethod
    def dist(c1: Client, c2: Client, dist_matrix: List[List[float]]) -> float:
        # return math.sqrt((c1.x - c2.x)**2 + (c1.y - c2.y)**2)
        return dist_matrix[c1.id][c2.id]

    @staticmethod
    def calculate_route_length(route: List[Client], depot: Client, dist_matrix: List[List[float]]) -> float:
        if not route: return 0.0
        d = RoutingEngine.dist(depot, route[0], dist_matrix)
        for i in range(len(route)-1):
            d += RoutingEngine.dist(route[i], route[i+1], dist_matrix)
        d += RoutingEngine.dist(route[-1], depot, dist_matrix)
        return d

    @staticmethod
    def optimize_route(route: List[Client], depot: Client, dist_matrix: List[List[float]]) -> Tuple[List[Client], float]:
        if len(route) <= 2:
            return route, RoutingEngine.calculate_route_length(route, depot, dist_matrix)

        improved = True
        best_route = list(route)
        best_len = RoutingEngine.calculate_route_length(best_route, depot, dist_matrix)
        
        while improved:
            improved = False
            for i in range(len(best_route) - 1):
                for j in range(i + 1, len(best_route)):
                    if j - i == 1: continue
                    new_route = best_route[:]
                    new_route[i:j] = reversed(best_route[i:j])
                    l = RoutingEngine.calculate_route_length(new_route, depot, dist_matrix)
                    if l < best_len - 0.001:
                        best_route = new_route
                        best_len = l
                        improved = True
                        break
                if improved: break
        
        return best_route, best_len

# ==============================================================================
# 4. TABU SEARCH ALGORITHM (With Time Limit)
# ==============================================================================

class TabuSearch3L:
    def __init__(self, depot: Client, clients: List[Client], num_vehicles: int, dist_matrix: List[List[float]]):
        self.depot = depot
        self.clients = clients
        self.min_vehicles = num_vehicles 
        self.vehicles = []
        self.dist_matrix = dist_matrix
        
        # avg_dist = sum(RoutingEngine.dist(depot, c) for c in clients) / len(clients) if clients else 10
        avg_dist = sum(c for c in dist_matrix[0]) / len(clients) if clients else 10
        self.alpha = 20 * avg_dist / GlobalState.VEHICLE_CAPACITY 
        self.beta = 20 * avg_dist / GlobalState.VEHICLE_L         
        self.gamma = math.sqrt(2 * len(clients) * num_vehicles) 
        
        self.tabu_list = {} 
        self.freq_matrix = {} 

    def solve(self, max_iters=100, time_limit=3600):
        """
        Executes Tabu Search until max_iters OR time_limit is reached.
        """
        start_time = time.time()
        
        # 1. Strict Feasible Initialization
        print("Constructing feasible initial solution (checking 3D packing)...")
        solution = self._initial_solution_robust()
        self.num_vehicles = len(solution)
        
        feasible_init = True
        for v in solution:
            if v.route:
                v.loading_length = PackingEngine.solve_single_vehicle(v)
                if v.loading_length > GlobalState.VEHICLE_L:
                    feasible_init = False
        
        best_solution = copy.deepcopy(solution)
        best_cost = self._evaluate_solution_true_cost(best_solution)
        
        if feasible_init:
            print(f"Initial Feasible Cost: {best_cost:.2f} (Vehicles used: {len(solution)})")
        else:
            print(f"Warning: Could not construct fully feasible initial solution. Cost: {best_cost:.2f}")

        # 2. Tabu Search Loop
        for it in range(max_iters):
        # it = -1
        # while True:
            it += 1
            best_neighbor = None
            best_neighbor_score = float('inf')
            move_details = None
            
            candidate_clients = random.sample(self.clients, min(len(self.clients), 20))
            
            for client in candidate_clients:
                source_v_idx = self._find_vehicle_idx(solution, client.id)
                if source_v_idx == -1: continue

                targets = list(range(self.num_vehicles))
                
                for target_v_idx in targets:
                    if source_v_idx == target_v_idx: continue
                    
                    neighbor_sol = copy.deepcopy(solution)
                    
                    try:
                        c_obj = next(c for c in neighbor_sol[source_v_idx].route if c.id == client.id)
                    except StopIteration:
                        continue 

                    neighbor_sol[source_v_idx].route.remove(c_obj)
                    neighbor_sol[source_v_idx].total_weight -= c_obj.total_weight
                    
                    neighbor_sol[target_v_idx].route.append(c_obj)
                    neighbor_sol[target_v_idx].total_weight += c_obj.total_weight
                    
                    for v_idx in [source_v_idx, target_v_idx]:
                        neighbor_sol[v_idx].route, neighbor_sol[v_idx].route_length = \
                            RoutingEngine.optimize_route(neighbor_sol[v_idx].route, self.depot, self.dist_matrix)
                    
                    score = self._calculate_score(neighbor_sol, client.id, target_v_idx)
                    
                    is_tabu = False
                    if (client.id, source_v_idx) in self.tabu_list:
                         if self.tabu_list[(client.id, source_v_idx)] > it:
                             is_tabu = True
                    
                    true_cost_neighbor = self._evaluate_solution_true_cost(neighbor_sol)
                    is_new_best = (true_cost_neighbor < best_cost)
                    
                    if is_new_best or (score < best_neighbor_score and not is_tabu):
                        best_neighbor = neighbor_sol
                        best_neighbor_score = score
                        move_details = (client.id, source_v_idx, target_v_idx)

            if best_neighbor:
                solution = best_neighbor
                client_id, old_v, new_v = move_details
                
                self.tabu_list[(client_id, old_v)] = it + random.randint(5, 10)
                
                key = (client_id, new_v)
                self.freq_matrix[key] = self.freq_matrix.get(key, 0) + 1
                
                total_w_excess = sum(max(0, v.total_weight - GlobalState.VEHICLE_CAPACITY) for v in solution)
                total_l_excess = sum(max(0, v.loading_length - GlobalState.VEHICLE_L) for v in solution)
                
                if total_w_excess > 0: self.alpha *= 1.2 
                else: self.alpha /= 1.1
                
                if total_l_excess > 0: self.beta *= 1.2
                else: self.beta /= 1.1

                true_cost = self._evaluate_solution_true_cost(solution)
                if true_cost < best_cost: 
                    best_solution = copy.deepcopy(solution)
                    best_cost = true_cost
                    print(f"Iter {it}: New Best Feasible Cost = {best_cost:.2f}")

            
            # --- Check Time Limit ---
            elapsed = time.time() - start_time
            if elapsed > time_limit:
                print(f"Stopping: Time limit reached ({elapsed:.2f}s > {time_limit}s)")
                break
            # ------------------------

        return best_solution

    def _initial_solution_robust(self):
        sol = [Vehicle(i) for i in range(self.min_vehicles)]
        unassigned = sorted(self.clients, key=lambda c: (c.total_volume, c.total_weight), reverse=True)
        
        for c in unassigned:
            inserted = False
            best_v_idx = -1
            min_cost_increase = float('inf')
            
            for v_idx, v in enumerate(sol):
                if v.total_weight + c.total_weight > GlobalState.VEHICLE_CAPACITY:
                    continue
                
                v.route.append(c)
                lam = PackingEngine.solve_single_vehicle(v)
                v.route.pop() 
                
                if lam <= GlobalState.VEHICLE_L:
                    current_dist = RoutingEngine.calculate_route_length(v.route, self.depot, self.dist_matrix)
                    v.route.append(c)
                    new_dist = RoutingEngine.calculate_route_length(v.route, self.depot, self.dist_matrix)
                    v.route.pop()
                    
                    increase = new_dist - current_dist
                    if increase < min_cost_increase:
                        min_cost_increase = increase
                        best_v_idx = v_idx

            if best_v_idx != -1:
                v = sol[best_v_idx]
                v.route.append(c)
                v.total_weight += c.total_weight
                v.route, v.route_length = RoutingEngine.optimize_route(v.route, self.depot, self.dist_matrix)
                inserted = True
            
            if not inserted:
                new_v_idx = len(sol)
                new_v = Vehicle(new_v_idx)
                new_v.route.append(c)
                new_v.total_weight += c.total_weight
                new_v.route, new_v.route_length = RoutingEngine.optimize_route(new_v.route, self.depot, self.dist_matrix)
                sol.append(new_v)

        sol = [v for v in sol if v.route]
        return sol

    def _calculate_score(self, vehicles: List[Vehicle], moved_client_id: int, target_v_idx: int) -> float:
        total_len = 0
        w_excess = 0
        l_excess = 0
        for v in vehicles:
            total_len += v.route_length
            w_excess += max(0, v.total_weight - GlobalState.VEHICLE_CAPACITY)
            
            if v.route:
                lam = PackingEngine.solve_single_vehicle(v)
                v.loading_length = lam
                l_excess += max(0, lam - GlobalState.VEHICLE_L)
        
        freq = self.freq_matrix.get((moved_client_id, target_v_idx), 0)
        return total_len + (self.alpha * w_excess) + (self.beta * l_excess) + (self.gamma * freq)

    def _evaluate_solution_true_cost(self, vehicles):
        cost = 0
        for v in vehicles:
            if v.total_weight > GlobalState.VEHICLE_CAPACITY:
                return float('inf')
            
            lam = PackingEngine.solve_single_vehicle(v)
            if lam > GlobalState.VEHICLE_L:
                return float('inf')
            
            cost += v.route_length
        return cost

    def _find_vehicle_idx(self, vehicles, client_id):
        for idx, v in enumerate(vehicles):
            for c in v.route:
                if c.id == client_id: return idx
        return -1

# ==============================================================================
# 5. FILE PARSER & EXPORT
# ==============================================================================

def load_instance_from_file(filepath: str):
    print(f"Reading instance file: {filepath}")
    
    current_section = None
    depot = None
    items_ref = {}
    clients_list = []
    customer_positions = {}
    num_vehicles = 5
    dist_matrix = []
    
    with open(filepath, 'r') as f:
        lines = [line.strip() for line in f if line.strip()]

    i = 0
    while i < len(lines):
        line = lines[i]
        
        if line == "VEHICLE":
            current_section = "VEHICLE"
            i += 1
            continue
        elif line == "CUSTOMERS":
            current_section = "CUSTOMERS"
            i += 2 
            continue
        elif line == "ITEMS":
            current_section = "ITEMS"
            i += 2 
            continue
        elif line == "DEMANDS PER CUSTOMER":
            current_section = "DEMANDS"
            i += 2 
            continue
        elif line == "DISTANCE MATRIX":
            current_section = "DISTANCE MATRIX"
            i += 1
            continue

        if current_section is None:
            parts = line.split()
            if len(parts) >= 2 and parts[0] == "Number_of_Vehicles":
                num_vehicles = int(parts[1])

        elif current_section == "VEHICLE":
            parts = line.split()
            key = parts[0]
            val = float(parts[1])
            if key == "Mass_Capacity": GlobalState.VEHICLE_CAPACITY = val
            elif key == "CargoSpace_Length": GlobalState.VEHICLE_L = int(val)
            elif key == "CargoSpace_Width": GlobalState.VEHICLE_W = int(val)
            elif key == "CargoSpace_Height": GlobalState.VEHICLE_H = int(val)

        elif current_section == "CUSTOMERS":
            parts = line.split()
            cid = int(parts[0])
            cx = float(parts[1])
            cy = float(parts[2])
            if cid == 0:
                depot = Client(0, cx, cy, [])
            else:
                customer_positions[cid] = (cx, cy)
        
        elif current_section == "ITEMS":
            parts = line.split()
            itype = parts[0]
            il = int(parts[1]) 
            iw = int(parts[2]) 
            ih = int(parts[3]) 
            imass = float(parts[4])
            ifragile = int(parts[5]) == 1
            items_ref[itype] = {'w': iw, 'h': ih, 'l': il, 'mass': imass, 'fragile': ifragile}

        elif current_section == "DEMANDS":
            parts = line.split()
            cid = int(parts[0])
            demand_data = parts[1:]
            client_items = []
            item_counter = 0
            for k in range(0, len(demand_data), 2):
                if k+1 >= len(demand_data): break
                t_name = demand_data[k]
                qty = int(demand_data[k+1])
                ref = items_ref.get(t_name)
                if not ref: continue
                for _ in range(qty):
                    unique_id = f"{cid}_{item_counter}"
                    new_item = Item(unique_id, cid, t_name, ref['w'], ref['h'], ref['l'], ref['mass'], ref['fragile'])
                    client_items.append(new_item)
                    item_counter += 1
            if cid in customer_positions:
                cx, cy = customer_positions[cid]
                clients_list.append(Client(cid, cx, cy, client_items))
        
        elif current_section == "DISTANCE MATRIX":
            parts = line.split()
            dist_matrix.append([float(x) for x in parts])

        i += 1
    if len(dist_matrix) == 0:
        all_nodes = [depot] + clients_list
        for i in range(len(all_nodes)):
            dist_matrix.append([])
            for j in range(len(all_nodes)):
                dist_matrix[-1].append(math.sqrt((all_nodes[i].x - all_nodes[j].x)**2 + (all_nodes[i].y - all_nodes[j].y)**2))

    return depot, clients_list, num_vehicles, dist_matrix

def export_results(solution: List[Vehicle], instance_file: str):
    os.makedirs("results", exist_ok=True)
    base_name = os.path.basename(instance_file)
    name_without_ext = os.path.splitext(base_name)[0]
    out_file = os.path.join("results", f"solution_{name_without_ext}.txt")
    
    total_dist = sum(v.route_length for v in solution)
    
    print(f"\nExporting detailed solution to: {out_file}")
    
    with open(out_file, "w") as f:
        f.write(f"Instance: {base_name}\n")
        f.write(f"Total Distance: {total_dist:.2f}\n")
        f.write("=" * 60 + "\n\n")
        
        for v in solution:
            if not v.route: continue
            
            lam = PackingEngine.solve_single_vehicle(v, persist_results=True)
            v.loading_length = lam
            
            f.write(f"VEHICLE {v.id}\n")
            f.write(f"Route: 0 -> {' -> '.join(str(c.id) for c in v.route)} -> 0\n")
            f.write(f"Route Length: {v.route_length:.2f}\n")
            f.write(f"Load Weight: {v.total_weight:.2f} / {GlobalState.VEHICLE_CAPACITY}\n")
            f.write(f"Load Length: {v.loading_length:.2f} / {GlobalState.VEHICLE_L}\n")
            f.write(f"Packed Items Count: {len(v.packed_items)}\n")
            f.write("-" * 20 + " ITEM PLACEMENT " + "-" * 20 + "\n")
            f.write(f"{'Item ID':<10} {'Client':<8} {'X':<5} {'Y':<5} {'Z':<5} {'Dim(WxHxL)':<15} {'Fragile'}\n")
            
            if not v.packed_items:
                 f.write("(NO FEASIBLE PACKING FOUND FOR THIS ROUTE CONFIGURATION)\n")
            else:
                for item in v.packed_items:
                    dim_str = f"{item.w}x{item.h}x{item.l}"
                    frag_str = "YES" if item.fragile else "NO"
                    f.write(f"{item.id:<10} {item.client_id:<8} {item.x:<5} {item.y:<5} {item.z:<5} {dim_str:<15} {frag_str}\n")
            
            f.write("\n")
        f.write("=" * 60 + "\n")

def append_results_to_csv(instance_file: str, solution: List[Vehicle], calc_time: float):
    os.makedirs("results", exist_ok=True)
    csv_file = os.path.join("results", "results.csv")
    file_exists = os.path.isfile(csv_file)
    instance_name = os.path.basename(instance_file)
    
    used_vehicles = [v for v in solution if v.route]
    vehicle_count = len(used_vehicles)
    total_dist = sum(v.route_length for v in used_vehicles)
    
    total_vehicle_vol = vehicle_count * (GlobalState.VEHICLE_W * GlobalState.VEHICLE_H * GlobalState.VEHICLE_L)
    total_item_vol = 0
    for v in used_vehicles:
        for client in v.route:
            total_item_vol += client.total_volume
            
    fill_rate = total_item_vol / total_vehicle_vol if total_vehicle_vol > 0 else 0.0
    
    print(f"Appending summary to: {csv_file}")
    
    with open(csv_file, mode='a', newline='') as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(["instance", "distance", "vehicle_count", "fill_rate", "calculation_time"])
        
        writer.writerow([instance_name, f"{total_dist}", vehicle_count, f"{fill_rate}", f"{calc_time:.4f}"])


def save_solution_txt_v2(
    out_path: str,
    instance_name: str,
    solution: List[Vehicle],
    elapsed_time: float
):
    used_vehicles = [v for v in solution if v.route]
    total_dist = sum(v.route_length for v in used_vehicles)

    with open(out_path, "w") as f:
        f.write(f"Name:\t\t\t\t{instance_name}\n")
        f.write("Problem:\t\t\t3L-CVRP\n")
        f.write(f"Number_of_used_Vehicles:\t{len(used_vehicles)}\n")
        f.write(f"Total_Travel_Distance:\t\t{total_dist}\n")
        f.write(f"Calculation_Time:\t\t{elapsed_time}\n\n")

        for vid, v in enumerate(used_vehicles):
            PackingEngine.solve_single_vehicle(v, persist_results=True)

            customers = [c.id for c in v.route]

            f.write("-" * 40 + "\n")
            f.write(f"Tour_Id:\t\t\t{vid}\n")
            f.write(f"No_of_Customers:\t\t{len(customers)}\n")
            f.write(f"No_of_Items:\t\t\t{len(v.packed_items)}\n")
            f.write(
                "Customer_Sequence:\t\t"
                + " ".join(map(str, customers)) + "\n\n"
            )

            f.write(
                "CustId\tId\tTypeId\tRotated\tx\ty\tz\t"
                "Length\tWidth\tHeight\tmass\tFragility\n"
            )

            for it in v.packed_items:
                f.write(
                    f"{it.client_id}\t"
                    f"{it.id}\t"
                    f"{it.type_id}\t"
                    f"{0}\t"
                    f"{it.x}\t{it.y}\t{it.z}\t"
                    f"{it.l}\t{it.w}\t{it.h}\t"
                    f"{it.weight}\t"
                    f"{int(it.fragile)}\n"
                )

            f.write("\n")


# ==============================================================================
# 6. MAIN EXECUTION
# ==============================================================================

# if __name__ == "__main__":
#     # Default folder is current directory if no argument provided
#     input_path = "."
#     if len(sys.argv) > 1:
#         input_path = sys.argv[1]

#     # Check if input is a directory or a single file
#     files_to_process = []
#     if os.path.isdir(input_path):
#         print(f"Scanning directory: {input_path}")
#         for entry in os.listdir(input_path):
#             if entry.endswith(".txt") and not entry.startswith("solution_"):
#                 if not entry.startswith("Overview"):
#                     files_to_process.append(os.path.join(input_path, entry))
#     elif os.path.isfile(input_path):
#         files_to_process.append(input_path)
#     else:
#         print(f"Error: Path {input_path} not found.")
#         sys.exit(1)

#     files_to_process.sort() # Sort to process in order (e.g. 01, 02, 03)
#     print(f"Found {len(files_to_process)} instances to process.")
#     print("="*60)

#     # Process each file
#     for instance_file in files_to_process:
#         print(f"\n>>> Processing Instance: {os.path.basename(instance_file)}")
        
#         try:
#             # 1. Load Data
#             depot, clients, file_num_vehicles = load_instance_from_file(instance_file)
#             print(f"    Loaded {len(clients)} clients. Vehicle Capacity: {GlobalState.VEHICLE_CAPACITY}")

#             # 2. Run Algorithm
#             start_time = time.time()
            
#             # Create Solver
#             # Use max(file_num_vehicles, 4) to ensure a minimum fleet size baseline
#             ts = TabuSearch3L(depot, clients, num_vehicles=max(file_num_vehicles, 4))
            
#             # Execute with Time Limit (e.g., 60 seconds per instance)
#             if len(clients) <= 25:
#                 time_limit = 1800
#             elif len(clients) <= 50:
#                 time_limit = 3600
#             else:
#                 time_limit = 7200
#             final_solution = ts.solve(max_iters=1000, time_limit=time_limit)
            
#             end_time = time.time()
#             duration = end_time - start_time
#             print(f"    Finished in {duration:.2f} seconds.")

#             # 3. Export Results
#             export_results(final_solution, instance_file)
#             append_results_to_csv(instance_file, final_solution, duration)
            
#         except Exception as e:
#             print(f"!!! Error processing {instance_file}: {e}")
#             # Log error to CSV as well for tracking
#             os.makedirs("results", exist_ok=True)
#             with open(os.path.join("results", "results.csv"), mode='a', newline='') as f:
#                 writer = csv.writer(f)
#                 writer.writerow([os.path.basename(instance_file), "ERROR", "0", "0", "0"])
#             continue

#     print("\n" + "="*60)
#     print("Batch Processing Complete.")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--instances", type=str, default=".", help="folder with instances (.txt)")
    parser.add_argument("--out", type=str, default="results", help="output folder")
    parser.add_argument("--time", type=int, default=3600, help="time limit (sec) per run")
    parser.add_argument("--iter", type=int, default=1000, help="max iterations")
    parser.add_argument("--repeats", type=int, default=1, help="repeats per instance")
    parser.add_argument("--start-instance", type=str, default=None, help="resume from instance filename")
    parser.add_argument("--start-repeat", type=int, default=0, help="resume from repeat index (0-based)")
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)

    # CSV
    # csv_path = os.path.join(args.out, "results.csv")
    csv_path = "output/results.csv"
    write_header = not os.path.exists(csv_path)
    csv_file = open(csv_path, "a", newline="")
    writer = csv.writer(csv_file)

    if write_header:
        writer.writerow([
            "instance", "distance", "fill_rate", "calculation_time"
        ])

    # Collect instance files
    instance_files = []
    for fn in sorted(os.listdir(args.instances)):
        if fn.lower().endswith(".txt") and not fn.lower().startswith("solution") and "overview" not in fn.lower():
            instance_files.append(fn)

    if args.start_instance is not None:
        instance_files = [f for f in instance_files if f >= args.start_instance]

    print(f"Found {len(instance_files)} instances.")

    for fn in instance_files:
        instance_path = os.path.join(args.instances, fn)

        for r in range(args.repeats):
            if args.start_instance == fn and r < args.start_repeat:
                continue

            print(f"\n>>> Running {fn}, repeat {r + 1}/{args.repeats}")

            try:
                depot, clients, file_num_vehicles, dist_matrix = load_instance_from_file(instance_path)

                start_time = time.time()

                ts = TabuSearch3L(
                    depot,
                    clients,
                    num_vehicles=max(file_num_vehicles, len(clients)),
                    dist_matrix=dist_matrix
                )

                solution = ts.solve(
                    max_iters=args.iter,
                    time_limit=args.time
                )

                elapsed = time.time() - start_time

                if not solution or all(not v.route for v in solution):
                    raise RuntimeError("NO_SOLUTION")

                # Export solution
                sol_name = f"{os.path.splitext(fn)[0]}_{r}.txt"
                sol_path = os.path.join(args.out, sol_name)

                save_solution_txt_v2(
                    out_path=sol_path,
                    instance_name=os.path.splitext(fn)[0],
                    solution=solution,
                    elapsed_time=elapsed
                )

                # CSV summary
                used = [v for v in solution if v.route]
                total_dist = sum(v.route_length for v in used)

                total_vehicle_vol = len(used) * (
                    GlobalState.VEHICLE_W *
                    GlobalState.VEHICLE_H *
                    GlobalState.VEHICLE_L
                )

                total_item_vol = sum(
                    c.total_volume for v in used for c in v.route
                )

                fill_rate = (
                    total_item_vol / total_vehicle_vol
                    if total_vehicle_vol > 0 else 0.0
                )

                writer.writerow([
                    fn,
                    f"{total_dist}",
                    len(used),
                    f"{fill_rate}",
                    f"{elapsed}"
                ])

                print(f"[DONE] dist={total_dist:.2f}, vehicles={len(used)}, time={elapsed:.2f}s")

            except Exception as e:
                print(f"[FAIL] {fn}, repeat {r}: {e}")

                writer.writerow([
                    fn,
                    "x",
                    "x",
                    "x",
                    "x"
                ])

                csv_file.flush()
                continue

    csv_file.close()
    print("\nBatch processing complete.")


if __name__ == "__main__":
    main()