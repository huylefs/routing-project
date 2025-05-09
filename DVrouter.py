####################################################
# DVrouter.py
# Name:
# HUID:
#####################################################

from router import Router
from packet import Packet
import json

class DVrouter(Router):
    """Distance vector routing protocol implementation."""

    def __init__(self, addr, heartbeat_time):
        """Initialize distance vector routing protocol."""
        Router.__init__(self, addr)
        self.heartbeat_time = heartbeat_time
        self.last_time = 0
        
        # Initialize routing data structures
        self.forwarding_table = {}  # Maps dest -> (port, cost)
        self.distance_vector = {addr: 0}  # Initial DV with only self
        self.neighbors = {}  # Maps port -> (endpoint, cost)
        self.route_through = {}  # Maps dest -> next hop router
        self.INFINITY = 16

    def handle_packet(self, port, packet):
        if packet.is_traceroute:
            if packet.dst_addr in self.forwarding_table:
                next_port, _ = self.forwarding_table[packet.dst_addr]
                self.send(next_port, packet)
        else:
            try:
                received_dv = json.loads(packet.content)
                sender = received_dv.pop('sender')
                updated = False
                
                if port in self.neighbors and self.neighbors[port][0] == sender:
                    link_cost = self.neighbors[port][1]
                    
                    for dest, received_cost in received_dv.items():
                        if dest == self.addr:
                            continue

                        # Calculate precise total cost
                        total_cost = self.INFINITY
                        if received_cost < self.INFINITY:
                            total_cost = received_cost + link_cost
                            if total_cost >= self.INFINITY:
                                total_cost = self.INFINITY
                        
                        current_cost = self.distance_vector.get(dest, self.INFINITY)
                        current_next_hop = self.route_through.get(dest)

                        # Update route if:
                        # 1. Found a better (lower cost) path
                        # 2. Current route through this neighbor has changed
                        # 3. Current route is broken (cost now INFINITY)
                        if ((total_cost < current_cost) or 
                            (current_next_hop == sender and total_cost != current_cost) or
                            (current_next_hop == sender and received_cost >= self.INFINITY)):
                            
                            if total_cost < self.INFINITY:
                                self.distance_vector[dest] = total_cost
                                self.forwarding_table[dest] = (port, total_cost)
                                self.route_through[dest] = sender
                            else:
                                self.remove_route(dest)
                            updated = True
                
                if updated:
                    self.broadcast_distance_vector()
                    
            except json.JSONDecodeError:
                pass

    def handle_new_link(self, port, endpoint, cost):
        """Handle new link."""
        self.neighbors[port] = (endpoint, cost)
        
        # Update direct route to neighbor
        if endpoint not in self.distance_vector or cost < self.distance_vector[endpoint]:
            self.distance_vector[endpoint] = cost
            self.forwarding_table[endpoint] = (port, cost)
            self.route_through[endpoint] = endpoint
            self.broadcast_distance_vector()

    def handle_remove_link(self, port):
        """Handle removed link."""
        if port not in self.neighbors:
            return
            
        removed_endpoint = self.neighbors[port][0]
        del self.neighbors[port]
        
        changed = False
        # Remove all routes that use this port
        for dest in list(self.forwarding_table.keys()):
            if (self.forwarding_table[dest][0] == port or 
                (dest in self.route_through and self.route_through[dest] == removed_endpoint)):
                self.remove_route(dest)
                changed = True
        
        if changed:
            self.broadcast_distance_vector()

    def remove_route(self, dest):
        """Helper to remove a route and all associated entries."""
        if dest in self.distance_vector:
            del self.distance_vector[dest]
        if dest in self.forwarding_table:
            del self.forwarding_table[dest]
        if dest in self.route_through:
            del self.route_through[dest]

    def handle_time(self, time_ms):
        """Handle current time."""
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            self.broadcast_distance_vector()

    def broadcast_distance_vector(self):
        """Send distance vector updates to all neighbors."""
        for port, (neighbor, _) in self.neighbors.items():
            dv_to_send = {'sender': self.addr}
            
            for dest, cost in self.distance_vector.items():
                # Apply split horizon with poison reverse
                if dest in self.route_through and self.route_through[dest] == neighbor:
                    dv_to_send[dest] = self.INFINITY
                else:
                    dv_to_send[dest] = cost
            
            packet = Packet(Packet.ROUTING, self.addr, neighbor, json.dumps(dv_to_send))
            self.send(port, packet)

    def __repr__(self):
        """Return string representation for debugging."""
        return f"DVrouter(addr={self.addr}, dv={self.distance_vector}, ft={self.forwarding_table})"