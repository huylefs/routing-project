from router import Router
from packet import Packet
from json import dumps, loads
from dijkstar import Graph, find_path

class LSrouter(Router):
    """Link state routing protocol implementation."""

    def __init__(self, addr, heartbeat_time):
        Router.__init__(self, addr)  # Initialize base class - DO NOT REMOVE
        self.heartbeat_time = heartbeat_time
        self.last_time = 0
        self.graph = Graph(undirected=True)
        self.global_seq = {}  # to store sequence numbers of all nodes
        self.neighbours = {}  # dict with nodes as key and [cost,port] as value
        self.seq_no = 0      # seq no
        self.address = addr                                         
        self.linked_state = [self.address, self.neighbours, self.seq_no]  # local link state
        self.global_view = {} # dict to store all link states (global view)
        self.fT = {} # forwarding table

    def handle_packet(self, port, packet):
        """Process incoming packet."""
        if packet.is_traceroute:
            send_add = self.path_finder(packet.dst_addr)  # finds appropriate nextHop 
            if send_add != 0:  # if path found
                try:
                    final_add = self.neighbours[send_add][1]  # finding port
                    self.send(final_add, packet)
                except:
                    return
        else:
            data = loads(packet.content)  # loading packet in data
            src = packet.src_addr
            addr = packet.dst_addr
            seq = data[2]
            node_neighbour = data[1]
            node_id = data[0]

            # Check for stale link state
            if node_id in self.global_seq and seq <= self.global_seq[node_id]:
                return
            
            self.global_seq[node_id] = seq  # adding seq no
            if node_id in self.graph:  # if node is in global graph view
                self.update_ft()  # update forwarding table
                self.update_graph(node_id, node_neighbour)  # update graph
                self.forward_received_ls(packet, node_id)  # broadcast link state

    def handle_new_link(self, port, endpoint, cost):
        """Handle new link."""
        self.graph.add_edge(self.addr, endpoint, cost)  # add edge
        self.neighbours[endpoint] = [cost, port]   # add item in neighbour dict
        self.global_view[port] = self.neighbours  # update global view
        self.seq_no += 1
        self.update_ft()  # update forwarding table
        self.broadcast_ls()  # broadcast LS

    def handle_remove_link(self, port):
        """Handle removed link."""
        add = self.add_finder(port)  # finding appropriate port
        self.neighbours.pop(add)  # removing from dict
        self.graph.remove_edge(self.addr, add)  # updating graph
        self.seq_no += 1
        self.update_ft()  # update forwarding table
        self.broadcast_ls()  # broadcast LS

    def handle_time(self, time_ms):
        """Handle current time."""
        if time_ms - self.last_time >= self.heartbeat_time:
            self.seq_no += 1
            self.broadcast_ls()  # broadcast LS
            self.last_time = time_ms

    def add_finder(self, port):
        """Find appropriate port from node_address in neighbour dict."""
        for i in self.neighbours:
            if self.neighbours[i][1] == port:
                return i

    def path_finder(self, dst):
        """Run Dijkstra algorithm to find path."""
        try:
            path = find_path(self.graph, self.address, dst)
            return path.nodes[1]
        except:
            return 0
    
    def update_ft(self):
        """Update forwarding table."""
        for j in self.global_view:
            for i in self.global_view[j]:
                try:
                    path = find_path(self.graph, self.address, i[1])
                    node = path.nodes[1]
                    cost = path.total_cost
                    if i[1] not in self.fT:
                        self.fT[i] = [node, cost]
                    elif cost < self.fT[i][1]:  # if efficient path
                        self.fT[i] = [node, cost]
                except:
                    continue
    
    def broadcast_ls(self):
        """Broadcast Local link state to neighbours."""
        for i in self.neighbours:
            self.linked_state[2] = self.seq_no
            pac = Packet(Packet.ROUTING, self.address, self.neighbours[i])
            pac.content = dumps(self.linked_state)
            self.send(self.neighbours[i][1], pac)
    
    def forward_received_ls(self, pac, sender_add):
        """Forward link state received to other neighbours."""
        for i in self.neighbours:
            if i != sender_add:
                self.send(self.neighbours[i][1], pac)

    def update_graph(self, node_id, node_neighbour):
        """Update Graph."""
        if len(list(node_neighbour)) < len(list(self.graph[node_id])):
            for node in self.graph[node_id]:
                if node in node_neighbour:
                    continue
                else:
                    self.graph.remove_edge(node_id, node)
                    break
        else:
            for key in node_neighbour:
                self.graph.add_edge(node_id, key, node_neighbour[key][0])

    def __repr__(self):
        """Return string representation for debugging."""
        return f"LSrouter(addr={self.addr}, ft={self.fT})"