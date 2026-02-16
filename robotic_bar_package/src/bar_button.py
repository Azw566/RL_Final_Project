#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty, Float64
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
import os
import threading
import time

class BarButton(Node):
    def __init__(self):
        super().__init__('bar_button_node')
        
        # Clients
        self.delete_client = self.create_client(DeleteEntity, '/world/robotic_bar_world/remove')
        self.spawn_client = self.create_client(SpawnEntity, '/world/robotic_bar_world/create')
        
        threading.Thread(target=self.wait_services, daemon=True).start()

        # Publishers
        self.order_pub = self.create_publisher(String, '/bar/new_order', 10)
        self.detach_pub_iiwa = self.create_publisher(Empty, '/iiwa/bar/gripper/detach', 10)
        self.detach_pub_fra2mo = self.create_publisher(Empty, '/fra2mo/bar/gripper/detach', 10)
        
        self.lift_pub = {
            1: self.create_publisher(Float64, '/lift_1/elevation', 10),
            2: self.create_publisher(Float64, '/lift_2/elevation', 10),
            3: self.create_publisher(Float64, '/lift_3/elevation', 10)
        }
        # Subscriber
        self.create_subscription(String, '/delivered', self.delivery_callback, 10)
        
        self.can_spawn = True
        self.last_spawned_name = None
        self.current_color_rgb = None 
        
        self.table_counts = {1: 0, 2: 0, 3: 0}
        self.table_can_names = {1: [], 2: [], 3: []} # Keep track of can names on each table for deletion
        
        self.TABLE_SLOTS = {
            1: [(-3.01, 0.15, 0.83), (-3.70, 0.15, 0.83), (-3.03, -0.87, 0.83), (-3.69, -0.87, 0.83)],
            2: [(0.34, 0.15, 0.83), (-0.33, 0.15, 0.83), (0.32, -0.87, 0.83), (-0.36, -0.87, 0.83)],
            3: [(3.65, 0.15, 0.83), (2.96, 0.15, 0.83), (3.63, -0.87, 0.83), (2.97, -0.87, 0.83)]
        }

        self.print_menu()

    def wait_services(self):
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            pass
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            pass

    def delete_entity(self, name):
        if not name: return
        req = DeleteEntity.Request()
        req.entity.name = name
        req.entity.type = 2 
        self.delete_client.call_async(req)

   
    def spawn_model(self, name, x, y, z, rgb):
        geo = "<cylinder><radius>0.04</radius><length>0.15</length></cylinder>"
        sdf = f"<sdf version='1.6'><model name='{name}'><pose>{x} {y} {z} 0 0 0</pose><link name='l'><inertial><mass>0.1</mass><inertia><ixx>0.0005</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.0005</iyy><iyz>0</iyz><izz>0.0005</izz></inertia></inertial><collision name='c'><pose>0 0 0 0 0 0</pose><geometry>{geo}</geometry></collision><visual name='v'><geometry><cylinder><radius>0.04</radius><length>0.15</length></cylinder></geometry><material><ambient>{rgb}</ambient><diffuse>{rgb}</diffuse></material></visual></link></model></sdf>"
        
        req = SpawnEntity.Request()
        req.entity_factory.name = name
        req.entity_factory.sdf = sdf
        req.entity_factory.pose.position.x = float(x)
        req.entity_factory.pose.position.y = float(y)
        req.entity_factory.pose.position.z = float(z)
        self.spawn_client.call_async(req)

    def move_lift(self, tid, val):
        msg = Float64(); msg.data = float(val)
        if tid in self.lift_pub: self.lift_pub[tid].publish(msg)

    def detach_all(self):
        msg = Empty()
        for _ in range(15): 
            self.detach_pub_iiwa.publish(msg)
            self.detach_pub_fra2mo.publish(msg)
            time.sleep(0.2)

    def clear_table_if_full(self, tid):
        if self.table_counts[tid] >= 4:
            self.get_logger().info(f"🧹 Table {tid} Full. Cleaning...")
            for name in self.table_can_names[tid]:
                self.delete_entity(name)
            time.sleep(2.0)
            self.table_counts[tid] = 0
            self.table_can_names[tid] = []

    def delivery_callback(self, msg):
        tname = msg.data
        if "table1" in tname: tid = 1
        elif "table2" in tname: tid = 2
        elif "table3" in tname: tid = 3
        else: return

        self.get_logger().info(f"📩 Delivering to {tname}. Unloading...")

        self.move_lift(tid, 0.77)
        time.sleep(6.0)

        self.detach_all()
        self.delete_entity("order_can")
        if self.last_spawned_name and self.last_spawned_name != "order_can":
            self.delete_entity(self.last_spawned_name)
        self.last_spawned_name = None
        time.sleep(1.5)

        if self.table_counts[tid] < 4:
            idx = self.table_counts[tid]
            new_name = f"can_t{tid}_{idx}"
            pos = self.TABLE_SLOTS[tid][idx]
          
            self.spawn_model(new_name, pos[0], pos[1], pos[2], self.current_color_rgb)
            
            self.table_counts[tid] += 1
            self.table_can_names[tid].append(new_name)

        time.sleep(2.0)
        self.move_lift(tid, 0.0)
        self.can_spawn = True
        self.print_menu()

    def spawn_can(self, color):
        if not self.can_spawn: return

        if color == 'r': rgb, tname, tid = "1 0 0 1", "table1", 1
        elif color == 'b': rgb, tname, tid = "0 0 1 1", "table2", 2
        elif color == 'y': rgb, tname, tid = "1 1 0 1", "table3", 3
        else: return

        self.clear_table_if_full(tid)

        self.delete_entity("order_can")
        time.sleep(0.5)

        self.current_color_rgb = rgb
        self.can_spawn = False

        self.get_logger().info(f"🍺 Calling order at Table {tid}...")

        self.spawn_model("order_can", -0.95, 5.25, 1.01, rgb)
        
        self.detach_all()
        
        msg = String(); msg.data = tname
        self.order_pub.publish(msg)

    def print_menu(self):
        print("\n[r] Red | [b] Blue | [y] Yellow | [q] Quit")

def input_thread(node):
    while rclpy.ok():
        try:
            cmd = input()
            if cmd in ['r','b','y']: node.spawn_can(cmd)
            elif cmd == 'q': os._exit(0)
        except: break

def main():
    rclpy.init()
    node = BarButton()
    t = threading.Thread(target=input_thread, args=(node,), daemon=True)
    t.start()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()