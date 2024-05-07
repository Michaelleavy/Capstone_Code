import xml.etree.ElementTree as ET
import tkinter as tk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import math
import random
import requests
import subprocess
from PIL import Image
from io import BytesIO

from codrone_edu.drone import *
import time
import numpy as np

import paramiko
from datetime import datetime

class Observation:
    def __init(self,id):
        self.id = id
        self.lat = 0
        self.lon = 0
        self.used = False

class GraphNode:
    def __init__(self, lat,lon,id,should_visit):
        self.lon = lon
        self.lat = lat
        self.id = id
        self.should_visit = should_visit
        self.visited = False
        self.level = -1
        self.connected_nodes = []
    def __str__(self):
        return "Id:" + self.id + " lon: " + self.lon + " lat: " + self.lat + " should_visit: " + str(self.should_visit) + " connections: " + str(self.connected_nodes) + " visited: " + str(self.visited)

class Instruction:
    def __init__(self,angle,dist,left,node):
        self.angle = angle
        self.dist = dist
        self.left = left
        self.node = node
    def print(self):
            if self.left:
                print("Turn: " + str(self.angle) + " to the left and travel: " + str(self.dist) + "m")
            else:
                print("Turn: " + str(self.angle) + " to the right and travel: " + str(self.dist) + "m")
            print("To node: ")
            print(self.node)
            print("===========================")

class MapVisualizer:
    def __init__(self, master, way_map):
        self.access_key = "673f60f8a69b26c57dcd9f7ed968e078b8a9a8e5"

        self.master = master
        self.way_map = way_map

        self.menu_frame = tk.Frame(master, width=400,height=400)
        self.menu_frame.pack(side = tk.LEFT,expand=True,fill=tk.BOTH)

        self.graph_frame = tk.Frame(master)
        self.graph_frame.pack(side=tk.RIGHT,expand=True,fill=tk.BOTH)

        self.starting_node = None

        self.canvas = None
        self.nodes = {}
        self.paths = []
        self.removed_nodes=[]
        self.removed_paths=[]
        self.axPrev = None
        
        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)

        self.run_right_frame()

        self.init_map()

        self.drone = None
        self.drone_instructions = []

    #Function to create the buttons on the right frame in the GUI
    def run_right_frame(self):
        #Config - include debug
        include_debug = False

        frame = self.menu_frame

        self.graph = None

        title = tk.Label(frame, text= "Options for map",font=("Calibri", 14, "bold"))
        title.place(x=130,y=0)

        self.remove_path_button = tk.Button(frame, text="Remove selected paths", command=self.remove_path_button_func)
        self.remove_path_button.place(x=10,y=30)

        self.remove_node_button = tk.Button(frame, text="Remove selected nodes", command=self.remove_node_button_func)
        self.remove_node_button.place(x=10,y=60)

        self.refresh_button = tk.Button(frame, text="Reload all nodes/paths", command=self.refresh_nodes_func)
        self.refresh_button.place(x=150,y=60)

        self.generate_button = tk.Button(frame, text="Set 'must visit' for selected nodes", command=self.must_visit_selected_nodes)
        self.generate_button.place(x=10,y=90)

        self.generate_button = tk.Button(frame, text="Unset 'must visit' for selected nodes", command=self.not_must_visit_selected_nodes)
        self.generate_button.place(x=10,y=120)

        self.generate_button = tk.Button(frame, text="Set selected node to drone start", command=self.set_starting_node)
        self.generate_button.place(x=10,y=150)


        self.generate_button = tk.Button(frame, text="Generate instructions", command=self.generate_instructions)
        self.generate_button.place(x=10,y=200)
        #was "self.fly_drone
        self.generate_button = tk.Button(frame, text="Fly Drone", command=self.fly_drone)
        self.generate_button.place(x=10,y=230)

        self.generate_button = tk.Button(frame, text="Emergency Stop", command=self.emergency_stop)
        self.generate_button.place(x=80, y=230)

        if include_debug:
            self.generate_button = tk.Button(frame, text="Get Auth", command=self.get_mage_auth)
            self.generate_button.place(x=10, y=300)

            self.generate_button = tk.Button(frame, text="run_hardcode", command=self.run_HC)
            self.generate_button.place(x=70, y=300)

            self.generate_button = tk.Button(frame, text="testObservation", command=self.test_func_observation)
            self.generate_button.place(x=10,y=350)

            self.generate_button = tk.Button(frame, text="Test", command=self.test_func)
            self.generate_button.place(x=150, y=400)

    #Test function to run a set of pre-defined instructions for the drone
    def run_HC(self):
        cook = False
        self.drone = Drone()
        print("Attempting to pair")
        self.drone.pair()
        self.drone.takeoff()
        time.sleep(1)
        self.drone.move_forward(distance=10,units="ft",speed=.4)
        time.sleep(20)
        self.drone.land()
        if cook:
            print("Drone paired, taking off")
            self.drone.takeoff()
            print("Drone taken off, moving forward")
            self.drone.move_forward(distance=15.9,units="m",speed=.4)
            self.d_a_c(10)
            print("Turning left 85 deg")
            self.drone.turn_left(86.6)
            self.d_a_c(2)
            print("Moving")
            self.drone.move_forward(12.08,units="m",speed=.4)
            self.d_a_c(10)
            print("Turning around")
            self.drone.turn_left(90)
            print("A")
            self.d_a_c(2)
            self.drone.turn_left(90)
            print("B")
            self.d_a_c(2)
            print("Moving forward 12.08")
            self.drone.move_forward(12.08,units="m",speed=.4)
            self.d_a_c(10)
            print("Turning right")
            self.drone.turn_right(86.6)
            self.d_a_c(2)
        print("Moving forward 15.8 m")
        self.drone.move_forward(15.88,units="m",speed=.4)
        self.d_a_c(10)
        print("Landing")
        self.drone.land()
        self.d_a_c(2)
        self.drone.disconnect()

    #This function creates a new observation ID (empty inside) to be modified by the function below
    def create_new_observationID(self):
        try:
            response = requests.post(("http://localhost:4242/api/events/1/observations/id?access_token=" + self.access_key))
            return response.json()
        except requests.exceptions.RequestException as e:
            print("Error getting data: ",e)
            return None
        
    #This function updates the given observationID to have the given lat/lon coordinates
    def call_mageAPI(self,lat,lon,id, image_name):
        headers = {'Content-Type' : 'application/json'}
        body = {
            "eventId": 1,
            "userId": "661ff058515d1a45509e3dce",
            "deviceId": "661ff058864b65146c23f4a6",
            "type": "Feature",
            "geometry": {
                "type": "Point",
                "coordinates": [lon, lat]
            },
            "properties": {
                "timestamp": "2024-04-17T19:55:21.248Z",
                "forms": [
                #     {
                #     "formId": 1,
                #     "field0": [{
                #         "name":"download.jpg",
                #         "size":8823,
                #         "contentType":"image/jpeg",
                #         "action":"add"
                #     }]
                # },
                {
                   "formId": 2,
                   "field0": image_name
                }
                ]
            },
            "favoriteUserIds": [],
            "attachments": [
                # {
                #     "fieldName": "field0",
                #     "contentType": "image/jpeg",
                #     "name": "download.jpg",
                #     "size": 8823
                # }
            ],
            "id": id,
        }
        url = ("http://localhost:4242/api/events/1/observations/" + id + "?access_token=" + self.access_key)
        try:
            response = requests.put(url, headers = headers, json= body)
            response.raise_for_status() #raise an exception if necessary
            json_data = response.json()
            # print("ID:" + json_data['attachments'][0]['id'])
            return json_data
        except requests.exceptions.RequestException as e:
            print("Error fetching data:", e)
            return None

#Function to create a new observation at the given lat/lon coordinates and add the given image name
    def create_observation(self,lat,lon,image_name):
        result = self.create_new_observationID()
        print("New observation:")
        print(result)
        print("ID of new observation:")
        print(result['id'])
        print("=============")
        print("Updated JSON for new observation (most importantly coordinates)")
        result2 = self.call_mageAPI(lat,lon,result['id'],image_name=image_name)
        print(result2)

#This function uploads an image to the observation (has errors)
    def add_image_to_observation(self,att_id,observation_id,path):
        print("Started uploading image to observation")

        img_data = {"attachment": open(file=path+".jpg",mode='rb')}
        url = ("http://localhost:4242/api/events/1/observations/" + observation_id + "/attachments/" + att_id + "?access_token=" + self.access_key)
        response = requests.put(url=url,data=img_data,headers={'contentType': 'image/jpg'})
        print("Response:")
        print(response)
        print(response.json())
        return response

#This function gets an authorization token for Mage and
    def get_mage_auth(self):
        try:
            print("Getting authentication for MAGE")
            #Get the JWT
            body = {
                "password": "myMagepWd123123",
                "passwordconfirm": "myMagepWd123123",
                "username": "Michaell1"
            }
            response = requests.post("http://localhost:4242/auth/local/signin", json= body)

            #(Should) Get authentication token from the JWT 
            header = {
                "Authorization": ("Bearer " + response.json()['token'])
            }
            response = requests.post("http://localhost:4242/auth/token?uid=134",headers=header)
            self.access_key = response.json()['token']
            print("Obtained MAGE authentication key")
        except requests.exceptions.RequestException as e:
            print("Error getting data: ",e)
            return None

#Test create observation function
    def test_func_observation(self):
        #self.add_image_to_observation("662814993d100482b411d2ff","662814973d100482b411d2fd","download.jpg")
        #list = self.graph.items()
        # for item in list:
        #     node = item[1]
        #     lat = float(node.lat)
        #     lon = float(node.lon)
        #     #lat2 = 38.6488582
        #     #lon2 = -30.301388#-90.301388
        #     # print(type(lat2))
        #     # print(type(lat))
        #     # print(type(lon2))
        #     # print(type(lon))
        self.create_observation(30,30,"download.jpg")

#This function tells the RPI Zero W (the drone camera) to take a picture
    def take_photo_ssh(image_name):
        # Example usage
        hostname = "gsm-pi.local"
        username = "gsm"
        password = "D0ntF0rg3t!"
        output_filename = (str(image_name)+".jpg")

        # Connect to the Raspberry Pi via SSH
        ssh_client = paramiko.SSHClient()
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh_client.connect(hostname, username=username, password=password)

        # Take a photo using raspistill command
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        command = f"raspistill -o {output_filename}"
        ssh_stdin, ssh_stdout, ssh_stderr = ssh_client.exec_command(command)

        # Wait for the command to finish
        ssh_stdout.channel.recv_exit_status()

        # Check for any errors
        error_output = ssh_stderr.read().decode("utf-8")
        if error_output:
            print("Error:", error_output)
        # Close SSH connection
        ssh_client.close()

        command = f"scp {username}@{hostname}:~/{output_filename} ~/Desktop/"
        process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(1)
        return output_filename

#This function is used for general purpose testing
    def test_func(self):
        self.fly_drone_demo()
        #self.take_photo_ssh(image_name="Test")
        #self.get_mage_auth()
        #self.create_observation(lat=38.6483842,lon=-90.3023468,image_name="drone_pic_5")

    def sleep(self,duration):
        run_DAC = True
        if run_DAC:
            return self.d_a_c(duration)
        else:
            time.sleep(duration)
            return False

    #Function with pre-defined operations for DEMO (based on instructions generated by MAGE data)
    def fly_drone_demo(self):
        self.drone = Drone()
        drone = self.drone
        drone.pair()
        drone.takeoff()
        self.sleep(1)
        drone.move_forward(distance=80.6, units="cm", speed=2)
        self.sleep(2.5)
        drone.turn_left(85.6)
        self.sleep(0.2)
        drone.move_forward(distance=58.4, units="cm", speed=2)
        self.sleep(2.5)
        drone.land()
        drone.close()

    def emergency_stop(self):
        self.drone = Drone()
        self.drone.pair()
        self.drone.land()
        self.drone.close()

#General function to fly the drone based on currently generated instructions
    def fly_drone(self):
        #Configs for this function

        #Pair with the drone
        drone_pair = True
        #Run in increments of 15(unit) (instead of going straight - for debugging)
        do_increments = False
        #Is the camera connected
        camera_connected = False
        #Is mage running/should it connect
        mage_connected = False
        #To run a miniature version of the directions, make this >1
        #decreases direction size by a factor of decrement_factor
        decrement_factor = 1
        #set what unit to do the directions in (instructions are written in M, but diff units can be used to
        #downscale them)
        unit = "m"

        if(drone_pair):
            print("Connecting drone")
            self.drone = Drone()
            self.drone.pair()
            print("Drone connected, starting to fly")

        if len(self.drone_instructions) <= 1:
            tk.messagebox.showerror("Missing directions error", "Please ensure you have generated instructions prior to flying the drone")
            return
        if drone_pair:
            self.drone.takeoff()
            time.sleep(3)

        num = 1
        for instr in self.drone_instructions:
            if instr.dist == 0:
                print("Land")
            elif instr.angle < 3:
                print("Don't turn ")
            elif instr.angle == 180:
                print("Turn around")
                if drone_pair:
                    self.drone.turn_left(degree=90)
                    self.d_a_c(.01,drone_pair)
                    self.drone.turn_left(degree=90)
                    self.d_a_c(.01, drone_pair)
            elif instr.left:
                print("Turn left " + str(instr.angle))
                if drone_pair:
                    self.drone.turn_left(degree=instr.angle)
                    self.d_a_c(.1, drone_pair)
            else:
                print("Turn right " + str(instr.angle))
                if drone_pair:
                    self.drone.turn_right(degree=instr.angle)
                    self.d_a_c(.1, drone_pair)
            if instr.dist > 0:
                total_dist = instr.dist

                #Loop to run in smaller increments if desired
                while total_dist > 15 and do_increments:
                    if drone_pair:
                        self.drone.move_forward(distance=15, units=unit, speed=1)
                    print("Moving 15")
                    if self.d_a_c(1,drone_pair):
                        return
                    total_dist = total_dist-15
                print("Moving: ", total_dist)

                if drone_pair:
                    self.drone.move_forward(distance=total_dist/decrement_factor,units=unit, speed=1)
                    if self.d_a_c(1,drone_pair):
                        return

                if camera_connected:
                    self.take_photo_ssh("drone_pic_"+str(num))
                    print("Take picture for node at lat/lon: " + str(instr.node.lat + "/" +str(instr.node.lon)))

                print("=============")
        if drone_pair:
            self.drone.land()
            time.sleep(2)
            self.drone.disconnect()

        if mage_connected:
            self.get_mage_auth()
            ob_num = 0
            for instr in self.drone_instructions:
                if instr.dist > 3 and ob_num > 0:
                    mage_image_name = "drone_pic_"+str(ob_num)
                    print("Create observation at lat: " + instr.node.lat + " lon: " + instr.node.lon + " with name: " + mage_image_name)
                    self.create_observation(lat=float(instr.node.lat), lon=float(instr.node.lon), image_name=mage_image_name)
                ob_num = ob_num + 1

    #Function with delta timing and to avoid collisions
    def d_a_c(self,time_length,running=True):
        #print("inside")
        for i in range(0,int(time_length*100)):
            if running:
                if self.drone.detect_wall(30):
                    print("Found wall! Landing")
                    self.drone.land()
                    time.sleep(1)
                    self.drone.disconnect()
                    return True
            #print("waiting...")
            time.sleep(.01)
        return False

    #Function to determine the directions for the drone based off selected nodes
    def generate_instructions(self):
        if self.starting_node == None:
            tk.messagebox.showerror("Missing starting node","Please ensure a node is selected as the drone starting node")
        graph = {}
        nodes_to_visit = []

        #Generate the graph data structure
        for path in self.paths:
            if path['active']:
                n1 = path['node1']
                n2 = path['node2']

                #Connect node1 to node2
                if not graph.__contains__(n1['id']):
                    new_node = GraphNode(n1['lat'],n1['lon'],n1['id'],self.nodes[n1['id']]['mustVisit'])
                    if(new_node.should_visit):
                        nodes_to_visit.append(new_node)
                    graph[n1['id']] = new_node
                #Connect node2 to node1
                if not graph.__contains__(n2['id']):
                    new_node = GraphNode(n2['lat'],n2['lon'],n2['id'],self.nodes[n2['id']]['mustVisit'])
                    if(new_node.should_visit):
                        nodes_to_visit.append(new_node)
                    graph[n2['id']] = new_node
                graph[n1['id']].connected_nodes.append(graph[n2['id']])
                graph[n2['id']].connected_nodes.append(graph[n1['id']])

        #Prepare to start generating instructions
        curNode = graph[self.starting_node]
        full_instructions = []

        self.graph = graph

        #Main loop to generate instructions - find closest node to the current node so long as there are nodes to visit
        while len(nodes_to_visit) > 0:
            curNode.level = 0
            #Get the path to the closest node that should be visited
            paths_taken,path_length = self.find_closest_node_path(graph,nodes_to_visit,curNode,0)
            nodes_to_visit.remove(paths_taken[0])
            curNode = paths_taken[0]
            full_instructions.append(paths_taken)
        
        #Find fastest path from last visited node back to start
        last_path,last_length=  self.find_closest_node_path(graph,[graph[self.starting_node]],curNode,0)
        full_instructions.append(last_path)

        #Print out the 2D instruction array
        full_ordered_instructions = []

        for n in full_instructions:
            #print("Section: ")
            for i in range(0,len(n)-1):
                index = len(n)-(i+1)
                full_ordered_instructions.append(n[index])
        full_ordered_instructions.append(graph[self.starting_node])

        index = 1
        cur_node = graph[self.starting_node]
        next_node = full_ordered_instructions[1]
        prev_node = graph[self.starting_node]

        for i in full_ordered_instructions:
            #print(cur_node)
            index = index + 1
            angle,dist,left = self.find_angle_and_distance(prev_node,cur_node,next_node)
            prev_node = cur_node
            cur_node = next_node
            if(index < len(full_ordered_instructions)):
                next_node = full_ordered_instructions[index]

            #This is where each instruction is created and added to the list
            new_instr = Instruction(angle,dist,left,prev_node)
            self.drone_instructions.append(new_instr)
            #new_instr.print()
        tk.messagebox.showinfo("Instructions generated","Instructions successfully generated, ready to fly!")

    #Function to calculate distance between two LAT/LON coordinates
    def haversine(self,lat1, lon1, lat2, lon2):
        R = 6370 #Earth radius (km)
        d_lat = math.radians(lat2 - lat1)
        d_lon = math.radians(lon2 - lon1)
        a = math.sin(d_lat/2) * math.sin(d_lat/2) + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(d_lon/2) * math.sin(d_lon/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c
        return distance

    #Function to calculate the angle and distance between
    def find_angle_and_distance(self,prev_node,cur_node, next_node):
        #distance between cur and next node in meters using haversine formula
        dist = self.haversine(float(cur_node.lat),float(cur_node.lon),float(next_node.lat),float(next_node.lon))*1000

        #Get angle
        a = [float(next_node.lon) - float(cur_node.lon),float(next_node.lat)-float(cur_node.lat)]
        b = [float(prev_node.lon) - float(cur_node.lon),float(prev_node.lat)-float(cur_node.lat)]

        vec_a = np.array(a)
        vec_b = np.array(b)

        dot = vec_a.dot(vec_b)

        mag_a = np.linalg.norm(vec_a)
        mag_b = np.linalg.norm(vec_b)
        cross_prod = np.cross(vec_a,vec_b)

        #angle = math.degrees(math.acos(dot/(mag_a*mag_b)))
        if cross_prod == 0:
            angle = 0
        else:
            angle = math.degrees(math.atan(cross_prod/dot))
        
        if prev_node.id == next_node.id:
            angle = 180
        left = False
        if(cross_prod > 0):
            left = True
        else:
            left = False
        return abs(angle),dist,left

    #Recursive function to find the shortest path between the desired start node and end node
    #This function is called in the generate_instructions function
    def find_closest_node_path(self,graph,nodes_to_visit,cur_node,cur_level):
        #Base case: node is a node that we should visit
        if cur_node in nodes_to_visit:
            new_paths_taken = []
            new_paths_taken.append(cur_node)
            if cur_level < cur_node.level:
                cur_node.level = cur_level
            return new_paths_taken,cur_level
        else:
            cur_node.visited = True
            min_level = -1
            min_paths = []
            #Check paths for all connected nodes
            for n in cur_node.connected_nodes:
                #only check if it could be a better path than already exists
                if (not n.visited) and ((n.level == -1) or n.level < (cur_level+1)):
                    n.visited = True
                    pot_paths,pot_level = self.find_closest_node_path(graph,nodes_to_visit,n,cur_level+1)
                    if (not (pot_paths == None)) and ((min_level == -1) or (pot_level < min_level)):
                        min_level = pot_level
                        min_paths = pot_paths
                    n.visited = False
            cur_node.visited = False

            if min_level > -1:
                min_paths.append(cur_node)
                return min_paths,min_level
            else:
                return None,None

#The next several functions are called by buttons in the GUI to manipulate the map and its data structures
    def set_starting_node(self):
        new_start = None
        sum=0
        for node_id,node_data in self.nodes.items():
            if node_data['selected'] and  (not self.removed_nodes.__contains__(node_id)):
                new_start = node_id
                sum = sum + 1
        if sum == 1:
            self.starting_node = new_start
        else:
            tk.messagebox.showerror("Incorrect number of selected nodes","Please ensure that only one node is selected to be the starting point")

    def refresh_nodes_func(self):
        self.removed_nodes.clear()
        self.init_map()

    def remove_path_button_func(self):
        for path in self.paths:
            if path['selected']:
                path['active'] = False
        self.ax.clear()
        self.canvas.draw()
        self.refresh_plot()
    
    def must_visit_selected_nodes(self):
        for node_id,node_data in self.nodes.items():
            if node_data['selected'] and  (not self.removed_nodes.__contains__(node_id)):
                self.nodes[node_id]['mustVisit'] = True
    
    def not_must_visit_selected_nodes(self):
        for node_id,node_data in self.nodes.items():
            if node_data['selected'] and  (not self.removed_nodes.__contains__(node_id)):
                self.nodes[node_id]['mustVisit'] = False

    def remove_node_button_func(self):
        for node_id,node_data in self.nodes.items():
            if node_data['selected'] and  (not self.removed_nodes.__contains__(node_id)):
                self.removed_nodes.append(node_id)
                if node_id == self.starting_node:
                    self.starting_node = None
        self.ax.clear()
        self.canvas.draw()
        self.refresh_plot()

#Function to reload the map in the GUI
    def refresh_plot(self):
        for path in self.paths:
            n1 = path['node1']
            n2 = path['node2']
            if not (self.removed_nodes.__contains__(n1['id']) or self.removed_nodes.__contains__(n2['id'])) and path['active']:
                path_obj = self.ax.plot([float(n1['lon']), float(n2['lon'])], [float(n1['lat']), float(n2['lat'])], color='black', picker=5)
                path['path'] = path_obj
            else:
                path['active'] = False

        self.nodes.clear()
        for way_id, way_data in self.way_map.items():
            for node in way_data['nodes']:
                lat = float(node['lat'])
                lon = float(node['lon'])
                if not self.removed_nodes.__contains__(node['id']):
                    if not self.nodes.__contains__(node['id']):
                        circle = self.ax.plot(lon, lat, 'bo', picker=2)
                        self.nodes[node['id']] = {'circle': circle, 'selected': False, 'uses': 1, 'mustVisit': False}
                    else:
                        self.nodes[node['id']]['uses'] = self.nodes[node['id']]['uses']+1

        self.ax.set_xlabel('Longitude')
        self.ax.set_ylabel('Latitude')
        self.ax.set_title('Map')
        if self.canvas != None:
            self.canvas.draw()
            self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    #Function to initialize the map of nodes based on OSM data
    def init_map(self):
        #initalize paths array
        self.paths.clear()
        for way_id, way_data in self.way_map.items():
            color = (random.random(), random.random(), random.random())
            for i in range(len(way_data['nodes']) - 1):
                node1 = way_data['nodes'][i]
                node2 = way_data['nodes'][i + 1]
                if not (self.removed_nodes.__contains__(node1['id']) or self.removed_nodes.__contains__(node2['id'])):
                    self.paths.append({'path': None, 'node1': node1, 'node2': node2, 'selected': False, 'active': True})

        self.refresh_plot()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.graph_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas.mpl_connect('pick_event', self.on_pick)

#Function which is called when a circle or path is clicked in the GUI map
    def on_pick(self, event):
        artist = event.artist
        node_id = None
        for id, data in self.nodes.items():
            if data['circle'][0] == artist:
                node_id = id
                break
        
        if node_id: #i.e. this is a node
            if self.nodes[node_id]['selected'] and self.starting_node == node_id:
                artist.set_color('purple')
            elif self.nodes[node_id]['selected'] and self.nodes[node_id]['mustVisit']:
                artist.set_color('lime')
            elif self.nodes[node_id]['selected']:
                artist.set_color('blue')
            else:
                artist.set_color('red')
            self.nodes[node_id]['selected'] = not self.nodes[node_id]['selected']
        else: #if not a node, this must be a path
            for path in self.paths:
                if path['path'][0] == artist:
                    if path['selected']:
                        artist.set_color('black')
                        path['selected'] = False
                    else:
                        artist.set_color('red')
                        path['selected'] = True
                    break

        self.canvas.draw()

#Function to parse the XML file extracted from OSM data
def parse_xml_file(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()

    node_map = {}
    way_map = {}
    for node in root.findall('node'):
        node_id = node.attrib['id']
        lat = node.attrib['lat']
        lon = node.attrib['lon']
        node_map[node_id] = {'lat': lat, 'lon': lon}

    for way in root.findall('way'):
        way_id = way.attrib['id']
        way_data = {'nodes': [], 'tags': {}}
        for nd in way.findall('nd'):
            node_ref = nd.attrib['ref']
            if node_ref in node_map:
                way_data['nodes'].append({'id': node_ref, 'lat': node_map[node_ref]['lat'], 'lon': node_map[node_ref]['lon']})

        is_sidewalk = False
        for tag in way.findall('tag'):
            key = tag.attrib['k']
            value = tag.attrib['v']
            way_data['tags'][key] = value
            if tag.attrib['k'] == 'highway':
                is_sidewalk = True
        if is_sidewalk:
            way_map[way_id] = way_data
    return way_map

def main():
    #xml_file_path = 'map.osm'
    xml_file_path = 'brauer.osm'
    way_map = parse_xml_file(xml_file_path)

    root = tk.Tk()
    root.title("Map Visualizer")

    app = MapVisualizer(root, way_map)
    
    root.mainloop()

if __name__ == "__main__":
    main()