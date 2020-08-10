# Mayank Kishore
# Solution Description:
    # GOAL: Try to create the lowest amount of stall time (no drones sent because all drones are active)
    # Create a graph of connected hospitals
        # Each HospitalNode has a set_of_adjacent_hospitals which includes every
        # hospital that can be reachable from the current hospital and still make it back to base
    # queue_order(received_time,hospital,priority)
        # Used a priority queue to rank an Emergency order higher than a Resupply order
    # schedule_next_flight(current_time)
        # When it is time to schedule the next route:
            # Make sure there is an inactive zip available
            # If it is an Emergency:
                # try to send the max combination of Emergencies and Resupplies, but send
                # the emergency as fast as possible. Decided to only send an Emergency with at 
                # least one other packageas it reduced the avereage stall time to 629.922 from 
                # 1070.948 when allowed to be sent alone
            # If there are only Resupplies:
                # only send it if there are 3 hospitals that are reachable with one drone
                # this is to try and make sure there are enough drones for emergencies
    # RESULTS:
        # Was able to achieve an average stall of 629.922 seconds which means that
        # the average amonunt of time spent waiting for the drone was around 10 minutes.
        # When tweaking the parameters of the best_combination function this was the lowest time I got
    
    # CONCLUSION:
        # As an end result, I think this solution does a good job of minimizing the number of drones 
        # sent while trying to minimize the amount of time spent in the air by finding the best
        # combination of current_orders to send with the next drone, all while prioritizing the
        # emergencies that are happening


import csv
import numpy as np
import time
import heapq

hospitals = 'hospitals.csv' # Hospital Name, North, East
orders = 'orders.csv'       # Recieved Time, Hospital Name, Priority

# Using this class to represent the Hospital as a node on a graph
class HospitalNode():
    def __init__(self, x, y):
        self.name = ''
        self.x = x
        self.y = y
        self.distance_to_base = np.linalg.norm(np.array((0,0))-np.array((x,y)))

        # this is used to keep track of all hospitals reachable from this HospitalNode
        self.set_of_adjacent_hospitals = set() 

# Keep track of zips with and ID, current route, and return_time to base
# Mostly created to help create cleaner code
class Zip():
    def __init__(self, id):
        self.id = id
        self.route = []
        self.return_time = 0

class ZipScheduler():
    def __init__(self):
        self.base = (0,0) # base coordinates
        self.num_of_zips = 10 # max zips available
        self.inactive_zips = [] # currently inactive zips
        self.active_zips = [] # currently active zips
        self.max_capacity = 3 # max number of packes a zip can hold  
        self.flight_speed = 30 # m/s
        self.max_range = 160000 #meters
        self.hospital_locations = {} # hashmap of {hospital name : HospitalNode}
        self.current_orders = [] # queued orders
        self.emergencies = [] # queued emergencies
        self.resupplies = [] # queued resupplies
        self.stall_start = 0 # Keep track of when a stall starts
        self.active = False # Keep track of if a stall is occuring
        self.indicies_to_delete = [] # used to delete orders from current_orders
        self.num_emergencies = 0 # keeps track of the current number of emergencies
        self.zip_return_time = [] # helps to check the activeness of a drone
        self.all_stalls = [] # statistic for checking how long a drone an order is waiting (used to get avg stall)
        self.stall = 0 # used to keep track of the stall start time
        self.current_time = 0 # seconds since midnight
        self.next_schedule_time = 0 # the next time the schedule_next_flight() function needs to be called 
        self.create_zips() 
        self.grab_hospital_data()
        self.process_orders()
        print("Avgerage Stall:", sum(self.all_stalls) / len(self.all_stalls))

    # 
    # Used to create Zip objects
    # Places them all in inactive_zips
    # 
    def create_zips(self):
        for x in range(self.num_of_zips):
            newZip = Zip(x)
            self.inactive_zips.append(newZip)
    
    # 
    # Used to parse the hospitals.csv
    # Populates the hospital_locations dictionary with {hospital name : HospitalNode} pairs
    # Calls the create_graph function
    #
    def grab_hospital_data(self):
        with open('hospitals.csv', newline='') as csvfile:
            data = list(csv.reader(csvfile))
        for location in data:
            newNode = HospitalNode(int(location[1]), int(location[2]))
            newNode.name = (location[0])
            self.hospital_locations[location[0]] = newNode
        self.create_graph(self.hospital_locations)
        return
    
    # 
    # This is simply used to find the Euclidean Distance between two coordinates
    #
    def euclidean_distance(self, point1, point2):
        return np.linalg.norm(np.array(point1)-np.array(point2))

    #
    # Used to attach each node to each other possible node in the route
    # by checking if it has reached it's max range.
    #
    def create_graph(self, nodes):
        node_list = [node for node in nodes.values()]
        for i, node in enumerate(node_list):
            distance_left = self.max_range
            distance_left -= node.distance_to_base
            for adjacent_node in node_list[i:]:
                distance_to_next_node = self.euclidean_distance(np.array((adjacent_node.x, adjacent_node.y)),np.array((node.x, node.y)))
                next_node_to_base = adjacent_node.distance_to_base
                if (distance_to_next_node + next_node_to_base <= distance_left):
                    # node and adjacent nodes both lie in eachothers set_of_adjacent_hospitals
                    # This is so the double for loop does not need to loop twice
                    node.set_of_adjacent_hospitals.add(adjacent_node.name)
                    adjacent_node.set_of_adjacent_hospitals.add(node.name)
            print(node.name, node.set_of_adjacent_hospitals)
        return

    #
    # This is the function used to read the incoming orders from orders.csv
    # Assigns a self.current_time as the time at which the next order is processed
    # 
    # Uses the self.next_schedule_time to made sure that 60s has passed before 
    # the self.schedule_next_flight() is called or to stall the function if necessary
    # 
    # num_orders and max_orders can be used to step through a certain number of orders
    # try changing max_orders to 15 to only see the first 16 orders get processed
    # 
    def process_orders(self):
        with open('orders.csv') as csvfile:
            line = csvfile.readline()
            self.current_time = int(line.split(',')[0])
            self.stall = self.current_time
            self.next_schedule_time = self.current_time + 60
            num_orders = 0
            max_orders = float('inf') # change this to step through a certain number of orders
            self.active = False
            while line:
                data = line.split(',')
                data[2] = data[2][:-1]
                self.current_time = int(data[0])
                self.queue_order(int(data[0]), data[1][1:], data[2][1:])
                next_route = self.schedule_next_flight(self.current_time)
                print("Next Route:", next_route)
                if not self.active and next_route == None:
                    self.stall_start = self.current_time
                    self.active = True
                if next_route != None:
                    self.active = False
                line = csvfile.readline()
                self.visualize_active_zips()
                num_orders += 1
                if num_orders == max_orders:
                    break
        #
        # This code is commented out as it was added just to finish off the remaining jobs at 
        # the end of the file. It can be uncommented to see the active_zips become inactive
        #
        # while len(self.active_zips) != 0:
        #     self.current_time += 60
        #     self.schedule_next_flight(self.current_time)
        #     if self.current_orders and len(self.inactive_zips) > 0:
        #         time_check = self.calculate_time_till_return(self.current_orders[0:1])
        #         self.send_current_order(self.current_time, time_check, self.current_orders[0:1])
        #     self.visualize_active_zips()
        
        # Used to delete the first stall
        del self.all_stalls[0]


    # 
    # This function goes through each hospital in the route and calculates the distance to complete the route
    # It uses a distance_left variable and finds the distance between two nodes using the euclidean_distance func
    # Checks with the set_of_adjacent_hospitals to make sure the next hospital is even potentially reachable
    #
    def calculate_time_till_return(self, route):
        if not route:
            return -1
        route = [hospital_name for priority, hospital_name in route]
        # Sorting the route to make sure cases where the same hospital occurs twice distance is not double counted
        # Ex: ['Kigeme', 'Nyanza', 'Nyanza']
        route.sort()
        distance_left = self.max_range
        starting_location = np.array((0,0))
        for i, hospital in enumerate(route):
            hospital_object = self.hospital_locations[hospital]
            next_location = np.array((hospital_object.x, hospital_object.y))
            distance_left -= self.euclidean_distance(next_location, starting_location)
            starting_location = next_location
            if distance_left < 0 or (i + 1 < len(route) and route[i + 1] not in hospital_object.set_of_adjacent_hospitals):
                return -1

        # Final trip to the base
        distance_left -= self.hospital_locations[route[-1]].distance_to_base
        if distance_left < 0:
            return -1        
        
        # Getting the total distance traveled to return time drone is gone
        distance_traveled = (self.max_range - distance_left)
        time_taken = distance_traveled / self.flight_speed
        return time_taken

        
    #
    # Simply uses a queue to keep track of incoming orders
    # Assigns a priority of 0 if an Emergency
    # Assigns a priority of 1 if a Resupply
    # Places all the emergencies in front of the resupplies
    # 
    def queue_order(self, received_time, hospital, priority):
        # (priority, hospital_name)
        if priority == "Emergency":
            self.emergencies.append((0,hospital))
        else:
            self.resupplies.append((1,hospital))
        self.current_orders = self.emergencies + self.resupplies
        return

    #
    # Sending out an order
    # Making an inactive zip an active zip
    #
    def send_current_order(self, current_time, time_check, min_orders):
        next_zip = self.inactive_zips.pop(0)
        next_zip.route = [hospital_name for priority, hospital_name in list(min_orders)]
        next_zip.return_time = current_time + time_check
        self.active_zips.append(next_zip)
        self.zip_return_time.append(next_zip.return_time)
        self.current_orders = []

    #
    # Used purely for debugging, very useful in checking which drones are active with what route
    #
    def visualize_active_zips(self):
        print("Current Time: ", self.current_time)
        for z in self.active_zips:
            print("Zip ID:", z.id, "End Time:", z.return_time, "Zip Route:", z.route)
        print()
        print()

    #
    # Called every time schedule_next_flight() is called to make active drones that 
    # have returned to base inactive again and ready to be deployed
    #
    def check_activeness(self, current_time):
        if self.zip_return_time:
            for ind, return_time in enumerate(self.zip_return_time):
                if return_time <= current_time:
                    self.zip_return_time.pop(ind)
                    new_inactive_zip = self.active_zips.pop(ind)
                    new_inactive_zip.route = []
                    new_inactive_zip.return_time = 0
                    self.inactive_zips.append(new_inactive_zip)

    #
    # Used to determine the set of orders to send on the next mission to minimize the amount of 
    # time spent resupplying and maximize the number of hospitals delivered too
    #
    # num_packages - number of packages that can still be added to trip
    # locked - packages that must be sent on this trip
    # working_Set - packages that can be iterated through to find the most optimal route
    #
    def best_combination(self, num_packages, locked, working_set):
        curr_order = locked
        min_time = float('inf')
        min_orders = []
        self.indicies_to_delete = []
        if num_packages == 0:
            curr_time = self.calculate_time_till_return(curr_order)
            min_orders = curr_order.copy()
        if num_packages == 1:
            for i, package in enumerate(working_set):
                curr_order.append(package)
                curr_time = self.calculate_time_till_return(curr_order)
                if curr_time != -1 and curr_time < min_time:
                    min_time = curr_time
                    min_orders = curr_order.copy()
                    self.indicies_to_delete = []
                    self.indicies_to_delete.append(self.num_emergencies + i)
                curr_order.pop()
        if num_packages == 2:
            for i, package in enumerate(working_set):
                curr_order.append(package)
                for j, second_package in enumerate(working_set[i + 1:]):
                    curr_order.append(second_package)
                    curr_time = self.calculate_time_till_return(curr_order)
                    if curr_time != -1 and curr_time < min_time:
                        min_time = curr_time
                        min_orders = curr_order.copy()
                        self.indicies_to_delete = []
                        self.indicies_to_delete.append(self.num_emergencies + i)
                        self.indicies_to_delete.append(self.num_emergencies + i + j + 1)
                    curr_order.pop()
                curr_order.pop()
        if num_packages == 3:
            for i, package in enumerate(working_set):
                curr_order.append(package)
                for j, second_package in enumerate(working_set[i + 1:]):
                    curr_order.append(second_package)
                    for k, third_package in enumerate(working_set[i + j + 2:]):
                        curr_order.append(third_package)
                        curr_time = self.calculate_time_till_return(curr_order)
                        if curr_time != -1 and curr_time < min_time:
                            min_time = curr_time
                            min_orders = curr_order.copy()
                            self.indicies_to_delete = []
                            self.indicies_to_delete.append(self.num_emergencies + i)
                            self.indicies_to_delete.append(self.num_emergencies + i + j + 1)
                            self.indicies_to_delete.append(self.num_emergencies + i + j + k + 2)
                        curr_order.pop()
                    curr_order.pop()
                curr_order.pop()
        return min_orders if len(min_orders) > 0 else []
                        
    # 
    # Creates the next order based on a couple principles
    # If there is an emergency, send as many emergencies as possible
    # Then try to maxmize the payload size, want to send 3 packages at a time
    # Finally, if there is an emergency, always send a drone
    # If there are only resupplies, send a drone only if a fully stocked drone can be sent
    #
    # three_emergencies, two_emergencies, one_emergency, and no_emergency are structured as such:
    #   Ordered from most desirable to least desirable
    #   (emergency packages, resupply packages)
    
    def next_order(self, current_time, emergencies, resupplies):
        min_orders = []
        three_emergencies = [(3,0),(2,1),(2,0),(1,2),(1,1)]
        two_emergencies = [(2,1),(2,0),(1,2),(1,1)]
        one_emergency = [(1,2),(1,1)]
        no_emergency = [(0,3)]
        emergency_indicies, final_emergency_indicies = [], []
        resupply_indicies, final_resupply_indicies = [], []
        select = []
        if len(emergencies) >= 3:
            select = three_emergencies
        elif len(emergencies) == 2:
            select = two_emergencies
        elif len(emergencies) == 1:
            select = one_emergency
        elif len(emergencies) == 0:
            select = no_emergency
        # Called the best_combination() function until it finds a viable route
        for E, R in select:
            best_emergencies = self.best_combination(E,[],emergencies)
            emergency_indicies = [index - self.num_emergencies for index in self.indicies_to_delete]
            if len(resupplies) >= R:
                min_orders = self.best_combination(R,best_emergencies,resupplies)
                resupply_indicies = [index - self.num_emergencies for index in self.indicies_to_delete]
            if len(min_orders) != 0:
                final_emergency_indicies = emergency_indicies
                final_resupply_indicies = resupply_indicies
                break
        # appending any of the emergency package indicies 
        for index in final_emergency_indicies:
            self.indicies_to_delete.append(index)
        
        #
        # If a viable path is found, delete all of the orders being send from
        # the emergencies, resupplies and current_orders queues
        # Then send the order 
        #
        if len(min_orders) > 0:
            temp, emergency_temp, resupply_temp = [], [], []
            time_check = self.calculate_time_till_return(min_orders)
            for i in range(len(self.current_orders)):
                if i not in set(self.indicies_to_delete): temp.append(self.current_orders[i])
            for i in range(len(self.emergencies)):
                if i not in set(final_emergency_indicies): emergency_temp.append(self.emergencies[i])
            for i in range(len(self.resupplies)):
                if i not in set(final_resupply_indicies): resupply_temp.append(self.resupplies[i])
            self.send_current_order(current_time, time_check, min_orders)
            self.current_orders = temp
            self.emergencies = emergency_temp
            self.resupplies = resupply_temp
        return min_orders


    #
    # Is called everytime a new order comes in
    # Will return None if there are no inactive_zips or if 60seconds have not passed
    # Then, will call the next_route() function to determine which route to output next
    # Returns a route if one is viable
    # 
    def schedule_next_flight(self, current_time):
        # This is the only way an active zip becomes inactive
        # make all the active zips that have returned to base active again
        self.check_activeness(current_time)

        # Setting the stall time
        if self.active:
            self.stall = current_time - self.stall_start

        # stall if it hasn't been 60 seconds or if no available zips
        if current_time + 60 < self.next_schedule_time or len(self.inactive_zips) == 0:
            self.next_schedule_time = current_time + 60
            return None
        
        # Used to add to list of all stalls 
        if not self.active and len(self.emergencies) != 0:
            self.all_stalls.append(self.stall)

        self.num_emergencies = len(self.emergencies)
        next_route = self.next_order(current_time, self.emergencies, self.resupplies)
        return next_route if len(next_route) > 0 else None

ZipScheduler()
