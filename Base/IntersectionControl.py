import xml.etree.ElementTree as ET
import random
import os
import math

import traci
import sumolib



class IntersectionControl:
    def __init__(self, inter_ID, lane_num, loop_num, sig_num, sig_phase):
        self.inter_ID = inter_ID
        self.lane_num = lane_num
        self.loop_num = loop_num
        self.sig_num = sig_num
        self.sig_phase = sig_phase

        self.flow_num = [["1","2","3"], ["4"] , ["5","6","7"], ["8"], ["9","10","11"], ["12"], ["13","14","15"], ["16"]]
        self.flow_list = [320, 200, 380, 230, 370, 250, 400, 260]
        self.red_sec = 3
        self.SIMUL_STEP = 1
        self.TRAFFIC_SCALE = 1

        self.cur_dir = os.getcwd()
        self.route_dir = 'simpleRou.rou.xml'
        self.config_dir = 'simpleConfig.sumocfg'
        self.detector_dir = 'simpleDet.det.xml'

        self.ROUTE_FILE = os.path.join(self.cur_dir, self.route_dir)
        self.CONFIG_FILE = os.path.join(self.cur_dir, self.config_dir)
        self.DETECTOR_FILE = os.path.join(self.cur_dir, self.detector_dir)

        self.tree = ET.parse(self.ROUTE_FILE)
        self.root = self.tree.getroot()

        # 'sumo-gui' or 'sumo'
        self.SUMO_CMD = [sumolib.checkBinary('sumo'), '-c', self.CONFIG_FILE, '--start',
            '--step-length', str(self.SIMUL_STEP), '--scale', str(self.TRAFFIC_SCALE), 
            '--additional-files', self.DETECTOR_FILE, '--quit-on-end']
        
    def set_flow(self, flow_id, new_volume):
        for flow in self.root.iter("flow"):
            for i in range(len(flow_id)):
                if flow.attrib.get("id") == flow_id[i]:
                    set_volume = random.randint(new_volume, new_volume+30)
                    flow.attrib["number"] = str(set_volume)
        self.tree.write(self.ROUTE_FILE)
    
    def get_q(self, lane_num):
        q_ = []
        for i in range(lane_num):
            q_.append(traci.lanearea.getJamLengthVehicle(str(i)))
        return q_
    
    def get_vehID(self, loop_num):
        veh_ID = []
        for i in range(loop_num):
            veh_ID.append(traci.inductionloop.getLastStepVehicleIDs(str(i)))
        return veh_ID

    def set_signal(self, phase_num, phase_sec):
        traci.trafficlight.setPhase(self.inter_ID, phase_num)
        traci.trafficlight.setPhaseDuration(self.inter_ID, phase_sec)
        
    def start_phase(self, phase_num, phase_sec):
        ID_list = []
        self.set_signal(phase_num, phase_sec)

        for i in range(phase_sec + self.red_sec):
            traci.simulationStep()
            veh_ID = self.get_vehID(self.loop_num)
            ID_list.append(veh_ID)

        ID = sum(ID_list, [])
        throughput = len(set(ID))-1
        return throughput, phase_sec+self.red_sec

    def start_ep(self, max_episode):
        throughput_list = []
        for ep in range(int(max_episode)):
            ep_throughput = []
            ep_phase = []
            traci.start(self.SUMO_CMD)

            for f in range(len(self.flow_num)):
                self.set_flow(self.flow_num[f], self.flow_list[f])

            cycle = sum(self.sig_phase) + (len(self.sig_phase)*self.red_sec)

            traci.simulationStep()
            for i in range(cycle*3):
                traci.simulationStep()

            sim_num = math.trunc(3600/cycle)-3
            for e in range(sim_num):
                for s in range(len(self.sig_num)):
                    self.set_signal(self.sig_num[s], self.sig_phase[s])
                    throughput, phase_sec = self.start_phase(self.sig_num[s], self.sig_phase[s])
                    ep_throughput.append(throughput)
                    ep_phase.append(phase_sec)
            throughput_list.append((sum(ep_throughput)/sum(ep_phase)))
            traci.close()
            print('Episode: ', ep+1, 'Throughput: ', (sum(ep_throughput)/sum(ep_phase)))
        return throughput_list
