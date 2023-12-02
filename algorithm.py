from __future__ import absolute_import
from __future__ import print_function
import os
import sys
import optparse
import torch
import xml.dom.minidom as xmldom
priority = None
THISDIR = os.path.dirname(__file__)
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci
from sumolib import checkBinary

MIN_GREEN_TIME = 15
VEHICLE_GREEN_PHASE = 0
PEDESTRIAN_GREEN_PHASE = 2
TLSID = 'C'
WALKINGAREAS = [':C_w3', ':C_w2', ':C_w1', ':C_w0']
CROSSINGS = [':C_c2', ':C_c1', ':C_c0', ':C_c3']
DETECTORS = ["e2det_SC_3", "e2det_SC_2", "e2det_SC_1", "e2det_EC_3", "e2det_EC_2", "e2det_EC_1", "e2det_NC_3", "e2det_NC_2", "e2det_NC_1", "e2det_WC_3", "e2det_WC_2", "e2det_WC_1"]

TLS_ID = '0'
GIDX = [1, 4, 7, 10, 12, 13, 14, 15]
last_light = [0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0]
def setTrafficlight(schedule,step):
    light_str = ''
    global last_light
    print('last_light',last_light)
    if step % 18< 3:
        for idx, light in enumerate(schedule):
            if idx < 12:
                if light == 0:
                    if last_light[idx] == 0:
                        light_str += 'r'
                    else:
                        light_str += 'y'
                elif light == 1:
                    if idx in GIDX:
                        light_str += 'G'
                    else:
                        light_str += 'g'
                else:
                    light_str += 'G'
            else:
                if light == 0:
                    light_str += 'r'
                elif light == 1:
                    if idx in GIDX:
                        light_str += 'G'
                    else:
                        light_str += 'g'
                else:
                    light_str += 'G'
    else:
        for idx, light in enumerate(schedule):

            if light == 0:
                light_str += 'r'
                last_light[idx] = 0
            elif light == 1:
                last_light[idx] = 1
                if idx in GIDX:
                    light_str += 'G'
                else:
                    light_str += 'g'
            else:
                light_str += 'G'
                last_light[idx] = 1
    print('last_light',last_light)
    print('light state: ', light_str)
    traci.trafficlight.setRedYellowGreenState(TLS_ID, light_str)
def run():
    global priority
    step = 0
    penalty = torch.zeros(16)
    priority = torch.zeros(16)
    light = torch.zeros(16)
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        if (step % 18 == 0):
            peoWaitTime,pedest=checkWaitingPersons()
            print('pedest:', pedest.int().data)
            vehicle=torch.zeros(12)
            for idx, det in enumerate(DETECTORS):
                vehicle[idx] = (traci.lanearea.getLastStepVehicleNumber(det) > 0)
            emer = traci.vehicle.getIDList()
            print('ID', emer)
            emer = [float(x) for x in emer]
            print(emer)
            for i in emer[::-1]:
                if i < 4:
                    emer.remove(i)
            emer = [str(x) for x in emer]
            post = []
            for i in emer:
                post.append(traci.vehicle.getLaneID(i))
            print(post)
            new_post = []
            for item in post:
                if not item.startswith("C"):
                    new_post.append(item)
            post = new_post
            print(post)
            dic = {'NC_1': 0, 'NC_2': 1, 'NC_3': 2,'EC_1': 3,'EC_2': 4,'EC_3': 5,'SC_1': 6,'SC_2': 7,'SC_3': 8,'WC_1': 9,'WC_2': 10,'WC_3': 11}
            print(dic)
            m=[]
            for item in new_post:
                 print(item)
                 if item in list(dic.keys()):
                     m.append(dic.get(item))
                     post = m
                     print(post)
            print('vehicle:', vehicle.int().data)
            traffic = torch.cat((vehicle, pedest*1), 0)  # 按维数0拼接（竖着拼）
            index_t = [0, 1, 2, 12, 3, 4, 5, 13, 6, 7, 8, 14, 9, 10, 11, 15]
            traffic = traffic[index_t]
            print('traffic:', traffic.int().data)
            print('light:',light)
            light_shift = 1 - light
            penalty = penalty * light_shift + 1
            penal_lift = [penalty * 2, penalty ** 2, torch.exp(penalty)]
            penalty_shift = penal_lift[0]
            penalty_shift = traffic * penalty_shift + traffic  # 车多 红灯->惩罚值大
            print('penalty:', penalty_shift)
            schedule, light = geneSchedule(penalty_shift,m)
        setTrafficlight(schedule,step)
        step += 1
    traci.close()
    sys.stdout.flush()
def checkWaitingPersons():
    peopleWaitingTime = 0
    persons = torch.zeros(4)
    for edge in WALKINGAREAS:
        index = 0
        peds = traci.edge.getLastStepPersonIDs(edge)
        for ped in peds:
            peopleWaitingTime+=traci.person.getWaitingTime(ped)
            if (traci.person.getNextEdge(ped) in CROSSINGS):
                    persons[CROSSINGS.index(traci.person.getNextEdge(ped))] = 1
        index+=1
    return peopleWaitingTime,persons
def averageWaiting(DETECTORS, people):
    veh_num=0
    speed=0
    for dec in DETECTORS:
        veh_num+=traci.lanearea.getLastStepHaltingNumber(dec)
        speed+=traci.lanearea.getLastStepMeanSpeed(dec)
    ped_num=torch.mean(people)*len(people.numpy().tolist())
    averageWait=veh_num+1.5*ped_num
    averageSpeed=speed/len(DETECTORS)
    return averageWait,averageSpeed,ped_num
def totalWaiting():
    peo_time,_=checkWaitingPersons()
    veh_time=extractVeh()
    return peo_time+veh_time
def extractVeh():
    vehWaitSum=0
    xml_filepath = os.path.abspath("./tripinfo.xml")
    dom_obj = xmldom.parse(xml_filepath)
    element_obj = dom_obj.documentElement
    sub_element_obj = element_obj.getElementsByTagName("tripinfo")
    for i in range(len(sub_element_obj)):
        vehWaitSum += float(sub_element_obj[i].getAttribute("waitingTime"))
    return vehWaitSum

def get_options():
    """define options for this script and interpret the command line"""
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    optParser.add_option("-d", "--default", action="store_true",
                         default=False, help="Run the default fixed length traffic light")
    optParser.add_option("-e", "--end-time", action="store",
                         type="string", dest="end_time", default="300", help="Simulation end time")
    optParser.add_option("--ped-binomial", action="store",
                         type="string", dest="ped_binomial", default="1", help="Pedestrian binomial")
    optParser.add_option("--ped-period", action="store",
                         type="string", dest="ped_period", default="3.6", help="Pedestrian period")
    optParser.add_option("--veh-binomial", action="store",
                         type="string", dest="veh_binomial", default="1", help="Vehicle binomial")
    optParser.add_option("--veh-period", action="store",
                         type="string", dest="veh_period", default="3.6", help="Vehicle period")
    optParser.add_option("-c", "--sumocfg", action="store",
                         type="string", default="test.sumocfg", dest="sumocfg", help="The sumocfg file")
    optParser.add_option("-o", "--tripinfo", action="store",
                         type="string", default="tripinfo.xml", dest="tripinfo_xml", help="The tripinfo output file")
    options, args = optParser.parse_args()
    return options

def geneSchedule(penalty,m):
    global priority
    global last_light
    line0 = [-1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1, 0]
    line1 = [1, -1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1]
    line2 = [1, 1, -1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1]
    line3 = [0, 0, 0, -1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1]
    state_temp = torch.FloatTensor([line0, line1, line2, line3])
    state = torch.zeros(16, 16)

    for i in range(16):
        for j in range(16):
            bias = i // 4
            line = i % 4
            state[i][j] = state_temp[line][(j + 12 * bias) % 16]

    state_prob = state.clone()
    state_prob[state_prob < 0] = 0
    state = state + 1
    state_copy = state.clone()
    state_copy[state_copy != 1] = 0
    state_nol = 1 - state_copy
    light = torch.zeros(16)
    print(penalty.size())
    penalty_tensor = penalty.reshape(1, -1).repeat(16, 1)

    loss = penalty_tensor * state_copy
    print('lossb: ', loss)
    loss_sum = loss.sum(1) - penalty
    print('lossa: ', loss_sum)

    index_loss = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
    loss_line = loss_sum[index_loss]
    min_mum = min(loss_line)

    max_p = 0
    index = -1
    prob = torch.ones(16)
    for i in range(loss_line.shape[0]):
        if loss_line[i] == min_mum:
            if priority[i] > max_p:

                max_p = priority[i]
                index = i
    list = [0] * 16
    if len(m) != 0:
        print("特种车辆！")
        for i in m:
            list[i] += 1
        non_zero_indices = [i for i in range(len(list)) if list[i] != 0]
        sorted_indices = sorted(non_zero_indices, key=lambda i: list[i], reverse=True)
        light[sorted_indices[0]] = 1
        priority[sorted_indices[0]] = 0
        list[sorted_indices[0]] = 0
        prob = prob * state_prob[sorted_indices[0]]
        nol = state_nol[sorted_indices[0]]
    else:
        light[index] = 1
        priority[index] = 0
        prob = prob * state_prob[index]
        nol = state_nol[index]
    print('light',light)
    print('prob',prob)
    print()
    while (prob.sum() != 0):
        max_p = 0
        min_loss = 9999
        index = -1
        for i in range(16):
            if prob[i] > 0:
                nol_temp = nol * state_nol[i]
                noloss = 1-nol_temp
                loss = (noloss * penalty * prob).sum() - penalty[i]
                print((i, loss))
                if loss < min_loss:
                    min_loss = loss
                    index = i
                    max_p = priority[i]
                elif loss == min_loss:
                    if priority[i] > max_p:
                        min_loss = loss
                        index = i
                        max_p = priority[i]
        non_zero_indices = [i for i in range(len(list)) if list[i] != 0]
        sorted_indices = sorted(non_zero_indices, key=lambda i: list[i], reverse=True)
        if len(sorted_indices) != 0:
            light[sorted_indices[0]] = 1
            priority[sorted_indices[0]] = 0
            list[sorted_indices[0]] = 0
            prob = prob * state_prob[sorted_indices[0]]
            nol = nol * state_nol[sorted_indices[0]]
        else:
            light[index] = 1
            priority[index] = 0
            prob = prob * state_prob[index]
            nol = nol * state_nol[index]
        print('light',light)
        print('prob',prob)
        print()
    vlight = torch.zeros(12)
    plight = torch.zeros(4)
    j = 0
    k = 0
    for i in range(16):
        if i != 3 and i != 7 and i != 11 and i != 15:
            vlight[j] = light[i]
            j += 1
        else:
            plight[k] = light[i]
            k += 1
    index_v = [8,7,6,5,4,3,2,1,0,11,10,9]
    index_p = [2,1,0,3]
    vlight = vlight[index_v]
    plight = plight[index_p]

    traflight = torch.cat((vlight, plight),0)
    print('traflight',traflight)
    print('priority before increment: ', priority)
    priority += 1
    return traflight, light

def run_default():
    step = 0
    traci.trafficlight.setPhase("0", 2)
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1
    traci.close()
    sys.stdout.flush()
if __name__ == "__main__":
    options = get_options()
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
    traci.start([sumoBinary, '-c', os.path.join(options.sumocfg), '--tripinfo-output', options.tripinfo_xml])
    if options.default:
        run_default()
    else:
        run()
