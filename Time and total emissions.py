import os
import numpy as np
import xml.dom.minidom as xmldom

def extractVeh():
    vehWait=[]
    pedWait=[]
    xml_filepath = os.path.abspath("./tripinfo.xml")
    dom_obj = xmldom.parse(xml_filepath)
    element_obj = dom_obj.documentElement
    sub_element_obj = element_obj.getElementsByTagName("tripinfo")
    for i in range(len(sub_element_obj)):
        vehWait.append(float(sub_element_obj[i].getAttribute("waitingTime")))
    veh_mean=np.mean(vehWait)
    veh_std=np.std(vehWait,ddof=1)
    sub_element_obj_1 = element_obj.getElementsByTagName("walk")
    for i in range(len(sub_element_obj_1)):
        pedWait.append(float(sub_element_obj_1[i].getAttribute("timeLoss")))
    ped_mean=np.mean(pedWait)
    ped_std=np.std(pedWait,ddof=1)
    return veh_mean,veh_std,ped_mean,ped_std

def extractFuelEmissions():
    fuel_sum = 0
    emissions_sum = 0
    xml_filepath = os.path.abspath("./tripinfo.xml")
    dom_obj = xmldom.parse(xml_filepath)
    element_obj = dom_obj.documentElement
    sub_element_obj = element_obj.getElementsByTagName("tripinfo")
    for i in range(len(sub_element_obj)):
        emissions_node = sub_element_obj[i].getElementsByTagName("emissions")[0]
        fuel_sum += float(emissions_node.getAttribute("fuel_abs"))
        emissions_sum += (float(emissions_node.getAttribute("CO2_abs")) + float(emissions_node.getAttribute("CO_abs")) + float(emissions_node.getAttribute("HC_abs")) + float(emissions_node.getAttribute("PMx_abs")) + float(emissions_node.getAttribute("NOx_abs"))) # 单位为mg
    fuel_total = fuel_sum /10000
    emissions_total = emissions_sum/10000

    return fuel_total, emissions_total


if __name__ == "__main__":
    veh_mean, veh_std, ped_mean, ped_std = extractVeh()
    print("Average Wait Time for Vehicles: " + str(veh_mean))
    print("Standard Deviation of Wait Time for Vehicles: " + str(veh_std))

    print("Average Wait Time for Pedestrians: " + str(ped_mean))
    print("Standard Deviation of Wait Time for Pedestrians: " + str(ped_std))

    fuel_total, emissions_total = extractFuelEmissions()
    print("Total Fuel Consumption for Vehicles: %.2f" % fuel_total)
    print("Total Gas Emissions from Vehicles: %.2f" % emissions_total)


