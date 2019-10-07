import rospkg
import csv
import os
import cmath
from em_interface_ros.msg import ts_RadarDetection, ts_Target
from em_interface_ros.msg import ts_EgoMotion
from em_interface_ros.msg import ts_RadarContext

directories_name = ['5m_lateral_move_2019_0717_063544', '5m-80m_car_2019_0717_064611', '10m_lateral_move_2019_0717_063935',
                    '15m_lateral_move_2019_0717_064411', '20m_circle_car_2019_0717_065837', '80m-5m_car_2019_0717_064838']
directories_suffix = ['_Specific_operation_mode', '_RSP_clusterlist', '_Sensor_mounting_data', '_Vehicle_dynamic_data']

father_path = os.path.abspath('.')
path_current = os.path.join(father_path, 'tracking_interface_data')

dir_store = {}


for directory_name in directories_name:
# 每次循环就进创建一个ros message
    mylist = []
# 这里是将6个场景下的每4个文件合成一个包
    for directory_suffix in directories_suffix:
        document_whole_name = directory_name + directory_suffix
        doc_current = os.path.join(path_current, directory_name, document_whole_name)
        doc_current = doc_current + '.csv'

        if directory_suffix is '_Specific_operation_mode':
            with open(doc_current) as f:
                document_work = csv.reader(f)
                for index, row in enumerate(document_work):
                    mydict = {}
                    if index != 0:
                        mydict['uiTimeStamp'] = row[0]
                        mydict['uiCycleCounter'] = row[1]
                        mydict['eSigStatus'] = row[2]
                        mylist.append(mydict)

        if directory_suffix is '_Sensor_mounting_data':
            with open(doc_current) as f:
                document_work = csv.reader(f)
                for index, row in enumerate(document_work):
                    mydict1 = {}
                    if index != 0:
                        mydict1 = mylist[index-1]
                        mydict1['VehicleWidth'] = row[3]
                        mydict1['VehicleLength'] = row[4]
                        mydict1['OverhangFront'] = row[5]
                        mydict1['LatPos'] = row[6]
                        mydict1['LongPos'] = row[7]
                        mydict1['VertPos'] = row[8]
                        mydict1['PitchAngle'] = row[9]
                        mydict1['RollAngle'] = row[10]
                        mydict1['YawAngle'] = row[11]

        if directory_suffix is '_Vehicle_dynamic_data':
            with open(doc_current) as f:
                document_work = csv.reader(f)
                for index, row in enumerate(document_work):
                    mydict2 = {}
                    if index != 0:
                        mydict2 = mylist[index-1]
                        mydict2['Velocity'] = row[3]
                        mydict2['Accel'] = row[4]
                        mydict2['varVelocity'] = row[5]
                        mydict2['varAccel'] = row[6]
                        mydict2['YawRate'] = row[7]
                        mydict2['YVariance'] = row[8]
                        mydict2['LatAccel'] = row[9]
                        mydict2['LVariance'] = row[10]
                        mydict2['MotState'] = row[11]

        if directory_suffix is '_RSP_clusterlist':
            cyctime = 0
            with open(doc_current) as f:
                document_work = csv.reader(f)
                mylist_f_VrelRad = []
                mylist_f_RangeRad = []
                mylist_a_AzAng = []
                mylist_f_RcsRaw = []
                mylist_f_AzAngVar = []
                mylist_f_VrelRadVar = []
                mylist_f_RangeRadVar = []
                mylist_a_Pdh0 = []
                for index, row in enumerate(document_work):
                    mydict3 = {}
                    if index != 0:
                        mydict3 = mylist[cyctime]
                        if mydict3['uiTimeStamp'] == row[0]:
                            pass
                        else:
                            cyctime = cyctime + 1
                            mylist_f_VrelRad = []
                            mylist_f_RangeRad = []
                            mylist_a_AzAng = []
                            mylist_f_RcsRaw = []
                            mylist_f_AzAngVar = []
                            mylist_f_VrelRadVar = []
                            mylist_f_RangeRadVar = []
                            mylist_a_Pdh0 = []
                        mydict3 = mylist[cyctime]
                        mylist_f_VrelRad.append(row[3])
                        mylist_f_RangeRad.append(row[4])
                        mylist_a_AzAng.append(row[5])
                        mylist_f_RcsRaw.append(row[6])
                        mylist_f_AzAngVar.append(row[7])
                        mylist_f_VrelRadVar.append(row[8])
                        mylist_f_RangeRadVar.append(row[9])
                        mylist_a_Pdh0.append(row[10])
                        mydict3['f_VrelRad'] = mylist_f_VrelRad
                        mydict3['f_RangeRad'] = mylist_f_RangeRad
                        mydict3['a_AzAng'] = mylist_a_AzAng
                        mydict3['f_RcsRaw'] = mylist_f_RcsRaw
                        mydict3['f_AzAngVar'] = mylist_f_AzAngVar
                        mydict3['f_VrelRadVar'] = mylist_f_VrelRadVar
                        mydict3['f_RangeRadVar'] = mylist_f_RangeRadVar
                        mydict3['a_Pdh0'] = mylist_a_Pdh0

# 创建包完成后，将每帧的数据放到对应的 ros message
# 一共创建了3个，frame_RadarDetection \ frame_EgoMotion \ frame_RadarContext
    for mylist_single in mylist:
        frame_RadarDetection = ts_RadarDetection()
        frame_EgoMotion = ts_EgoMotion()
        frame_RadarContext = ts_RadarContext()
        frame_RadarDetection.generic_context.timestamp = mylist_single['uiTimeStamp']
        frame_RadarDetection.generic_context.cycle_counter = mylist_single['uiCycleCounter']
        frame_EgoMotion.abs_velocity.v_x = mylist_single['Velocity']
        frame_EgoMotion.abs_velocity_std.v_x_std = cmath.sqrt(float(mylist_single['varVelocity']))
        if mylist_single['MotState'] is 1:
            frame_EgoMotion.driving_status = 2
        elif mylist_single['MotState'] is 0:
            frame_EgoMotion.driving_status = 3
        frame_RadarContext.mounting_pose.x = mylist_single['LatPos']
        frame_RadarContext.mounting_pose.y = mylist_single['LongPos']
        frame_RadarContext.mounting_pose.z = mylist_single['VertPos']
        frame_RadarContext.mounting_pose.theta = mylist_single['PitchAngle']
        frame_RadarContext.mounting_pose.phi = mylist_single['RollAngle']
        frame_RadarContext.mounting_pose.psi = mylist_single['YawAngle']

        store_clusterlist_list = []
        for mylist_f_VrelRad_single in mylist_single['f_VrelRad']:
            store_clusterlist_dictory = {}
            store_clusterlist_dictory['f_VrelRad'] = mylist_f_VrelRad_single
            store_clusterlist_list.append(store_clusterlist_dictory)
        store_cycle_time = 0
        for mylist_f_RangeRad_single in mylist_single['f_RangeRad']:
            store_clusterlist_list[store_cycle_time]['f_RangeRad'] = mylist_f_RangeRad_single
            store_cycle_time = store_cycle_time + 1
        store_cycle_time = 0
        for mylist_a_AzAng_single in mylist_single['a_AzAng']:
            store_clusterlist_list[store_cycle_time]['a_AzAng'] = mylist_a_AzAng_single
            store_cycle_time = store_cycle_time + 1
        store_cycle_time = 0
        for mylist_f_RcsRaw_single in mylist_single['f_RcsRaw']:
            store_clusterlist_list[store_cycle_time]['f_RcsRaw'] = mylist_f_RcsRaw_single
            store_cycle_time = store_cycle_time + 1
        store_cycle_time = 0
        for mylist_f_AzAngVar_single in mylist_single['f_AzAngVar']:
            store_clusterlist_list[store_cycle_time]['f_AzAngVar'] = mylist_f_AzAngVar_single
            store_cycle_time = store_cycle_time + 1
        store_cycle_time = 0
        for mylist_f_VrelRadVar_single in mylist_single['f_VrelRadVar']:
            store_clusterlist_list[store_cycle_time]['f_VrelRadVar'] = mylist_f_VrelRadVar_single
            store_cycle_time = store_cycle_time + 1
        store_cycle_time = 0
        for mylist_f_RangeRadVar_single in mylist_single['f_RangeRadVar']:
            store_clusterlist_list[store_cycle_time]['f_RangeRadVar'] = mylist_f_RangeRadVar_single
            store_cycle_time = store_cycle_time + 1
        store_cycle_time = 0
        for mylist_a_Pdh0_single in mylist_single['a_Pdh0']:
            store_clusterlist_list[store_cycle_time]['a_Pdh0'] = mylist_a_Pdh0_single
            store_cycle_time = store_cycle_time + 1

        for store_clusterlist_list_single in store_clusterlist_list:
            frame_Target = ts_Target()
            frame_Target.radial_velocity = store_clusterlist_list_single['f_VrelRad']
            frame_Target.range = store_clusterlist_list_single['f_RangeRad']
            frame_Target.azimuth = store_clusterlist_list_single['a_AzAng']
            frame_Target.RCS = store_clusterlist_list_single['f_RcsRaw']
            frame_Target.azimuth_std = cmath.sqrt(float(store_clusterlist_list_single['f_AzAngVar']))
            frame_Target.radial_velocity_std = cmath.sqrt(float(store_clusterlist_list_single['f_VrelRadVar']))
            frame_Target.range_std = cmath.sqrt(float(store_clusterlist_list_single['f_RangeRadVar']))
            frame_Target.SNR = store_clusterlist_list_single['a_Pdh0']
            frame_RadarDetection.targets.append(frame_Target)
# 生成rosbag



























