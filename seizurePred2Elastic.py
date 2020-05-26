import schedule
import time
import requests
import json
import pickle as p
import pandas as pd
from elasticsearch import Elasticsearch
from datetime import datetime
import codecs
import csv
import pandas as pd
import numpy as np

es = Elasticsearch([{'host': 'localhost', 'port': 9200}])
i = 0
def job(data):
    url = 'http://0.0.0.0:5000/api/'
    demo_data = data
    X_test = demo_data.iloc[:,:672]
    scalerFile = './scaler.pickle'
    scaler = p.load(open(scalerFile, 'rb'))
    X_test_scaled = scaler.transform(X_test)
    j = 0
    data = X_test_scaled[j].tolist()
    j_data = json.dumps(data)
    headers = {'content-type': 'application/json', 'Accept-Charset': 'UTF-8'}
    r = requests.post(url, data=j_data, headers=headers)
    return (int(r.text[1]))

# Define Body Parts List and Segment Lists  
full_body = []

def partPosNOrien(part,source):
    source.append(part+"x")
    source.append(part+"y")
    source.append(part+"z")
    source.append("Pitch_"+part+"_orientation")
    source.append("Yaw_"+part+"_orientation")
    source.append("Roll_"+part+"_orientation")
    return source
body_parts = ["pelvis", "naval", "chest", "neck", "clavicle_left", "shoulder_left", "elbow_left", "wrist_left", "hand_left", "handtip_left", "thumb_left", "clavicle_right", "shoulder_right", "elbow_right", "wrist_right", "hand_right", "handtip_right", "thumb_right", "hip_left", "knee_left", "ankle_left", "foot_left", "hip_right", "knee_right", "ankle_right", "foot_right", "head", "nose", "eye_left", "ear_left", "eye_right", "ear_right"]
for part in body_parts:
    full_body = partPosNOrien(part,full_body)
    
# Calculate velocity and acceleration of joints then generate table of std, skew, and kurt of data based on predefined timeframe

def normal_body_dist_data(df,full_body,dist_timeFrame):
    normal_body_df = df[full_body]

    normal_body_positions = []
    normal_body_x_positions = []
    normal_body_x_velocity_dict = {}
    normal_body_x_acceleration_dict = {}
    normal_body_y_positions = []
    normal_body_y_velocity_dict = {}
    normal_body_y_acceleration_dict = {}
    normal_body_z_positions = []
    normal_body_z_velocity_dict = {}
    normal_body_z_acceleration_dict = {}
    normal_body_orientations = []
    # Create an completely empty Dataframe without any column names, indices or data
    dfObj = pd.DataFrame()
    vel_time = np.divide(list(df["Azure_time"])[1:],1000000).tolist()
    accl_time = np.divide(list(df["Azure_time"])[2:],1000000).tolist()
    for name in full_body:
        if "orientation" not in name:
            normal_body_positions.append(name)
            if name[-1] == 'x':
                normal_body_x_positions.append(name)
                normal_body_x_velocity_dict[name] = []
                normal_body_x_acceleration_dict[name] =[]
                for i in range(0,len(list(df[name]))-1):
                    delta_time = (list(df["Azure_time"])[i+1]-list(df["Azure_time"])[i])/1000000
                    delta_x = (list(df[name])[i+1]-list(df[name])[i])
                    normal_body_x_velocity_dict[name].append(delta_x/delta_time)
                dfObj[name+"_velocity"]=normal_body_x_velocity_dict[name][1:]
                for i in range(0,len(list(df[name]))-2):
                    delta_time = (list(df["Azure_time"])[i+1]-list(df["Azure_time"])[i])/1000000
                    delta_vx = (normal_body_x_velocity_dict[name][i+1]-normal_body_x_velocity_dict[name][i])
                    normal_body_x_acceleration_dict[name].append(delta_vx/delta_time)
                dfObj[name+"_acceleration"]=normal_body_x_acceleration_dict[name]
            elif name[-1] == 'y':
                normal_body_y_positions.append(name)
                normal_body_y_velocity_dict[name] = []
                normal_body_y_acceleration_dict[name] =[]
                for i in range(0,len(list(df[name]))-1):
                    delta_time = (list(df["Azure_time"])[i+1]-list(df["Azure_time"])[i])/1000000
                    delta_y = (list(df[name])[i+1]-list(df[name])[i])
                    normal_body_y_velocity_dict[name].append(delta_y/delta_time)
                dfObj[name+"_velocity"]=normal_body_y_velocity_dict[name][1:]
                for i in range(0,len(list(df[name]))-2):
                    delta_time = (list(df["Azure_time"])[i+1]-list(df["Azure_time"])[i])/1000000
                    delta_vy = (normal_body_y_velocity_dict[name][i+1]-normal_body_y_velocity_dict[name][i])
                    normal_body_y_acceleration_dict[name].append(delta_vy/delta_time)
                dfObj[name+"_acceleration"]=normal_body_y_acceleration_dict[name]
            else:
                normal_body_z_positions.append(name)
                normal_body_z_velocity_dict[name] = []
                normal_body_z_acceleration_dict[name] =[]
                for i in range(0,len(list(df[name]))-1):
                    delta_time = (list(df["Azure_time"])[i+1]-list(df["Azure_time"])[i])/1000000
                    delta_z = (list(df[name])[i+1]-list(df[name])[i])
                    normal_body_z_velocity_dict[name].append(delta_z/delta_time)
                dfObj[name+"_velocity"]=normal_body_z_velocity_dict[name][1:]
                for i in range(0,len(list(df[name]))-2):
                    delta_time = (list(df["Azure_time"])[i+1]-list(df["Azure_time"])[i])/1000000
                    delta_z = (normal_body_z_velocity_dict[name][i+1]-normal_body_z_velocity_dict[name][i])
                    normal_body_z_acceleration_dict[name].append(delta_z/delta_time)
                dfObj[name+"_acceleration"]=normal_body_z_acceleration_dict[name]
        if "orientation" in name:
            normal_body_orientations.append(name)
    normal_body_x_positions_df = normal_body_df[normal_body_x_positions]
    normal_body_y_positions_df = normal_body_df[normal_body_y_positions]
    normal_body_z_positions_df = normal_body_df[normal_body_z_positions]
    # Create an completely empty Dataframe without any column names, indices or data
    dfObj2 = pd.DataFrame()

    for name in list(dfObj):
        std_list = []
        skew_list = []
        kurt_list = []
        accel_change_count_list = []
        for i in range(dist_timeFrame-1,dfObj.shape[0]):
            accel_change_count = 0
            neg_accel_count = 0
            pos_accel_count = 0
            std_list.append(dfObj[name].iloc[i-(dist_timeFrame-1):i].std())
            skew_list.append(dfObj[name].iloc[i-(dist_timeFrame-1):i].skew())
            kurt_list.append(dfObj[name].iloc[i-(dist_timeFrame-1):i].kurt())
            for j in range(i-dist_timeFrame,i):
                if name[-13] == '_' and dfObj[name].iloc[j] > 500:
                    pos_accel_count +=1
                elif name[-13] == '_' and dfObj[name].iloc[j] < -500:
                    neg_accel_count +=1
            accel_change_count = abs(pos_accel_count - neg_accel_count)
            accel_change_count_list.append(accel_change_count)
        dfObj2[name+"_std"] = std_list
        dfObj2[name+"_skew"] = skew_list
        dfObj2[name+"_kurt"] = kurt_list
        if name[-1] == 'n':
            dfObj2[name+"_accel_change_count"] = accel_change_count_list
    dfObj2['response'] = "normal"
    return dfObj2

#(Real-time) Load data, run predictions, create elasticsearch doc with prediction and timestamp
k = 0
for i in range(0,5000):
    df = pd.read_csv("main.csv")
    if df.shape[0]>45:
        num = 47 +i
        results = normal_body_dist_data(df[i:num],full_body,45)
        if results.shape[0] == 1:
            k += 1
            # _source data for the Elasticsearch document
            doc_source = {
                "prediction": job(results),
                # must be string for JSON seralization
                "timestamp": datetime.utcnow().isoformat()
            }
            # build the Elasticsearch document from a dict
            build_doc = {}
            build_doc["_index"] = "test_index"
            build_doc["_id"] = i+1
            build_doc["doc_type"] = "_doc" # doc type deprecated
            build_doc["_source"] = doc_source
            try:
                # create JSON string of doc _source data
                json_source = json.dumps(build_doc["_source"])

                # get the dict object's _id
                json_id = build_doc["_id"]

                # make an API call to the Elasticsearch cluster
                response = es.index(
                    index = 'test_index',
                    doc_type = '_doc',
                    id = json_id,
                    body = json_source
                )

                # print a pretty response to the index() method call response
                print ("\nclient.index response:", json.dumps(response, indent=4))

            except Exception as error:
                print ("Error type:", type(error))
                print ("client.index() ERROR:", error)
            i += 1
