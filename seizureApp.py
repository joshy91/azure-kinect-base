from flask import Flask, request, redirect, url_for, flash, jsonify
import numpy as np
import pickle as p
import json
# import test.py as test

app = Flask(__name__)


@app.route('/api/', methods=['POST'])
# Generate seizure prediction value based on input data from POST method
def makecalc():
    data = request.get_json()
    new_list = []
    for item in data:
        new_list.append(float(item))
    inputData = np.asarray(new_list)
    inputData = inputData.reshape(1,672)
    y_prob = model.predict_proba(inputData)
    y_pred = y_prob[:,1]
    for i in range(0,y_pred.size):
        if y_pred[i]>.3:  
            y_pred[i]=1
        else:
            y_pred[i]=0
    prediction = np.array2string(y_pred)
    
    return prediction

# Load model from saved pickle object
if __name__ == '__main__':
    modelfile = './final_prediction.pickle'
    model = p.load(open(modelfile, 'rb'))
    app.run(debug=True, host='0.0.0.0')
#     while True:
#         test.prediction()
#         time.sleep(10)
